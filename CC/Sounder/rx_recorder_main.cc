/*
 Copyright (c) 2018-2026, Rice University
 RENEW OPEN SOURCE LICENSE: http://renew-wireless.org/license

---------------------------------------------------------------------
 rx-recorder: timed RX capture from a SoapySDR device (Houdini SDR by
 default) to an HDF5 file.

 Opens the device, applies rate/freq/gain from the JSON config,
 activates a continuous RX stream immediately, records duration_sec
 worth of CS16 samples through a RecorderThread into HDF5, then tears
 down and reports timing gaps / overruns. No beacons, no schedules,
 no TDD -- plain receive-now-for-x-seconds.
---------------------------------------------------------------------
*/

#include <H5Cpp.h>
#include <SoapySDR/Constants.h>
#include <gflags/gflags.h>

#include <SoapySDR/Device.hpp>
#include <SoapySDR/Errors.hpp>
#include <SoapySDR/Formats.hpp>
#include <SoapySDR/Types.hpp>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <memory>
#include <vector>

#include "include/logger.h"
#include "include/macros.h"
#include "include/recorder_thread.h"
#include "include/rx_recorder_config.h"
#include "include/rx_recorder_worker.h"
#include "include/signalHandler.hpp"
#include "include/version_config.h"

DEFINE_string(conf_file, "files/rx-record.json",
              "JSON configuration file name");
DEFINE_string(storepath, "logs", "Dataset store path");

namespace {

struct DeviceUnmaker {
  void operator()(SoapySDR::Device* dev) const {
    if (dev != nullptr) {
      SoapySDR::Device::unmake(dev);
    }
  }
};
using DevicePtr = std::unique_ptr<SoapySDR::Device, DeviceUnmaker>;

struct CaptureStats {
  size_t slots_recorded = 0;
  size_t slots_dropped = 0;  // ring full: HDF5 writer could not keep up
  size_t read_timeouts = 0;
  size_t overflow_returns = 0;     // OVERFLOW returned by readStream itself
  size_t gap_events = 0;           // forward timeNs jumps between reads
  size_t backward_time_jumps = 0;  // timeNs went backward (resync/rollover)
  double est_lost_samples = 0.0;
  size_t status_overflows = 0;  // OVERFLOW events from readStreamStatus
  size_t status_other = 0;
  long long first_sample_time_ns = 0;  // timeNs of the first stamped read
  bool has_first_sample_time = false;
};

// Detects sample loss from per-read timestamps: each read's timeNs should
// equal the previous stamped time plus the samples read since. This is the
// reliable drop signal on Houdini -- kernel-level UDP drops never show up
// in the overflow counters, only as time gaps.
class TimeGapTracker {
 public:
  explicit TimeGapTracker(double rate) : rate_(rate) {}

  void onRead(int num_samps, int flags, long long time_ns,
              CaptureStats& stats) {
    if ((flags & SOAPY_SDR_HAS_TIME) != 0) {
      if (stats.has_first_sample_time == false) {
        stats.first_sample_time_ns = time_ns;
        stats.has_first_sample_time = true;
      }
      if (has_last_ == true) {
        const double expected_ns =
            static_cast<double>(last_time_ns_) +
            (static_cast<double>(samps_since_last_) * 1e9 / rate_);
        const double gap_ns = static_cast<double>(time_ns) - expected_ns;
        const double sample_period_ns = 1e9 / rate_;
        if (gap_ns > sample_period_ns) {
          stats.gap_events++;
          stats.est_lost_samples += gap_ns * rate_ / 1e9;
        } else if (gap_ns < -sample_period_ns) {
          // Time went backward (hardware-time resync / tick rollover):
          // count it separately, never subtract from the loss estimate.
          stats.backward_time_jumps++;
        }
      }
      has_last_ = true;
      last_time_ns_ = time_ns;
      samps_since_last_ = num_samps;
    } else if (has_last_ == true) {
      samps_since_last_ += num_samps;
    }
  }

 private:
  double rate_;
  bool has_last_ = false;
  long long last_time_ns_ = 0;
  size_t samps_since_last_ = 0;
};

// Reads exactly samps_per_slot samples into dest (interleaved CS16 shorts).
// Returns false when the capture should stop (fatal error or shutdown).
bool fillSlot(SoapySDR::Device* dev, SoapySDR::Stream* stream, short* dest,
              const Sounder::RxRecorderConfig& cfg, TimeGapTracker& gaps,
              CaptureStats& stats, std::atomic<bool>& running) {
  size_t filled = 0;
  size_t consecutive_timeouts = 0;
  size_t consecutive_overflows = 0;
  constexpr size_t kMaxConsecutiveTimeouts = 10;
  // Overflow returns are instant (event pops, no wait); a driver wedged in
  // a permanent overflow state would otherwise spin here forever.
  constexpr size_t kMaxConsecutiveOverflows = 10000;

  while ((filled < cfg.samps_per_slot()) && (running.load() == true) &&
         (SignalHandler::gotExitSignal() == false)) {
    void* buffs[1] = {dest + (2 * filled)};
    int flags = 0;
    long long time_ns = 0;
    const int ret =
        dev->readStream(stream, buffs, cfg.samps_per_slot() - filled, flags,
                        time_ns, cfg.rx_timeout_us());
    // A 0-element return makes no progress either -- treat it as a timeout
    // so a stalled stream cannot spin this loop forever.
    if ((ret == SOAPY_SDR_TIMEOUT) || (ret == 0)) {
      stats.read_timeouts++;
      if (++consecutive_timeouts >= kMaxConsecutiveTimeouts) {
        MLPD_ERROR("readStream: %zu consecutive timeouts, aborting capture\n",
                   consecutive_timeouts);
        running = false;
        return false;
      }
      continue;
    }
    if (ret == SOAPY_SDR_OVERFLOW) {
      stats.overflow_returns++;
      consecutive_timeouts = 0;  // the stream is alive, just lossy
      if (++consecutive_overflows >= kMaxConsecutiveOverflows) {
        MLPD_ERROR(
            "readStream: %zu consecutive overflows with no data, aborting "
            "capture\n",
            consecutive_overflows);
        running = false;
        return false;
      }
      continue;
    }
    if (ret < 0) {
      MLPD_ERROR("readStream error: %d - %s\n", ret, SoapySDR::errToStr(ret));
      running = false;
      return false;
    }
    consecutive_timeouts = 0;
    consecutive_overflows = 0;
    gaps.onRead(ret, flags, time_ns, stats);
    filled += static_cast<size_t>(ret);
  }
  return (filled == cfg.samps_per_slot());
}

int runCapture(const Sounder::RxRecorderConfig& cfg) {
  std::atomic<bool> running(true);

  // ---- Device bring-up ------------------------------------------------
  SoapySDR::Kwargs dev_args(cfg.device_args().begin(), cfg.device_args().end());
  std::cout << "Opening device: " << SoapySDR::KwargsToString(dev_args)
            << std::endl;
  DevicePtr dev(SoapySDR::Device::make(dev_args));
  if (dev == nullptr) {
    throw std::runtime_error("SoapySDR::Device::make returned null");
  }

  const size_t chan = cfg.channels().at(0);
  if (cfg.rate() > 0.0) {
    dev->setSampleRate(SOAPY_SDR_RX, chan, cfg.rate());
  }
  if (cfg.has_freq() == true) {
    dev->setFrequency(SOAPY_SDR_RX, chan, cfg.freq());
  }
  if (cfg.has_gain() == true) {
    dev->setGain(SOAPY_SDR_RX, chan, cfg.gain());
  }
  if (cfg.antenna().empty() == false) {
    dev->setAntenna(SOAPY_SDR_RX, chan, cfg.antenna());
  }

  Sounder::RxCaptureMeta meta;
  meta.hardware_key = dev->getHardwareKey();
  meta.hardware_info = SoapySDR::KwargsToString(dev->getHardwareInfo());
  meta.actual_rate = dev->getSampleRate(SOAPY_SDR_RX, chan);
  meta.actual_freq = dev->getFrequency(SOAPY_SDR_RX, chan);
  meta.actual_gain = dev->getGain(SOAPY_SDR_RX, chan);
  meta.antenna = dev->getAntenna(SOAPY_SDR_RX, chan);

  if (meta.actual_rate <= 0.0) {
    throw std::runtime_error(
        "Device reports a sample rate of 0; cannot size the capture");
  }
  const size_t total_slots = static_cast<size_t>(
      std::ceil(cfg.duration_sec() * meta.actual_rate / cfg.samps_per_slot()));
  meta.total_slots = std::max<size_t>(total_slots, 1);

  std::printf("Capture plan: %.3f s @ %.3f MSPS on channel %zu\n",
              cfg.duration_sec(), meta.actual_rate / 1e6, chan);
  std::printf("  freq %.6f MHz, gain %.1f dB, antenna '%s'\n",
              meta.actual_freq / 1e6, meta.actual_gain, meta.antenna.c_str());
  std::printf("  %zu slots x %zu samples (%.1f MB) -> %s\n", meta.total_slots,
              cfg.samps_per_slot(),
              meta.total_slots * cfg.getPacketDataLength() / 1e6,
              cfg.output_file().c_str());

  // ---- Stream + buffers ------------------------------------------------
  SoapySDR::Kwargs stream_args(cfg.stream_args().begin(),
                               cfg.stream_args().end());
  SoapySDR::Stream* stream = dev->setupStream(SOAPY_SDR_RX, SOAPY_SDR_CS16,
                                              cfg.channels(), stream_args);
  if (stream == nullptr) {
    throw std::runtime_error("setupStream failed");
  }

  const size_t packet_length = sizeof(Packet) + cfg.getPacketDataLength();
  const size_t num_slots = cfg.buffer_slots();
  SampleBuffer ring;
  ring.buffer.resize(num_slots * packet_length);
  const size_t intsize = sizeof(std::atomic_int);
  const size_t arraysize = (num_slots + intsize - 1) / intsize;
  ring.pkt_buf_inuse = new std::atomic_int[arraysize];
  std::fill_n(ring.pkt_buf_inuse, arraysize, 0);
  // Scratch packet: keeps the stream drained when the ring is full.
  std::vector<char> scratch(packet_length);

  CaptureStats stats;
  TimeGapTracker gaps(meta.actual_rate);

  {
    auto worker = std::make_unique<Sounder::RxRecorderWorker>(&cfg, meta);
    // Raw handle outlives the move: used to hand the capture-start times to
    // the worker before its Stop event (see setStartTimes's ordering note).
    Sounder::RxRecorderWorker* worker_raw = worker.get();
    // The ring claim protocol caps in-flight events at num_slots; 2x is
    // ample queue slack.
    Sounder::RecorderThread recorder(std::move(worker),
                                     cfg.getPacketDataLength(), 0 /*id*/,
                                     -1 /*no core pin*/, num_slots * 2, true);
    recorder.Start();

    // ---- Timed receive loop: activate NOW, continuous ------------------
    const int act = dev->activateStream(stream, 0 /*flags*/, 0 /*timeNs*/,
                                        0 /*numElems: continuous*/);
    if (act != 0) {
      MLPD_ERROR("activateStream failed: %d - %s\n", act,
                 SoapySDR::errToStr(act));
      running = false;
    }
    // Approximate capture start; the exact time of file sample 0 is the
    // first read's timeNs (FIRST_SAMPLE_TIME_NS), captured by the tracker.
    long long start_hw_time_ns = 0;
    if (act == 0) {
      try {
        start_hw_time_ns = dev->getHardwareTime("");
      } catch (const std::exception& e) {
        MLPD_WARN("getHardwareTime unavailable: %s\n", e.what());
      }
    }

    for (size_t slot_idx = 0;
         (slot_idx < meta.total_slots) && (running.load() == true) &&
         (SignalHandler::gotExitSignal() == false);
         slot_idx++) {
      const size_t cursor = slot_idx % num_slots;
      if (sample_buf_try_claim(ring.pkt_buf_inuse, cursor) == false) {
        // Writer is behind and the ring wrapped: drop this slot but keep
        // the stream drained so the device-side ring does not also fill.
        // The slot's row stays zeros in the file; time alignment is kept.
        stats.slots_dropped++;
        Packet* trash = reinterpret_cast<Packet*>(scratch.data());
        fillSlot(dev.get(), stream, trash->data, cfg, gaps, stats, running);
        continue;
      }

      Packet* pkt = reinterpret_cast<Packet*>(ring.buffer.data() +
                                              cursor * packet_length);
      new (pkt) Packet(slot_idx, 0 /*slot*/, 0 /*cell*/, 0 /*ant*/);
      if (fillSlot(dev.get(), stream, pkt->data, cfg, gaps, stats, running) ==
          false) {
        // Incomplete slot (shutdown or error): release the claim, don't record.
        sample_buf_release(ring.pkt_buf_inuse, cursor);
        break;
      }

      Event_data event;
      event.event_type = kTaskRecord;
      event.node_type = kBS;
      event.frame_id = pkt->frame_id;
      event.slot_id = 0;
      event.ant_id = 0;
      event.offset = cursor;
      event.buff_size = num_slots;
      event.buffer = &ring;
      if (recorder.DispatchWork(event) == false) {
        MLPD_ERROR("Record task enqueue failed\n");
        sample_buf_release(ring.pkt_buf_inuse, cursor);
        running = false;
        break;
      }
      stats.slots_recorded++;
    }

    // ---- Teardown -------------------------------------------------------
    dev->deactivateStream(stream);
    // Drain async status events; the event type is the return code. Bounded:
    // a driver that keeps returning a non-exit code must not spin us forever
    // (the Houdini status FIFO holds at most 256 events).
    constexpr size_t kMaxStatusDrain = 1024;
    for (size_t i = 0; i < kMaxStatusDrain; i++) {
      size_t chan_mask = 0;
      int flags = 0;
      long long time_ns = 0;
      const int ret =
          dev->readStreamStatus(stream, chan_mask, flags, time_ns, 0);
      if ((ret == SOAPY_SDR_TIMEOUT) || (ret == SOAPY_SDR_NOT_SUPPORTED)) {
        break;
      }
      if (ret == SOAPY_SDR_OVERFLOW) {
        stats.status_overflows++;
      } else {
        stats.status_other++;
      }
    }
    dev->closeStream(stream);

    // Ordered before Stop(): the queue dispatch sequences these writes
    // ahead of the writer thread's finalize.
    worker_raw->setStartTimes(start_hw_time_ns, stats.first_sample_time_ns,
                              stats.has_first_sample_time);
    recorder.Stop();
    // RecorderThread's destructor joins after the queue drains, closing the
    // HDF5 file, so the summary below reports on a finished file.
  }
  delete[] ring.pkt_buf_inuse;

  // ---- Summary ----------------------------------------------------------
  std::printf("\n==== rx-recorder summary ====\n");
  std::printf("Recorded slots      : %zu / %zu (%.3f s)\n",
              stats.slots_recorded, meta.total_slots,
              stats.slots_recorded * cfg.samps_per_slot() / meta.actual_rate);
  std::printf("Dropped slots (host): %zu\n", stats.slots_dropped);
  std::printf("Time-gap events     : %zu (est. %.0f samples lost)\n",
              stats.gap_events, stats.est_lost_samples);
  if (stats.backward_time_jumps > 0) {
    std::printf("Backward time jumps : %zu (timestamps not monotonic)\n",
                stats.backward_time_jumps);
  }
  std::printf("Overflows           : %zu readStream / %zu status events\n",
              stats.overflow_returns, stats.status_overflows);
  std::printf("Read timeouts       : %zu, other status events: %zu\n",
              stats.read_timeouts, stats.status_other);
  std::printf("Output file         : %s\n", cfg.output_file().c_str());

  const bool interrupted = SignalHandler::gotExitSignal();
  const bool complete = (stats.slots_recorded == meta.total_slots);
  if (interrupted == true) {
    std::printf("Capture interrupted by signal.\n");
  }
  return (complete || interrupted) ? EXIT_SUCCESS : EXIT_FAILURE;
}

}  // namespace

int main(int argc, char* argv[]) {
  gflags::SetVersionString(GetSounderProjectVersion());
  gflags::SetUsageMessage("rx-recorder Options: -conf_file -storepath");
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  int ret = EXIT_FAILURE;
  try {
    Sounder::RxRecorderConfig config(FLAGS_conf_file, FLAGS_storepath);

    SignalHandler signal_handler;
    signal_handler.setupSignalHandlers();

    ret = runCapture(config);
  } catch (H5::Exception& e) {
    // H5::Exception does not derive from std::exception; without this the
    // process would std::terminate on any HDF5 failure.
    std::cerr << "rx-recorder terminated (HDF5): " << e.getDetailMsg()
              << std::endl;
    ret = EXIT_FAILURE;
  } catch (const std::exception& e) {
    std::cerr << "rx-recorder terminated: " << e.what() << std::endl;
    ret = EXIT_FAILURE;
  }
  gflags::ShutDownCommandLineFlags();
  return ret;
}
