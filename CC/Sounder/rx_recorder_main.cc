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

 Sample-time integrity: the file promises sample k lives at
 FIRST_SAMPLE_TIME_NS + k/RATE. Stream gaps (dropped packets) are
 detected from per-read timestamps, repaired by inserting placeholder
 zeros so one gap cannot shift the rest of the file, and recorded as
 untrusted extents in /Data/Gaps (see rx_recorder_grid.h and
 docs/RX_GAP_AWARENESS.md).
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
#include <cstring>
#include <iostream>
#include <memory>
#include <vector>

#include "include/logger.h"
#include "include/macros.h"
#include "include/recorder_thread.h"
#include "include/rx_recorder_config.h"
#include "include/rx_recorder_grid.h"
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
  size_t gap_events = 0;           // stream gaps detected via timestamps
  size_t backward_time_jumps = 0;  // timeNs went backward (resync/rollover)
  double est_lost_samples = 0.0;   // sum of inserted placeholder samples
  size_t status_overflows = 0;     // OVERFLOW events from readStreamStatus
  size_t status_other = 0;
  long long first_sample_time_ns = 0;  // grid time of file sample 0
  bool has_first_sample_time = false;
};

// Capture-thread bookkeeping threaded through fillSlot: the sample-time
// grid, the untrusted-extent list, and the staging machinery that lets
// placeholder zeros be inserted BEFORE a late-arriving read's samples.
struct CaptureState {
  explicit CaptureState(double rate, size_t chunk_samps)
      : grid(rate), chunk(2 * chunk_samps) {}

  CaptureStats stats;
  Sounder::TimeGridTracker grid;
  std::vector<Sounder::GapExtent> extents;
  int64_t grid_pos = 0;        // absolute samples emitted onto the grid
  size_t pad_remaining = 0;    // placeholder zeros still owed to the grid
  size_t last_read_samps = 0;  // uncertainty span for widened extents
  std::vector<short> chunk;    // staging buffer for one readStream call
  size_t chunk_len = 0;        // samples staged
  size_t chunk_off = 0;        // samples already emitted from staging
  // Driver guarantees every readStream buffer is time-contiguous (breaks
  // at gaps). Without it a mid-read gap is located only to the enclosing
  // read, so extents widen by last_read_samps. See docs/RX_GAP_AWARENESS.md.
  bool exact_gaps = false;
};

// Checks a stamped read against the grid: anchors t0, schedules placeholder
// zeros for forward gaps, records untrusted extents.
void onStampedChunk(long long time_ns, CaptureState& state) {
  const bool first = (state.grid.has_t0() == false);
  const Sounder::GridCheck check = state.grid.onStamp(time_ns, state.grid_pos);
  if (first == true) {
    state.stats.first_sample_time_ns = state.grid.t0();
    state.stats.has_first_sample_time = true;
    return;
  }
  if (check.pad_samples > 0) {
    state.stats.gap_events++;
    state.stats.est_lost_samples += static_cast<double>(check.pad_samples);
    int64_t start = state.grid_pos;
    if (state.exact_gaps == false) {
      // Gap position is only known to the enclosing read: widen the
      // untrusted extent backward over the previous read's samples.
      start = std::max<int64_t>(
          0, state.grid_pos - static_cast<int64_t>(state.last_read_samps));
    }
    const int64_t length =
        static_cast<int64_t>(check.pad_samples) + (state.grid_pos - start);
    state.extents.push_back({start, length, Sounder::kGapTimeJump});
    state.pad_remaining = check.pad_samples;
  } else if (check.backward == true) {
    state.stats.backward_time_jumps++;
    state.extents.push_back({state.grid_pos, 0, Sounder::kGapBackward});
  }
}

// Fills dest with exactly samps_per_slot grid samples: placeholder zeros
// owed from detected gaps, staged samples, then fresh readStream chunks.
// Returns false when the capture should stop (fatal error or shutdown).
bool fillSlot(SoapySDR::Device* dev, SoapySDR::Stream* stream, short* dest,
              const Sounder::RxRecorderConfig& cfg, CaptureState& state,
              std::atomic<bool>& running) {
  const size_t slot_samps = cfg.samps_per_slot();
  size_t filled = 0;
  size_t consecutive_timeouts = 0;
  size_t consecutive_overflows = 0;
  constexpr size_t kMaxConsecutiveTimeouts = 10;
  // Overflow returns are instant (event pops, no wait); a driver wedged in
  // a permanent overflow state would otherwise spin here forever.
  constexpr size_t kMaxConsecutiveOverflows = 10000;

  while ((filled < slot_samps) && (running.load() == true) &&
         (SignalHandler::gotExitSignal() == false)) {
    // 1) Placeholder zeros owed to the grid from a detected gap.
    if (state.pad_remaining > 0) {
      const size_t n = std::min(state.pad_remaining, slot_samps - filled);
      std::memset(dest + (2 * filled), 0, n * 2 * sizeof(short));
      filled += n;
      state.grid_pos += static_cast<int64_t>(n);
      state.pad_remaining -= n;
      continue;
    }
    // 2) Staged samples from the last readStream call.
    if (state.chunk_off < state.chunk_len) {
      const size_t n =
          std::min(state.chunk_len - state.chunk_off, slot_samps - filled);
      std::memcpy(dest + (2 * filled),
                  state.chunk.data() + (2 * state.chunk_off),
                  n * 2 * sizeof(short));
      filled += n;
      state.chunk_off += n;
      state.grid_pos += static_cast<int64_t>(n);
      continue;
    }
    // 3) Read a fresh chunk into staging.
    const size_t request =
        std::min(state.chunk.size() / 2, slot_samps - filled);
    void* buffs[1] = {state.chunk.data()};
    int flags = 0;
    long long time_ns = 0;
    const int ret = dev->readStream(stream, buffs, request, flags, time_ns,
                                    cfg.rx_timeout_us());
    // A 0-element return makes no progress either -- treat it as a timeout
    // so a stalled stream cannot spin this loop forever.
    if ((ret == SOAPY_SDR_TIMEOUT) || (ret == 0)) {
      state.stats.read_timeouts++;
      if (++consecutive_timeouts >= kMaxConsecutiveTimeouts) {
        MLPD_ERROR("readStream: %zu consecutive timeouts, aborting capture\n",
                   consecutive_timeouts);
        running = false;
        return false;
      }
      continue;
    }
    if (ret == SOAPY_SDR_OVERFLOW) {
      state.stats.overflow_returns++;
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
    state.chunk_len = static_cast<size_t>(ret);
    state.chunk_off = 0;
    if ((flags & SOAPY_SDR_HAS_TIME) != 0) {
      // May schedule pads: the staged chunk then emits AFTER them, which
      // places its first sample exactly where the timestamp says it belongs.
      onStampedChunk(time_ns, state);
    }
    state.last_read_samps = state.chunk_len;
  }
  return (filled == slot_samps);
}

// Applies the requested sample rate. Sequence [user]: check the device's
// advertised rates; while the request lies outside them, step the global
// RX fabric clock (doubling toward faster rates, halving toward slower —
// the advertised set scales with the clock) and re-check. The device's
// fail-loud on an unrealizable clock (with full revert on its side)
// defines both the ceiling and the floor; an 8-step bound is the local
// backstop. Then select the rate normally. RX_FAB_CLK requires all RX
// streams closed, which holds here (before setupStream); every fresh
// make() resets the device to defaults, so an off-default run never
// leaks into the next session.
void applySampleRate(SoapySDR::Device* dev, size_t chan, double rate) {
  const auto on_ladder = [&](void) {
    for (double rung : dev->listSampleRates(SOAPY_SDR_RX, chan)) {
      if (std::abs(rung - rate) <= (1e-6 * rate)) {
        return true;
      }
    }
    return false;
  };
  // Span of reachable rates at the current clock: the advertised rungs
  // plus the effective fabric rate (fabric_clock x vld).
  const auto span = [&](void) {
    double lo = dev->getSampleRate(SOAPY_SDR_RX, chan);
    double hi = lo;
    for (double rung : dev->listSampleRates(SOAPY_SDR_RX, chan)) {
      lo = std::min(lo, rung);
      hi = std::max(hi, rung);
    }
    return std::make_pair(lo, hi);
  };

  constexpr int kMaxClockSteps = 8;
  for (int step = 0; step < kMaxClockSteps; step++) {
    if (on_ladder() == true) {
      dev->setSampleRate(SOAPY_SDR_RX, chan, rate);
      return;
    }
    const double eff = dev->getSampleRate(SOAPY_SDR_RX, chan);
    if (std::abs(eff - rate) <= (1e-6 * rate)) {
      return;  // the clock walk landed the effective rate on it exactly
    }
    const auto [lo, hi] = span();
    double factor = 0.0;
    if (rate > hi * (1.0 + 1e-6)) {
      factor = 2.0;
    } else if (rate < lo * (1.0 - 1e-6)) {
      factor = 0.5;
    } else {
      throw std::runtime_error(
          "Requested rate is within the advertised span (" +
          std::to_string(lo / 1e6) + " - " + std::to_string(hi / 1e6) +
          " MSPS) but is not an advertised rate at any clock");
    }
    double cur_fab_hz = 0.0;
    try {
      // readSetting format: "<hz> Hz (code C, realized R Hz; ...)"
      cur_fab_hz = std::stod(dev->readSetting("RX_FAB_CLK"));
    } catch (const std::exception&) {
      throw std::runtime_error(
          "Requested rate is outside the advertised rates and RX_FAB_CLK "
          "is not available to move them (pre-1.12 bitstream?)");
    }
    if (cur_fab_hz <= 0.0) {
      throw std::runtime_error("RX_FAB_CLK readback is not positive");
    }
    const long long next_fab_hz =
        static_cast<long long>(std::llround(cur_fab_hz * factor));
    std::printf(
        "Advertised rates span %.2f - %.2f MSPS, requested %.2f MSPS; "
        "stepping RX_FAB_CLK %.0f -> %lld Hz\n",
        lo / 1e6, hi / 1e6, rate / 1e6, cur_fab_hz, next_fab_hz);
    dev->writeSetting("RX_FAB_CLK", std::to_string(next_fab_hz));
  }
  const auto [lo, hi] = span();
  throw std::runtime_error(
      "Requested rate unreachable: advertised rates still span " +
      std::to_string(lo / 1e6) + " - " + std::to_string(hi / 1e6) +
      " MSPS after clock stepping");
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
    applySampleRate(dev.get(), chan, cfg.rate());
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

  const SoapySDR::Kwargs hw_info = dev->getHardwareInfo();
  Sounder::RxCaptureMeta meta;
  meta.hardware_key = dev->getHardwareKey();
  meta.hardware_info = SoapySDR::KwargsToString(hw_info);
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

  CaptureState state(meta.actual_rate, cfg.read_chunk_samps());
  // Proposed driver capability (see docs/RX_GAP_AWARENESS.md): every read
  // buffer is time-contiguous, so gap extents need no widening.
  state.exact_gaps = (hw_info.count("rx_gap_break") != 0) &&
                     (hw_info.at("rx_gap_break") == "1");
  CaptureStats& stats = state.stats;

  {
    auto worker = std::make_unique<Sounder::RxRecorderWorker>(&cfg, meta);
    // Raw handle outlives the move: used to hand the capture-start times and
    // gap extents to the worker before its Stop event (queue-ordered).
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
    // grid anchor (FIRST_SAMPLE_TIME_NS), set at the first stamped read.
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
        state.extents.push_back({state.grid_pos,
                                 static_cast<int64_t>(cfg.samps_per_slot()),
                                 Sounder::kGapHostRing});
        Packet* trash = reinterpret_cast<Packet*>(scratch.data());
        fillSlot(dev.get(), stream, trash->data, cfg, state, running);
        continue;
      }

      Packet* pkt = reinterpret_cast<Packet*>(ring.buffer.data() +
                                              cursor * packet_length);
      new (pkt) Packet(slot_idx, 0 /*slot*/, 0 /*cell*/, 0 /*ant*/);
      if (fillSlot(dev.get(), stream, pkt->data, cfg, state, running) ==
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
    worker_raw->setGapExtents(state.extents);
    std::printf(
        "Capture done (%zu slots); draining queued slots to disk "
        "(RAM-window captures may take a while)...\n",
        stats.slots_recorded);
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
  std::printf(
      "Stream gaps         : %zu (%.0f placeholder samples inserted, "
      "%s extents)\n",
      stats.gap_events, stats.est_lost_samples,
      state.exact_gaps ? "exact" : "read-widened");
  if (stats.backward_time_jumps > 0) {
    std::printf("Backward time jumps : %zu (timestamps not monotonic)\n",
                stats.backward_time_jumps);
  }
  std::printf("Untrusted extents   : %zu (see /Data/Gaps)\n",
              state.extents.size());
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
