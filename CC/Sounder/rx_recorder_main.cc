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
#include <functional>
#include <iostream>
#include <memory>
#include <vector>

#include "include/logger.h"
#include "include/macros.h"
#include "include/recorder_thread.h"
#include "include/rx_recorder_capture.h"
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

// AP-5 loopback verification tone: the capture's OWN device handle arms a
// tx_mode=replay stream so a known tone rides the DAC->ADC loopback for
// the capture's whole lifetime (a second handle is out-of-contract, and
// the tone's lifetime IS the stream's lifetime: closeStream releases the
// DAC tile refcount). Load contract (validated flow rx_tx_loopback.ipynb
// + the host writeStream(TX,replay) source): one writeStream element =
// one complex CS16 sample = one u32 replay-RAM word (I in [15:0]), the
// playout loop length = the total loaded fill, activateStream starts the
// continuous RTL loop, deactivate/close stop and release it.
class TxReplayTone {
 public:
  TxReplayTone(SoapySDR::Device* dev, const Sounder::RxRecorderConfig& cfg)
      : dev_(dev) {
    const size_t chan = cfg.tx_replay_channel();
    const size_t n = cfg.tx_replay_n_addrs();
    // The bin grid is set by the DAC COMPLEX input rate (Fs_dac/interp),
    // exposed via TX channel info; getSampleRate(TX) is the fallback.
    double dac_rate = 0.0;
    const SoapySDR::Kwargs info = dev_->getChannelInfo(SOAPY_SDR_TX, chan);
    if (info.count("rfdc_effective_rate_hz") != 0) {
      dac_rate = std::stod(info.at("rfdc_effective_rate_hz"));
    } else {
      dac_rate = dev_->getSampleRate(SOAPY_SDR_TX, chan);
    }
    if (dac_rate <= 0.0) {
      throw std::runtime_error(
          "tx_replay: DAC complex rate unavailable on TX channel " +
          std::to_string(chan));
    }
    // Snap to the nearest n-sample loop bin: the replayed loop is then
    // phase-seamless at the wrap — the exact property the --tone
    // phase-continuity verification leans on.
    const long long k = std::max<long long>(
        1LL, std::llround(cfg.tx_replay_freq() * static_cast<double>(n) /
                          dac_rate));
    actual_freq_ = static_cast<double>(k) * dac_rate / static_cast<double>(n);

    // Complex tone, interleaved I,Q with I in the low half — the DAC's
    // CHAI,CHAQ order (flip here if a rig run shows the mirrored image).
    std::vector<short> ram(2 * n);
    const double amp = cfg.tx_replay_amp() * 32767.0;
    for (size_t i = 0; i < n; i++) {
      const double phase = (2.0 * M_PI * static_cast<double>(k) *
                            static_cast<double>(i)) /
                           static_cast<double>(n);
      ram[2 * i] = static_cast<short>(std::lround(amp * std::cos(phase)));
      ram[(2 * i) + 1] = static_cast<short>(std::lround(amp * std::sin(phase)));
    }

    SoapySDR::Kwargs tx_args;
    tx_args["tx_mode"] = "replay";
    stream_ = dev_->setupStream(SOAPY_SDR_TX, SOAPY_SDR_CS16,
                                std::vector<size_t>{chan}, tx_args);
    if (stream_ == nullptr) {
      throw std::runtime_error("tx_replay: setupStream(TX, replay) failed");
    }
    try {
      const void* buffs[1] = {ram.data()};
      int flags = 0;
      const int ret = dev_->writeStream(stream_, buffs, n, flags,
                                        0 /*timeNs: a-temporal load*/, 1000000);
      if (ret != static_cast<int>(n)) {
        throw std::runtime_error(
            "tx_replay: replay-RAM load wrote " + std::to_string(ret) +
            " of " + std::to_string(n) + " samples");
      }
      const int act = dev_->activateStream(stream_);
      if (act != 0) {
        throw std::runtime_error(std::string("tx_replay: activateStream: ") +
                                 SoapySDR::errToStr(act));
      }
    } catch (...) {
      dev_->closeStream(stream_);
      stream_ = nullptr;
      throw;
    }
  }

  ~TxReplayTone() {
    if (stream_ != nullptr) {
      dev_->deactivateStream(stream_);
      dev_->closeStream(stream_);
    }
  }
  TxReplayTone(const TxReplayTone&) = delete;
  TxReplayTone& operator=(const TxReplayTone&) = delete;

  inline double actual_freq(void) const { return actual_freq_; }

 private:
  SoapySDR::Device* dev_;
  SoapySDR::Stream* stream_ = nullptr;
  double actual_freq_ = 0.0;
};

// readStream-backed SampleSource: fetch fills a staging chunk. The span
// stamp is the read's leading timeNs; with the driver's break-at-gap
// contract (rx_gap_break=1) every span is time-contiguous.
class ReadStreamSource : public Sounder::SampleSource {
 public:
  ReadStreamSource(SoapySDR::Device* dev, SoapySDR::Stream* stream,
                   size_t chunk_samps, long timeout_us)
      : dev_(dev),
        stream_(stream),
        chunk_(2 * chunk_samps),
        timeout_us_(timeout_us) {}

  Sounder::FetchStatus fetch(void) override {
    void* buffs[1] = {chunk_.data()};
    int flags = 0;
    long long time_ns = 0;
    const int ret = dev_->readStream(stream_, buffs, chunk_.size() / 2, flags,
                                     time_ns, timeout_us_);
    // A 0-element return makes no progress either -- treat it as a timeout
    // so a stalled stream cannot spin the capture loop forever.
    if ((ret == SOAPY_SDR_TIMEOUT) || (ret == 0)) {
      return Sounder::FetchStatus::kTimeout;
    }
    if (ret == SOAPY_SDR_OVERFLOW) {
      return Sounder::FetchStatus::kOverflow;
    }
    if (ret < 0) {
      MLPD_ERROR("readStream error: %d - %s\n", ret, SoapySDR::errToStr(ret));
      return Sounder::FetchStatus::kFatal;
    }
    len_ = static_cast<size_t>(ret);
    off_ = 0;
    has_time_ = ((flags & SOAPY_SDR_HAS_TIME) != 0);
    time_ns_ = time_ns;
    return Sounder::FetchStatus::kData;
  }

  size_t pending(void) const override { return len_ - off_; }
  const short* data(void) const override { return chunk_.data() + (2 * off_); }
  void consume(size_t samples) override { off_ += samples; }
  bool has_time(void) const override { return has_time_; }
  long long time_ns(void) const override { return time_ns_; }

 private:
  SoapySDR::Device* dev_;
  SoapySDR::Stream* stream_;
  std::vector<short> chunk_;  // staging buffer for one readStream call
  long timeout_us_;
  size_t len_ = 0;
  size_t off_ = 0;
  bool has_time_ = false;
  long long time_ns_ = 0;
};

// Direct-buffer (SH-254 acquire/release) SampleSource: one acquire = one
// wire packet borrowed from the driver's ring — samples are assembled
// straight from the ring slot into the HDF5 row (the staging copy is
// gone). Each packet carries its own hardware tick stamp, so consecutive
// span stamps locate every gap sample-exactly, independent of the
// rx_gap_break readStream contract. At most one buffer is held: consume()
// releases the slot the moment its span is drained, and the destructor
// releases a partially-consumed hold (run it BEFORE closeStream).
class DirectBufferSource : public Sounder::SampleSource {
 public:
  DirectBufferSource(SoapySDR::Device* dev, SoapySDR::Stream* stream,
                     long timeout_us)
      : dev_(dev), stream_(stream), timeout_us_(timeout_us) {}

  ~DirectBufferSource() override {
    if (held_ == true) {
      dev_->releaseReadBuffer(stream_, handle_);
    }
  }

  Sounder::FetchStatus fetch(void) override {
    const void* buffs[1] = {nullptr};
    int flags = 0;
    long long time_ns = 0;
    const int ret = dev_->acquireReadBuffer(stream_, handle_, buffs, flags,
                                            time_ns, timeout_us_);
    // 0 = a finite burst completed; this stream is continuous, so treat a
    // progress-less return like a timeout rather than spinning.
    if ((ret == SOAPY_SDR_TIMEOUT) || (ret == 0)) {
      return Sounder::FetchStatus::kTimeout;
    }
    if (ret == SOAPY_SDR_OVERFLOW) {
      return Sounder::FetchStatus::kOverflow;
    }
    if (ret < 0) {
      MLPD_ERROR("acquireReadBuffer error: %d - %s\n", ret,
                 SoapySDR::errToStr(ret));
      return Sounder::FetchStatus::kFatal;
    }
    held_ = true;
    payload_ = static_cast<const short*>(buffs[0]);
    len_ = static_cast<size_t>(ret);
    off_ = 0;
    has_time_ = ((flags & SOAPY_SDR_HAS_TIME) != 0);
    time_ns_ = time_ns;
    return Sounder::FetchStatus::kData;
  }

  size_t pending(void) const override { return len_ - off_; }
  const short* data(void) const override { return payload_ + (2 * off_); }
  void consume(size_t samples) override {
    off_ += samples;
    if (off_ >= len_) {
      // Hand the ring slot back the moment the span is drained, so the
      // driver ring never backs up behind the app (<= 1 buffer held).
      dev_->releaseReadBuffer(stream_, handle_);
      held_ = false;
      payload_ = nullptr;
      len_ = 0;
      off_ = 0;
    }
  }
  bool has_time(void) const override { return has_time_; }
  long long time_ns(void) const override { return time_ns_; }

 private:
  SoapySDR::Device* dev_;
  SoapySDR::Stream* stream_;
  long timeout_us_;
  size_t handle_ = 0;
  const short* payload_ = nullptr;
  bool held_ = false;
  size_t len_ = 0;
  size_t off_ = 0;
  bool has_time_ = false;
  long long time_ns_ = 0;
};

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
  // One device snapshot per iteration (each query is a remote round-trip):
  // the advertised rungs plus the effective fabric rate (fabric_clock x
  // vld), from which ladder membership and the reachable span both derive.
  struct RateView {
    bool on_ladder = false;
    double eff = 0.0;
    double lo = 0.0;
    double hi = 0.0;
  };
  const auto snapshot = [&](void) {
    RateView v;
    v.eff = dev->getSampleRate(SOAPY_SDR_RX, chan);
    v.lo = v.eff;
    v.hi = v.eff;
    for (double rung : dev->listSampleRates(SOAPY_SDR_RX, chan)) {
      v.on_ladder |= (std::abs(rung - rate) <= (1e-6 * rate));
      v.lo = std::min(v.lo, rung);
      v.hi = std::max(v.hi, rung);
    }
    return v;
  };

  constexpr int kMaxClockSteps = 8;
  for (int step = 0; step < kMaxClockSteps; step++) {
    const RateView view = snapshot();
    if (view.on_ladder == true) {
      dev->setSampleRate(SOAPY_SDR_RX, chan, rate);
      return;
    }
    if (std::abs(view.eff - rate) <= (1e-6 * rate)) {
      return;  // the clock walk landed the effective rate on it exactly
    }
    const double lo = view.lo;
    const double hi = view.hi;
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
  const RateView view = snapshot();
  throw std::runtime_error(
      "Requested rate unreachable: advertised rates still span " +
      std::to_string(view.lo / 1e6) + " - " + std::to_string(view.hi / 1e6) +
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

  // AP-5: arm the loopback verification tone before the RX stream opens;
  // the object's lifetime (this scope) keeps it playing past the capture.
  std::unique_ptr<TxReplayTone> tx_tone;
  if (cfg.has_tx_replay() == true) {
    tx_tone = std::make_unique<TxReplayTone>(dev.get(), cfg);
    meta.tx_replay_enabled = true;
    meta.tx_replay_freq_actual = tx_tone->actual_freq();
    meta.tx_replay_amp = cfg.tx_replay_amp();
    meta.tx_replay_channel = cfg.tx_replay_channel();
    std::printf(
        "  tx_replay tone armed: %.6f MHz baseband (requested %.6f MHz), "
        "amp %.2f, DAC channel %zu\n",
        meta.tx_replay_freq_actual / 1e6, cfg.tx_replay_freq() / 1e6,
        meta.tx_replay_amp, meta.tx_replay_channel);
  }
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

  // SH-254 zero-copy read path. Support is advertised by a non-zero
  // direct-access buffer count (on the Houdini driver: single-channel RX
  // only); there is deliberately no getHardwareInfo key for it.
  const bool direct_supported = (dev->getNumDirectAccessBuffers(stream) > 0);
  bool use_direct = false;
  if (cfg.direct_rx() == "require") {
    if (direct_supported == false) {
      dev->closeStream(stream);
      throw std::runtime_error(
          "direct_rx=require, but the driver advertises no direct-access "
          "buffers on this stream (needs the SH-254 acquire/release API, "
          "single-channel RX)");
    }
    use_direct = true;
  } else if (cfg.direct_rx() == "auto") {
    use_direct = direct_supported;
  }

  // Extent exactness: every source span must be time-contiguous. Direct
  // mode has it structurally (one acquire = one wire packet, per-packet
  // stamps); readStream needs the driver's break-at-gap capability AND
  // the per-stream rx_gap_break knob (default on) not disabled by the
  // config's stream args (driver bool spelling: false/0/empty = off).
  bool gap_break = (hw_info.count("rx_gap_break") != 0) &&
                   (hw_info.at("rx_gap_break") == "1");
  if (const auto it = stream_args.find("rx_gap_break");
      it != stream_args.end()) {
    const std::string& v = it->second;
    gap_break = gap_break && (v != "false") && (v != "0") && (v.empty() == false);
  }
  const bool exact_gaps = (use_direct == true) || (gap_break == true);
  meta.read_path = use_direct ? "direct" : "readstream";
  meta.gaps_exact = exact_gaps;
  meta.stream_args = SoapySDR::KwargsToString(stream_args);
  std::printf("  read path: %s (%s extents)\n", meta.read_path.c_str(),
              exact_gaps ? "sample-exact" : "read-widened");
  if (meta.stream_args.empty() == false) {
    std::printf("  stream args: %s\n", meta.stream_args.c_str());
  }

  std::unique_ptr<Sounder::SampleSource> source;
  if (use_direct == true) {
    source = std::make_unique<DirectBufferSource>(dev.get(), stream,
                                                  cfg.rx_timeout_us());
  } else {
    source = std::make_unique<ReadStreamSource>(
        dev.get(), stream, cfg.read_chunk_samps(), cfg.rx_timeout_us());
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

  Sounder::CaptureState state(meta.actual_rate);
  state.exact_gaps = exact_gaps;
  Sounder::CaptureStats& stats = state.stats;
  const std::function<bool(void)> interrupt_poll = [](void) {
    return SignalHandler::gotExitSignal();
  };

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
        Sounder::fillSlot(*source, trash->data, cfg.samps_per_slot(), state,
                          running, interrupt_poll);
        continue;
      }

      Packet* pkt = reinterpret_cast<Packet*>(ring.buffer.data() +
                                              cursor * packet_length);
      new (pkt) Packet(slot_idx, 0 /*slot*/, 0 /*cell*/, 0 /*ant*/);
      if (Sounder::fillSlot(*source, pkt->data, cfg.samps_per_slot(), state,
                            running, interrupt_poll) == false) {
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
    // Drop the source first: a held direct-buffer borrow is invalidated
    // by (de)activation and must be released while the stream is open.
    source.reset();
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
  std::printf("Read path           : %s\n", meta.read_path.c_str());
  std::printf(
      "Stream gaps         : %zu (%.0f placeholder samples inserted, "
      "%s extents)\n",
      stats.gap_events, stats.est_lost_samples,
      state.exact_gaps ? "exact" : "read-widened");
  if (stats.backward_time_jumps > 0) {
    std::printf("Backward time jumps : %zu (timestamps not monotonic)\n",
                stats.backward_time_jumps);
  }
  if (stats.time_resyncs > 0) {
    std::printf(
        "Time-base resyncs   : %zu (absolute time alignment broken at the "
        "marked samples)\n",
        stats.time_resyncs);
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
