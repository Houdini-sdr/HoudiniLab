/** @file RadioHoudini.cc
  * @brief The Houdini RFSoC backend: stream arguments, the receive drain with
  *        its gap ledger, the TDD transmit grid. Moved out of Radio.cc.
  *
  * RENEW OPEN SOURCE LICENSE: http://renew-wireless.org/license
*/
#include "include/RadioHoudini.h"

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <vector>

#include "SoapySDR/Errors.hpp"
#include "SoapySDR/Time.hpp"
#include "include/logger.h"
#include "include/macros.h"
#include "include/node_version.h"
#include "include/rx_gap_sink.h"       // RxGapSink (UDP gap -> /Data/Gaps bridge)
#include "include/rx_recorder_grid.h"  // TimeGridTracker
#include "include/utils.h"

SoapySDR::Kwargs RadioHoudini::deviceArgs(const RadioParams& p) {
  // SoapyHoudiniSDR node: the id is the board IP. Address the remote node
  // directly (C++ SoapyRemote auto-discovery is unreliable here); this
  // matches SoapySDRUtil's enumerated kwargs.
  SoapySDR::Kwargs args;
  args["driver"] = "houdinisdr";
  args["remote"] = "tcp://" + p.id + ":" + p.remote_port;
  args["remote:driver"] = "houdinisdr-device";
  args["remote:type"] = "houdinisdr";
  args["timeout"] = p.timeout;
  return args;
}

SoapySDR::Kwargs RadioHoudini::rxStreamArgs(const RadioParams& p) {
  SoapySDR::Kwargs rx;
  // The host UDP port a single-channel RX stream binds. On a COMBINED (>1
  // channel) stream the driver rejects local_port: the wire fixes each channel's
  // port at 10001 + channel (SH-142/SH-159), so leave it unset and let the
  // driver assign per channel.
  if (p.rx_channels.size() <= 1) {
    rx["local_port"] = std::to_string(p.rx_local_port);
  }
  // Break-at-gap (SH-253). The driver defaults this ON, but the whole gap
  // account depends on it: recv only compares timestamps BETWEEN reads, so a
  // splice INSIDE one returned buffer would be invisible. Asked for explicitly
  // rather than inherited from a default another repo owns (AP-10).
  rx["rx_gap_break"] = "1";
  // MTS (AP-23): pin the converter bring-up latency and align the ADC/DAC
  // tiles the RX-stamp -> TX-time arithmetic crosses.
  if (p.mts) rx["mts"] = "true";
  return rx;
}

SoapySDR::Kwargs RadioHoudini::txStreamArgs(const RadioParams& p) {
  SoapySDR::Kwargs tx;
  tx["tx_mode"] = p.tx_mode;  // "replay" (the BS beacon RAM) or "stream" (the UE)
  // The driver's TxTickAnchor accepts HAS_TIME starts on the 3.125 us TDD
  // window grid (SH-248/SH-301) instead of whole milliseconds.
  if (p.tdd) tx["tdd"] = "1";
  if (p.mts) tx["mts"] = "true";
  return tx;
}

houdini::modev::Plan RadioHoudini::modeVPlan(const RadioParams& p) {
  houdini::modev::Plan m;
  m.tx_channels = p.tx_channels;
  m.rx_channels = p.rx_channels;
  m.tx_rate_hz = p.tx_rate_hz > 0.0 ? p.tx_rate_hz : p.rate_hz;
  m.rx_rate_hz = p.rate_hz;
  m.adc_fs_hz = p.adc_fs_hz;
  m.dac_fs_hz = p.dac_fs_hz;
  m.default_nco_hz = p.nco_hz;
  m.nco_by_channel = p.nco_by_channel;
  m.half_bw_hz = p.half_bw_hz;
  m.tx_gain_db = p.tx_gain_db;
  m.rx_gain_db = p.rx_gain_db;
  m.rx_freq_offset_hz = p.rx_freq_offset_hz;
  m.tx_freq_offset_hz = p.tx_freq_offset_hz;
  return m;
}

RadioHoudini::RadioHoudini(const RadioParams& params)
    : RadioHoudini(params, params.adc_fs_hz > 0.0
                               ? std::make_shared<houdini::modev::Result>()
                               : nullptr) {}

RadioHoudini::RadioHoudini(const RadioParams& params,
                           std::shared_ptr<houdini::modev::Result> mv)
    // One-rate path: RX and TX both at the app rate, tuned to the NCO, all
    // before setupStream (Houdini forbids a live rate change); the beacon
    // replay RAM plays at THIS rate and the RFDC interpolates to the DAC.
    // Mode V (AP-79, converter Fs configured): the whole converter bring-up
    // runs instead, in the device's order, with the per-channel NCO and the
    // derived zones, calibration modes and inverse sinc; the result is kept
    // so each RX lane knows whether it needs the channel filter.
    : RadioSoapy(params, Type::kSoapyHoudini, deviceArgs(params), rxStreamArgs(params),
                 txStreamArgs(params), params.rate_hz, params.rate_hz, params.nco_hz, true,
                 mv == nullptr
                     ? std::function<void(SoapySDR::Device&)>()
                     : [mv, plan = modeVPlan(params), label = params.label](SoapySDR::Device& dev) {
                         *mv = houdini::modev::bringUp(dev, plan);
                         for (const auto& entry : mv->log) {
                           MLPD_INFO("%s mode V: %s\n", label.c_str(), entry.c_str());
                         }
                       }),
      mode_v_(std::move(mv)) {
  if (params.tx_rate_hz > 0.0 && params.tx_rate_hz != params.rate_hz) {
    if (params.tx_rate_hz != 2.0 * params.rate_hz) {
      throw std::invalid_argument("RadioHoudini: only TX = 2 x sample_rate is interpolated");
    }
    tx_interp_ = std::make_unique<houdini::boundary::TxBurstInterpolator>();
  }
  if (mode_v_ != nullptr) {
    auto on = houdini::boundary::laneFlags(
        params.rx_channels, [this](size_t ch) { return rxChannelFilter(ch); });
    auto f = std::make_unique<houdini::boundary::RxLaneFilters>(std::move(on));
    if (f->any()) rx_filters_ = std::move(f);
  }
}

void RadioHoudini::setup(int ch, double rxgain, double txgain) {
  // The mixer NCO is the only tuning knob and there is no antenna, analog
  // bandwidth, gain or DC-offset stage to program. Rate and NCO were applied
  // before the streams opened, so this reports.
  (void)rxgain;
  (void)txgain;
  MLPD_INFO("Houdini channel %d: rate %.2f MSPS, NCO %.2f MHz\n", ch,
            dev_->getSampleRate(SOAPY_SDR_RX, ch) / 1e6,
            dev_->getFrequency(SOAPY_SDR_RX, ch) / 1e6);
}

void RadioHoudini::printSettings() const {
  // No CBRS/UHF front end and no LNA/PGA/TIA gain stages to report.
  const size_t rx0 = params_.rx_channels.empty() ? 0 : params_.rx_channels.front();
  const size_t tx0 = params_.tx_channels.empty() ? 0 : params_.tx_channels.front();
  std::cout << params_.label << ": Houdini RFSoC, RX "
            << (dev_->getSampleRate(SOAPY_SDR_RX, rx0) / 1e6) << " MSPS, TX "
            << (dev_->getSampleRate(SOAPY_SDR_TX, tx0) / 1e6) << " MSPS" << std::endl;
  // Register this node's gateware/firmware/host stack for the cross-node
  // skew check the Receiver runs once every radio set is up.
  Sounder::NodeVersions::instance().add(params_.label, dev_->getHardwareInfo());
}

int RadioHoudini::xmit(const void* const* buffs, int samples, int flags,
                       long long& frameTime) {
  // AP-79: at TX = 2 x sample_rate every burst, built at the tick rate, is
  // interpolated here as a whole and padded to a whole 8-sample TX beat. The
  // time is in ns and does not change; the caller counts in ticks, so a full
  // write reports its own sample count back.
  if (!tx_interp_ || samples <= 0) return RadioSoapy::xmit(buffs, samples, flags, frameTime);
  const auto o = tx_interp_->run(buffs, params_.tx_channels.size(), static_cast<size_t>(samples));
  if (o.saturated > 0) {
    static size_t warned = 0;
    if (warned++ < 5) {
      MLPD_WARN("%s: %zu I/Q components saturated in the x2 TX interpolation; "
                "lower the TX level (the halfband overshoots near edges)\n",
                params_.label.c_str(), o.saturated);
    }
  }
  const int r = RadioSoapy::xmit(o.buffs.data(), static_cast<int>(o.samples), flags, frameTime);
  if (r < 0) return r;
  return r >= static_cast<int>(o.samples) ? samples : r / 2;
}

long long RadioHoudini::txTimeNs(long long frame_ticks, double rate_hz, bool tdd_pilot,
                                 long long advance_ticks) const {
  // With the `tdd=1` TX stream arg the driver's TxTickAnchor accepts HAS_TIME
  // starts on the 3.125 us TDD window grid, so the beacon-referenced time is
  // snapped to it -- fine enough to land in the BS rx_gate and, unlike the
  // whole-ms fallback, with NO 1 ms drift-cliff. The tick advance is the fine
  // calibration added before the snap (ue_tx_advance_ticks).
  long long ft = frame_ticks;
  if (tdd_pilot) ft += advance_ticks;
  long long ns = SoapySDR::ticksToTimeNs(ft, rate_hz);
  constexpr long long kTddGridNs = 3125;  // 384 ticks, the TDD window grid
  constexpr long long kNsPerMs = 1000000LL;
  const long long q = tdd_pilot ? kTddGridNs : kNsPerMs;
  return ((ns + q / 2) / q) * q;  // snap to the accepted grid
}

// SoapyHoudiniSDR delivers ~1 MTU (~1016 samples) per readStream and lets the
// host socket buffer a backlog while the caller is busy (e.g. running
// find_beacon between windows), so a single readStream can neither fill a
// multi-thousand-sample sync window nor guarantee it is contiguous. Drain any
// stale backlog non-blocking, then accumulate a fresh, contiguous window --
// this is the in-radio equivalent of the client_sync_cuda drain-before-frame.
int RadioHoudini::recv(void* const* buffs, int samples, long long& frameTime) {
  constexpr size_t kBytesPerSamp = 4;  // CS16 = 2 x int16
  static thread_local std::vector<uint8_t> junk;
  const size_t drain_samps = 16384;
  if (junk.size() < drain_samps * kBytesPerSamp * num_rx_ch_)
    junk.resize(drain_samps * kBytesPerSamp * num_rx_ch_);
  std::vector<void*> jb(num_rx_ch_);
  for (size_t c = 0; c < num_rx_ch_; c++)
    jb[c] = junk.data() + c * drain_samps * kBytesPerSamp;
  int jf = 0;
  long long jt = 0;
  // Split drain from read: the loop spends 93% of an iteration inside the 30
  // radioRx calls it makes per frame (29 of them purely to throw the slot
  // away), at ~880 us each, and the fix differs depending on whether that cost
  // is the drain loop or the read itself. HOUDINI_LOOP_PROFILE reports both.
  // Its OWN knob: HOUDINI_LOOP_PROFILE counts loop ITERATIONS while this
  // counts radioRx CALLS, and coalescing changes the ratio between them from
  // ~30:1 to ~2:1. One shared setting would silently report two different
  // scales, which is a hazard given how much of this branch's evidence rests
  // on those numbers being comparable.
  static const size_t rx_profile_every = [] {
    const char* e = getenv("HOUDINI_RX_PROFILE");
    return e != nullptr ? static_cast<size_t>(atol(e)) : 0;
  }();
  static thread_local double p_drain = 0, p_read = 0;
  static thread_local size_t p_calls = 0, p_chunks = 0, p_drained = 0;
  // Gated, not unconditional: this is the RX hot path (~30 calls per frame) and
  // an always-taken clock read is cost the shipped build should not carry for
  // an instrument that is off by default.
  const auto p_t0 = rx_profile_every > 0
                        ? std::chrono::steady_clock::now()
                        : std::chrono::steady_clock::time_point{};
  int drained_chunks = 0, drained_samps = 0;
  int dr = 0;
  while ((dr = dev_->readStream(rxs_, jb.data(), drain_samps, jf, jt, 0)) > 0) {
    ++drained_chunks;
    drained_samps += dr;
  }
  const auto p_t1 = rx_profile_every > 0
                        ? std::chrono::steady_clock::now()
                        : std::chrono::steady_clock::time_point{};

  // A dropped UDP packet splices a gap between two reads of THIS window. Detect it
  // from each read's own timestamp (the window used to keep only the first read's
  // time and concatenate the rest as if contiguous -- silently mis-aligning every
  // post-gap sample, which corrupts the correlation window / CSI). A per-window
  // TimeGridTracker compares where each read's samples land vs. where its stamp says
  // they belong; a gap is zero-padded so post-gap samples stay on their true offset,
  // and the extent is logged (absolute RX sample position) for the /Data/Gaps table.
  if (rx_profile_every > 0) {
    p_drain += std::chrono::duration<double, std::micro>(p_t1 - p_t0).count();
    p_chunks += static_cast<size_t>(drained_chunks);
    p_drained += static_cast<size_t>(drained_samps);
  }
  if (rx_rate_ == 0.0) rx_rate_ = dev_->getSampleRate(SOAPY_SDR_RX, 0);
  Sounder::TimeGridTracker grid(rx_rate_);
  std::vector<void*> cur(num_rx_ch_);
  int got = 0;
  size_t padded = 0;  // zeros inserted into THIS window (see lastPadSamples)
  last_pad_samples_ = 0;  // cleared up front so an early return can't leave a stale count
  while (got < samples) {
    for (size_t c = 0; c < num_rx_ch_; c++)
      cur[c] = static_cast<uint8_t*>(buffs[c]) +
               static_cast<size_t>(got) * kBytesPerSamp;
    int flags = 0;
    long long t = 0;
    int r =
        dev_->readStream(rxs_, cur.data(), samples - got, flags, t, 1000000);
    if (r <= 0) {
      if (got == 0) {
      // Account the call before leaving, or the drain cost already added above
      // is divided across a call count that never saw it -- over-reporting
      // drain per call on the very instrument this branch's cost evidence
      // rests on.
      if (rx_profile_every > 0) ++p_calls;
      return r;
    }
      break;
    }
    if (got == 0) frameTime = t;  // first (grid-anchoring) read stamps the window
    size_t pad = 0;
    if (rx_rate_ > 0.0 && (flags & SOAPY_SDR_HAS_TIME) != 0) {
      const Sounder::GridCheck gc = grid.onStamp(t, got);
      pad = std::min(gc.pad_samples, static_cast<size_t>(samples - got));
    } else {
      // No usable stamp, so this read is spliced onto the previous one with no
      // continuity check: precisely the corruption the grid tracker exists to
      // prevent. HOUDINI_PROTOCOL stamps every packet, so on a conformant device
      // this cannot fire; if it does, the guarantee is gone and the window is a
      // guess. Say so rather than degrading silently (AP-10).
      static std::atomic<int> unstamped{0};
      const int n_unstamped = unstamped.fetch_add(1);
      // Braces are load-bearing: MLPD_WARN expands to several statements, so an
      // unbraced guard would gate only the header and print the body every read.
      if ((n_unstamped % 200) == 0) {
        MLPD_WARN(
            "RX read without a usable timestamp (rate=%.0f, flags=0x%x), count "
            "%d: splicing with NO gap check, so this window's timing is not "
            "guaranteed.\n",
            rx_rate_, flags, n_unstamped + 1);
      }
    }
    if (pad > 0) {
      // The r samples just read belong at got+pad: shift them forward and zero-fill
      // the gap so the window stays sample-exact (one gap can't time-shift the rest).
      const size_t keep = std::min(static_cast<size_t>(r),
                                   static_cast<size_t>(samples - got) - pad);
      for (size_t c = 0; c < num_rx_ch_; c++) {
        uint8_t* d = static_cast<uint8_t*>(buffs[c]) +
                     static_cast<size_t>(got) * kBytesPerSamp;
        std::memmove(d + pad * kBytesPerSamp, d, keep * kBytesPerSamp);
        std::memset(d, 0, pad * kBytesPerSamp);
      }
      Sounder::RxGapSink::instance().push({rx_sample_pos_ + got,
                                           static_cast<int64_t>(pad),
                                           Sounder::kGapTimeJump});
      padded += pad;
      got += static_cast<int>(pad + keep);
    } else {
      got += r;
    }
  }
  rx_sample_pos_ += got;
  last_pad_samples_ = padded;
  if ((getenv("HOUDINI_CL_RX_DEBUG") != nullptr ||
       getenv("HOUDINI_DUMP_WIN") != nullptr) &&
      got > 0 && buffs[0] != nullptr) {
    const int16_t* p = static_cast<const int16_t*>(buffs[0]);
    double s = 0;
    int amax = 0;
    for (int k = 0; k < got * 2; ++k) {
      s += double(p[k]) * p[k];
      amax = std::max(amax, std::abs((int)p[k]));
    }
    const double rms = std::sqrt(s / (got * 2));
    if (getenv("HOUDINI_CL_RX_DEBUG") != nullptr) {
      static std::atomic<int> cnt{0};
      if ((cnt.fetch_add(1) % 40) == 0) {  // braces load-bearing: MLPD_INFO
        MLPD_INFO("Houdini client RX dbg: got=%d rms=%.2f absmax=%d\n", got,
                  rms, amax);                 // is a multi-statement macro
      }
    }
    // Dump the first strong (beacon-present) window for offline correlation.
    if (getenv("HOUDINI_DUMP_WIN") != nullptr && rms > 100.0) {
      static std::atomic<bool> done{false};
      bool expected = false;
      if (done.compare_exchange_strong(expected, true)) {
        const std::string path = Utils::dumpPath("cl_win.bin");
        FILE* f = std::fopen(path.c_str(), "wb");
        if (f == nullptr) {
          MLPD_WARN("HOUDINI_DUMP_WIN: cannot open %s (%s)\n", path.c_str(), std::strerror(errno));
        }
        if (f) {
          std::fwrite(p, sizeof(int16_t), static_cast<size_t>(got) * 2, f);
          std::fclose(f);
          MLPD_INFO("Dumped client beacon window rms=%.1f got=%d -> %s\n",
                    rms, got, path.c_str());
        }
      }
    }
  }
  if (rx_profile_every > 0) {
    const auto p_t2 = std::chrono::steady_clock::now();
    p_read += std::chrono::duration<double, std::micro>(p_t2 - p_t1).count();
    if (++p_calls >= rx_profile_every) {
      MLPD_INFO(
          "RX PROFILE over %zu radioRx calls: drain %.0f us (%.1f chunks, "
          "%.0f samples) + read %.0f us = %.0f us/call\n",
          p_calls, p_drain / p_calls, 1.0 * p_chunks / p_calls,
          1.0 * p_drained / p_calls, p_read / p_calls,
          (p_drain + p_read) / p_calls);
      p_drain = p_read = 0;
      p_calls = p_chunks = p_drained = 0;
    }
  }
  // AP-79: the lanes whose channel's mirror lands in the output get the
  // +-25 MHz channel filter before any consumer (detector, CFO, framer, CSI)
  // sees them. Every Houdini read comes through here.
  if (rx_filters_ != nullptr && got > 0) {
    rx_filters_->apply(buffs, num_rx_ch_, static_cast<size_t>(got));
  }
  return got;
}

