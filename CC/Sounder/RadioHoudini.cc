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
#include <ctime>
#include <cstring>
#include <functional>
#include <iostream>
#include <set>
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
  // The host UDP port a single-channel RX stream binds. The FPGA sends each
  // RX channel to a FIXED destination port, 10001 + channel (the RX stream
  // contract, SH-142/SH-159), whatever the host binds; the driver accepts any
  // local_port and a mismatched one delivers NOTHING (every datagram lands on
  // NoPorts). AP-79's R0 hit exactly that: a port fixed at 10002 (ch1's,
  // right for the old channel-B demo) on channel A. So derive it from the
  // channel. On a COMBINED (>1 channel) stream the driver rejects local_port
  // and assigns the per-channel ports itself, so leave it unset.
  if (p.rx_channels.size() == 1) {
    rx["local_port"] = std::to_string(10001 + p.rx_channels.front());
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

void RadioHoudini::logModeV(const std::string& label, const std::vector<std::string>& lines) {
  for (const auto& entry : lines) {
    if (entry.rfind("WARNING", 0) == 0) {
      MLPD_WARN("%s mode V: %s\n", label.c_str(), entry.c_str());
    } else {
      MLPD_INFO("%s mode V: %s\n", label.c_str(), entry.c_str());
    }
  }
}

void RadioHoudini::writeModeVRecord(const std::string& label, SoapySDR::Device& dev,
                                    const houdini::modev::Result& r,
                                    const houdini::modev::PostSetup& ps, const std::string& failure) {
 try {
  // The converter state this session ran with, beside the run (plan rule 5:
  // a capture carries its state). One file per node per bring-up, under
  // HOUDINI_DUMP_DIR (Utils::dumpPath). Best effort: a record that cannot be
  // written is warned about, never fatal.
  std::string tag = label;
  for (auto& ch : tag)
    if (ch == ' ' || ch == '.' || ch == '/') ch = '_';
  const std::time_t now = std::time(nullptr);
  char stamp[32];
  std::strftime(stamp, sizeof stamp, "%Y%m%d-%H%M%S", std::localtime(&now));
  const std::string name = "modev_" + tag + "_" + stamp + ".txt";
  const std::string path = Utils::dumpPath(name.c_str());
  FILE* f = std::fopen(path.c_str(), "w");
  if (f == nullptr) {
    MLPD_WARN("%s mode V: cannot write the session record %s (%s)\n", label.c_str(), path.c_str(),
              std::strerror(errno));
    return;
  }
  std::fprintf(f, "# %s mode-V session record\n", label.c_str());
  if (!failure.empty()) std::fprintf(f, "RESULT: %s\n", failure.c_str());
  try {
    for (const auto& kv : dev.getHardwareInfo()) std::fprintf(f, "hw %s=%s\n", kv.first.c_str(), kv.second.c_str());
  } catch (const std::exception& e) {
    std::fprintf(f, "hw: getHardwareInfo failed: %s\n", e.what());
  }
  std::fprintf(f, "## bring-up\n");
  for (const auto& l : r.log) std::fprintf(f, "%s\n", l.c_str());
  std::fprintf(f, "## after the setups\n");
  for (const auto& l : ps.log) std::fprintf(f, "%s\n", l.c_str());
  std::fprintf(f, "## RFDC_SNAPSHOT (before the setups)\n%s\n", r.snapshot.c_str());
  std::fclose(f);
  MLPD_INFO("%s mode V: session record %s\n", label.c_str(), path.c_str());
 } catch (const std::exception& e) {
  // Best effort, never fatal: a record that cannot be written must not
  // refuse a healthy radio (review).
  MLPD_WARN("%s mode V: the session record could not be written: %s\n", label.c_str(), e.what());
 }
}

void RadioHoudini::writeStateRecord(const std::string& label, SoapySDR::Device& dev, const std::string& stage,
                                    const std::vector<size_t>& rx_channels,
                                    const std::vector<size_t>& tx_channels) {
  try {
    std::string tag = label;
    for (auto& ch : tag)
      if (ch == ' ' || ch == '.' || ch == '/') ch = '_';
    const std::time_t now = std::time(nullptr);
    char stamp[32];
    std::strftime(stamp, sizeof stamp, "%Y%m%d-%H%M%S", std::localtime(&now));
    const std::string name = "rfdc_" + tag + "_" + stage + "_" + stamp + ".txt";
    const std::string path = Utils::dumpPath(name.c_str());
    FILE* f = std::fopen(path.c_str(), "w");
    if (f == nullptr) {
      MLPD_WARN("%s: cannot write the RFDC state record %s (%s)\n", label.c_str(), path.c_str(),
                std::strerror(errno));
      return;
    }
    std::fprintf(f, "# %s RFDC state, %s, %s\n", label.c_str(), stage.c_str(), stamp);
    // Each read on its own: one that fails is noted and the rest still land.
    auto section = [&](const char* title, const std::function<std::string()>& read) {
      std::fprintf(f, "## %s\n", title);
      try {
        std::fprintf(f, "%s\n", read().c_str());
      } catch (const std::exception& e) {
        std::fprintf(f, "(read failed: %s)\n", e.what());
      }
    };
    section("hardware", [&] {
      std::string s;
      for (const auto& kv : dev.getHardwareInfo()) s += kv.first + "=" + kv.second + "\n";
      return s;
    });
    section("RFDC_PREFLIGHT", [&] { return dev.readSetting("RFDC_PREFLIGHT"); });
    auto info = [&](int dir, const char* name, const std::vector<size_t>& chans) {
      for (auto ch : chans) {
        const std::string title = std::string("getChannelInfo ") + name + " ch" + std::to_string(ch);
        section(title.c_str(), [&] {
          std::string s;
          for (const auto& kv : dev.getChannelInfo(dir, ch)) s += kv.first + "=" + kv.second + "\n";
          return s;
        });
      }
    };
    info(SOAPY_SDR_RX, "RX", rx_channels);
    info(SOAPY_SDR_TX, "TX", tx_channels);
    section("RFDC_SNAPSHOT", [&] { return dev.readSetting("RFDC_SNAPSHOT"); });
    std::fclose(f);
    MLPD_INFO("%s: RFDC state record (%s) %s\n", label.c_str(), stage.c_str(), path.c_str());
  } catch (const std::exception& e) {
    MLPD_WARN("%s: the RFDC state record (%s) could not be written: %s\n", label.c_str(), stage.c_str(),
              e.what());
  }
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
                         try {
                           *mv = houdini::modev::bringUp(dev, plan);
                         } catch (const std::exception& e) {
                           // The failed bring-up is the run most worth a record.
                           writeModeVRecord(label, dev, *mv, houdini::modev::PostSetup{},
                                            std::string("bring-up FAILED: ") + e.what());
                           throw;
                         }
                         logModeV(label, mv->log);
                       },
                 // After the last setup, before any activate: every Houdini run
                 // records its converter state here; mode V checks it first.
                 mv == nullptr
                     ? std::function<void(SoapySDR::Device&)>(
                           [label = params.label, rx = params.rx_channels,
                            tx = params.tx_channels](SoapySDR::Device& dev) {
                             writeStateRecord(label, dev, "pre-activate", rx, tx);
                           })
                     : [mv, plan = modeVPlan(params), label = params.label, rx = params.rx_channels,
                        tx = params.tx_channels](SoapySDR::Device& dev) {
                         houdini::modev::PostSetup ps;
                         try {
                           ps = houdini::modev::postSetupCheck(dev, plan, *mv);
                         } catch (const std::exception& e) {
                           writeModeVRecord(label, dev, *mv, ps, std::string("post-setup check FAILED: ") + e.what());
                           throw;
                         }
                         logModeV(label, ps.log);
                         writeModeVRecord(label, dev, *mv, ps, "");
                         writeStateRecord(label, dev, "pre-activate", rx, tx);
                       }),
      mode_v_(std::move(mv)) {
  if (params.tx_rate_hz > 0.0 && params.tx_rate_hz != params.rate_hz) {
    if (params.tx_rate_hz != 2.0 * params.rate_hz) {
      throw std::invalid_argument("RadioHoudini: only TX = 2 x sample_rate is interpolated");
    }
    // Shape the TX spectrum with the channel filter whenever the waveform fits
    // its passband (every first-pass channel does): unshaped splatter beyond
    // 40.2 MHz folds back onto the sub-6 channel at the far ADC.
    const bool prefilter = params.half_bw_hz > 0.0 &&
                           params.half_bw_hz <= houdini::rfplan::Rules{}.filter_pass_hz;
    tx_interp_ = std::make_unique<houdini::boundary::TxBurstInterpolator>(prefilter);
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

RadioHoudini::~RadioHoudini() {
  {
    std::lock_guard<std::mutex> lk(health_mtx_);
    health_stop_ = true;
  }
  health_cv_.notify_all();
  if (health_thread_.joinable()) health_thread_.join();
  // The end-of-run state, while the streams are still open (the base class
  // closes them after this), so drift across the run is visible.
  if (dev_ != nullptr) writeStateRecord(params_.label, *dev_, "end", params_.rx_channels, params_.tx_channels);
}

void RadioHoudini::maybeStartHealth() {
  // AP-79: the software lane's link-health checks (houdini/link_health.h) on
  // THIS session's handle, once streaming. HOUDINI_LINK_HEALTH_S sets the
  // period (default 5 s in mode V, off otherwise; 0 turns it off).
  if (health_started_.exchange(true)) return;
  // On by default in mode V only; the one-rate path keeps its control plane
  // as it was unless asked.
  double period = mode_v_ != nullptr ? 5.0 : 0.0;
  if (const char* e = std::getenv("HOUDINI_LINK_HEALTH_S")) period = std::atof(e);
  if (!(period > 0.0)) return;
  health_thread_ = std::thread([this, period] { healthLoop(period); });
}

void RadioHoudini::healthLoop(double period_s) {
  auto wait = [this](double s) {
    std::unique_lock<std::mutex> lk(health_mtx_);
    return !health_cv_.wait_for(lk, std::chrono::duration<double>(s), [this] { return health_stop_; });
  };
  if (!wait(2.0)) return;  // let the streams settle before the baseline
  const std::string label = params_.label;
  try {
    // The bring-up latches benign ADC flags (SH-372 class): clear them now
    // that the group is up and active (about 2 s after the first read), so
    // the baseline and every later verdict describe this session (software
    // lane, SH-422 silicon check). What is latched is logged first, so the
    // clear erases nothing without a trace. The driver REFUSES the clear when
    // a judged bit is still set after it (level-asserted, re-latched, or new):
    // that refusal names the bits, so it is logged and the baseline is taken
    // anyway rather than stopping the monitor (review).
    {
      const std::string before = dev_->readSetting("RFDC_PREFLIGHT");
      MLPD_INFO("%s link health: preflight before the post-activate clear: %s\n", label.c_str(),
                before.substr(0, before.find('\n')).c_str());
      try {
        dev_->writeSetting("RFDC_PREFLIGHT", "clear");
      } catch (const std::exception& e) {
        MLPD_WARN("%s link health: the preflight clear was refused (a bit still set after it): %s\n",
                  label.c_str(), e.what());
      }
      const std::string after = dev_->readSetting("RFDC_PREFLIGHT");
      MLPD_INFO("%s link health: preflight after the post-activate clear: %s\n", label.c_str(),
                after.substr(0, after.find('\n')).c_str());
    }
    houdini::health::LinkHealth h([this](const std::string& k) { return dev_->readSetting(k); }, label);
    std::string at_start;
    for (const auto& f : h.baselineFailures()) at_start += (at_start.empty() ? "" : "; ") + f;
    MLPD_INFO("%s link health: baseline taken; preflight FAILs standing at start: %s\n", label.c_str(),
              at_start.empty() ? "none" : at_start.c_str());
    std::set<std::string> reported;  // blind/drift alarms already warned about
    unsigned long long p_err = app_rx_err_, p_short = app_rx_short_, p_pad = app_rx_pad_,
                       p_txs = app_tx_short_, p_sat = app_tx_sat_;
    for (unsigned n = 1; wait(period_s); ++n) {
      const auto rep = h.check();
      const unsigned long long c_err = app_rx_err_, c_short = app_rx_short_, c_pad = app_rx_pad_,
                               c_txs = app_tx_short_, c_sat = app_tx_sat_;
      char app[160];
      std::snprintf(app, sizeof app, " | app: rx_err +%llu, rx_short +%llu, rx_pad +%llu, tx_short +%llu, tx_sat +%llu",
                    c_err - p_err, c_short - p_short, c_pad - p_pad, c_txs - p_txs, c_sat - p_sat);
      const bool app_bad = c_err != p_err || c_pad != p_pad || c_txs != p_txs || c_sat != p_sat;
      p_err = c_err; p_short = c_short; p_pad = c_pad; p_txs = c_txs; p_sat = c_sat;
      // Unattended, a condition that cannot clear within a session (a sticky
      // or saturated egress counter, a drifted config section) would WARN
      // every period and bury the new alarms: warn when one first appears or
      // changes; the periodic line still carries it. Counter rises and new
      // preflight items are new by construction and always warn.
      bool fresh = !rep.increases.empty() || !rep.new_failures.empty();
      std::set<std::string> standing;
      for (const auto* v : {&rep.blind, &rep.drift})
        for (const auto& s : *v) {
          standing.insert(s);
          if (reported.count(s) == 0) fresh = true;
        }
      reported = standing;
      if (fresh || app_bad) {
        MLPD_WARN("%s link health: %s%s\n", label.c_str(), rep.line().c_str(), app);
      } else if (n % 12 == 0) {
        MLPD_INFO("%s link health: %s%s\n", label.c_str(), rep.line().c_str(), app);
      }
    }
  } catch (const std::exception& e) {
    MLPD_WARN("%s link health stopped: %s (a key this device does not report?)\n", label.c_str(), e.what());
  }
}

int RadioHoudini::xmit(const void* const* buffs, int samples, int flags,
                       long long& frameTime) {
  // AP-79: at TX = 2 x sample_rate every burst, built at the tick rate, is
  // interpolated here as a whole and padded to a whole 8-sample TX beat. The
  // time is in ns and does not change; the caller counts in ticks, so a full
  // write reports its own sample count back.
  if (!tx_interp_ || samples <= 0) {
    const int r0 = RadioSoapy::xmit(buffs, samples, flags, frameTime);
    // RadioSoapy::xmit returns 0 on a radio with no TX stream: not a short write.
    if (!params_.tx_channels.empty() && r0 < samples) app_tx_short_.fetch_add(1, std::memory_order_relaxed);
    return r0;
  }
  const auto o = tx_interp_->run(buffs, params_.tx_channels.size(), static_cast<size_t>(samples));
  if (o.saturated > 0) {
    app_tx_sat_.fetch_add(1, std::memory_order_relaxed);
    static std::atomic<unsigned> warned{0};
    if (warned.fetch_add(1) < 5) {
      MLPD_WARN("%s: %zu I/Q components saturated in the x2 TX interpolation; "
                "lower the TX level (the halfband overshoots near edges)\n",
                params_.label.c_str(), o.saturated);
    }
  }
  const int r = RadioSoapy::xmit(o.buffs.data(), static_cast<int>(o.samples), flags, frameTime);
  if (!params_.tx_channels.empty() && r < static_cast<int>(o.samples))
    app_tx_short_.fetch_add(1, std::memory_order_relaxed);
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
      if (r < 0) app_rx_err_.fetch_add(1, std::memory_order_relaxed);
      if (got > 0) app_rx_short_.fetch_add(1, std::memory_order_relaxed);
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
  // AP-79: the lanes whose channel's mirror lands in the output get the
  // +-25 MHz channel filter before any consumer (detector, CFO, framer, CSI)
  // sees them. On the UE every read is filtered here and the window dump below
  // shows the filtered samples, the ones the detector sees. On the BS the
  // framer turns this off and filters the slots it extracts instead
  // (HoudiniFramer::rx), so there the dump shows the RAW capture. The RX
  // PROFILE's "read" time includes this filter (review).
  if (rx_filters_ != nullptr && recv_filter_ && got > 0) {
    rx_filters_->apply(buffs, num_rx_ch_, static_cast<size_t>(got));
  }
  rx_sample_pos_ += got;
  last_pad_samples_ = padded;
  if (padded > 0) app_rx_pad_.fetch_add(1, std::memory_order_relaxed);
  if (got > 0) maybeStartHealth();
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
  return got;
}

