/**
 * @file houdini/mode_v_bringup.h
 * @brief The mode-V converter bring-up of one Houdini node, in the order the
 *        device requires (AP-79), between make() and the first setupStream.
 *
 * THE ORDER, and why each step sits where it does. It is the software lane's
 * recipe (handoff 2026-09-22, the reference driver host/examples/dualband_link.py)
 * and the HS-202 plan section 3.2, which the device enforces:
 *   1. FORCE_IDLE: a known-idle device, leaks from an earlier session reported.
 *   2. setSampleRate(TX) on EVERY TX channel, then RFDC_DAC_FS: the TX rate
 *      and the DAC Fs together fix interpolation 24.
 *   3. RFDC_ADC_FS, then setSampleRate(RX) on every RX channel: decimation 40
 *      is derived from the Fs, so the Fs goes first.
 *   4. The DAC zones, then the inverse sinc (it must match the zone, and a
 *      later zone change re-keys or drops it), then the ADC zones: each zone's
 *      band check is computed from the live Fs, so after step 3.
 *   5. RFDC_ADC_CAL: the calibration mode is stored now and written at the
 *      tile's StartUp, which the setupStream performs; every RX channel's
 *      mode is written explicitly so the snapshot shows the intent.
 *   6. The NCOs: a tune outside the current zone's band is refused, so after
 *      the zones.
 *   7. The gains, explicitly: make() resets RX DSA to 0 dB and TX VOP to
 *      20 mA, and nothing survives from an earlier session.
 *   8. RFDC_SNAPSHOT, kept for the record.
 * Every rate, NCO and gain is read back and a mismatch throws: a setter
 * returning is not evidence the device holds the value (the SH-338 class).
 *
 * The zone, calibration mode and inverse sinc are derived per channel from
 * the NCO by houdini/rf_plan.h; only the NCO is configured per channel.
 *
 * It takes the device by reference and opens no stream, so the test drives it
 * with a recording fake device and checks the order and the values.
 *
 * RENEW OPEN SOURCE LICENSE: http://renew-wireless.org/license
 */
#pragma once

#include <cmath>
#include <cstdio>
#include <tuple>
#include <cstdlib>
#include <limits>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>

#include "SoapySDR/Constants.h"
#include "SoapySDR/Device.hpp"
#include "houdini/rf_plan.h"

namespace houdini {
namespace modev {

struct Plan {
  std::vector<size_t> tx_channels;  ///< the channels this node opens for TX
  std::vector<size_t> rx_channels;  ///< .. and for RX
  double tx_rate_hz = 0.0;
  double rx_rate_hz = 0.0;
  double adc_fs_hz = 0.0;
  double dac_fs_hz = 0.0;
  double default_nco_hz = 0.0;             ///< the common NCO
  std::map<size_t, double> nco_by_channel; ///< the per-channel overrides
  double half_bw_hz = 0.0;                 ///< occupied half bandwidth of the waveform
  double tx_gain_db = std::numeric_limits<double>::quiet_NaN();  ///< NaN: not written
  double rx_gain_db = std::numeric_limits<double>::quiet_NaN();
  double rx_freq_offset_hz = 0.0;          ///< deliberate detune (AP-33), normally 0
  double tx_freq_offset_hz = 0.0;

  double ncoFor(size_t ch) const {
    const auto it = nco_by_channel.find(ch);
    return it != nco_by_channel.end() ? it->second : default_nco_hz;
  }
  rfplan::ConverterPoint converters() const { return {adc_fs_hz, dac_fs_hz, rx_rate_hz, tx_rate_hz}; }
};

struct ChannelResult {
  size_t channel = 0;
  double nco_hz = 0.0;
  int zone = 0;
  int cal_mode = 0;          ///< RX only
  bool channel_filter = false;  ///< RX only
};

struct Result {
  std::vector<ChannelResult> tx, rx;
  std::vector<std::string> log;  ///< one line per step, with the device's readbacks
  std::string snapshot;          ///< RFDC_SNAPSHOT after the writes

  /// Whether RX `ch` needs the +-25 MHz channel filter.
  bool rxFilter(size_t ch) const {
    for (const auto& r : rx)
      if (r.channel == ch) return r.channel_filter;
    return false;
  }
};

namespace detail {
inline std::string fmt(const char* f, double v) {
  char b[64];
  std::snprintf(b, sizeof b, f, v);
  return b;
}
inline std::string join(const std::vector<std::string>& v) {
  std::string s;
  for (const auto& x : v) s += (s.empty() ? "" : ";") + x;
  return s;
}
/// RFDC_ADC_FS / RFDC_DAC_FS: '0:fs=4915.200,max=.. 2:fs=4915.200,..' (MHz,
/// one entry per tile; the DAC's `mode=DUC(0->Fs/2) IQ-in` has spaces, so
/// only the `fs=` fields are read). Every tile's fs, in MHz.
inline std::vector<double> fsValuesMhz(const std::string& raw) {
  std::vector<double> out;
  for (size_t p = raw.find("fs="); p != std::string::npos; p = raw.find("fs=", p + 3)) {
    if (p > 0 && raw[p - 1] != ':' && raw[p - 1] != ',') continue;  // not e.g. queried_max... (safety)
    out.push_back(std::atof(raw.c_str() + p + 3));
  }
  return out;
}
/// 'ch0:1;ch1:2' (zones, inverse sinc) -> {0: "1", 1: "2"}.
inline std::map<size_t, std::string> chanList(const std::string& raw) {
  std::map<size_t, std::string> out;
  size_t p = 0;
  while (p < raw.size()) {
    const size_t e = std::min(raw.find(';', p), raw.size());
    std::string item = raw.substr(p, e - p);
    // Whitespace around an item (a newline at the end, a space after ';')
    // must not fail a healthy readback (review).
    const size_t a = item.find_first_not_of(" \t\r\n"), b = item.find_last_not_of(" \t\r\n");
    item = (a == std::string::npos) ? std::string() : item.substr(a, b - a + 1);
    const size_t c = item.find(':');
    if (item.rfind("ch", 0) == 0 && c != std::string::npos && c > 2)
      out[static_cast<size_t>(std::atoi(item.c_str() + 2))] = item.substr(c + 1);
    p = e + 1;
  }
  return out;
}
/// One field of one block of RFDC_ADC_CAL '0.0:dither=on,cal=mode2,.. 0.1:..':
/// the value of `key` in the entry for `addr` ("<tile>.<block>"), or "".
inline std::string blockField(const std::string& raw, const std::string& addr, const std::string& key) {
  size_t p = raw.find(addr + ":");
  while (p != std::string::npos && p > 0 && raw[p - 1] != ' ') p = raw.find(addr + ":", p + 1);
  if (p == std::string::npos) return "";
  const size_t end = std::min(raw.find(' ', p), raw.size());
  const std::string entry = raw.substr(p + addr.size() + 1, end - p - addr.size() - 1);
  size_t q = 0;
  while (q < entry.size()) {
    const size_t e = std::min(entry.find(',', q), entry.size());
    const std::string kv = entry.substr(q, e - q);
    if (kv.rfind(key + "=", 0) == 0) return kv.substr(key.size() + 1);
    q = e + 1;
  }
  return "";
}

inline void expectNear(const char* what, size_t ch, double got, double want, double tol) {
  if (!(std::fabs(got - want) <= tol)) {
    char b[160];
    std::snprintf(b, sizeof b, "mode V bring-up: %s ch%zu reads back %.6f, wanted %.6f", what, ch, got, want);
    throw std::runtime_error(b);
  }
}
}  // namespace detail

/// Run the bring-up. Throws std::invalid_argument for a plan the rules refuse
/// (before anything is written) and std::runtime_error for a readback that
/// disagrees with what was written.
inline Result bringUp(SoapySDR::Device& dev, const Plan& p) {
  using detail::fmt;
  if (!(p.tx_rate_hz > 0.0 && p.rx_rate_hz > 0.0 && p.adc_fs_hz > 0.0 && p.dac_fs_hz > 0.0)) {
    throw std::invalid_argument("mode V bring-up: rates and converter Fs must all be set");
  }
  Result res;
  // Derive and validate every channel BEFORE any write, so a refused plan
  // leaves the device untouched.
  const auto cp = p.converters();
  for (size_t ch : p.tx_channels) {
    const auto t = rfplan::planTx(p.ncoFor(ch), p.half_bw_hz, cp);
    res.tx.push_back({ch, p.ncoFor(ch), t.zone, 0, false});
  }
  for (size_t ch : p.rx_channels) {
    const auto r = rfplan::planRx(p.ncoFor(ch), p.half_bw_hz, cp);
    res.rx.push_back({ch, p.ncoFor(ch), r.zone, r.cal_mode, r.channel_filter});
  }
  auto logLine = [&res](const std::string& s) { res.log.push_back(s); };
  if (p.rx_freq_offset_hz != 0.0 || p.tx_freq_offset_hz != 0.0) {
    // The one-rate path warns about a deliberate detune; mode V must too, or a
    // test injection reads as a nominal run (AP-33).
    logLine("WARNING: DELIBERATE frequency offset in effect: RX " + fmt("%+.1f", p.rx_freq_offset_hz) +
            " Hz, TX " + fmt("%+.1f", p.tx_freq_offset_hz) +
            " Hz off the NCOs. A test injection; results are NOT nominal.");
  }

  // 1
  dev.writeSetting("FORCE_IDLE", "");
  logLine("FORCE_IDLE");
  // 2
  const size_t n_tx = dev.getNumChannels(SOAPY_SDR_TX), n_rx = dev.getNumChannels(SOAPY_SDR_RX);
  for (size_t ch = 0; ch < n_tx; ++ch) dev.setSampleRate(SOAPY_SDR_TX, ch, p.tx_rate_hz);
  dev.writeSetting("RFDC_DAC_FS", fmt("%.4f", p.dac_fs_hz / 1e6));
  const std::string dac_fs = dev.readSetting("RFDC_DAC_FS");
  logLine("RFDC_DAC_FS -> " + dac_fs);
  // 3
  dev.writeSetting("RFDC_ADC_FS", fmt("%.4f", p.adc_fs_hz / 1e6));
  const std::string adc_fs = dev.readSetting("RFDC_ADC_FS");
  logLine("RFDC_ADC_FS -> " + adc_fs);
  // Every tile must read the Fs written (plan 3.1: "readback must equal"): a
  // discarded or re-picked Fs would change every decimation, zone and alias.
  for (const auto& [raw, want, what] : {std::tuple<std::string, double, const char*>{dac_fs, p.dac_fs_hz, "RFDC_DAC_FS"},
                                        {adc_fs, p.adc_fs_hz, "RFDC_ADC_FS"}}) {
    const auto v = detail::fsValuesMhz(raw);
    bool ok = !v.empty();
    for (double f : v) ok = ok && std::fabs(f - want / 1e6) < 1e-3;
    if (!ok) throw std::runtime_error(std::string("mode V bring-up: ") + what + " reads '" + raw + "', wanted " + fmt("%.3f", want / 1e6) + " MHz on every tile");
  }
  for (size_t ch = 0; ch < n_rx; ++ch) dev.setSampleRate(SOAPY_SDR_RX, ch, p.rx_rate_hz);
  for (const auto& t : res.tx)
    detail::expectNear("TX rate", t.channel, dev.getSampleRate(SOAPY_SDR_TX, t.channel), p.tx_rate_hz, 1.0);
  for (const auto& r : res.rx)
    detail::expectNear("RX rate", r.channel, dev.getSampleRate(SOAPY_SDR_RX, r.channel), p.rx_rate_hz, 1.0);
  // 4
  if (!res.tx.empty()) {
    std::vector<std::string> z, s;
    for (const auto& t : res.tx) {
      z.push_back("ch" + std::to_string(t.channel) + ":" + std::to_string(t.zone));
      s.push_back("ch" + std::to_string(t.channel) + ":zone");
    }
    dev.writeSetting("RFDC_TX_NYQUIST_ZONE", detail::join(z));
    dev.writeSetting("RFDC_TX_INVSINC", detail::join(s));
    const std::string tz = dev.readSetting("RFDC_TX_NYQUIST_ZONE"), ti = dev.readSetting("RFDC_TX_INVSINC");
    logLine("RFDC_TX_NYQUIST_ZONE " + detail::join(z) + " -> " + tz);
    logLine("RFDC_TX_INVSINC " + detail::join(s) + " -> " + ti);
    // The zone must read back, and the inverse sinc resolves `zone` to the
    // block's zone: 'zone<k>' (software lane's reader).
    const auto tzm = detail::chanList(tz), tim = detail::chanList(ti);
    for (const auto& t : res.tx) {
      const auto a = tzm.find(t.channel), b = tim.find(t.channel);
      if (a == tzm.end() || a->second != std::to_string(t.zone))
        throw std::runtime_error("mode V bring-up: RFDC_TX_NYQUIST_ZONE reads '" + tz + "', wanted ch" + std::to_string(t.channel) + ":" + std::to_string(t.zone));
      if (b == tim.end() || b->second != "zone" + std::to_string(t.zone))
        throw std::runtime_error("mode V bring-up: RFDC_TX_INVSINC reads '" + ti + "', wanted ch" + std::to_string(t.channel) + ":zone" + std::to_string(t.zone));
    }
  }
  if (!res.rx.empty()) {
    std::vector<std::string> z, c;
    for (const auto& r : res.rx) {
      z.push_back("ch" + std::to_string(r.channel) + ":" + std::to_string(r.zone));
      c.push_back("ch" + std::to_string(r.channel) + ":cal=mode" + std::to_string(r.cal_mode));
    }
    dev.writeSetting("RFDC_RX_NYQUIST_ZONE", detail::join(z));
    const std::string rz = dev.readSetting("RFDC_RX_NYQUIST_ZONE");
    logLine("RFDC_RX_NYQUIST_ZONE " + detail::join(z) + " -> " + rz);
    const auto rzm = detail::chanList(rz);
    for (const auto& r : res.rx) {
      const auto a = rzm.find(r.channel);
      if (a == rzm.end() || a->second != std::to_string(r.zone))
        throw std::runtime_error("mode V bring-up: RFDC_RX_NYQUIST_ZONE reads '" + rz + "', wanted ch" + std::to_string(r.channel) + ":" + std::to_string(r.zone));
    }
    // 5
    dev.writeSetting("RFDC_ADC_CAL", detail::join(c));
    logLine("RFDC_ADC_CAL " + detail::join(c) + " -> " + dev.readSetting("RFDC_ADC_CAL"));
  }
  // 6
  for (const auto& r : res.rx) {
    const double f = r.nco_hz + p.rx_freq_offset_hz;
    dev.setFrequency(SOAPY_SDR_RX, r.channel, f);
    detail::expectNear("RX NCO", r.channel, dev.getFrequency(SOAPY_SDR_RX, r.channel), f, 1.0);
  }
  for (const auto& t : res.tx) {
    const double f = t.nco_hz + p.tx_freq_offset_hz;
    dev.setFrequency(SOAPY_SDR_TX, t.channel, f);
    detail::expectNear("TX NCO", t.channel, dev.getFrequency(SOAPY_SDR_TX, t.channel), f, 1.0);
  }
  // 7
  if (!std::isnan(p.tx_gain_db)) {
    for (const auto& t : res.tx) {
      dev.setGain(SOAPY_SDR_TX, t.channel, p.tx_gain_db);
      detail::expectNear("TX gain", t.channel, dev.getGain(SOAPY_SDR_TX, t.channel), p.tx_gain_db, 0.5);
    }
  }
  if (!std::isnan(p.rx_gain_db)) {
    for (const auto& r : res.rx) {
      dev.setGain(SOAPY_SDR_RX, r.channel, p.rx_gain_db);
      detail::expectNear("RX gain", r.channel, dev.getGain(SOAPY_SDR_RX, r.channel), p.rx_gain_db, 0.5);
    }
  }
  for (const auto& t : res.tx)
    logLine("TX ch" + std::to_string(t.channel) + fmt(": NCO %.3f MHz", t.nco_hz / 1e6) + ", zone " +
            std::to_string(t.zone) + ", inverse sinc follows the zone");
  for (const auto& r : res.rx)
    logLine("RX ch" + std::to_string(r.channel) + fmt(": NCO %.3f MHz", r.nco_hz / 1e6) + ", zone " +
            std::to_string(r.zone) + ", cal Mode " + std::to_string(r.cal_mode) +
            (r.channel_filter ? ", +-25 MHz channel filter ON" : ", no channel filter"));
  // 8
  res.snapshot = dev.readSetting("RFDC_SNAPSHOT");
  // NOT cleared here: the bring-up itself latches benign over-voltage and
  // common-mode flags on the streamed ADCs (SH-372 class), so the preflight is
  // cleared once streaming (about 2 s after the first read) and judged from
  // there (software lane, SH-422 silicon check), by the link-health baseline
  // (RadioHoudini).
  return res;
}

/// What the check after the LAST setupStream found (the MTS sync runs inside
/// each mts=true setup, so the group's state is final here, before activate).
struct PostSetup {
  std::vector<std::string> log;
  std::string preflight;              ///< the verdict's first line
  std::vector<std::string> failures;  ///< FAIL items outside the known part
  std::string irq_count;
};

/// After the setups, before activate: every opened channel MTS-synced (the
/// device also refuses to activate an unsynced group; this names it first),
/// its latency T1 recorded; each RX channel's calibration mode as the tile
/// started it (SH-420: `cal=` is valid right after the setups); the preflight
/// verdict and the interrupt count. Throws on an unsynced channel or a wrong
/// calibration mode.
inline PostSetup postSetupCheck(SoapySDR::Device& dev, const Plan& p, const Result& r) {
  PostSetup ps;
  auto chan = [&](int dir, size_t ch, const char* tag) {
    const auto info = dev.getChannelInfo(dir, ch);
    auto get = [&info](const char* k) { const auto it = info.find(k); return it == info.end() ? std::string() : it->second; };
    const std::string synced = get("rfdc_mts_synced"), t1 = get("rfdc_mts_latency");
    ps.log.push_back(std::string(tag) + " ch" + std::to_string(ch) + ": " + get("rfdc_label") + " mts_synced=" +
                     (synced.empty() ? "?" : synced) + " T1=" + (t1.empty() ? "-" : t1));
    if (synced != "1")
      throw std::runtime_error("mode V: " + std::string(tag) + " ch" + std::to_string(ch) +
                               " is not MTS-synced after the setups (rfdc_mts_synced=" + synced + ")");
    return info;
  };
  for (const auto& t : r.tx) chan(SOAPY_SDR_TX, t.channel, "TX");
  // The driver re-applies the zones and the inverse sinc at each tile's
  // StartUp (inside the setups): read them again now that the tiles are up.
  {
    const auto tz = detail::chanList(dev.readSetting("RFDC_TX_NYQUIST_ZONE"));
    const auto ti = detail::chanList(dev.readSetting("RFDC_TX_INVSINC"));
    const auto rz = detail::chanList(dev.readSetting("RFDC_RX_NYQUIST_ZONE"));
    for (const auto& t : r.tx) {
      const auto a = tz.find(t.channel), b = ti.find(t.channel);
      if (a == tz.end() || a->second != std::to_string(t.zone) || b == ti.end() ||
          b->second != "zone" + std::to_string(t.zone))
        throw std::runtime_error("mode V: TX ch" + std::to_string(t.channel) +
                                 " zone/inverse sinc changed across the setups (wanted zone " + std::to_string(t.zone) + ")");
    }
    for (const auto& x : r.rx) {
      const auto a = rz.find(x.channel);
      if (a == rz.end() || a->second != std::to_string(x.zone))
        throw std::runtime_error("mode V: RX ch" + std::to_string(x.channel) + " zone changed across the setups (wanted " +
                                 std::to_string(x.zone) + ")");
    }
    ps.log.push_back("zones and inverse sinc re-read after the setups: as written");
  }
  const std::string cal = dev.readSetting("RFDC_ADC_CAL");
  ps.log.push_back("RFDC_ADC_CAL after the setups -> " + cal);
  for (const auto& x : r.rx) {
    const auto info = chan(SOAPY_SDR_RX, x.channel, "RX");
    const auto ti = info.find("rfdc_tile_index"), bl = info.find("rfdc_block");
    if (ti == info.end() || bl == info.end()) continue;
    const std::string addr = ti->second + "." + bl->second;
    const std::string mode = detail::blockField(cal, addr, "cal");
    if (mode.empty()) {
      ps.log.push_back("WARNING: RFDC_ADC_CAL has no entry for " + addr + " (RX ch" + std::to_string(x.channel) + "); calibration mode not verified");
    } else if (mode != "mode" + std::to_string(x.cal_mode)) {
      throw std::runtime_error("mode V: RX ch" + std::to_string(x.channel) + " (ADC " + addr + ") runs cal=" + mode +
                               ", wanted mode" + std::to_string(x.cal_mode));
    }
  }
  (void)p;
  const std::string pf = dev.readSetting("RFDC_PREFLIGHT");
  ps.preflight = pf.substr(0, pf.find('\n'));
  if (ps.preflight.rfind("FAIL ", 0) == 0) {
    std::string body = ps.preflight.substr(5);
    const size_t k = body.find(" known ");
    if (k != std::string::npos) body = body.substr(0, k);
    size_t q = 0;
    while (q <= body.size()) {
      const size_t e = std::min(body.find(';', q), body.size());
      if (e > q) ps.failures.push_back(body.substr(q, e - q));
      q = e + 1;
    }
  }
  ps.irq_count = dev.readSetting("RFDC_INTR_FIRE_COUNT");
  ps.log.push_back("RFDC_PREFLIGHT after the setups -> " + ps.preflight);
  ps.log.push_back("RFDC_INTR_FIRE_COUNT -> " + ps.irq_count);
  // Informational at this point: the bring-up latches benign flags (SH-372
  // class) that read FAIL until cleared after activate. The judged verdict is
  // the link-health baseline, taken after that clear.
  if (!ps.failures.empty())
    ps.log.push_back("preflight items latched by the bring-up (cleared after activate, then judged): " +
                     std::to_string(ps.failures.size()));
  return ps;
}

}  // namespace modev
}  // namespace houdini
