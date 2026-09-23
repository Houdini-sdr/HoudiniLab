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
  logLine("RFDC_DAC_FS -> " + dev.readSetting("RFDC_DAC_FS"));
  // 3
  dev.writeSetting("RFDC_ADC_FS", fmt("%.4f", p.adc_fs_hz / 1e6));
  logLine("RFDC_ADC_FS -> " + dev.readSetting("RFDC_ADC_FS"));
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
    logLine("RFDC_TX_NYQUIST_ZONE " + detail::join(z) + " -> " + dev.readSetting("RFDC_TX_NYQUIST_ZONE"));
    logLine("RFDC_TX_INVSINC " + detail::join(s) + " -> " + dev.readSetting("RFDC_TX_INVSINC"));
  }
  if (!res.rx.empty()) {
    std::vector<std::string> z, c;
    for (const auto& r : res.rx) {
      z.push_back("ch" + std::to_string(r.channel) + ":" + std::to_string(r.zone));
      c.push_back("ch" + std::to_string(r.channel) + ":cal=mode" + std::to_string(r.cal_mode));
    }
    dev.writeSetting("RFDC_RX_NYQUIST_ZONE", detail::join(z));
    logLine("RFDC_RX_NYQUIST_ZONE " + detail::join(z) + " -> " + dev.readSetting("RFDC_RX_NYQUIST_ZONE"));
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
  return res;
}

}  // namespace modev
}  // namespace houdini
