/**
 * @file houdini/rf_plan.h
 * @brief Per-channel RF-DC settings DERIVED from a channel's NCO and the
 *        converter rates (AP-79): the Nyquist zone of each direction, the ADC
 *        calibration mode, and whether the RX lane needs the +-25 MHz channel
 *        filter. Pure arithmetic, no radio.
 *
 * WHY DERIVED, NOT CONFIGURED. [user]: keep the config common and split only
 * what must differ. Across the two bands only the NCO must differ (2425 sub-6,
 * 4380 X-band IF); everything else below follows from it and the converter
 * rates by PG269's rules, so writing them into the config would be four more
 * per-channel values that can disagree with the one that decides them. The
 * sounder logs what it derived and the preflight snapshot shows what the
 * device holds, so a disagreement is visible, not silent.
 *
 * THE RULES, each with its source (HS-202 plan sections 2 and 3):
 *   - RX zone: the channel [nco - hb, nco + hb] lies inside one Nyquist zone
 *     of the ADC; the device firmware accepts zones 1 and 2 only. A channel
 *     that straddles a zone edge is refused (PG269 p.75).
 *   - ADC calibration mode by where the channel ALIASES: Mode 2 for 0 to
 *     0.4 Fs, Mode 1 for 0.4 Fs to Fs/2 (PG269 p.88). A channel whose alias
 *     straddles 0.4 Fs is refused rather than guessed.
 *   - RX channel filter: a real input at f is sampled with its mirror about
 *     the nearest zone edge, and when that mirror lands inside the RX output
 *     (+-rx_rate/2) it arrives at 0 dBc beside the channel (W2 measured it
 *     30/30). Then the lane needs the channel filter, and the filter's fixed
 *     design (pass +-24, stop from +-40.2 MHz, dsp/band_filters.h) must be
 *     able to separate them, or the placement is refused.
 *   - TX zone from the DAC's usable bands (PG269 p.130 Table 63): NRZ zone 1
 *     is 0 to 0.45 Fs, mix-mode zone 2 is 0.55 to 0.95 Fs; the channel must
 *     sit inside one of them. The inverse sinc then follows the zone
 *     (`RFDC_TX_INVSINC ch<n>:zone`, set after the zone, PG269 p.115).
 *
 * Rules carries every threshold so the test can build a mutation of each one
 * and require the plan assertion that names it to fail.
 *
 * RENEW OPEN SOURCE LICENSE: http://renew-wireless.org/license
 */
#pragma once

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <stdexcept>
#include <string>

namespace houdini {
namespace rfplan {

struct ConverterPoint {
  double adc_fs_hz = 0.0;   ///< RF-ADC sample rate
  double dac_fs_hz = 0.0;   ///< RF-DAC sample rate
  double rx_rate_hz = 0.0;  ///< RX stream rate (the decimated output)
  double tx_rate_hz = 0.0;  ///< TX stream rate
};

struct Rules {
  double cal_split_frac = 0.4;        ///< PG269 p.88: Mode 2 below, Mode 1 above
  double nrz_max_frac = 0.45;         ///< PG269 p.130 Table 63: NRZ usable to 0.45 Fs
  double mix_min_frac = 0.55;         ///< mix-mode usable 0.55 ..
  double mix_max_frac = 0.95;         ///< .. 0.95 Fs
  double filter_pass_hz = 24.0e6;     ///< dsp::ChannelFilter passband edge
  double filter_stop_hz = 40.2e6;     ///< dsp::ChannelFilter stopband edge
  int max_rx_zone = 2;                ///< the device firmware accepts zones 1 and 2
};

struct RxPlan {
  int zone = 0;
  int cal_mode = 0;                   ///< 1 or 2 (RFDC_ADC_CAL ch<n>:cal=mode<k>)
  bool channel_filter = false;        ///< this lane needs the +-25 MHz filter
  double mirror_offset_hz = std::numeric_limits<double>::infinity();  ///< nearest mirror, from the NCO
};

struct TxPlan {
  int zone = 0;                       ///< 1 = NRZ, 2 = mix-mode; inverse sinc follows it
};

namespace detail {
inline std::string mhz(double hz) {
  char b[32];
  std::snprintf(b, sizeof b, "%.3f MHz", hz / 1e6);
  return b;
}
/// Where a real line at f lands in [0, fs/2] once sampled at fs.
inline double fold(double f, double fs) {
  const double r = std::fmod(f, fs);
  return r > fs / 2.0 ? fs - r : r;
}
/// Smallest |x| over [a, b] (0 when the interval contains 0).
inline double nearestToZero(double a, double b) {
  if (a > b) std::swap(a, b);
  if (a <= 0.0 && b >= 0.0) return 0.0;
  return std::min(std::fabs(a), std::fabs(b));
}
}  // namespace detail

/// The RX side of one channel whose content occupies nco +- half_bw_hz.
inline RxPlan planRx(double nco_hz, double half_bw_hz, const ConverterPoint& c,
                     const Rules& r = Rules{}) {
  using detail::mhz;
  if (!(c.adc_fs_hz > 0.0) || !(c.rx_rate_hz > 0.0)) throw std::invalid_argument("planRx: converter point not set");
  if (!(nco_hz > 0.0) || !(half_bw_hz > 0.0)) throw std::invalid_argument("planRx: nco and half bandwidth must be positive");
  const double nyq = c.adc_fs_hz / 2.0;
  const double lo = nco_hz - half_bw_hz, hi = nco_hz + half_bw_hz;
  RxPlan p;
  p.zone = static_cast<int>(std::floor(nco_hz / nyq)) + 1;
  if (std::floor(lo / nyq) != std::floor(hi / nyq)) {
    throw std::invalid_argument("planRx: channel " + mhz(lo) + " to " + mhz(hi) +
                                " straddles an ADC Nyquist edge (multiple of " + mhz(nyq) + ")");
  }
  if (p.zone < 1 || p.zone > r.max_rx_zone) {
    throw std::invalid_argument("planRx: channel at " + mhz(nco_hz) + " is in ADC zone " +
                                std::to_string(p.zone) + "; the device accepts zones 1 to " +
                                std::to_string(r.max_rx_zone));
  }
  // Calibration mode by the alias position of the whole channel.
  const double a_lo = detail::fold(lo, c.adc_fs_hz) / c.adc_fs_hz;
  const double a_hi = detail::fold(hi, c.adc_fs_hz) / c.adc_fs_hz;
  const bool below = std::max(a_lo, a_hi) < r.cal_split_frac;
  const bool above = std::min(a_lo, a_hi) >= r.cal_split_frac;
  if (!below && !above) {
    throw std::invalid_argument("planRx: channel at " + mhz(nco_hz) + " aliases across " +
                                std::to_string(r.cal_split_frac) + " Fs, where the ADC calibration mode changes");
  }
  p.cal_mode = below ? 2 : 1;
  // The channel's mirror about each edge of its zone, as offsets from the NCO.
  for (const double edge : {(p.zone - 1) * nyq, p.zone * nyq}) {
    const double near = detail::nearestToZero(2.0 * edge - hi - nco_hz, 2.0 * edge - lo - nco_hz);
    p.mirror_offset_hz = std::min(p.mirror_offset_hz, near);
  }
  if (p.mirror_offset_hz < c.rx_rate_hz / 2.0) {
    p.channel_filter = true;
    if (half_bw_hz > r.filter_pass_hz) {
      throw std::invalid_argument("planRx: channel half width " + mhz(half_bw_hz) +
                                  " exceeds the channel filter passband " + mhz(r.filter_pass_hz));
    }
    if (p.mirror_offset_hz < r.filter_stop_hz) {
      throw std::invalid_argument("planRx: the channel's mirror starts " + mhz(p.mirror_offset_hz) +
                                  " from the NCO, inside the channel filter's stop edge " +
                                  mhz(r.filter_stop_hz) + "; move the NCO away from the zone edge");
    }
  }
  return p;
}

/// The TX side of one channel whose content occupies nco +- half_bw_hz.
inline TxPlan planTx(double nco_hz, double half_bw_hz, const ConverterPoint& c,
                     const Rules& r = Rules{}) {
  using detail::mhz;
  if (!(c.dac_fs_hz > 0.0)) throw std::invalid_argument("planTx: converter point not set");
  if (!(nco_hz > 0.0) || !(half_bw_hz > 0.0)) throw std::invalid_argument("planTx: nco and half bandwidth must be positive");
  const double lo = (nco_hz - half_bw_hz) / c.dac_fs_hz, hi = (nco_hz + half_bw_hz) / c.dac_fs_hz;
  TxPlan p;
  if (lo >= 0.0 && hi <= r.nrz_max_frac) {
    p.zone = 1;
  } else if (lo >= r.mix_min_frac && hi <= r.mix_max_frac) {
    p.zone = 2;
  } else {
    throw std::invalid_argument("planTx: channel " + mhz(nco_hz - half_bw_hz) + " to " +
                                mhz(nco_hz + half_bw_hz) + " is outside the DAC's usable bands (NRZ 0 to " +
                                mhz(r.nrz_max_frac * c.dac_fs_hz) + ", mix-mode " +
                                mhz(r.mix_min_frac * c.dac_fs_hz) + " to " + mhz(r.mix_max_frac * c.dac_fs_hz) + ")");
  }
  return p;
}

}  // namespace rfplan
}  // namespace houdini
