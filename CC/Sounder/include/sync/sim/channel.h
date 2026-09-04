/**
 * @file sync/sim/channel.h
 * @brief A synthetic link for the radio-free tests: place a beacon core in a
 *        window through a tapped channel with carrier offset, fractional
 *        timing and Gaussian noise, and hand back int16 samples the way the
 *        radio would.
 *
 * Extracted from beacon_geometry_test (architecture review 2026-09-03, item
 * 19) so the detector, the guard, the estimator and the policy all get sweep
 * tests against ONE channel model. Test-only and header-only: it links
 * nothing. The noise is drawn per sample in a fixed order (real then
 * imaginary) from a seeded generator, so a run is reproducible.
 *
 * Scale is by PEAK, because that is what the transmit path constrains: the
 * core goes out at a fixed fraction of full scale. Normalising by rms instead
 * would hand every candidate the same average power and hide the PAPR cost.
 */
#pragma once

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstdint>
#include <random>
#include <utility>
#include <vector>

namespace houdini {
namespace sync {
namespace sim {

using cf = std::complex<float>;

/// Delay a waveform by a fractional sample with a windowed sinc (2R+1 taps).
/// Returns len+1 samples so the tail is kept. A bench link has an arbitrary
/// fractional timing; integer placement cannot show what a detector does
/// BETWEEN samples (DEMO_VERIFICATION 8.164).
inline std::vector<cf> fracDelay(const std::vector<cf>& x, double tau, int R = 16) {
  std::vector<cf> y(x.size() + 1, cf(0.f, 0.f));
  for (size_t k = 0; k < y.size(); ++k) {
    std::complex<double> acc(0.0, 0.0);
    for (int m = static_cast<int>(k) - R; m <= static_cast<int>(k) + R; ++m) {
      if (m < 0 || m >= static_cast<int>(x.size())) continue;
      const double t = static_cast<double>(k) - static_cast<double>(m) - tau;
      const double sinc = (std::fabs(t) < 1e-9) ? 1.0 : std::sin(M_PI * t) / (M_PI * t);
      const double w = 0.5 + 0.5 * std::cos(M_PI * t / (R + 1));  // Hann
      acc += std::complex<double>(x[m].real(), x[m].imag()) * (sinc * w);
    }
    y[k] = cf(static_cast<float>(acc.real()), static_cast<float>(acc.imag()));
  }
  return y;
}

/// One path: an integer delay in samples and a complex gain.
struct Tap {
  long long delay = 0;
  std::complex<double> gain{1.0, 0.0};
};

struct Channel {
  std::vector<Tap> taps{Tap{}};  ///< a single direct path by default
  double cfo_hz = 0.0;           ///< carrier offset applied along the core
  double rate_hz = 122.88e6;     ///< the rate the offset is applied at
  double snr_db = 45.0;          ///< noise relative to the core's mean power
  double peak_counts = 3200.0;   ///< the core's peak in int16 counts
  double frac_delay = 0.0;       ///< fractional timing, samples (0 = none)

  /// The received window: `n` samples starting `s0` samples into an absolute
  /// timeline where the core's first sample sits at `pos`. Each tap adds the
  /// core at pos + delay, rotated by the carrier offset; noise is drawn from
  /// `seed`. Saturated at +-32000 as the radio's ADC would.
  std::vector<std::complex<int16_t>> receive(const std::vector<cf>& core_in, long long pos,
                                             long long s0, long long n, unsigned seed) const {
    const std::vector<cf> core = frac_delay != 0.0 ? fracDelay(core_in, frac_delay) : core_in;
    const long long clen = static_cast<long long>(core.size());
    std::mt19937 g(seed);
    auto u01 = [&g]() { return (static_cast<double>(g()) + 0.5) / 4294967296.0; };
    auto gauss = [&]() {
      return std::sqrt(-2.0 * std::log(u01())) * std::cos(2.0 * M_PI * u01());
    };
    double mean_p = 0.0, peak_p = 0.0;
    for (const auto& v : core_in) {
      mean_p += std::norm(v);
      peak_p = std::max(peak_p, static_cast<double>(std::norm(v)));
    }
    mean_p /= static_cast<double>(std::max<size_t>(1, core_in.size()));
    const double scale = peak_counts / std::sqrt(peak_p);
    const double ns = std::sqrt((mean_p / std::pow(10.0, snr_db / 10.0)) / 2.0);

    std::vector<std::complex<double>> sig(static_cast<size_t>(n), {0.0, 0.0});
    for (const auto& tp : taps) {
      for (long long k = 0; k < clen; ++k) {
        const long long a = pos + tp.delay + k - s0;
        if (a < 0 || a >= n) continue;
        const double ph = 2.0 * M_PI * cfo_hz * static_cast<double>(k) / rate_hz;
        sig[static_cast<size_t>(a)] +=
            tp.gain * std::complex<double>(core[k].real(), core[k].imag()) *
            std::exp(std::complex<double>(0.0, ph));
      }
    }
    std::vector<std::complex<int16_t>> buf(static_cast<size_t>(n));
    for (long long i = 0; i < n; ++i) {
      const double re = sig[i].real() * scale + gauss() * ns * scale;
      const double im = sig[i].imag() * scale + gauss() * ns * scale;
      buf[i] = std::complex<int16_t>(
          static_cast<int16_t>(std::max(-32000.0, std::min(32000.0, re))),
          static_cast<int16_t>(std::max(-32000.0, std::min(32000.0, im))));
    }
    return buf;
  }
};

}  // namespace sim
}  // namespace sync
}  // namespace houdini
