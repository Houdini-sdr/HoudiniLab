/**
 * @file houdini/pilot_slope_fit.h
 * @brief The recorder's stage-2 timing fit over the U slot's pilot tones: a
 *        least-squares line through each tone's accumulated phase (AP-79).
 *
 * The tones' phases are UNWRAPPED along frequency (each from its neighbour),
 * so the fit depends only on adjacent tones being within pi of each other.
 * Taking arg() of each tone directly, as this fit first did, wraps when the
 * common phase sits near +-180 degrees: some tones read +179, others -179, and
 * the fitted slope (the timing correction) is garbage. (Referencing the tones
 * to their unit-vector mean instead, tried next, fixed that but shrank the
 * unambiguous range at fft 64: an Opus review measured a 1.2-sample residual
 * read as -0.63.) At R3's
 * 30 kHz spacing the pilot-to-data phase from an uncorrected carrier offset is
 * 94-124 degrees and drifts through the wrap (a metric audit, 2026-09-23).
 */
#pragma once

#include <algorithm>
#include <cmath>
#include <complex>
#include <vector>

namespace houdini {
namespace csi {

struct SlopeFit {
  bool ok = false;
  double slope = 0.0;      ///< radians per bin
  double intercept = 0.0;  ///< radians at bin offset 0, the common phase
};

/// kk[i]: tone i's offset from the band centre (bins); acc[i]: its accumulated
/// value against the known pilot. Needs at least two tones.
inline SlopeFit pilotSlopeFit(const std::vector<double>& kk, const std::vector<std::complex<double>>& acc) {
  SlopeFit f;
  const size_t n = kk.size();
  if (n < 2 || acc.size() != n) return f;
  // Unwrap along frequency: tones sorted by offset, each phase taken as the
  // previous one plus the angle between neighbours. Valid while ADJACENT tones
  // differ by under pi (the slope times the tone spacing), independent of the
  // common phase: +-2.3 samples at fft 64 with 14-bin pilots, far more at R3.
  std::vector<size_t> ord(n);
  for (size_t i = 0; i < n; ++i) ord[i] = i;
  std::sort(ord.begin(), ord.end(), [&](size_t a, size_t b) { return kk[a] < kk[b]; });
  std::vector<double> ph(n);
  ph[ord[0]] = std::arg(acc[ord[0]]);
  for (size_t q = 1; q < n; ++q)
    ph[ord[q]] = ph[ord[q - 1]] + std::arg(acc[ord[q]] * std::conj(acc[ord[q - 1]]));
  double sk = 0, sp = 0, skk = 0, skp = 0;
  for (size_t i = 0; i < n; ++i) {
    sk += kk[i];
    sp += ph[i];
    skk += kk[i] * kk[i];
    skp += kk[i] * ph[i];
  }
  const double denom = static_cast<double>(n) * skk - sk * sk;
  if (std::fabs(denom) <= 1e-9) return f;
  f.slope = (static_cast<double>(n) * skp - sk * sp) / denom;
  f.intercept = std::remainder((sp - f.slope * sk) / static_cast<double>(n), 2.0 * M_PI);
  f.ok = true;
  return f;
}

}  // namespace csi
}  // namespace houdini
