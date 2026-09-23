/**
 * @file houdini/pilot_slope_fit.h
 * @brief The recorder's stage-2 timing fit over the U slot's pilot tones: a
 *        least-squares line through each tone's accumulated phase (AP-79).
 *
 * The phase of each tone is taken RELATIVE to the tones' common phase (the
 * argument of their unit-vector mean), and that common phase is added back to
 * the intercept. Taking arg() of each tone directly, as this fit first did,
 * wraps when the common phase sits near +-180 degrees: some tones read +179,
 * others -179, and the fitted slope (the timing correction) is garbage. At R3's
 * 30 kHz spacing the pilot-to-data phase from an uncorrected carrier offset is
 * 94-124 degrees and drifts through the wrap (a metric audit, 2026-09-23).
 */
#pragma once

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
  std::complex<double> m(0.0, 0.0);
  for (const auto& a : acc)
    if (std::abs(a) > 0.0) m += a / std::abs(a);
  if (std::abs(m) <= 0.0) return f;
  const double c0 = std::arg(m);
  double sk = 0, sp = 0, skk = 0, skp = 0;
  for (size_t i = 0; i < n; ++i) {
    const double ph = std::arg(acc[i] * std::polar(1.0, -c0));  // near 0, no wrap
    sk += kk[i];
    sp += ph;
    skk += kk[i] * kk[i];
    skp += kk[i] * ph;
  }
  const double denom = static_cast<double>(n) * skk - sk * sk;
  if (std::fabs(denom) <= 1e-9) return f;
  f.slope = (static_cast<double>(n) * skp - sk * sp) / denom;
  f.intercept = c0 + (sp - f.slope * sk) / static_cast<double>(n);
  f.ok = true;
  return f;
}

}  // namespace csi
}  // namespace houdini
