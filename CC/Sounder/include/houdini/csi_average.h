/**
 * @file houdini/csi_average.h
 * @brief The pilot slot's per-symbol channel estimates averaged into one H,
 *        with each symbol's common rotation removed first (AP-79 #5).
 *
 * The pilot repeats one symbol, so the per-symbol estimates G_s = F_s x
 * conj(ref) differ only by noise and by a COMMON rotation: the residual carrier
 * offset turns every tone by 2 pi f T_sym per symbol. At fft 64 or 256 that is
 * a fraction of a degree. At R3's 30 kHz spacing a 0.1 ppm residual at 2425 MHz
 * is ~240 Hz, about 3 degrees a symbol and 43 across fourteen, and a plain
 * average of rotated copies shrinks |H| (by sin(N x/2) / (N sin(x/2))) and
 * blurs it. Each symbol's rotation is measured against the one before it (so
 * up to +-180 degrees a symbol is tracked), removed, and the MEAN rotation
 * restored, so H's phase sits where the plain average put it and nothing
 * downstream sees a change on a stationary link.
 */
#pragma once

#include <cmath>
#include <complex>
#include <vector>

namespace houdini {
namespace csi {

using cf = std::complex<float>;

/// The average of the rows of G (one per symbol, each N tones), each row
/// de-rotated by its common phase relative to the first. Returns the plain
/// average when `derotate` is false. `rot_rad`, when given, receives the
/// rotation measured for each row (radians, unwrapped, the first row 0).
inline std::vector<cf> derotatedAverage(const std::vector<std::vector<cf>>& G, bool derotate = true,
                                        std::vector<double>* rot_rad = nullptr) {
  std::vector<cf> acc;
  if (G.empty()) return acc;
  const size_t n = G.front().size();
  acc.assign(n, cf(0.0f, 0.0f));
  std::vector<double> th(G.size(), 0.0);
  for (size_t s = 1; derotate && s < G.size(); ++s) {
    std::complex<double> c(0.0, 0.0);
    for (size_t k = 0; k < n; ++k)
      c += std::complex<double>(G[s][k].real(), G[s][k].imag()) *
           std::conj(std::complex<double>(G[s - 1][k].real(), G[s - 1][k].imag()));
    th[s] = th[s - 1] + (std::abs(c) > 0.0 ? std::arg(c) : 0.0);
  }
  double mean = 0.0;
  for (double t : th) mean += t;
  mean /= static_cast<double>(th.size());
  for (size_t s = 0; s < G.size(); ++s) {
    const std::complex<double> r = std::polar(1.0, mean - th[s]);
    const cf rf(static_cast<float>(r.real()), static_cast<float>(r.imag()));
    for (size_t k = 0; k < n; ++k) acc[k] += G[s][k] * rf;
  }
  const float inv = 1.0f / static_cast<float>(G.size());
  for (auto& v : acc) v *= inv;
  if (rot_rad != nullptr) *rot_rad = th;
  return acc;
}

}  // namespace csi
}  // namespace houdini
