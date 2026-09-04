/**
 * @file sync/cfo_estimator.h
 * @brief The beacon's own carrier-offset estimate from its repeated field.
 *
 * Receiver::estimateCFO, moved. Two repetition correlations on the shape's
 * geometry: the fine field (rep2 against rep1 at lag fine_len) gives the
 * estimate, the coarse field (consecutive short symbols) unwraps it when the
 * shape has one. Both are the plain repetition-phase estimator: for
 * x[n] = s[n] exp(j 2 pi f n) with s[n+N] = s[n], sum conj(x[n]) x[n+N] has
 * argument 2 pi f N.
 *
 * WHAT THE WINDOWS MUST NOT TOUCH. DEMO_VERIFICATION 8.164: on a real link the
 * beacon sits between samples, so the samples after its last repetition are
 * the interpolation tail of the edge, not zero, and a window that reaches them
 * multiplies real samples by that tail -- up to 4.9 kHz of bias for a
 * 64-sample field. `margin` shrinks both windows away from both edges; the
 * durable fix is a cyclic postfix in the waveform (AP-69). `guard` is the
 * older +8 slide of AP-39, kept so the shipped behaviour is reproducible.
 *
 * EVERY FAILURE PATH RETURNS NaN, NOT ZERO. A failed estimate is not a
 * measurement of zero offset, and the SYN1 wire drops a NaN rather than
 * averaging a fabricated 0 Hz into the panel.
 */
#pragma once

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <limits>

namespace houdini {
namespace sync {

/// The core layout the estimator reads, in samples from the core start.
struct FieldGeometry {
  int core_len = 0;
  int fine_off = 0, fine_len = 0, fine_reps = 0;
  int coarse_off = 0, coarse_len = 0, coarse_reps = 0;
  bool usable() const {
    return fine_len > 0 && fine_reps >= 2 &&
           fine_off + fine_len * fine_reps <= core_len;
  }
};

class RepetitionPhaseEstimator {
 public:
  /// @param g             the shape's fields (Config::beacon_* accessors)
  /// @param margin        samples shrunk from both ends of each window
  /// @param conjugated    true when the receive path delivers baseband
  ///                      conjugated (Houdini's matched-NCO R2C mixer), so a
  ///                      +f offset reads as -f and the sign is undone here
  RepetitionPhaseEstimator(const FieldGeometry& g, int margin, bool conjugated)
      : g_(g), margin_(margin), conj_(conjugated) {}

  /// Normalised offset in cycles per sample (multiply by the rate for Hz), or
  /// NaN. `end_index` is the beacon END in `buf`, so the core occupies
  /// [end_index - core_len, end_index).
  float estimate(const std::complex<int16_t>* buf, size_t buf_len,
                 int end_index) const {
    const float kNoEstimate = std::numeric_limits<float>::quiet_NaN();
    if (buf == nullptr || !g_.usable()) return kNoEstimate;
    const int start = end_index - g_.core_len;
    if (start < 0 || end_index < 0 || static_cast<size_t>(end_index) > buf_len)
      return kNoEstimate;
    const int m = std::max(0, std::min(margin_, g_.fine_len / 2 - 1));
    auto at = [buf](int i) {
      return std::complex<double>(static_cast<double>(buf[i].real()),
                                  static_cast<double>(buf[i].imag()));
    };
    const int g1 = start + g_.fine_off + m;
    const int g2 = g1 + g_.fine_len;
    std::complex<double> r_fine(0.0, 0.0);
    for (int i = 0; i < g_.fine_len - 2 * m; ++i)
      r_fine += std::conj(at(g1 + i)) * at(g2 + i);
    if (std::abs(r_fine) == 0.0) return kNoEstimate;
    const double f_fine = std::arg(r_fine) / (2.0 * M_PI * g_.fine_len);
    double f = f_fine;
    // The coarse field unwraps the fine one. A shape without one is not a
    // defect: the fine stage alone is unambiguous to +-rate/(2 fine_len),
    // 480 kHz at 128, against a link offset of at most a few kHz.
    if (g_.coarse_reps >= 2 && g_.coarse_len > 0 &&
        g_.coarse_off + g_.coarse_len * g_.coarse_reps <= g_.core_len) {
      std::complex<double> r_coarse(0.0, 0.0);
      for (int k = 0; k + 1 < g_.coarse_reps; ++k) {
        for (int i = 0; i < g_.coarse_len; ++i) {
          r_coarse += std::conj(at(start + g_.coarse_off + k * g_.coarse_len + i)) *
                      at(start + g_.coarse_off + (k + 1) * g_.coarse_len + i);
        }
      }
      if (std::abs(r_coarse) == 0.0) return kNoEstimate;
      const double f_coarse = std::arg(r_coarse) / (2.0 * M_PI * g_.coarse_len);
      const double ambiguity = 1.0 / g_.fine_len;
      f = f_fine + std::round((f_coarse - f_fine) / ambiguity) * ambiguity;
    }
    if (conj_) f = -f;
    return static_cast<float>(f);
  }

  int margin() const { return margin_; }
  const FieldGeometry& geometry() const { return g_; }

 private:
  FieldGeometry g_;
  int margin_;
  bool conj_;
};

}  // namespace sync
}  // namespace houdini
