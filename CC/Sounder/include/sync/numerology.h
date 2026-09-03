/**
 * @file sync/numerology.h
 * @brief The sample-rate facts a shape, a geometry or a knob may depend on,
 *        in one struct, so a rate literal never appears in a shape builder or
 *        a test again.
 *
 * WHAT SCALES AND WHAT DOES NOT. Two kinds of quantity live side by side in
 * this library and the rule for each is stated here once:
 *
 *   - TIME quantities scale with the rate: the tracking and acquisition
 *     tolerances (`resync.scatter_tol_us`, `confirm_tol_us`), the resync
 *     cadence, the OFDM prefix as a duration. samplesFor() converts them.
 *   - CORRELATOR quantities are SAMPLE counts by nature and do not scale: a
 *     replica's taps, the correlator run-up (two replicas), the first-path
 *     back window (half a replica), the estimator's edge guard
 *     (`cfo.index_guard`, `cfo.window_margin`) and the SNR guard floor of 8.
 *     They are properties of the discrete sequence, whatever the rate.
 *   - SEQUENCE-defined shapes (legacy, legacy_guard, dot11) are fixed sample
 *     counts: an STS is 16 samples and a gold symbol 128 at any rate, so
 *     their bandwidth follows the rate. STANDARD-defined shapes (nr, nr_pss)
 *     hold their subcarrier spacing: the PSS is 127 tones at `scs_hz`, so its
 *     IFFT size is rate / scs. A rate that cannot carry 127 tones at that
 *     spacing (no whole power-of-two size) builds the shipped 128-point
 *     symbols instead and says so (Desc::numerology_held), because refusing
 *     to build would abort every Iris config that names an NR shape.
 */
#pragma once

#include <cmath>
#include <cstddef>
#include <optional>
#include <stdexcept>
#include <string>

namespace houdini {
namespace sync {

struct Numerology {
  double rate_hz = 122.88e6;      ///< sample rate
  size_t samps_per_slot = 4096;   ///< the TDD slot, samples
  size_t samps_per_frame = 122880;///< the frame, samples
  size_t prefix_samples = 160;    ///< OFDM zero prefix, samples
  double scs_hz = 960e3;          ///< subcarrier spacing the NR shapes hold (mu = 6)

  /// The shipped Houdini numerology: 122.88 MSPS, 4096-sample slots, 30
  /// slots a frame, a 160-sample prefix, 960 kHz spacing (4096 x 30 kHz =
  /// 122.88 MHz; 128-point PSS symbols).
  static constexpr Numerology houdiniDefault() { return Numerology{}; }

  double samplesFor(double seconds) const { return seconds * rate_hz; }
  double secondsFor(double samples) const { return samples / rate_hz; }
  double slotSeconds() const { return secondsFor(static_cast<double>(samps_per_slot)); }
  double frameSeconds() const { return secondsFor(static_cast<double>(samps_per_frame)); }

  /// The IFFT size that carries `scs_hz` at this rate, when one exists: a
  /// whole number, a power of two (the FFT library is radix 2/4/8 and does
  /// not check its plan), and at least `min_tones + 1` points, because the
  /// tone mapper nulls DC so an n-point symbol carries n - 1 tones. Empty
  /// otherwise, so a shape can fall back rather than refuse to build.
  std::optional<size_t> ifftSizeIfExact(size_t min_tones) const {
    if (!(rate_hz > 0.0) || !(scs_hz > 0.0)) return std::nullopt;
    const double n = rate_hz / scs_hz;
    const double r = std::round(n);
    if (std::fabs(n - r) > 1e-6 || r < static_cast<double>(min_tones) + 1.0) return std::nullopt;
    const size_t k = static_cast<size_t>(r);
    if ((k & (k - 1)) != 0) return std::nullopt;
    return k;
  }
  /// The same, throwing with the reason when no such size exists.
  size_t ifftSize(size_t min_tones) const {
    const auto k = ifftSizeIfExact(min_tones);
    if (!k.has_value()) {
      throw std::invalid_argument(
          "numerology: " + std::to_string(rate_hz) + " Hz at " + std::to_string(scs_hz) +
          " Hz spacing gives " + std::to_string(rate_hz / scs_hz) +
          " points; a power of two of at least " + std::to_string(min_tones + 1) + " is needed");
    }
    return *k;
  }
};

}  // namespace sync
}  // namespace houdini
