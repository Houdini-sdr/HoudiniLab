/**
 * @file houdini/pre_cfo.h
 * @brief The carrier offset from a slot of identical symbols, removed in the
 *        time domain before the FFT (a weakly steered UE clock otherwise drags
 *        the wired X-band's MER down).
 *
 * Taking the carrier offset out per symbol after the FFT is not enough: that
 * removes the common phase but not the leakage between subcarriers the offset
 * causes, so a residual offset still caps MER (DEMO_VERIFICATION 9.13: 691 Hz
 * held it near 25 dB), and the X-band, at 1.81x the sub-6's offset in Hz,
 * follows every swing of the UE's steering. The pilot slot is identical
 * symbols, so the phase from each to the next (one symbol, cp + fft samples)
 * is the offset, measured on that lane's own pilot; rotating it out of the
 * samples before the FFT removes the leakage too.
 */
#pragma once

#include <algorithm>
#include <cmath>
#include <complex>

namespace houdini {
namespace precfo {

struct Est {
  double hz = 0.0;         ///< the carrier offset, Hz (unambiguous to rate / (2 * L))
  double coherence = 0.0;  ///< |sum x[i+L] x*[i]| / power: 1 for clean repeats, about 0 for noise
};

/// The offset of `count` identical symbols of L samples starting at `start` in
/// the int16 IQ slot `d` of n samples, at `rate`.
inline Est estimate(const short* d, int n, int start, int L, int count, double rate) {
  std::complex<double> acc(0.0, 0.0);
  double pw = 0.0;
  const int end = std::min(n - L, std::max(0, start) + (count - 1) * L);
  for (int i = std::max(0, start); i < end; ++i) {
    const std::complex<double> a(d[2 * i], d[2 * i + 1]), b(d[2 * (i + L)], d[2 * (i + L) + 1]);
    acc += b * std::conj(a);
    pw += 0.5 * (std::norm(a) + std::norm(b));
  }
  Est e;
  if (pw > 0.0) {
    e.coherence = std::abs(acc) / pw;
    e.hz = std::arg(acc) * rate / (2.0 * M_PI * L);
  }
  return e;
}

/// Rotate the offset out: out[i] = in[i] * exp(-j 2 pi hz (t0 + i) / rate), for
/// n int16 IQ samples, t0 the first sample's index from the reference (the pilot
/// slot's start), so a later slot of the same frame continues its phase.
/// Returns how many values (I or Q) saturated: a rotation keeps |x|, so a sample above
/// 32767 in magnitude (both rails large, the ADC within 3 dB of full scale) can
/// exceed the int16 range once rotated even where neither rail clipped before.
inline long long derotate(const short* in, short* out, int n, double hz, double rate, long long t0) {
  const double w = -2.0 * M_PI * hz / rate;
  long long saturated = 0;
  auto clamp16 = [&saturated](double v) {
    const double r = std::round(v);
    if (r > 32767.0 || r < -32768.0) {
      ++saturated;
      return static_cast<short>(r > 0 ? 32767 : -32768);
    }
    return static_cast<short>(r);
  };
  std::complex<double> ph, step(std::cos(w), std::sin(w));
  for (int i = 0; i < n; ++i) {
    if ((i & 1023) == 0) {  // re-seed the phasor every 1024 samples (no drift)
      const double a = w * static_cast<double>(t0 + i);
      ph = std::complex<double>(std::cos(a), std::sin(a));
    }
    const std::complex<double> x(in[2 * i], in[2 * i + 1]), y = x * ph;
    out[2 * i] = clamp16(y.real());
    out[2 * i + 1] = clamp16(y.imag());
    ph *= step;
  }
  return saturated;
}

}  // namespace precfo
}  // namespace houdini
