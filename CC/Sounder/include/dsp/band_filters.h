/**
 * @file dsp/band_filters.h
 * @brief The two fixed filters the dual-band mode-V point needs (AP-79): the
 *        x2 TX interpolator (122.88 -> 245.76 MSPS) and the sub-6 RX channel
 *        filter (+-25 MHz of the 122.88 output).
 *
 * WHY THEY EXIST. Mode V runs the DAC at 5898.24 with interpolation 24, so the
 * TX stream is 245.76 MSPS while the RX stream, and the device's timestamp
 * tick, stay at 122.88. The sounder keeps generating every TX waveform at
 * 122.88, in ticks, exactly as the config schedule defines it [user], and this
 * interpolator doubles it at the radio TX boundary. The RFDC's own
 * interpolation passband reaches about 0.4 x 245.76 = 98 MHz and there is no
 * analog filter after the DAC, so the halfband's images (97.88 to 147.88 MHz
 * from the centre for a +-25 MHz waveform) must be removed HERE: the software
 * lane asks for at least 60 dB.
 *
 * On the RX side, at NCO 2425 a full-strength (0 dBc) copy of the sub-6
 * channel's upper content aliases to 40.2 to 61.4 MHz from the NCO, inside the
 * 122.88 output (HS-202 plan section 3; measured 30/30 on the prediction). The
 * application therefore uses +-25 MHz and never the full output. The OFDM FFT
 * separates that alias by orthogonality, but the beacon correlator, the CFO
 * estimator and the energy search run in the time domain and do not, so the
 * sub-6 lanes are filtered before any of them. The X-band IF lanes need no
 * digital filter: their output straddles no Nyquist boundary.
 *
 * ZERO PHASE, BY CONSTRUCTION. Both filters are odd-length and linear-phase and
 * are applied centred over a whole buffer, so they add NO delay: an
 * interpolated sample 2k is input sample k exactly, and a filtered sample k is
 * centred on input sample k. A causal filter would add a fixed offset that
 * BS and UE would have to carry identically in their timing calibration
 * (software lane); centring removes the offset rather than calibrating it.
 * The price is the edge contract: samples outside the buffer are taken as
 * ZERO, so the buffer must carry at least contextBefore()/contextAfter()
 * samples of zeros, or real neighbouring content, at each end. The TDD slots
 * carry 128-tick zero prefixes and postfixes, far more than either needs.
 * runCircular() is the variant for a looped buffer (the beacon replay RAM).
 * Neither filter runs in place (the output overwrites input it still needs);
 * both refuse it.
 *
 * THE DESIGNS, and why these numbers. Kaiser-windowed sinc, the taps computed
 * here at construction so no coefficient table can drift from its stated
 * design. Chosen by a sweep for a 10 dB margin over the requirement:
 *   - halfband: 23 taps, beta 9. Images at least 91 dB down (need 60),
 *     passband ripple 0.0003 dB over +-25 MHz. 12 multiplies per odd output.
 *   - channel: 35 taps, beta 7, cutoff 31 MHz at 122.88. Stopband at least
 *     72 dB down from +-40.2 MHz (need 40; the alias arrives at 0 dBc),
 *     passband ripple 0.043 dB over +-24 MHz (96 subcarriers at 480 kHz
 *     occupy +-23.28 MHz).
 * tests/comms-func/band_filters_test.cc measures both on the implementation
 * (tones through run(), not a formula on the taps) and checks that each named
 * mutation of them fails.
 *
 * RENEW OPEN SOURCE LICENSE: http://renew-wireless.org/license
 */
#pragma once

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <vector>

namespace houdini {
namespace dsp {

/// Zeroth-order modified Bessel function of the first kind, by its series.
inline double besselI0(double x) {
  double sum = 1.0, term = 1.0;
  for (int k = 1; k < 200; ++k) {
    term *= (x / (2.0 * k)) * (x / (2.0 * k));
    sum += term;
    if (term < 1e-17 * sum) break;
  }
  return sum;
}

/// Odd-length Kaiser-windowed sinc lowpass, unity gain at DC. `cutoff` is the
/// -6 dB point as a fraction of the sample rate (0 to 0.5).
inline std::vector<double> kaiserLowpass(size_t taps, double cutoff, double beta) {
  if (taps < 3 || taps % 2 == 0) throw std::invalid_argument("kaiserLowpass: taps must be odd and >= 3");
  if (!(cutoff > 0.0 && cutoff < 0.5)) throw std::invalid_argument("kaiserLowpass: cutoff outside (0, 0.5)");
  const double c = static_cast<double>(taps - 1) / 2.0;
  const double i0b = besselI0(beta);
  std::vector<double> h(taps);
  double sum = 0.0;
  for (size_t n = 0; n < taps; ++n) {
    const double m = static_cast<double>(n) - c;
    const double x = 2.0 * cutoff * m;
    const double sinc = (m == 0.0) ? 1.0 : std::sin(M_PI * x) / (M_PI * x);
    const double r = m / c;
    const double w = besselI0(beta * std::sqrt(std::max(0.0, 1.0 - r * r))) / i0b;
    h[n] = 2.0 * cutoff * sinc * w;
    sum += h[n];
  }
  for (auto& v : h) v /= sum;
  return h;
}

/// x2 halfband interpolator, zero phase: out[2k] == in[k] exactly and
/// out[2k+1] is the band-limited midpoint. Output length is 2n.
class HalfbandInterp2 {
 public:
  static constexpr size_t kTaps = 23;   ///< 4m+3, so the outermost taps are non-zero
  static constexpr double kBeta = 9.0;

  explicit HalfbandInterp2(size_t taps = kTaps, double beta = kBeta) {
    if (taps % 4 != 3) throw std::invalid_argument("HalfbandInterp2: taps must be 4m+3");
    // A halfband is a lowpass at fs/4 of the OUTPUT rate. As an interpolator
    // it has gain 2 (half its input samples are the inserted zeros), which
    // makes the centre tap exactly 1 and every other even-offset tap exactly
    // 0: those are forced rather than left to rounding, so the passthrough
    // samples are bit-exact.
    full_ = kaiserLowpass(taps, 0.25, beta);
    const long c = static_cast<long>(taps - 1) / 2;
    for (size_t n = 0; n < taps; ++n) {
      const long off = static_cast<long>(n) - c;
      full_[n] = (off == 0) ? 1.0 : ((off % 2 == 0) ? 0.0 : 2.0 * full_[n]);
    }
    // Odd output 2k+1 = sum over odd offsets i = 2t+1 of h[c+i] * in[k-t],
    // t from -(c+1)/2 to (c-1)/2: in[k+(c+1)/2] .. in[k-(c-1)/2].
    const long half = (c + 1) / 2;
    for (long t = -half; t <= half - 1; ++t) {
      odd_.push_back(static_cast<float>(full_[static_cast<size_t>(c + 2 * t + 1)]));
      odd_off_.push_back(-t);  // input index offset from k
    }
    // Input p reaches odd outputs k = p - half .. p + half - 1, so content
    // needs `half` zeros before it and `half - 1` after it for every output
    // it touches to land inside the buffer.
    before_ = static_cast<size_t>(half);
    after_ = static_cast<size_t>(half - 1);
  }

  /// Input samples of zero margin the buffer needs before its first and after
  /// its last non-zero sample for run() to equal the filter on the unbounded
  /// signal.
  size_t contextBefore() const { return before_; }
  size_t contextAfter() const { return after_; }
  const std::vector<double>& taps() const { return full_; }

  /// Samples outside [0, n) are zero. `out` holds 2n samples.
  void run(const std::complex<float>* in, size_t n, std::complex<float>* out) const {
    if (static_cast<const void*>(in) == static_cast<const void*>(out)) throw std::invalid_argument("HalfbandInterp2: in-place is not supported");
    for (size_t k = 0; k < n; ++k) {
      out[2 * k] = in[k];
      std::complex<float> acc(0.0f, 0.0f);
      for (size_t j = 0; j < odd_.size(); ++j) {
        const long idx = static_cast<long>(k) + odd_off_[j];
        if (idx >= 0 && idx < static_cast<long>(n)) acc += odd_[j] * in[idx];
      }
      out[2 * k + 1] = acc;
    }
  }

  /// The buffer is one period of a looped signal (the replay RAM).
  void runCircular(const std::complex<float>* in, size_t n, std::complex<float>* out) const {
    if (n == 0) return;
    if (static_cast<const void*>(in) == static_cast<const void*>(out)) throw std::invalid_argument("HalfbandInterp2: in-place is not supported");
    const long ln = static_cast<long>(n);
    for (size_t k = 0; k < n; ++k) {
      out[2 * k] = in[k];
      std::complex<float> acc(0.0f, 0.0f);
      for (size_t j = 0; j < odd_.size(); ++j) {
        long idx = (static_cast<long>(k) + odd_off_[j]) % ln;
        if (idx < 0) idx += ln;
        acc += odd_[j] * in[idx];
      }
      out[2 * k + 1] = acc;
    }
  }

  /// The CS16 path the radio boundary uses: interpolate, then round to int16
  /// with saturation. Returns the number of I or Q components that had to be
  /// saturated; the caller treats any non-zero count as a level fault (the
  /// software lane: never let a sample reach full scale). Scale BEFORE this
  /// call with the halfband's overshoot in mind, or check `peak` after.
  size_t runCs16(const std::complex<int16_t>* in, size_t n, std::complex<int16_t>* out,
                 bool circular = false, float* peak = nullptr) const {
    std::vector<std::complex<float>> fin(n), fout(2 * n);
    for (size_t k = 0; k < n; ++k) fin[k] = {static_cast<float>(in[k].real()), static_cast<float>(in[k].imag())};
    if (circular) runCircular(fin.data(), n, fout.data());
    else run(fin.data(), n, fout.data());
    return quantize(fout.data(), 2 * n, out, peak);
  }

  static size_t quantize(const std::complex<float>* in, size_t n, std::complex<int16_t>* out,
                         float* peak = nullptr) {
    size_t sat = 0;
    float pk = 0.0f;
    auto q = [&sat](float v) -> int16_t {
      const float r = std::nearbyint(v);
      if (r > 32767.0f) { ++sat; return 32767; }
      if (r < -32767.0f) { ++sat; return -32767; }
      return static_cast<int16_t>(r);
    };
    for (size_t k = 0; k < n; ++k) {
      pk = std::max(pk, std::max(std::fabs(in[k].real()), std::fabs(in[k].imag())));
      out[k] = {q(in[k].real()), q(in[k].imag())};
    }
    if (peak != nullptr) *peak = pk;
    return sat;
  }

 private:
  std::vector<double> full_;
  std::vector<float> odd_;
  std::vector<long> odd_off_;
  size_t before_ = 0, after_ = 0;
};

/// The sub-6 RX channel filter at 122.88 MSPS, zero phase: out[k] is centred
/// on in[k]. Samples outside [0, n) are zero.
class ChannelFilter {
 public:
  static constexpr size_t kTaps = 35;
  static constexpr double kBeta = 7.0;
  static constexpr double kRateHz = 122.88e6;
  static constexpr double kCutoffHz = 31.0e6;

  explicit ChannelFilter(size_t taps = kTaps, double beta = kBeta, double cutoff_hz = kCutoffHz,
                         double rate_hz = kRateHz) {
    const auto h = kaiserLowpass(taps, cutoff_hz / rate_hz, beta);
    taps_.assign(h.begin(), h.end());
    half_ = (taps - 1) / 2;
  }

  size_t halfLength() const { return half_; }
  const std::vector<float>& taps() const { return taps_; }

  void run(const std::complex<float>* in, size_t n, std::complex<float>* out) const {
    runRange(in, n, 0, n, out);
  }

  /// Filter only outputs [start, start + count) of a signal `in` of `n`
  /// samples (zero outside it), written to out[0 .. count). The samples of
  /// `in` around the range are used as real context, so a slice cut from a
  /// longer capture is filtered exactly as the whole capture would be.
  ///
  /// SPEED (AP-79: the scalar form measured 94 to 138 ms per 10 ms of one
  /// lane). The taps are symmetric, so out[k] = h_c x[k] + sum_j h_{c-j}
  /// (x[k-j] + x[k+j]): half the multiplies. The taps are real, so the I and
  /// Q of the interleaved float array filter identically with a stride of 2,
  /// and the interior runs as `half` passes of a branch-free, contiguous loop
  /// the compiler vectorizes, in blocks that stay in cache. Only the few
  /// outputs within `half` of the signal's ends take the bounds-checked path.
  void runRange(const std::complex<float>* in, size_t n, size_t start, size_t count,
                std::complex<float>* out) const {
    if (static_cast<const void*>(in) == static_cast<const void*>(out)) throw std::invalid_argument("ChannelFilter: in-place is not supported");
    if (start > n || count > n - start) throw std::invalid_argument("ChannelFilter: range outside the signal");
    const long h = static_cast<long>(half_);
    const long ln = static_cast<long>(n);
    const long k0 = static_cast<long>(start), k1 = k0 + static_cast<long>(count);
    // Interior: outputs whose whole support lies inside the signal.
    const long i0 = std::max(k0, h), i1 = std::min(k1, ln - h);
    const float* x = reinterpret_cast<const float*>(in);
    float* y = reinterpret_cast<float*>(out);
    const float hc = taps_[static_cast<size_t>(h)];
    constexpr long kBlock = 2048;  // complex samples per block
    for (long b0 = i0; b0 < i1; b0 += kBlock) {
      const long b1 = std::min(i1, b0 + kBlock);
      float* yb = y + 2 * (b0 - k0);
      const float* xb = x + 2 * b0;
      const long m = 2 * (b1 - b0);
      for (long i = 0; i < m; ++i) yb[i] = hc * xb[i];
      for (long j = 1; j <= h; ++j) {
        const float hj = taps_[static_cast<size_t>(h - j)];
        const float* xl = xb - 2 * j;
        const float* xr = xb + 2 * j;
        for (long i = 0; i < m; ++i) yb[i] += hj * (xl[i] + xr[i]);
      }
    }
    // Edges: the bounds-checked form, for outputs whose support leaves the signal.
    auto edge = [&](long k) {
      std::complex<float> acc(0.0f, 0.0f);
      const long lo = std::max(0L, k - h), hi = std::min(ln - 1, k + h);
      for (long i = lo; i <= hi; ++i) acc += taps_[static_cast<size_t>(i - k + h)] * in[i];
      out[k - k0] = acc;
    };
    for (long k = k0; k < std::min(k1, i0); ++k) edge(k);
    for (long k = std::max(k0, i1); k < k1; ++k) edge(k);
    // (when the range is shorter than the support, i0 >= i1 and every output
    // is an edge; the two loops then cover [k0, k1) without overlap)
    if (i0 >= i1) {
      for (long k = std::max(k0, std::min(k1, i0)); k < std::min(k1, std::max(k0, i1)); ++k) edge(k);
    }
  }

 private:
  std::vector<float> taps_;
  size_t half_ = 0;
};

}  // namespace dsp
}  // namespace houdini
