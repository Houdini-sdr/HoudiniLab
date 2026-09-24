/**
 * @file houdini/cir.h
 * @brief The channel impulse response for the CSI view: the inverse FFT of the
 *        DC-centred channel estimate H, as power per delay tap (AP-79).
 *
 * One N-point inverse FFT per SENT CSI frame (the view's ~30 fps, not every
 * pilot): 0.17-0.42 ms at N 4096 on the rig host, about 1 % of a core per
 * antenna. The view gets a short window of taps around the strongest one, in
 * dB relative to it; the tap spacing is 1 / sample rate.
 *
 * H's phase carries the FFT window's back-off into the cyclic prefix (the
 * recorder reads the body early by design), so the whole CIR is circularly
 * shifted: the peak tap is found, not assumed, and the window wraps.
 */
#pragma once

#include <algorithm>
#include <cmath>
#include <complex>
#include <stdexcept>
#include <vector>

#include "houdini/mufft_c2c.h"

namespace houdini {

class CirFromH {
 public:
  explicit CirFromH(int n) : fft_(n, MUFFT_INVERSE, "CirFromH"), n_(n) {}

  /// Power per tap, |h[t]|^2, t = 0..N-1, from the DC-centred H (index N/2 is
  /// DC). H is Hann-windowed over its occupied span (the tones where it is
  /// non-zero) first: an unwindowed band edge makes a single path a sinc whose
  /// sidelobes (-13, -18, -21 dB ...) read as echoes and put a ~11 ns floor
  /// under the RMS delay spread; the Hann's first sidelobe is -31 dB, at the
  /// cost of a mainlobe twice as wide (sounder practice; ITU-R P.1407).
  std::vector<float> power(const std::vector<std::complex<float>>& h_dc) {
    if (static_cast<int>(h_dc.size()) != n_) throw std::invalid_argument("CirFromH: H is not N points");
    int lo = n_, hi = -1;
    for (int k = 0; k < n_; ++k)
      if (std::norm(h_dc[static_cast<size_t>(k)]) > 0.0f) {
        lo = std::min(lo, k);
        hi = std::max(hi, k);
      }
    const double span = static_cast<double>(hi - lo + 1);
    for (int k = 0; k < n_; ++k) {
      const int kc = (k + n_ / 2) % n_;  // natural bin k is DC-centred index kc
      const float w = (hi < lo) ? 0.0f
                                : (kc < lo || kc > hi)
                                      ? 0.0f
                                      : static_cast<float>(0.5 - 0.5 * std::cos(2.0 * M_PI * (kc - lo + 0.5) / span));
      fft_.in()[k] = h_dc[static_cast<size_t>(kc)] * w;
    }
    fft_.execute();
    std::vector<float> p(static_cast<size_t>(n_));
    for (int t = 0; t < n_; ++t) p[static_cast<size_t>(t)] = std::norm(fft_.out()[t]);
    return p;
  }

 private:
  MufftC2C fft_;
  int n_;
};

/// `len` taps starting `pre` taps before the strongest one (circularly), in dB
/// relative to that tap, floored at `floor_db`. `peak` receives its index.
inline std::vector<float> cirWindowDb(const std::vector<float>& p, int pre, int len, int* peak, float floor_db = -60.0f) {
  const int n = static_cast<int>(p.size());
  std::vector<float> out;
  if (n == 0 || len <= 0) return out;
  const int pk = static_cast<int>(std::max_element(p.begin(), p.end()) - p.begin());
  if (peak != nullptr) *peak = pk;
  const float ref = std::max(p[static_cast<size_t>(pk)], 1e-30f);
  out.reserve(static_cast<size_t>(len));
  for (int i = 0; i < len; ++i) {
    const int t = ((pk - pre + i) % n + n) % n;
    out.push_back(std::max(floor_db, 10.0f * std::log10(std::max(p[static_cast<size_t>(t)], 1e-30f) / ref)));
  }
  return out;
}

}  // namespace houdini
