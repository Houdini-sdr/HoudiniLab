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

extern "C" {
#include "fft.h"  // muFFT
}

namespace houdini {

class CirFromH {
 public:
  explicit CirFromH(int n) : n_(n) {
    if (n <= 0 || (n & (n - 1)) != 0) throw std::invalid_argument("CirFromH: n must be a power of two");
    in_ = static_cast<std::complex<float>*>(mufft_alloc(static_cast<size_t>(n) * sizeof(std::complex<float>)));
    out_ = static_cast<std::complex<float>*>(mufft_alloc(static_cast<size_t>(n) * sizeof(std::complex<float>)));
    plan_ = mufft_create_plan_1d_c2c(static_cast<unsigned>(n), MUFFT_INVERSE, MUFFT_FLAG_CPU_ANY);
    if (in_ == nullptr || out_ == nullptr || plan_ == nullptr) throw std::runtime_error("CirFromH: muFFT allocation failed");
  }
  ~CirFromH() {
    mufft_free_plan_1d(plan_);
    mufft_free(in_);
    mufft_free(out_);
  }
  CirFromH(const CirFromH&) = delete;
  CirFromH& operator=(const CirFromH&) = delete;

  /// Power per tap, |h[t]|^2, t = 0..N-1, from the DC-centred H (index N/2 is DC).
  std::vector<float> power(const std::vector<std::complex<float>>& h_dc) {
    if (static_cast<int>(h_dc.size()) != n_) throw std::invalid_argument("CirFromH: H is not N points");
    for (int k = 0; k < n_; ++k) in_[k] = h_dc[static_cast<size_t>((k + n_ / 2) % n_)];  // natural order
    mufft_execute_plan_1d(plan_, out_, in_);
    std::vector<float> p(static_cast<size_t>(n_));
    for (int t = 0; t < n_; ++t) p[static_cast<size_t>(t)] = std::norm(out_[t]);
    return p;
  }

 private:
  int n_;
  std::complex<float>* in_ = nullptr;
  std::complex<float>* out_ = nullptr;
  mufft_plan_1d* plan_ = nullptr;
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
