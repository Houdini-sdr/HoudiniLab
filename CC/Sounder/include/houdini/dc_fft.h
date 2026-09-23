/**
 * @file houdini/dc_fft.h
 * @brief The CSI view's per-symbol spectrum as an FFT (AP-79): the same
 *        DC-centred forward transform the recorder built as an explicit N x N
 *        DFT matrix, in O(N log N) with a plan made once.
 *
 * WHY. RecorderWorker::symbolFft multiplied every symbol by an N x N matrix.
 * At the old fft 64 that was 4096 multiply-adds a symbol; at the 5G-like
 * fft 4096 it is 16.7 M a symbol and a 134 MB matrix, about 14 G multiply-adds
 * a second at the dashboard's 30 fps on two antennas, which the view cannot
 * keep up with.
 *
 * THE DEFINITION IT KEEPS, exactly: Xs[k] = sum_n x[n] exp(-j 2 pi m n / N)
 * with m = (k + N/2) mod N, i.e. the forward DFT rotated so DC sits at index
 * N/2, matching the DC-centred pilot reference; x is the CS16 slot sample
 * with its imaginary part negated when `conj` (the R2C mixer's inversion).
 * dc_fft_test checks it against that sum.
 *
 * RENEW OPEN SOURCE LICENSE: http://renew-wireless.org/license
 */
#pragma once

#include <complex>
#include <cstddef>
#include <stdexcept>
#include <vector>

extern "C" {
#include "fft.h"  // muFFT
}

namespace houdini {

class DcCenteredFft {
 public:
  explicit DcCenteredFft(int n) : n_(n) {
    if (n <= 0 || (n & (n - 1)) != 0) throw std::invalid_argument("DcCenteredFft: n must be a power of two");
    in_ = static_cast<std::complex<float>*>(mufft_alloc(static_cast<size_t>(n) * sizeof(std::complex<float>)));
    out_ = static_cast<std::complex<float>*>(mufft_alloc(static_cast<size_t>(n) * sizeof(std::complex<float>)));
    plan_ = mufft_create_plan_1d_c2c(static_cast<unsigned>(n), MUFFT_FORWARD, MUFFT_FLAG_CPU_ANY);
    if (in_ == nullptr || out_ == nullptr || plan_ == nullptr) throw std::runtime_error("DcCenteredFft: muFFT allocation failed");
  }
  ~DcCenteredFft() {
    mufft_free_plan_1d(plan_);
    mufft_free(in_);
    mufft_free(out_);
  }
  DcCenteredFft(const DcCenteredFft&) = delete;
  DcCenteredFft& operator=(const DcCenteredFft&) = delete;

  int size() const { return n_; }

  /// The DC-centred spectrum of the n CS16 samples at d[2*base ..].
  std::vector<std::complex<float>> run(const short* d, int base, bool conj) {
    const float qs = conj ? -1.0f : 1.0f;
    for (int i = 0; i < n_; ++i)
      in_[i] = {static_cast<float>(d[2 * (base + i)]), qs * static_cast<float>(d[2 * (base + i) + 1])};
    mufft_execute_plan_1d(plan_, out_, in_);
    std::vector<std::complex<float>> xs(static_cast<size_t>(n_));
    for (int k = 0; k < n_; ++k) xs[static_cast<size_t>(k)] = out_[(k + n_ / 2) % n_];
    return xs;
  }

 private:
  int n_;
  std::complex<float>* in_ = nullptr;
  std::complex<float>* out_ = nullptr;
  mufft_plan_1d* plan_ = nullptr;
};

}  // namespace houdini
