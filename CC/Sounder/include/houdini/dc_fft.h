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
#include <vector>

#include "houdini/mufft_c2c.h"

namespace houdini {

class DcCenteredFft {
 public:
  explicit DcCenteredFft(int n) : fft_(n, MUFFT_FORWARD, "DcCenteredFft") {}

  int size() const { return fft_.size(); }

  /// The DC-centred spectrum of the n CS16 samples at d[2*base ..].
  std::vector<std::complex<float>> run(const short* d, int base, bool conj) {
    const int n = fft_.size();
    const float qs = conj ? -1.0f : 1.0f;
    std::complex<float>* in = fft_.in();
    for (int i = 0; i < n; ++i)
      in[i] = {static_cast<float>(d[2 * (base + i)]), qs * static_cast<float>(d[2 * (base + i) + 1])};
    fft_.execute();
    std::vector<std::complex<float>> xs(static_cast<size_t>(n));
    for (int k = 0; k < n; ++k) xs[static_cast<size_t>(k)] = fft_.out()[(k + n / 2) % n];
    return xs;
  }

 private:
  MufftC2C fft_;
};

}  // namespace houdini
