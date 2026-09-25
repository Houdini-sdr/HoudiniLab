/**
 * @file houdini/spectrum.h
 * @brief The dashboard's spectrum of the raw pilot slot: a Welch average in
 *        dBFS per display bin, DC-centred (the Spectrum tab, SPC1).
 *
 * NORMALISATION. Back-to-back segments of `seg` samples (no overlap: 50 %
 * would double the FFTs for a smoother trace the display does not need) are
 * Hann-windowed (periodic) and transformed; fine bin k gets
 * |X_k|^2 / (seg * sum(w^2) * fs^2), so by Parseval the fine bins of one segment sum to its windowed mean power
 * in units of fs^2. Segments are averaged in linear power, and each of the
 * `nbins` display bins SUMS r = seg / nbins adjacent fine bins: a display bin
 * is the power falling in its rate / nbins band. So a full-scale complex tone
 * (|x| = fs) reads 0 dBFS in the bin that holds it, white noise of mean power
 * P reads P - 10 log10(nbins) in every bin, and the bins sum to the slot's
 * power. A tone near a display-bin edge splits its power between the two
 * bins (about -3 dB each exactly on the edge).
 *
 * ORDER. Fine bin j is frequency (j - seg/2) * rate / seg, so display bin i
 * holds fine bins [i r, (i+1) r) and is centred at
 * (i r + (r - 1) / 2 - seg / 2) * rate / seg: DC-centred, -rate/2 first.
 *
 * RENEW OPEN SOURCE LICENSE: http://renew-wireless.org/license
 */
#pragma once

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

#include "houdini/mufft_c2c.h"

namespace houdini {

class WelchSpectrum {
 public:
  WelchSpectrum(int seg, int nbins) : fft_(seg, MUFFT_FORWARD, "WelchSpectrum"), nbins_(nbins) {
    if (nbins <= 0 || seg % nbins != 0) throw std::invalid_argument("WelchSpectrum: nbins must divide seg");
    w_.resize(static_cast<size_t>(seg));
    for (int n = 0; n < seg; ++n) {
      w_[static_cast<size_t>(n)] = static_cast<float>(0.5 - 0.5 * std::cos(2.0 * M_PI * n / seg));
      w2_ += static_cast<double>(w_[static_cast<size_t>(n)]) * w_[static_cast<size_t>(n)];
    }
    acc_.resize(static_cast<size_t>(seg));
  }

  int seg() const { return fft_.size(); }
  int nbins() const { return nbins_; }

  /// dBFS per display bin from the n CS16 samples at d (I, Q interleaved),
  /// Q negated when `conj` (the R2C mixer's inversion); fs is full scale.
  /// Empty when n is shorter than one segment. Floored at -200 dB (finite).
  std::vector<float> run(const short* d, int n, bool conj, double fs) {
    const int m = fft_.size();
    if (n < m) return {};
    const float qs = conj ? -1.0f : 1.0f;
    std::fill(acc_.begin(), acc_.end(), 0.0);
    int nseg = 0;
    for (int s = 0; s + m <= n; s += m, ++nseg) {
      std::complex<float>* in = fft_.in();
      const short* p = d + 2 * static_cast<ptrdiff_t>(s);
      for (int i = 0; i < m; ++i)
        in[i] = {w_[static_cast<size_t>(i)] * p[2 * i], qs * w_[static_cast<size_t>(i)] * p[2 * i + 1]};
      fft_.execute();
      const std::complex<float>* out = fft_.out();
      for (int j = 0; j < m; ++j) acc_[static_cast<size_t>(j)] += std::norm(out[(j + m / 2) % m]);
    }
    const double scale = 1.0 / (static_cast<double>(nseg) * m * w2_ * fs * fs);
    const int r = m / nbins_;
    std::vector<float> db(static_cast<size_t>(nbins_));
    for (int i = 0; i < nbins_; ++i) {
      double p = 0.0;
      for (int j = i * r; j < (i + 1) * r; ++j) p += acc_[static_cast<size_t>(j)];
      db[static_cast<size_t>(i)] = static_cast<float>(10.0 * std::log10(std::max(p * scale, 1e-20)));
    }
    return db;
  }

 private:
  MufftC2C fft_;
  int nbins_;
  std::vector<float> w_;
  double w2_ = 0.0;
  std::vector<double> acc_;
};

}  // namespace houdini
