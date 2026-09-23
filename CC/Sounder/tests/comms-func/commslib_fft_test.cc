/**
 * @file commslib_fft_test.cc
 * @brief CommsLib::FFT / IFFT on an input SHORTER than the transform (AP-79).
 *
 * The inherited code sized its output to the input and memcpy'd fftSize
 * samples in and out, so getPilotScValue's ofdm_data_num-long sequence
 * overran the heap at every fft_size larger than it (a crash at 256 / 96, a
 * silent 20 KB overrun at 4096 / 1596; the old fft 64 configs took the 802.11
 * path and never reached it). The fixed transform is fftSize points of the
 * zero-padded input; this pins the length and the values against a direct DFT.
 *
 * Build: CMake target commslib_fft_test. Run: ./commslib_fft_test (or ctest).
 */
#include <cmath>
#include <complex>
#include <cstdio>
#include <string>
#include <vector>

#include "comms-lib.h"

namespace {
int g_fail = 0;
void check(bool ok, const std::string& what) {
  std::printf("%s  %s\n", ok ? "PASS" : "FAIL", what.c_str());
  if (!ok) ++g_fail;
}
using cf = std::complex<float>;
std::vector<cf> dft(const std::vector<cf>& x, size_t n, int sign) {
  std::vector<cf> y(n);
  for (size_t k = 0; k < n; ++k) {
    std::complex<double> a(0, 0);
    for (size_t t = 0; t < x.size() && t < n; ++t) {
      const double ph = sign * 2.0 * M_PI * static_cast<double>((k * t) % n) / static_cast<double>(n);
      a += std::complex<double>(x[t].real(), x[t].imag()) * std::complex<double>(std::cos(ph), std::sin(ph));
    }
    y[k] = cf(static_cast<float>(a.real()), static_cast<float>(a.imag()));
  }
  return y;
}
double maxErr(const std::vector<cf>& a, const std::vector<cf>& b) {
  double e = 0.0;
  for (size_t i = 0; i < a.size() && i < b.size(); ++i) e = std::max(e, static_cast<double>(std::abs(a[i] - b[i])));
  return e;
}
}  // namespace

int main() {
  for (const auto& pr : {std::pair<size_t, size_t>{96, 256}, {1596, 4096}, {256, 256}}) {
    const size_t m = pr.first, n = pr.second;
    std::vector<cf> x(m);
    for (size_t i = 0; i < m; ++i) x[i] = cf(std::cos(0.37f * i), std::sin(0.11f * i * i));
    const auto y = CommsLib::FFT(x, static_cast<int>(n));
    const auto ref = dft(x, n, -1);
    check(y.size() == n && maxErr(y, ref) < 1e-2 * std::sqrt(static_cast<double>(m)),
          "FFT of " + std::to_string(m) + " samples at " + std::to_string(n) +
              " points: " + std::to_string(n) + " outputs, the zero-padded DFT");
    const auto z = CommsLib::IFFT(x, static_cast<int>(n), 1.0f, false, false);
    const auto zref = dft(x, n, +1);
    // IFFT scales by 1 (scale) and the library's own convention; compare shape
    // by the ratio at the strongest bin, then the whole vector.
    size_t kmax = 0;
    for (size_t k = 1; k < n; ++k) if (std::abs(zref[k]) > std::abs(zref[kmax])) kmax = k;
    const cf g = z.size() == n ? z[kmax] / zref[kmax] : cf(0, 0);
    std::vector<cf> zs(zref);
    for (auto& v : zs) v *= g;
    check(z.size() == n && std::abs(g) > 0.0f && maxErr(z, zs) < 1e-2 * std::sqrt(static_cast<double>(m)) * std::abs(g),
          "IFFT of " + std::to_string(m) + " samples at " + std::to_string(n) + " points: " + std::to_string(n) +
              " outputs, the zero-padded inverse DFT up to the library's scale");
  }
  std::printf("%s: %d failure(s)\n", g_fail == 0 ? "ALL PASS" : "FAILED", g_fail);
  return g_fail == 0 ? 0 : 1;
}
