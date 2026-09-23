/**
 * @file dc_fft_test.cc
 * @brief houdini::DcCenteredFft against the explicit DFT-matrix definition it
 *        replaced in RecorderWorker::symbolFft (AP-79), NO hardware.
 *
 * The definition: Xs[k] = sum_n x[n] exp(-j 2 pi m n / N), m = (k + N/2) mod N,
 * x the CS16 sample with the imaginary part negated when `conj`. Checked in
 * full at N = 64 and 256 and on a spread of bins at N = 4096. The mutant is
 * the unrotated transform (DC left at index 0): the same comparison must fail.
 *
 * Build: CMake target dc_fft_test. Run: ./dc_fft_test (or ctest).
 */
#include <cmath>
#include <complex>
#include <cstdio>
#include <string>
#include <vector>

#include "houdini/dc_fft.h"

namespace {
int g_fail = 0;
void check(bool ok, const std::string& what) {
  std::printf("%s  %s\n", ok ? "PASS" : "FAIL", what.c_str());
  if (!ok) ++g_fail;
}
std::complex<double> defn(const std::vector<short>& d, int base, int n, int k, bool conj, bool rotate) {
  const int m = rotate ? (k + n / 2) % n : k;
  std::complex<double> a(0, 0);
  for (int t = 0; t < n; ++t) {
    const std::complex<double> x(d[2 * (base + t)], (conj ? -1.0 : 1.0) * d[2 * (base + t) + 1]);
    const double ph = -2.0 * M_PI * static_cast<double>((static_cast<long long>(m) * t) % n) / n;
    a += x * std::complex<double>(std::cos(ph), std::sin(ph));
  }
  return a;
}
// Largest error relative to the spectrum's peak, over `step`-spaced bins.
double relErr(houdini::DcCenteredFft& f, const std::vector<short>& d, int base, bool conj, bool rotate, int step) {
  const int n = f.size();
  const auto xs = f.run(d.data(), base, conj);
  double peak = 0.0, err = 0.0;
  for (int k = 0; k < n; k += step) {
    const auto want = defn(d, base, n, k, conj, rotate);
    peak = std::max(peak, std::abs(want));
    err = std::max(err, std::abs(std::complex<double>(xs[k].real(), xs[k].imag()) - want));
  }
  return err / peak;
}
}  // namespace

int main() {
  for (int n : {64, 256, 4096}) {
    const int base = 37;
    std::vector<short> d(2 * (base + n + 8));
    for (size_t i = 0; i < d.size(); ++i) d[i] = static_cast<short>(std::lround(9000.0 * std::sin(0.013 * i * i + 0.7 * i)));
    houdini::DcCenteredFft f(n);
    const int step = n <= 256 ? 1 : 97;
    for (bool conj : {false, true}) {
      const double e = relErr(f, d, base, conj, true, step);
      char buf[160];
      std::snprintf(buf, sizeof buf, "N=%d conj=%d: the FFT equals the DC-centred DFT definition (max error %.1e of the peak)", n, conj ? 1 : 0, e);
      check(e < 1e-4, buf);
    }
    const double em = relErr(f, d, base, false, false, step);
    check(em > 1e-2, "mutant N=" + std::to_string(n) + ": the unrotated transform (DC at index 0) does NOT match");
  }
  std::printf("%s: %d failure(s)\n", g_fail == 0 ? "ALL PASS" : "FAILED", g_fail);
  return g_fail == 0 ? 0 : 1;
}
