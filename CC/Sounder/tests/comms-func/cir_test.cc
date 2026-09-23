// AP-79: the CSI view's channel impulse response (houdini/cir.h), against a
// naive IDFT on a known two-path channel. Each assertion names its mutation.
#include <cmath>
#include <complex>
#include <cstdio>
#include <vector>

#include "houdini/cir.h"

static int failures = 0;
static void check(bool ok, const char* what) {
  std::printf("%s  %s\n", ok ? "PASS" : "FAIL", what);
  if (!ok) ++failures;
}

int main() {
  const int N = 256;
  // A direct path at tap 40 and an echo 6 dB down at tap 52, as H on the
  // DC-centred grid (index k is frequency k - N/2).
  std::vector<std::complex<float>> H(N);
  for (int k = 0; k < N; ++k) {
    const double f = static_cast<double>(k - N / 2);
    const std::complex<double> h = std::polar(1.0, -2 * M_PI * f * 40 / N) + 0.5 * std::polar(1.0, -2 * M_PI * f * 52 / N);
    H[k] = std::complex<float>(static_cast<float>(h.real()), static_cast<float>(h.imag()));
  }
  houdini::CirFromH cir(N);
  const auto p = cir.power(H);
  // The naive IDFT, unnormalised like muFFT's inverse.
  double worst = 0.0;
  for (int t = 0; t < N; ++t) {
    std::complex<double> a(0, 0);
    for (int m = 0; m < N; ++m) {
      const int k = (m + N / 2) % N;  // natural bin m is DC-centred index k
      a += std::complex<double>(H[k].real(), H[k].imag()) * std::polar(1.0, 2 * M_PI * m * t / N);
    }
    worst = std::max(worst, std::fabs(std::norm(a) - p[t]) / (N * N));
  }
  // (Skipping the DC-centred to natural reorder cannot be caught here: a shift
  // of N/2 in frequency only multiplies h by (-1)^t, leaving |h|^2 unchanged.)
  check(worst < 1e-4, "power() equals the naive IDFT's |h|^2 (mutation: a forward FFT, the delays mirror)");
  int pk = -1;
  const auto w = houdini::cirWindowDb(p, 8, 32, &pk);
  check(pk == 40, "the strongest tap is the direct path, tap 40 (mutation: a forward FFT puts it at N - 40)");
  check(w.size() == 32 && std::fabs(w[8]) < 1e-3, "the window starts 8 taps before the peak, which reads 0 dB");
  check(std::fabs(w[8 + 12] - (-6.02f)) < 0.05f, "the echo 12 taps later reads -6.0 dB (mutation: amplitude, not power, gives -3)");
  // The window wraps: a peak at tap 2 with 8 pre-taps reads taps N-6..N-1 first.
  std::vector<float> q(N, 1e-6f);
  q[2] = 1.0f;
  int pk2 = -1;
  const auto w2 = houdini::cirWindowDb(q, 8, 16, &pk2);
  check(pk2 == 2 && w2.size() == 16 && std::fabs(w2[8]) < 1e-3 && w2[0] <= -59.9f,
        "a peak near tap 0 wraps the window circularly (mutation: no modulo, it reads out of range)");
  if (failures) std::printf("FAILED: %d failure(s)\n", failures);
  return failures ? 1 : 0;
}
