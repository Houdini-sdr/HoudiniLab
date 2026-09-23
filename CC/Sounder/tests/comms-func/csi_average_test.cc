// AP-79 #5: the pilot slot's de-rotated per-symbol average (houdini/csi_average.h).
// Each assertion names the mutation that breaks it.
#include <cmath>
#include <complex>
#include <cstdio>
#include <random>
#include <vector>

#include "houdini/csi_average.h"

using cf = std::complex<float>;
static int failures = 0;
static void check(bool ok, const char* what) {
  std::printf("%s  %s\n", ok ? "PASS" : "FAIL", what);
  if (!ok) ++failures;
}

// nsym copies of a channel H, symbol s turned by s x step radians, with noise.
static std::vector<std::vector<cf>> rows(const std::vector<cf>& H, int nsym, double step, double noise, unsigned seed) {
  std::mt19937 g(seed);
  std::normal_distribution<float> n(0.0f, static_cast<float>(noise));
  std::vector<std::vector<cf>> G(nsym, std::vector<cf>(H.size()));
  for (int s = 0; s < nsym; ++s) {
    const cf r = std::polar(1.0f, static_cast<float>(s * step));
    for (size_t k = 0; k < H.size(); ++k) G[s][k] = H[k] * r + cf(n(g), n(g));
  }
  return G;
}

int main() {
  const int N = 96;
  std::vector<cf> H(N);
  for (int k = 0; k < N; ++k) H[k] = std::polar(1.0f + 0.2f * std::sin(0.1f * k), 0.05f * k);
  const int nsym = 12;  // the symbols the recorder averages at R3 (14 less the guard eighths)
  auto gain = [&](const std::vector<cf>& a) {
    double num = 0, den = 0;
    for (int k = 0; k < N; ++k) { num += std::abs(a[k]); den += std::abs(H[k]); }
    return num / den;
  };
  auto phaseErr = [&](const std::vector<cf>& a, double want) {  // mean |arg(a/H) - want|
    double e = 0;
    for (int k = 0; k < N; ++k) e += std::fabs(std::remainder(std::arg(a[k] / H[k]) - want, 2 * M_PI));
    return e / N;
  };
  // R3's case: 3 degrees a symbol. Then a hard one: 40 degrees a symbol, 440 in all.
  for (double deg : {3.0, 40.0}) {
    const double step = deg * M_PI / 180.0;
    const auto G = rows(H, nsym, step, 0.02, 7);
    std::vector<double> th;
    const auto d = houdini::csi::derotatedAverage(G, true, &th);
    const auto p = houdini::csi::derotatedAverage(G, false);
    const double mid = step * (nsym - 1) / 2.0;  // the plain average's phase centre
    std::printf("      %.0f deg/sym: |H| kept %.4f (plain %.4f); phase error %.4f rad; last rotation %.1f deg\n", deg,
                gain(d), gain(p), phaseErr(d, mid), th.back() * 180.0 / M_PI);
    if (deg == 3.0) {
      check(gain(d) > 0.999 && phaseErr(d, mid) < 0.01,
            "3 deg/sym (R3, 0.1 ppm): |H| and its phase kept (mutation: no de-rotation loses 1.5%; drop the "
            "mean-rotation restore and the phase moves 16 deg)");
    } else {
      check(gain(d) > 0.999 && gain(p) < 0.5 && phaseErr(d, mid) < 0.01,
            "40 deg/sym, 440 in all: de-rotation keeps |H| where the plain average loses over half (mutation: "
            "no de-rotation)");
      check(std::fabs(th.back() - step * (nsym - 1)) < 0.05,
            "the rotation is unwrapped symbol to symbol (mutation: arg against the first symbol wraps it, which "
            "moves the restored mean phase)");
    }
  }
  // A stationary link: nothing changes (the plain average and the de-rotated agree).
  const auto G0 = rows(H, nsym, 0.0, 0.02, 9);
  const auto d0 = houdini::csi::derotatedAverage(G0, true), p0 = houdini::csi::derotatedAverage(G0, false);
  double diff = 0;
  for (int k = 0; k < N; ++k) diff = std::max(diff, static_cast<double>(std::abs(d0[k] - p0[k])));
  check(diff < 0.005, "no rotation: de-rotated equals the plain average to noise (a stationary link sees no change)");
  if (failures) std::printf("FAILED: %d failure(s)\n", failures);
  return failures ? 1 : 0;
}
