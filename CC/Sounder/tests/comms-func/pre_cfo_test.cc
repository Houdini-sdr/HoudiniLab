// houdini/pre_cfo.h: the carrier offset from a slot of identical symbols, removed
// before the FFT. Each assertion names the mutation that breaks it.
#include <cmath>
#include <complex>
#include <cstdio>
#include <random>
#include <vector>

#include "houdini/pre_cfo.h"

static int failures = 0;
static void check(bool ok, const char* what) {
  std::printf("%s  %s\n", ok ? "PASS" : "FAIL", what);
  if (!ok) ++failures;
}

int main() {
  namespace pc = houdini::precfo;
  const int N = 4096, cp = 288, L = N + cp, nsym = 14, pre = 32, n = pre + nsym * L + 32;
  const double rate = 122.88e6;
  std::mt19937 g(5);
  std::normal_distribution<double> w(0.0, 1.0);
  std::vector<std::complex<double>> sym(N);
  for (auto& v : sym) v = {400.0 * w(g), 400.0 * w(g)};
  auto slotWith = [&](double hz, double snr_noise, long long t0) {
    std::vector<short> d(2 * static_cast<size_t>(n), 0);
    for (int s = 0; s < nsym; ++s)
      for (int i = 0; i < L; ++i) {
        const int k = pre + s * L + i;
        const auto x = sym[static_cast<size_t>((i - cp + N) % N)];  // the CP is the body's tail
        const double a = 2.0 * M_PI * hz * static_cast<double>(t0 + k) / rate;
        const auto y = x * std::complex<double>(std::cos(a), std::sin(a));
        d[2 * k] = static_cast<short>(std::lround(y.real() + snr_noise * w(g)));
        d[2 * k + 1] = static_cast<short>(std::lround(y.imag() + snr_noise * w(g)));
      }
    return d;
  };
  const auto d = slotWith(600.0, 0.0, 0);
  const auto e = pc::estimate(d.data(), n, pre, L, nsym, rate);
  std::printf("      600 Hz read %.2f Hz, coherence %.3f\n", e.hz, e.coherence);
  check(std::fabs(e.hz - 600.0) < 1.0 && e.coherence > 0.95,
        "the offset of identical symbols within 1 Hz (mutation: the lag or the rate scale wrong)");
  const auto dn = slotWith(-450.0, 400.0 / std::sqrt(10.0), 0);  // 10 dB SNR, the weak sub-6 over the air
  const auto en = pc::estimate(dn.data(), n, pre, L, nsym, rate);
  check(std::fabs(en.hz + 450.0) < 5.0 && en.coherence > 0.8,
        "at 10 dB SNR still within 5 Hz, the sign kept (mutation: conj on the wrong factor)");
  std::vector<short> noise(2 * static_cast<size_t>(n));
  for (auto& v : noise) v = static_cast<short>(std::lround(300.0 * w(g)));
  check(pc::estimate(noise.data(), n, pre, L, nsym, rate).coherence < 0.2,
        "noise has no coherence, so the correction is not applied to it (mutation: coherence not normalised)");
  std::vector<short> out(d.size());
  pc::derotate(d.data(), out.data(), n, e.hz, rate, 0);
  const auto after = pc::estimate(out.data(), n, pre, L, nsym, rate);
  check(std::fabs(after.hz) < 1.0, "derotating by the estimate leaves under 1 Hz (mutation: the rotation's sign)");
  // A later slot of the same frame, one slot on, continues the phase from t0 = n.
  const auto d2 = slotWith(600.0, 0.0, n);
  std::vector<short> out2(d2.size());
  pc::derotate(d2.data(), out2.data(), n, e.hz, rate, n);
  double worst = 0.0;
  for (int i = pre; i < n - 32; i += 97) {
    const std::complex<double> a(out[2 * i], out[2 * i + 1]), b(out2[2 * i], out2[2 * i + 1]);
    if (std::abs(a) > 100.0) worst = std::max(worst, std::fabs(std::arg(b * std::conj(a))));
  }
  check(worst < 0.02, "the next slot derotated from t0 = n matches the pilot's phase (mutation: t0 ignored)");
  if (failures) std::printf("FAILED: %d failure(s)\n", failures);
  else std::printf("ALL PASS\n");
  return failures ? 1 : 0;
}
