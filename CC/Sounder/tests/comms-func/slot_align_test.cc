// AP-79: the BS framer's whole-burst slot alignment (houdini/slot_align.h).
// Each assertion names the mutation that breaks it.
#include <cmath>
#include <cstdio>
#include <random>
#include <vector>

#include "houdini/slot_align.h"

static int failures = 0;
static void check(bool ok, const char* what) {
  std::printf("%s  %s\n", ok ? "PASS" : "FAIL", what);
  if (!ok) ++failures;
}

// A capture with the UE burst's slots at `start + rel[k] * n`, each [prefix |
// content at rms[k] | postfix], over a noise floor; returns the cumulative energy.
static std::vector<double> capture(long long cg, long long start, long long n, const std::vector<long long>& rel,
                                   const std::vector<double>& rms, int prefix, int postfix, unsigned seed) {
  std::mt19937 g(seed);
  std::normal_distribution<double> w(0.0, 3.0);
  std::vector<double> e(static_cast<size_t>(cg), 0.0);
  for (auto& v : e) v = w(g) * w(g);
  for (size_t k = 0; k < rel.size(); ++k)
    for (long long i = prefix; i < n - postfix; ++i) {
      const long long t = start + rel[k] * n + i;
      if (t >= 0 && t < cg) {
        const double a = rms[k] * (1.0 + 0.3 * std::sin(0.37 * static_cast<double>(i)));
        e[static_cast<size_t>(t)] += a * a;
      }
    }
  std::vector<double> cse(e.size() + 1, 0.0);
  for (size_t i = 0; i < e.size(); ++i) cse[i + 1] = cse[i] + e[i];
  return cse;
}

// The rule this replaces: each slot centroid-aligned on its own window.
static long long perSlot(const std::vector<double>& cse, long long guess, long long n) {
  double c = 0.0;
  if (!houdini::slotalign::countCentroid(cse, guess - n / 8, guess - n / 8 + 5 * n / 4, &c)) return guess;
  return std::llround(c) - n / 2;
}

int main() {
  const long long n = 4096, cg = 5 * n;
  const long long truth = 9000;
  const std::vector<long long> pu = {0, 1};
  // R1: P and U ADJACENT. The per-slot rule, for the record and as the defect.
  const auto cse = capture(cg, truth, n, pu, {1000.0, 700.0}, 128, 128, 1);
  const long long old_p = perSlot(cse, truth + 150, n), old_u = perSlot(cse, truth + n + 150, n);
  std::printf("      per-slot rule: P %+lld, U %+lld, P-to-U error %+lld samples\n", old_p - truth,
              old_u - truth - n, old_u - old_p - n);
  check(std::llabs(old_u - old_p - n) > 50, "the per-slot rule mis-spaces adjacent P and U (the defect this test pins)");
  // The edge anchor, for a U at any level relative to P (the burst centroid
  // tried first failed below ~0.7x and lost U entirely below ~0.3x).
  bool all = true;
  for (double u : {1500.0, 1000.0, 700.0, 300.0, 100.0, 0.0}) {
    for (long long err : {-900LL, -150LL, 0LL, 150LL, 900LL}) {  // the coarse guess, within n/4
      const auto c = capture(cg, truth, n, pu, {1000.0, u}, 128, 128, static_cast<unsigned>(u) + 7);
      const long long got = houdini::slotalign::burstPilotStart(c, truth + err, n, 128);
      if (std::llabs(got - truth) > 2) {
        std::printf("      U %.0f guess %+lld: %+lld\n", u, err, got - truth);
        all = false;
      }
    }
  }
  check(all, "adjacent P+U, U from +3.5 dB to absent, guess within +-900: the pilot start within 2 samples "
             "(mutation: a burst centroid, or an edge at 0.15 of the plateau instead of 0.5)");
  // houdini-ul: P and U with a guard slot between them.
  const auto cse2 = capture(cg, truth, n, {0, 2}, {1000.0, 700.0}, 128, 128, 2);
  check(std::llabs(houdini::slotalign::burstPilotStart(cse2, truth - 300, n, 128) - truth) <= 2,
        "guarded P, G, U: exact");
  // R3-like layout: a short prefix, a long postfix.
  const auto cse3 = capture(cg, truth, n, pu, {1000.0, 1000.0}, 32, 256, 3);
  check(std::llabs(houdini::slotalign::burstPilotStart(cse3, truth + 100, n, 32) - truth) <= 2,
        "prefix 32 / postfix 256: exact (mutation: subtract a fixed 128 instead of the prefix)");
  const std::vector<long long> p = {0};
  const auto cse4 = capture(cg, truth, n, p, {1000.0}, 128, 128, 4);
  check(houdini::slotalign::burstPilotStart(cse4, cg, n, 128) <= cg - n,
        "the start is clamped so the pilot slot lies inside the capture (mutation: no clamp)");
  if (failures) std::printf("FAILED: %d failure(s)\n", failures);
  return failures ? 1 : 0;
}
