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

/// Mean index of the samples whose 128-sample energy (from the cumulative
/// energy `cse`, cse[i] = sum of |x|^2 over [0, i)) exceeds 15 % of the
/// window's peak, over [w0, w1). Returns false when nothing qualifies.
static bool countCentroid(const std::vector<double>& cse, long long w0, long long w1, double* centroid) {
  const long long cg = static_cast<long long>(cse.size()) - 1;
  w0 = std::max(0LL, w0);
  w1 = std::min(cg, w1);
  double peak = 0.0;
  for (long long i = w0 + 64; i + 64 <= w1; ++i) peak = std::max(peak, cse[i + 64] - cse[i - 64]);
  const double thr = 0.15 * peak;
  long long cnt = 0;
  double isum = 0.0;
  for (long long i = w0 + 64; i + 64 <= w1; ++i)
    if (cse[i + 64] - cse[i - 64] > thr) {
      ++cnt;
      isum += static_cast<double>(i);
    }
  if (cnt == 0) return false;
  *centroid = isum / static_cast<double>(cnt);
  return true;
}

// The rule this replaces: each slot centroid-aligned on its own window.
static long long perSlot(const std::vector<double>& cse, long long guess, long long n) {
  double c = 0.0;
  if (!countCentroid(cse, guess - n / 8, guess - n / 8 + 5 * n / 4, &c)) return guess;
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
  // The review's case (L1, L2, L9): OFDM-like content (Gaussian I/Q, so the
  // windowed energy fluctuates), a POSITIVE noise floor, U within +-5 % of P,
  // and the coarse guess off by up to 0.45 of a slot either way. A noise-like
  // envelope moves the half-plateau crossing by a waveform-dependent bias
  // (tens of samples), which the bench tx_advance absorbs for a given pilot,
  // so the tolerance here is 32 samples: half R1's zero prefix, and well
  // inside R3's 288-sample CP window.
  {
    bool ok = true;
    long long worst = 0;
    for (unsigned seed = 1; seed <= 20; ++seed)
      for (double u : {0.95, 1.0, 1.05})
        for (double gerr : {-0.45, -0.25, 0.0, 0.25, 0.45}) {
          std::mt19937 g(seed * 97u + static_cast<unsigned>(u * 100));
          std::normal_distribution<double> nd(0.0, 1.0);
          std::vector<double> e(static_cast<size_t>(cg), 0.0);
          for (auto& v : e) v = 9.0 * (nd(g) * nd(g) + nd(g) * nd(g));  // |noise|^2, positive
          for (int k = 0; k < 2; ++k)
            for (long long i = 128; i < n - 128; ++i) {
              const double a = (k ? u : 1.0) * 1000.0;
              const double re = a * nd(g), im = a * nd(g);
              e[static_cast<size_t>(truth + k * n + i)] += re * re + im * im;
            }
          std::vector<double> c(e.size() + 1, 0.0);
          for (size_t i = 0; i < e.size(); ++i) c[i + 1] = c[i] + e[i];
          const long long guess = truth + static_cast<long long>(gerr * static_cast<double>(n));
          const long long d = houdini::slotalign::burstPilotStart(c, guess, n, 128) - truth;
          worst = std::max(worst, std::llabs(d));
          if (std::llabs(d) > 32) ok = false;
        }
    std::printf("      OFDM-like, U 0.95-1.05, guess +-0.45 slot: worst error %lld samples (300 trials)\n", worst);
    check(ok, "OFDM-like P+U, guess off by up to 0.45 slot: the pilot within 32 samples (mutation: the old "
              "median over guess+n/4..3n/4 and a search from guess-n/4)");
  }
  const std::vector<long long> p = {0};
  const auto cse4 = capture(cg, truth, n, p, {1000.0}, 128, 128, 4);
  check(houdini::slotalign::burstPilotStart(cse4, cg, n, 128) <= cg - n,
        "the start is clamped so the pilot slot lies inside the capture (mutation: no clamp)");
  if (failures) std::printf("FAILED: %d failure(s)\n", failures);
  return failures ? 1 : 0;
}
