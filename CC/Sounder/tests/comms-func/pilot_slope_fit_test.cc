// AP-79: the recorder's stage-2 pilot-tone timing fit (houdini/pilot_slope_fit.h).
// Each assertion names the mutation that breaks it.
#include <cmath>
#include <complex>
#include <cstdio>
#include <vector>

#include "houdini/pilot_slope_fit.h"

static int failures = 0;
static void check(bool ok, const char* what) {
  std::printf("%s  %s\n", ok ? "PASS" : "FAIL", what);
  if (!ok) ++failures;
}

// Pilot tones at R3-like offsets carrying a common phase c0 and a timing slope
// (rad per bin), as the U slot's pilot tones do after the pilot-derived H.
static void tones(double c0, double slope, std::vector<double>* kk, std::vector<std::complex<double>>* acc) {
  kk->clear();
  acc->clear();
  for (int i = -4; i <= 4; ++i) {
    const double k = 110.0 * i;
    kk->push_back(k);
    acc->push_back(std::polar(1.0, c0 + slope * k));
  }
}

int main() {
  std::vector<double> kk;
  std::vector<std::complex<double>> acc;
  // A small timing residual (0.3 sample at N 4096 = 2 pi 0.3 / 4096 rad/bin).
  const double slope = 2.0 * M_PI * 0.3 / 4096.0;
  tones(0.4, slope, &kk, &acc);
  auto f = houdini::csi::pilotSlopeFit(kk, acc);
  check(f.ok && std::fabs(f.slope - slope) < 1e-9 && std::fabs(f.intercept - 0.4) < 1e-9,
        "common phase 0.4 rad: slope and intercept exact");
  // The case that broke: the common phase at 180 degrees, so the tones straddle
  // the +-pi cut (some +179, some -179 degrees).
  tones(M_PI, slope, &kk, &acc);
  f = houdini::csi::pilotSlopeFit(kk, acc);
  check(f.ok && std::fabs(f.slope - slope) < 1e-9 && std::fabs(std::remainder(f.intercept - M_PI, 2 * M_PI)) < 1e-9,
        "common phase at 180 degrees: slope still exact (mutation: arg() of each tone directly, as before, "
        "fits across the +-pi cut)");
  // Every common phase round the circle.
  bool all = true;
  for (double c0 = -M_PI; c0 < M_PI; c0 += 0.05) {
    tones(c0, slope, &kk, &acc);
    f = houdini::csi::pilotSlopeFit(kk, acc);
    all = all && f.ok && std::fabs(f.slope - slope) < 1e-9 && std::fabs(std::remainder(f.intercept - c0, 2 * M_PI)) < 1e-9;
  }
  check(all, "every common phase from -180 to +180 degrees: exact");
  // The 802.11 layout at fft 64 (pilots at +-7 and +-21) with a 1.2-sample
  // residual and the common phase near 0: the unwrapped fit reads it exactly
  // (an Opus review measured the unit-mean form reading -0.63).
  {
    std::vector<double> k64{-21, -7, 7, 21};
    std::vector<std::complex<double>> a64;
    const double s64 = 2.0 * M_PI * 1.2 / 64.0;
    for (double k : k64) a64.push_back(std::polar(1.0, 0.1 + s64 * k));
    const auto g = houdini::csi::pilotSlopeFit(k64, a64);
    check(g.ok && std::fabs(g.slope * 64.0 / (2.0 * M_PI) - 1.2) < 1e-9,
          "fft 64, pilots +-7/+-21, a 1.2-sample residual: read as 1.2 (mutation: reference the tones to their "
          "unit-vector mean, which flips and reads -0.63)");
  }
  kk.resize(1);
  acc.resize(1);
  check(!houdini::csi::pilotSlopeFit(kk, acc).ok, "one tone: no fit");
  if (failures) std::printf("FAILED: %d failure(s)\n", failures);
  return failures ? 1 : 0;
}
