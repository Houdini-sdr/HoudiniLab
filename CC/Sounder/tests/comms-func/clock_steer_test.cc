// AP-79: the in-sounder clock steering loop (sync/clock_steer.h), closed
// against a simulated UE clock. Each assertion names the mutation that breaks it.
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <random>

#include "sync/clock_steer.h"

using houdini::sync::ClockSteer;
using houdini::sync::ClockSteerConfig;

static int failures = 0;
static void check(bool ok, const char* what) {
  std::printf("%s  %s\n", ok ? "PASS" : "FAIL", what);
  if (!ok) ++failures;
}

struct Run {
  double worst_after = 0.0;  // max |eps| after the settle time, ppm
  int max_abs_offset = 0, max_abs_push = 0, pushes = 0;
  double min_gap_s = 1e9;    // shortest time between two pushes
};

// The UE sees eps(t) = e0 + ramp * t - offset * ppc (steering raises f_UE,
// which lowers eps), observed once a second with noise.
static Run simulate(ClockSteerConfig c, double e0, double ramp_ppm_per_s, double noise_ppm, double seconds,
                    double settle_s, unsigned seed) {
  std::mt19937 g(seed);
  std::normal_distribution<double> n(0.0, noise_ppm);
  ClockSteer s(c);
  s.start(0, 0.0);
  Run r;
  double last_push_t = -1e9;
  for (double t = 1.0; t <= seconds; t += 1.0) {
    const double eps = e0 + ramp_ppm_per_s * t - s.offset() * c.ppm_per_count;
    if (t >= settle_s) r.worst_after = std::max(r.worst_after, std::fabs(eps));
    const int push = s.observe(eps + n(g), t);
    if (push != 0) {
      s.applied(push);
      r.max_abs_push = std::max(r.max_abs_push, std::abs(push));
      r.min_gap_s = std::min(r.min_gap_s, t - last_push_t);
      last_push_t = t;
    }
    r.max_abs_offset = std::max(r.max_abs_offset, std::abs(s.offset()));
  }
  r.pushes = s.pushes();
  return r;
}

int main() {
  const ClockSteerConfig c;  // the shipped defaults
  // The AP-79 case: 0.4 ppm at bring-up, then the thermal ramp measured in
  // R2 (about 0.3 ppm over 300 s), sensor noise 0.01 ppm.
  const Run a = simulate(c, 0.4, 0.3 / 300.0, 0.01, 900.0, 120.0, 1);
  std::printf("      AP-79 ramp: worst |eps| after 120 s %.3f ppm, offset %d, pushes %d\n", a.worst_after,
              a.max_abs_offset, a.pushes);
  // The bound is the loop's own: half the quantum (0.063) plus the ramp over
  // one period (0.02) plus what gain 0.7 leaves of a push's error, about 0.15.
  check(a.worst_after < 0.15,
        "a 0.4 ppm offset plus a 1 ppm/1000 s ramp held under 0.15 ppm after 120 s (mutation: push with the "
        "wrong sign diverges to the authority bound)");
  // No steering, for scale: the same clock left alone.
  ClockSteerConfig off = c;
  off.max_offset = 0;  // no authority at all: the clock is left alone
  const Run b = simulate(off, 0.4, 0.3 / 300.0, 0.01, 900.0, 120.0, 1);
  std::printf("      unsteered: worst |eps| %.3f ppm\n", b.worst_after);
  check(b.worst_after > 1.0, "the unsteered clock drifts past 1 ppm over the same run (the harness can fail)");
  // Bounded authority: a 10 ppm offset needs 80 counts; the loop stops at 30.
  const Run d = simulate(c, 10.0, 0.0, 0.01, 3600.0, 1e9, 2);
  check(d.max_abs_offset == c.max_offset, "a 10 ppm offset parks at max_offset, 30 counts (mutation: no bound)");
  check(d.max_abs_push <= c.max_push, "no push exceeds max_push, 2 counts (mutation: no per-push clamp)");
  check(d.min_gap_s >= c.period_s - 1e-9, "at most one push per period_s (mutation: decide on every sample)");
  // Deadband. With the shipped 0.1251 ppm quantum, rounding alone already
  // refuses anything under ~0.09 ppm at gain 0.7, so the deadband is tested on
  // a finer actuator, where 0.05 ppm would round to a full count without it.
  ClockSteerConfig fine = c;
  fine.ppm_per_count = 0.05;
  const Run e = simulate(fine, 0.05, 0.0, 0.002, 600.0, 0.0, 3);
  check(e.pushes == 0,
        "0.05 ppm on a 0.05 ppm/count actuator, inside the 0.06 deadband: no push (mutation: no deadband, "
        "it pushes a count)");
  // The feed-forward's sign against receiver.cc's eps = samps_per_frame / period - 1.
  ClockSteer s(c);
  const double N = 122880.0, e_before = 0.37e-6;
  const double P = N / (1.0 + e_before);
  for (int k : {-2, -1, 1, 2}) {
    const double e_after = N / (P * s.periodScale(k)) - 1.0;
    const double want = e_before - k * c.ppm_per_count * 1e-6;
    if (std::fabs(e_after - want) > 1e-12) {
      std::printf("      k %+d: eps after %.6f ppm, want %.6f\n", k, e_after * 1e6, want * 1e6);
      check(false, "periodScale moves the tracked eps by exactly -k x ppm_per_count (mutation: 1 - k x ...)");
      return 1;
    }
  }
  check(true, "periodScale moves the tracked eps by exactly -k x ppm_per_count (mutation: 1 - k x ...)");
  // A session that INHERITS an offset outside the authority (a node left
  // steered +50 counts) steps back at most max_push a push, never in one jump.
  {
    ClockSteer w(c);
    w.start(50, 0.0);
    int worst = 0;
    for (double t = 1.0; t <= 600.0; t += 1.0) {
      // A steady +0.5 ppm demand: every decision pushes, and the authority
      // clamp is what turns the +2 into a step back toward +30.
      const int p = w.observe(0.5, t);
      if (p != 0) {
        worst = std::max(worst, std::abs(p));
        w.applied(p);
      }
    }
    check(worst <= c.max_push && w.offset() == c.max_offset,
          "an inherited +50 offset returns inside the authority in max_push steps (mutation: clamp the step "
          "only before the authority clamp, a -20 jump)");
  }
  // A non-finite sensor value is ignored, not averaged.
  ClockSteer z(c);
  z.start(0, 0.0);
  int got = 0;
  for (double t = 1.0; t <= 60.0; t += 1.0) got += z.observe(t == 30.0 ? NAN : 0.0, t);
  check(got == 0 && z.offset() == 0, "a NaN observation is dropped (mutation: sum it, the mean goes NaN)");
  if (failures) std::printf("FAILED: %d failure(s)\n", failures);
  return failures ? 1 : 0;
}
