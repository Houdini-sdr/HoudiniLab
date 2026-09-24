// AP-79: the in-sounder clock steering loop (sync/clock_steer.h), closed
// against a simulated UE clock. Each assertion names the mutation that breaks it.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <mutex>
#include <random>
#include <string>
#include <thread>
#include <vector>

#include "sync/clock_steer.h"

using houdini::sync::ClockSteer;
using houdini::sync::ClockSteerConfig;
using houdini::sync::ClockSteerSession;

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

// A UE's CLOCK_ADJ as the device reports it, with the faults a session must
// survive: a slow write, a write that reports failure yet lands (a link
// timeout), one that lands short of the request (which the device reports as
// a failure: it verifies the landed code), and reads that fail.
struct FakeNode {
  std::mutex m;
  int cal = 408, dac = 408;
  std::string ref = "calibrated";
  bool held = true;  // holdover=1: the calibrated hold is in force
  int write_delay_ms = 0;
  bool write_result = true;
  int land_short = 0;  // pair with write_result = false, as the device reports it
  int fail_reads = 0;  // the next this-many reads return ""
  int reads = 0;
  std::vector<std::string> writes;
  std::string read() {
    std::lock_guard<std::mutex> lk(m);
    ++reads;
    if (fail_reads > 0) {
      --fail_reads;
      return "";
    }
    return std::string("holdover=") + (held ? "1" : "0") + " man_dac=" + std::to_string(dac) + " rb_dac=" + std::to_string(dac) +
           " pll1_locked=1 ref=" + ref + " cal_dac=" + std::to_string(cal) + " offset=" + std::to_string(dac - cal);
  }
  bool write(const std::string& v) {
    // Only the push is slow: a slow release would let a push still in flight
    // win the lock anyway, and hide a release that does not wait for it.
    if (v != "release") std::this_thread::sleep_for(std::chrono::milliseconds(write_delay_ms));
    std::lock_guard<std::mutex> lk(m);
    writes.push_back(v);
    dac = (v == "release") ? cal : std::stoi(v) - land_short;
    return write_result;
  }
  std::vector<std::string> written() {
    std::lock_guard<std::mutex> lk(m);
    return writes;
  }
};

struct Bench {
  FakeNode node;
  double t = 0.0;
  std::vector<std::string> log;
  ClockSteerConfig cfg;
  Bench() {
    cfg.enable = true;
    cfg.period_s = 10.0;
  }
  ClockSteerSession make() {
    return ClockSteerSession(
        cfg, [this] { return node.read(); }, [this](const std::string& v) { return node.write(v); },
        [this](bool, const std::string& msg) { log.push_back(msg); }, [this] { return t; });
  }
  // One decision window: four observations 3 s apart close a 10 s window.
  void decide(ClockSteerSession& s, double eps_ppm) {
    for (int i = 0; i < 4; ++i) {
      t += 3.0;
      s.onUpdate(eps_ppm);
    }
  }
};

// Wait for the job in flight and return what its landing fed forward.
static double landed(ClockSteerSession& s) {
  double scale = 1.0;
  while (s.busy()) {
    scale = s.poll();
    if (s.busy()) std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  return scale;
}

// eps 0.36 ppm asks for +2 counts at gain 0.7 and 0.1251 ppm/count; 0.18 for +1.
static void sessionTests() {
  using houdini::sync::clockAdjCode;
  using houdini::sync::clockAdjField;
  const std::string st = "holdover=1 man_dac=410 rb_dac=410 pll1_locked=1 ref=calibrated cal_dac=408 offset=2";
  // Fails under: matching the key anywhere (every "dac=" here is the tail of a longer name).
  check(clockAdjField(st, "rb_dac") == "410" && clockAdjField(st, "dac").empty(),
        "CLOCK_ADJ fields are read by their whole name (mutation: no word boundary, 'dac' reads 410)");
  // Fails under: dropping the end-of-string check (408x reads 408) or the sign check (-3 reads -3).
  check(clockAdjCode("408") == 408 && clockAdjCode("408x") == -1 && clockAdjCode("-3") == -1 &&
            clockAdjCode("none") == -1 && clockAdjCode("") == -1,
        "a CLOCK_ADJ code is a whole non-negative decimal (mutation: accept trailing text or a sign)");
  const double k1 = 1.0 + 1 * 0.1251e-6, k2 = 1.0 + 2 * 0.1251e-6;
  {
    Bench b;
    b.cfg.enable = false;
    {
      auto s = b.make();
      // Fails under: arm() ignoring steer.enable (it reads the node and arms).
      check(!s.arm() && b.node.reads == 0, "steering off: arm() reads nothing and stays off");
    }
    // Two guards cover this (release() returns when not armed, and nothing
    // was moved), so no single mutation breaks it: it pins the pair.
    check(b.node.written().empty(), "steering off: nothing written at exit");
  }
  {
    Bench b;
    b.node.ref = "internal";
    auto s = b.make();
    // Fails under: arming on any ref (the actuator exists only under calibrated).
    check(!s.arm(), "a node not on ref=calibrated is not armed");
  }
  {
    Bench b;
    b.node.held = false;  // calibrated, but PLL1 tracking: rb_dac is no offset
    auto s = b.make();
    // Fails under: dropping the holdover=1 requirement from arm().
    check(!s.arm(), "a calibrated node whose hold is not in force is not armed");
  }
  {
    Bench b;
    {
      auto s = b.make();
      s.arm();
      b.decide(s, 0.01);  // inside the deadband: no push
    }
    // Fails under: releasing whenever armed (a clean node gets a needless write).
    check(b.node.written().empty(), "a clean session that never pushed writes nothing at exit");
  }
  {
    Bench b;
    b.node.dac = 412;  // left steered +4 by an earlier session
    {
      auto s = b.make();
      check(s.arm() && s.offset() == 4, "an inherited offset is taken over at arm");
    }
    // Fails under: not marking an inherited offset for release.
    check(b.node.written() == std::vector<std::string>{"release"}, "an inherited offset is released at exit");
  }
  {
    Bench b;
    auto s = b.make();
    s.arm();
    b.decide(s, 0.36);
    const double sc = landed(s);
    // Fails under: flipping the push's sign (writes 406) or the feed-forward's
    // (periodScale(-2)).
    check(b.node.written() == std::vector<std::string>{"410"} && s.offset() == 2 && std::fabs(sc - k2) < 1e-15,
          "a push of +2 writes cal+2 and feeds forward exactly periodScale(2)");
    // Fails under: reading CLOCK_ADJ back after a write the device took (one
    // more RPC under the stream lock per push; only arm() reads here).
    check(b.node.reads == 1, "a write the device took is not read back");
  }
  {
    Bench b;
    b.node.land_short = 1;  // the DAC lands one count short of the request,
    b.node.write_result = false;  // which the device reports as a failed write
    auto s = b.make();
    s.arm();
    b.decide(s, 0.36);
    const double sc = landed(s);
    // Fails under: taking the requested push (+2) instead of the read-back move (+1).
    check(s.offset() == 1 && std::fabs(sc - k1) < 1e-15,
          "the offset and the feed-forward follow the readback, not the request (mutation: offset += push)");
  }
  {
    Bench b;
    b.node.write_result = false;  // the write reports failure, yet the DAC moved
    auto s = b.make();
    s.arm();
    b.decide(s, 0.36);
    landed(s);
    // Fails under: applying a push only when its write reported success.
    check(s.offset() == 2, "a write that reports failure but lands still moves the offset");
  }
  {
    // The review's case: the write reports failure yet lands (+2), and its
    // readback fails.
    Bench b;
    b.node.write_result = false;
    auto s = b.make();
    s.arm();
    b.node.fail_reads = 1;
    b.decide(s, 0.36);
    const double sc1 = landed(s);
    b.decide(s, 0.18);  // this decision reads the node instead of pushing
    const double sc2 = landed(s);
    const int after_read = s.offset();
    b.decide(s, 0.18);  // and this one pushes from where the node really is
    landed(s);
    // Fails under: pushing on from the stale offset after a failed readback (the
    // second write is then 409, a step DOWN from the real 410, fed forward UP).
    check(b.node.written() == std::vector<std::string>{"410", "411"} && after_read == 2 && s.offset() == 3,
          "after a failed readback the next decision reads the node, then pushes from there (writes 410, 411)");
    // Fails under: feeding forward the move the read found (it happened a window ago).
    check(sc1 == 1.0 && sc2 == 1.0, "neither the unconfirmed push nor the read that found it is fed forward");
  }
  {
    Bench b;
    b.node.write_delay_ms = 150;
    auto s = b.make();
    s.arm();
    b.decide(s, 0.36);
    b.decide(s, 0.36);  // decided while the first write is still in flight
    landed(s);
    // Fails under: launching a job while one is pending (two writes).
    check(b.node.written().size() == 1, "one push in flight: a decision taken while one is pending is dropped");
  }
  {
    Bench b;
    b.node.write_delay_ms = 100;
    auto s = b.make();
    s.arm();
    b.decide(s, 0.36);
    s.periodReplaced();  // an escalation took a fresh confirm meanwhile
    const double sc = landed(s);
    // Fails under: periodReplaced() doing nothing (the step is counted twice).
    check(sc == 1.0 && s.offset() == 2, "a push in flight when the period is replaced lands without feed-forward");
    b.decide(s, 0.18);
    const double sc2 = landed(s);
    // Fails under: onUpdate not re-arming the feed-forward (every push after
    // one replacement would land unfed).
    check(std::fabs(sc2 - k1) < 1e-15 && s.offset() == 3, "the next push is fed forward again");
  }
  {
    Bench b;
    b.node.write_result = false;  // the write reports failure yet lands,
    {
      auto s = b.make();
      s.arm();
      b.node.fail_reads = 1;  // and its readback fails: the offset is unknown
      b.decide(s, 0.36);
      landed(s);
      check(!s.known(), "a failed push whose readback fails leaves the offset unknown");
    }
    // Fails under: skipping the release while the offset is unknown (the node
    // stays at +2 for every later run).
    check(b.node.written() == std::vector<std::string>{"410", "release"} && b.node.dac == b.node.cal,
          "a session that ends with the offset unknown still releases the node");
  }
  {
    Bench b;
    b.cfg.keep = true;
    {
      auto s = b.make();
      s.arm();
      b.decide(s, 0.36);
      landed(s);
    }
    // Fails under: ignoring steer.keep at exit.
    check(b.node.written() == std::vector<std::string>{"410"}, "steer.keep leaves the steered code at exit");
  }
  {
    Bench b;
    b.node.write_delay_ms = 100;
    {
      auto s = b.make();
      s.arm();
      b.decide(s, 0.36);
      s.release();  // the job is still writing
    }
    // Fails under: releasing without waiting for the job (release lands first,
    // then the push re-steers the node), or a second release from the destructor.
    check(b.node.written() == std::vector<std::string>{"410", "release"} && b.node.dac == b.node.cal,
          "a job still running at exit lands first, then one release (the node ends at its calibration code)");
  }
}

int main() {
  sessionTests();
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
