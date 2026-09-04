/**
 * @file resync_policy_test.cc
 * @brief Every transition of the resync state machine, driven without a
 *        radio: the cadence (timed and frame-counted), the miss budget and
 *        the exhausted episode, escalation by episodes and by off-grid
 *        streak, the hold, the stop without an anchor, and reset.
 */
#include <chrono>
#include <cstdio>
#include <string>

#include "sync/resync_policy.h"

using houdini::sync::ResyncAction;
using houdini::sync::ResyncPolicy;
using houdini::sync::ResyncPolicyConfig;

namespace {
int g_fail = 0;
void check(bool ok, const std::string& what) {
  std::printf("%s  %s\n", ok ? "PASS" : "FAIL", what.c_str());
  if (!ok) ++g_fail;
}
using tp = ResyncPolicy::Clock::time_point;
tp at(double s) { return tp{} + std::chrono::duration_cast<tp::duration>(std::chrono::duration<double>(s)); }
}  // namespace

int main() {
  ResyncPolicyConfig cfg;
  cfg.interval_s = 2.6;
  cfg.retry_max = 3;
  cfg.escalate_episodes = 2;
  cfg.hold_offgrid = 2;

  // 1. Starts looking (the receiver's resync = true), and an alive accept
  //    stops the look and counts a success.
  {
    ResyncPolicy p(cfg, 0, at(0));
    check(p.looking(), "starts looking");
    check(p.onAlive() == ResyncAction::kNone && !p.looking() && p.successes() == 1,
          "alive: stops looking, one success");
  }
  // 2. The timed cadence opens a look after the interval, not before.
  {
    ResyncPolicy p(cfg, 0, at(0));
    p.onAlive();
    check(!p.due(1, at(1.0)) && !p.looking(), "timed: not due at 1.0 s");
    check(p.due(2, at(2.7)) && p.looking(), "timed: due at 2.7 s");
    check(!p.due(3, at(3.0)), "timed: the clock restarted at the look");
  }
  // 3. The frame-counted cadence (Iris/UHD).
  {
    ResyncPolicyConfig f = cfg;
    f.timed = false;
    f.period_frames = 5;
    ResyncPolicy p(f, 0, at(0));
    p.onAccept();
    check(!p.due(4, at(0)) && p.due(5, at(0)) && p.looking(), "frames: due at period frames");
  }
  // 4. Disabled: due() never opens a look.
  {
    ResyncPolicyConfig d = cfg;
    d.enabled = false;
    ResyncPolicy p(d, 0, at(0));
    p.onAlive();
    check(!p.due(1, at(10.0)) && !p.looking(), "disabled: never looks again");
  }
  // 5. Misses: retry_max misses are tolerated, one more is an exhausted
  //    episode; two episodes escalate; an alive accept clears the streak.
  {
    ResyncPolicy p(cfg, 0, at(0));
    for (size_t i = 0; i < cfg.retry_max; ++i)
      check(p.onMiss(true) == ResyncAction::kNone && p.looking(), "miss " + std::to_string(i + 1) + ": still looking");
    check(p.onMiss(true) == ResyncAction::kExhausted && !p.looking() && p.retries() == 0 &&
              p.exhaustedStreak() == 1,
          "miss retry_max + 1: exhausted, look closed, retries reset");
    p.due(1, at(3.0));
    for (size_t i = 0; i < cfg.retry_max; ++i) p.onMiss(true);
    check(p.onMiss(true) == ResyncAction::kEscalate, "second consecutive exhausted episode escalates");
    p.reset(2, at(3.1));
    check(p.exhaustedStreak() == 0 && !p.looking() && p.retries() == 0, "reset clears the streak and the look");
    p.due(3, at(6.0));
    for (size_t i = 0; i <= cfg.retry_max; ++i) p.onMiss(true);
    p.due(4, at(9.0));
    p.onAlive();
    check(p.exhaustedStreak() == 0, "an alive accept clears the exhausted streak");
  }
  // 6. Without an anchor an exhausted episode stops the run.
  {
    ResyncPolicy p(cfg, 0, at(0));
    for (size_t i = 0; i < cfg.retry_max; ++i) p.onMiss(false);
    check(p.onMiss(false) == ResyncAction::kStop, "no anchor: exhausted means stop");
  }
  // 7. Off-grid: one is held, hold_offgrid consecutive escalate; an alive
  //    accept in between clears the streak.
  {
    ResyncPolicy p(cfg, 0, at(0));
    check(p.onOffGrid() == ResyncAction::kHold && p.holdPending() && p.looking() && p.offgridStreak() == 1,
          "one off-grid: held, still looking");
    check(p.onAlive() == ResyncAction::kNone && p.offgridStreak() == 0 && !p.holdPending(),
          "alive after a hold clears it");
    p.due(1, at(3.0));
    p.onOffGrid();
    check(p.onOffGrid() == ResyncAction::kEscalate, "two consecutive off-grid: escalate");
  }
  // 8. The untargeted accept (no grid) closes the look and counts.
  {
    ResyncPolicy p(cfg, 0, at(0));
    p.onMiss(true);
    check(p.onAccept() == ResyncAction::kNone && !p.looking() && p.retries() == 0 && p.successes() == 1,
          "untargeted accept: look closed, retries reset, one success");
  }
  std::printf("\nRESULT: %s (%d failure(s))\n", g_fail ? "FAIL" : "PASS", g_fail);
  return g_fail ? 1 : 0;
}
