/**
 * @file sync/resync_policy.h
 * @brief The UE's resync state machine as one object: when to look for the
 *        beacon, how many misses make an exhausted episode, how many episodes
 *        or off-grid detections make an escalation, and what a caller should
 *        do next.
 *
 * Extracted from Receiver::clientSyncTxRx (architecture review 2026-09-03,
 * item 18), where the same rules lived in fifteen loop-local variables and a
 * lambda that did `continue` on the caller's loop. The transitions are the
 * ones the ledger paid for and nothing here changes them:
 *
 *   - LOOK when the cadence says so: on Houdini a wall-clock interval
 *     (ResyncSchedule::resync_interval_s, 8.136); on Iris/UHD a frame count.
 *   - An on-grid accept clears every streak (the grid is alive, 4.18).
 *   - An off-grid accept is HELD, not applied: one is scatter, `hold_offgrid`
 *     consecutive ones mean the beacon MOVED and the caller re-acquires
 *     (AP-52 [user 2026-08-30]).
 *   - `retry_max` misses in one period is an EXHAUSTED episode: under the
 *     targeted search an attempt only counts when the grid predicted the
 *     beacon inside the window, so one episode is ~100 predicted positions in
 *     a row with nothing there; `escalate_episodes` consecutive episodes mean
 *     the beacon is LOST and the caller re-acquires (AP-18, Opus review M4).
 *     Without an anchored grid (Iris/UHD, or before the first confirm) an
 *     exhausted episode STOPS the run, as the original receiver did.
 *   - After the caller re-acquires it calls reset(), which is exactly what
 *     the old escalation lambda did to the counters.
 *
 * The object holds counters and clocks only; it never touches a radio, so
 * resync_policy_test drives every transition without one.
 */
#pragma once

#include <chrono>
#include <cstddef>

namespace houdini {
namespace sync {

/// What the caller should do after an event.
enum class ResyncAction {
  kNone,        ///< carry on (looking or not, see looking())
  kHold,        ///< off-grid detection held, keep looking, do not apply it
  kEscalate,    ///< re-acquire the anchor, then reset()
  kExhausted,   ///< this period's misses are spent; look again next period
  kStop,        ///< no anchored grid to fall back on: the run is over
};

struct ResyncPolicyConfig {
  bool enabled = true;                 ///< frame_mode == continuous_resync
  bool timed = true;                   ///< Houdini: wall-clock cadence; else frames
  double interval_s = 2.6;             ///< the timed cadence
  size_t period_frames = 1;            ///< the frame-count cadence (Iris/UHD)
  size_t retry_max = 100;              ///< misses per period before exhausted
  size_t escalate_episodes = 2;        ///< exhausted episodes before escalation
  size_t hold_offgrid = 2;             ///< off-grid detections before escalation
};

class ResyncPolicy {
 public:
  using Clock = std::chrono::steady_clock;

  ResyncPolicy(const ResyncPolicyConfig& cfg, size_t frame_now, Clock::time_point now)
      : cfg_(cfg), last_frame_(frame_now), last_tp_(now) {}

  /// The cadence: called once per frame BEFORE the search. Returns true when
  /// this call opened a look (the caller may log it).
  bool due(size_t frame_now, Clock::time_point now) {
    if (cfg_.timed) {
      if (std::chrono::duration<double>(now - last_tp_).count() >= cfg_.interval_s) {
        looking_ = cfg_.enabled;
        last_tp_ = now;
        return looking_;
      }
      return false;
    }
    if (frame_now - last_frame_ >= cfg_.period_frames) {
      looking_ = cfg_.enabled;
      last_frame_ = frame_now;
      return looking_;
    }
    return false;
  }
  bool looking() const { return looking_; }

  /// A detection accepted ON the tracked grid.
  ResyncAction onAlive() {
    hold_pending_ = false;
    offgrid_streak_ = 0;
    exhausted_streak_ = 0;
    looking_ = false;
    retry_ = 0;
    ++successes_;
    return ResyncAction::kNone;
  }
  /// A detection accepted OFF the tracked grid.
  ResyncAction onOffGrid() {
    ++offgrid_streak_;
    if (offgrid_streak_ >= cfg_.hold_offgrid) return ResyncAction::kEscalate;
    hold_pending_ = true;
    return ResyncAction::kHold;  // stay looking: a moved beacon repeats, noise does not
  }
  /// A detection accepted with no tracked grid to judge it against (the
  /// untargeted search: Iris/UHD, or Houdini before the first confirm).
  ResyncAction onAccept() {
    looking_ = false;
    retry_ = 0;
    ++successes_;
    return ResyncAction::kNone;
  }
  /// An attempt that found nothing. `anchored` says whether a tracked grid
  /// keeps the pilots seated meanwhile (an exhausted episode is then survivable).
  ResyncAction onMiss(bool anchored) {
    ++retry_;
    if (retry_ <= cfg_.retry_max) return ResyncAction::kNone;
    looking_ = false;
    retry_ = 0;
    if (!anchored) return ResyncAction::kStop;
    if (++exhausted_streak_ >= cfg_.escalate_episodes) return ResyncAction::kEscalate;
    return ResyncAction::kExhausted;
  }
  /// After the caller re-acquired (or tried to): every streak and the cadence
  /// restart, exactly as the old escalation did.
  void reset(size_t frame_now, Clock::time_point now) {
    exhausted_streak_ = 0;
    hold_pending_ = false;
    offgrid_streak_ = 0;
    looking_ = false;
    retry_ = 0;
    last_frame_ = frame_now;
    last_tp_ = now;
  }

  size_t retries() const { return retry_; }
  size_t successes() const { return successes_; }
  size_t offgridStreak() const { return offgrid_streak_; }
  size_t exhaustedStreak() const { return exhausted_streak_; }
  bool holdPending() const { return hold_pending_; }
  const ResyncPolicyConfig& config() const { return cfg_; }

 private:
  ResyncPolicyConfig cfg_;
  bool looking_ = true;  // the original receiver started with resync = true
  size_t retry_ = 0;
  size_t successes_ = 0;
  size_t offgrid_streak_ = 0;
  size_t exhausted_streak_ = 0;
  bool hold_pending_ = false;
  size_t last_frame_;
  Clock::time_point last_tp_;
};

}  // namespace sync
}  // namespace houdini
