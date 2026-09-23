/**
 * @file sync/clock_steer.h
 * @brief The UE's in-sounder clock steering (AP-79): the tracked grid rate as
 *        the sensor, CLOCK_ADJ on the UE as the actuator.
 *
 * WHY IN THE SOUNDER. The rig's clocks run a calibrated hold: an open-loop
 * VCXO at a fixed DAC code, so its frequency follows temperature. A session
 * warms the boards 2-3 C and moved the pair 0.3-0.4 ppm in five minutes (AP-79
 * R1/R2, 2026-09-23). The grid tracker updates about once a second and lags a
 * ramp: the re-sync residual held a -5..-14 sample bias with -46 at the onset,
 * and once the pilot seat walked 272 samples before a re-anchor. R3's zero
 * prefix is 32 samples. clock_steer_loop.py (AP-47) closed this loop from
 * outside, but it opens its own device connection, which resets a running
 * session. Inside the sounder the loop also gets the one thing the separate
 * script could not do: FEED-FORWARD. A push is a known frequency step
 * (k x 0.1251 ppm), so the caller moves the tracked period by the same step at
 * the moment of the push, and the tracker never has to chase it.
 *
 * WHERE. In the UE's sync thread, not a thread of its own: that thread owns
 * the tracked period, so the feed-forward needs no lock, and a push (one RPC,
 * a few ms) every period_s is well inside its slack.
 *
 * SIGN, the thing most likely to be got backwards (the Python loop says the
 * same): eps = (f_BS - f_UE) / f_UE, which receiver.cc computes as
 * samps_per_frame / period - 1. Raising the CLOCK_ADJ code raises f_UE and so
 * LOWERS eps: to remove a positive eps, push a positive count. The BS frame
 * then spans more UE ticks, so the period grows by the factor periodScale().
 *
 * DELIBERATELY SLOW and bounded: one push per period at most, at most
 * max_push counts, never beyond max_offset from the calibration point. A slow
 * loop is also what a later OTA link wants: on a one-way link a range rate
 * looks like a clock offset, and a fast steer would integrate motion into the
 * oscillator.
 *
 * Header-only and hardware-free: clock_steer_test closes the loop against a
 * simulated drifting clock.
 */
#pragma once

#include <algorithm>
#include <cmath>

#include "sync/sync_config.h"

namespace houdini {
namespace sync {

class ClockSteer {
 public:
  explicit ClockSteer(const ClockSteerConfig& c = ClockSteerConfig{}) : cfg_(c) {}

  /// Begin a session at `offset` counts from the calibration point (0 when
  /// the node is at its calibrated hold), at time `t_s`.
  void start(int offset, double t_s) {
    offset_ = offset;
    window_start_ = t_s;
    sum_ = 0.0;
    n_ = 0;
    started_ = true;
  }

  /// One accepted tracker observation of eps (ppm) at time `t_s`. Returns the
  /// push to apply now, in counts (0: none). The caller writes the actuator
  /// and, only if that succeeded, calls applied(); a failed write leaves the
  /// window running so the next decision retries with fresh data.
  int observe(double eps_ppm, double t_s) {
    if (!started_ || !std::isfinite(eps_ppm)) return 0;
    sum_ += eps_ppm;
    ++n_;
    if (t_s - window_start_ < cfg_.period_s || n_ < kMinSamples) return 0;
    const double mean = sum_ / n_;
    last_mean_ = mean;
    // Decide once per window, pushed or not: the next window averages only
    // what follows this decision.
    window_start_ = t_s;
    sum_ = 0.0;
    n_ = 0;
    if (std::fabs(mean) < cfg_.deadband_ppm) return 0;
    long long push = std::llround(cfg_.gain * mean / cfg_.ppm_per_count);
    push = std::max<long long>(-cfg_.max_push, std::min<long long>(cfg_.max_push, push));
    const long long target =
        std::max<long long>(-cfg_.max_offset, std::min<long long>(cfg_.max_offset, offset_ + push));
    return static_cast<int>(target - offset_);
  }

  /// The actuator took `push` counts.
  void applied(int push) {
    offset_ += push;
    pushes_ += 1;
  }

  /// The factor the tracked BS frame period (in UE ticks) moves by when the
  /// UE clock is pushed `push` counts: +1 count raises f_UE by ppm_per_count,
  /// so a BS frame spans that much MORE of the UE's ticks.
  double periodScale(int push) const { return 1.0 + static_cast<double>(push) * cfg_.ppm_per_count * 1e-6; }

  int offset() const { return offset_; }
  int pushes() const { return pushes_; }
  double lastMeanPpm() const { return last_mean_; }
  const ClockSteerConfig& config() const { return cfg_; }

  /// Tracker observations averaged per decision, at the least; about one a
  /// second arrive, so a 20 s period sees ~20.
  static constexpr int kMinSamples = 3;

 private:
  ClockSteerConfig cfg_;
  int offset_ = 0;
  int pushes_ = 0;
  double window_start_ = 0.0;
  double sum_ = 0.0;
  int n_ = 0;
  double last_mean_ = 0.0;
  bool started_ = false;
};

}  // namespace sync
}  // namespace houdini
