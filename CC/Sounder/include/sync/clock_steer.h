/**
 * @file sync/clock_steer.h
 * @brief The UE's in-sounder clock steering (AP-79): the tracked grid rate as
 *        the sensor, CLOCK_ADJ on the UE as the actuator. ClockSteer decides;
 *        ClockSteerSession drives the actuator and releases the node.
 *
 * WHY IN THE SOUNDER. The rig's clocks run a calibrated hold: an open-loop
 * VCXO at a fixed DAC code, so its frequency follows temperature. A session
 * warms the boards 2-3 C and moved the pair 0.3-0.4 ppm in five minutes (AP-79
 * R1/R2). The grid tracker updates once per targeted re-sync, about every
 * 2.6 s, and lags a ramp: the re-sync residual held a -5..-14 sample bias with -46 at the onset,
 * and once the pilot seat walked 272 samples before a re-anchor. R3's zero
 * prefix is 32 samples. clock_steer_loop.py (AP-47) closed this loop from
 * outside, but it opens its own device connection, which resets a running
 * session. Inside the sounder the loop also gets the one thing the separate
 * script could not do: FEED-FORWARD. A push is a known frequency step
 * (k x 0.1251 ppm), so the caller moves the tracked period by the same step at
 * the moment of the push, and the tracker never has to chase it.
 *
 * WHERE. The DECISION and the feed-forward are in the UE's sync thread, which
 * owns the tracked period, so they need no lock. The ACTUATOR write is not: a
 * CLOCK_ADJ write holds the device's stream lock about 200 ms, longer than the
 * pilot horizon, so ClockSteerSession runs it as a job the sync thread polls
 * once a frame. The same lock is what every other RPC on
 * that handle waits behind (the host pacer's time polls among them), and each
 * push is a rate step the pacer re-learns: steering stays off by default.
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
 * simulated drifting clock and drives the session against a fake device.
 */
#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <functional>
#include <future>
#include <string>
#include <system_error>

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
  /// and calls applied() with the move the node actually made (the landed
  /// code, or a readback), which can differ from the push or be none. The
  /// window closes at every decision, so a push that did not land is retried
  /// at the next one, a period later, with fresh data.
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
    // The step clamp again AFTER the authority clamp: a session that starts
    // outside the authority (an inherited offset) would otherwise be pulled
    // back in one large step.
    return static_cast<int>(std::max<long long>(-cfg_.max_push, std::min<long long>(cfg_.max_push, target - offset_)));
  }

  /// The actuator took `push` counts.
  void applied(int push) {
    offset_ += push;
    pushes_ += 1;
  }

  /// The node's offset as read back, where the one assumed was unknown.
  void resync(int offset) { offset_ = offset; }

  /// The factor the tracked BS frame period (in UE ticks) moves by when the
  /// UE clock is pushed `push` counts: +1 count raises f_UE by ppm_per_count,
  /// so a BS frame spans that much MORE of the UE's ticks.
  double periodScale(int push) const { return 1.0 + static_cast<double>(push) * cfg_.ppm_per_count * 1e-6; }

  int offset() const { return offset_; }
  int pushes() const { return pushes_; }
  double lastMeanPpm() const { return last_mean_; }

  /// Tracker observations averaged per decision, at the least; one arrives
  /// per targeted re-sync, about every 2.6 s, so a 20 s period sees about 8.
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

/// One field of a CLOCK_ADJ read ("holdover=1 man_dac=408 rb_dac=408
/// pll1_locked=1 ref=calibrated cal_dac=408 offset=0"), by its whole name;
/// "" when absent.
inline std::string clockAdjField(const std::string& st, const std::string& key) {
  const std::string k = key + "=";
  size_t p = 0;
  while ((p = st.find(k, p)) != std::string::npos) {
    if (p == 0 || st[p - 1] == ' ') {
      const size_t e = st.find(' ', p);
      return st.substr(p + k.size(), (e == std::string::npos ? st.size() : e) - p - k.size());
    }
    ++p;
  }
  return "";
}

/// A whole non-negative decimal field, or -1.
inline long clockAdjCode(const std::string& v) {
  if (v.empty()) return -1;
  char* end = nullptr;
  const long x = std::strtol(v.c_str(), &end, 10);
  return (end != nullptr && *end == '\0' && x >= 0) ? x : -1;
}

/// One UE's steering session: ClockSteer's decisions, the CLOCK_ADJ actuator
/// through `read` and `write`, each push run as a job off the caller's thread,
/// and the release at the end. The caller's thread owns the tracked period: it
/// calls poll() once a frame and scales the period by what poll() returns.
///
/// The offset is always the node's own. A write the device takes landed
/// exactly at its code: the device refuses a write whose hold did not verify
/// at that code (SoapyHoudiniSDR, WriteClockAdj and lmk::HoldLanded), and the
/// remote link returns only after the device does. A write that reports
/// failure may still have landed, or elsewhere (a link timeout, a refused
/// verify), so only that one is read back, and the offset and feed-forward
/// follow what the DAC did. When that readback fails the offset is unknown,
/// and the next decision reads the node instead of pushing from a guess (the
/// step already happened, and the tracker has been following it).
class ClockSteerSession {
 public:
  using Read = std::function<std::string()>;              ///< CLOCK_ADJ read, "" when it fails
  using Write = std::function<bool(const std::string&)>;  ///< CLOCK_ADJ write, true when taken
  using Log = std::function<void(bool warn, const std::string&)>;
  using Clock = std::function<double()>;                  ///< monotonic seconds

  ClockSteerSession(const ClockSteerConfig& c, Read read, Write write, Log log, Clock now = steadySeconds)
      : cfg_(c), steer_(c), read_(std::move(read)), write_(std::move(write)), log_(std::move(log)),
        now_(std::move(now)) {}
  ~ClockSteerSession() { release(); }
  ClockSteerSession(const ClockSteerSession&) = delete;
  ClockSteerSession& operator=(const ClockSteerSession&) = delete;

  /// Arm when steering is enabled and the node's CLOCK_ADJ reads
  /// ref=calibrated, in its hold (holdover=1), with its codes: the actuator
  /// exists only then. A calibrated node whose hold is not in force has PLL1
  /// tracking and rb_dac wandering with it, which is no offset to steer from.
  /// A node left steered by an earlier session is taken over, and released at
  /// the end like our own. Returns armed().
  bool arm() {
    if (!cfg_.enable || armed()) return armed();
    const std::string st = safeRead();
    const long cal = clockAdjCode(clockAdjField(st, "cal_dac")), rb = clockAdjCode(clockAdjField(st, "rb_dac"));
    if (clockAdjField(st, "ref") != "calibrated" || clockAdjField(st, "holdover") != "1" || cal < 0 || rb < 0) {
      say(true, "requested but OFF: CLOCK_ADJ reads '%s' (needs ref=calibrated in its hold, holdover=1, with "
                "cal_dac and rb_dac)", st.c_str());
      return false;
    }
    cal_ = static_cast<int>(cal);
    steer_.start(static_cast<int>(rb - cal), now_());
    if (rb != cal) {
      dirty_ = true;
      say(true, "the node is ALREADY steered %+ld counts from its calibration code; steering from there "
                "and releasing at exit", rb - cal);
    }
    say(false, "ON: CLOCK_ADJ %s; period %.0f s, gain %.2f, deadband %.3f ppm, max push %d, authority +-%d "
               "counts, %.4f ppm/count", st.c_str(), cfg_.period_s, cfg_.gain, cfg_.deadband_ppm, cfg_.max_push,
        cfg_.max_offset, cfg_.ppm_per_count);
    return true;
  }
  bool armed() const { return cal_ >= 0; }
  /// A job is in flight (or finished and not yet polled).
  bool busy() const { return job_.valid(); }

  /// One accepted tracker observation of eps, ppm. One job at a time: the
  /// window still closes on schedule, but a decision taken while a job is
  /// pending is dropped (the next window sees its effect).
  void onUpdate(double eps_ppm) {
    if (!armed() || released_) return;
    const int push = steer_.observe(eps_ppm, now_());
    if (push == 0 || job_.valid()) return;
    feed_forward_ = true;
    Read rd = read_;
    if (!known_) {
      // Learn where the node is before moving it again.
      launch([rd] { return Job{false, readCode(rd)}; });
      return;
    }
    const int code = cal_ + steer_.offset() + push;
    Write wr = write_;
    const bool started = launch([rd, wr, code] {
      bool wrote = false;
      try {
        wrote = wr(std::to_string(code));
      } catch (...) {
      }
      // A write the device took landed at `code`; only a failed one is read.
      return Job{true, wrote ? static_cast<long>(code) : readCode(rd), wrote};
    });
    if (!started) return;
    pending_ = push;
    ++writes_;
    dirty_ = true;  // from here the node may have moved
  }

  /// Once a frame: land a finished job. Returns the factor to scale the
  /// tracked period by: the pushed step's feed-forward, else 1.
  double poll() {
    if (!job_.valid() || job_.wait_for(std::chrono::seconds(0)) != std::future_status::ready) return 1.0;
    const Job j = job_.get();
    if (j.rb < 0) {
      known_ = false;
      say(true, "CLOCK_ADJ readback failed after %s: the offset is unknown until the next decision reads it",
          j.pushed ? "a failed push" : "a read");
      return 1.0;
    }
    const int now_offset = static_cast<int>(j.rb - cal_);
    const int moved = now_offset - steer_.offset();
    if (!j.pushed) {
      known_ = true;
      steer_.resync(now_offset);
      say(moved != 0, "CLOCK_ADJ reads back offset %+d (%+d from the last known; the tracker has followed it)",
          now_offset, moved);
      return 1.0;
    }
    if (moved != 0) steer_.applied(moved);
    const bool ff = feed_forward_ && moved != 0;
    const double scale = ff ? steer_.periodScale(moved) : 1.0;
    say(moved != pending_, "tracked eps %+.4f ppm averaged over %.0f s -> push %+d: %s %ld, offset %+d; period "
                           "scaled by %.9f%s",
        steer_.lastMeanPpm(), cfg_.period_s, pending_,
        j.wrote ? "landed at" : "the write FAILED, CLOCK_ADJ reads back", j.rb, steer_.offset(), scale,
        (moved != 0 && !feed_forward_) ? " (not fed forward: the period was replaced while it was in flight)" : "");
    return scale;
  }

  /// The caller replaced the tracked period wholesale (an escalation took a
  /// fresh confirm): a push in flight may already be in that period, so its
  /// landing is not fed forward.
  void periodReplaced() {
    if (job_.valid()) feed_forward_ = false;
  }

  /// Release the node to its calibrated hold, once; the destructor calls it.
  /// A job in flight lands first, so the release is the last word. Nothing to
  /// do when the node was never moved, or with steer.keep.
  void release() {
    if (released_ || !armed()) return;
    released_ = true;
    if (job_.valid()) {
      job_.wait();
      poll();
    }
    if (!dirty_) return;
    const std::string where = known_ ? std::to_string(steer_.offset()) + " counts" : "unknown (a readback failed)";
    if (cfg_.keep) {
      say(false, "keeping the offset at exit: %s", where.c_str());
      return;
    }
    bool ok = false;
    try {
      ok = write_("release");
    } catch (...) {
    }
    say(!ok, "%s to the calibrated hold after %d push(es), offset was %s",
        ok ? "released" : "RELEASE FAILED, node left steered", writes_, where.c_str());
  }

  /// The offset as last known (see known()).
  int offset() const { return steer_.offset(); }
  /// False from a failed readback until the next decision reads the node.
  bool known() const { return known_; }
  /// CLOCK_ADJ pushes written, whether or not they landed or were read back.
  int pushes() const { return writes_; }

  static double steadySeconds() {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
  }

 private:
  struct Job {
    bool pushed = false;  ///< a write was attempted (else a read only)
    long rb = -1;         ///< rb_dac read back, -1 when unreadable
    bool wrote = false;   ///< the write reported success
  };
  static long readCode(const Read& rd) {
    try {
      return clockAdjCode(clockAdjField(rd(), "rb_dac"));
    } catch (...) {
      return -1;
    }
  }
  std::string safeRead() {
    try {
      return read_();
    } catch (...) {
      return "";
    }
  }
  /// Start a job; false (the decision is dropped, the next window retries)
  /// when no thread can be started.
  bool launch(std::function<Job()> f) {
    try {
      job_ = std::async(std::launch::async, std::move(f));
      return true;
    } catch (const std::system_error& e) {
      if (!launch_warned_) {
        launch_warned_ = true;
        say(true, "could not start the CLOCK_ADJ job (%s): decision dropped", e.what());
      }
      return false;
    }
  }
  // printf-checked at every call (argument 1 is `this`).
  __attribute__((format(printf, 3, 4))) void say(bool warn, const char* fmt, ...) {
    if (!log_) return;
    char buf[512];
    va_list ap;
    va_start(ap, fmt);
    std::vsnprintf(buf, sizeof buf, fmt, ap);
    va_end(ap);
    log_(warn, buf);
  }

  ClockSteerConfig cfg_;
  ClockSteer steer_;
  Read read_;
  Write write_;
  Log log_;
  Clock now_;
  int cal_ = -1;              ///< the calibration DAC code; -1 = not armed
  bool known_ = true;         ///< the offset is the node's (false after a failed readback)
  bool dirty_ = false;        ///< the node may be off its calibration point
  bool feed_forward_ = true;  ///< the job in flight may feed its step forward
  bool released_ = false;
  bool launch_warned_ = false;
  int pending_ = 0;           ///< the push in flight
  int writes_ = 0;            ///< pushes written
  std::future<Job> job_;
};

}  // namespace sync
}  // namespace houdini
