/*
 Copyright (c) 2018-2026, Rice University
 RENEW OPEN SOURCE LICENSE: http://renew-wireless.org/license

----------------------------------------------------------------------
 Sample-time grid bookkeeping for the rx-recorder tool.

 The capture file promises a linear map: sample k lives at hardware
 time t0 + k/rate. TimeGridTracker anchors t0 at the first stamped
 read and, at every later stamp, compares where the read's samples
 WOULD land (the emit position) against where the timestamp says they
 BELONG. A positive difference is a stream gap (dropped packets): the
 capture loop inserts that many placeholder zeros first, so one gap
 cannot time-shift the whole remainder of the file. Extents of
 untrusted samples are recorded for the file's /Data/Gaps table.
---------------------------------------------------------------------
*/
#ifndef SOUNDER_RX_RECORDER_GRID_H_
#define SOUNDER_RX_RECORDER_GRID_H_

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

namespace Sounder {

// Why samples in a /Data/Gaps extent are untrusted.
enum GapCause : int64_t {
  kGapTimeJump = 0,    // stream gap detected via timestamps (UDP/kernel loss)
  kGapHostRing = 1,    // host ring wrapped; slot never recorded (row zeroed)
  kGapWriteError = 2,  // HDF5 write failed for the slot
  kGapBackward = 3,    // timestamp went backward (anomaly marker, n = 0)
  kGapResync = 4,      // time-base jump: grid re-anchored (marker, n = 0)
};

struct GapExtent {
  int64_t start_sample;  // first untrusted sample index in the file
  int64_t n_samples;     // extent length (0 for anomaly markers)
  int64_t cause;         // GapCause
};

// Result of checking one stamped read against the grid.
struct GridCheck {
  size_t pad_samples = 0;  // zeros to emit BEFORE the read's samples
  bool backward = false;   // timestamp moved backward (no pad)
  bool resync = false;     // time-base jump: t0 re-anchored (no pad)
};

// The one grid<->file time convention: sample index -> nanoseconds at
// `rate`. Every producer of file timestamps (the tracker's t0 anchor,
// the /Data/Gaps start_time_ns column) must use this same rounding.
inline long long sampleToNs(int64_t sample, double rate) {
  return static_cast<long long>(std::llround(sample * 1e9 / rate));
}

class TimeGridTracker {
 public:
  // Jitter tolerance: timeNs is integer nanoseconds and both the stamp and
  // the t0 anchor round independently (+-1 ns each), so above ~1 GSPS the
  // quantization exceeds one sample period — tolerate 2 ns worth of
  // samples, floor 1. (Measured: false +-2-sample backward jumps at
  // 1.966 GSPS with a fixed +-1 tolerance.) This is also the DETECTION
  // RESOLUTION: a real drop of <= tolerance_ samples is indistinguishable
  // from stamp rounding and is absorbed as a bounded standing offset until
  // cumulative drift exceeds the tolerance, at which point the FULL
  // accumulated amount pads and the grid re-aligns.
  explicit TimeGridTracker(double rate)
      : rate_(rate),
        tolerance_(std::max<int64_t>(
            1, static_cast<int64_t>(std::ceil(2e-9 * rate)))) {}

  // A jump larger than this is a hardware time-base change (resync,
  // rollover, a concurrent setHardwareTime from another tool), not sample
  // loss: padding it would zero-fill the rest of the capture. The grid
  // re-anchors instead and the caller records a kGapResync marker. Real
  // stream outages shorter than this still pad normally.
  static constexpr double kMaxGapSeconds = 10.0;

  // Check a stamped read whose first sample is about to be emitted at
  // absolute file position `position` (in samples). The first stamp
  // anchors t0 so that grid time of sample 0 = t0.
  GridCheck onStamp(long long time_ns, int64_t position) {
    GridCheck result;
    if (has_t0_ == false) {
      // Anchor: t0 = stamp time projected back to sample 0.
      t0_ = time_ns - Sounder::sampleToNs(position, rate_);
      has_t0_ = true;
      return result;
    }
    // Compute the delta in double BEFORE any llround: the sanity cap
    // below both bounds the pad and keeps an absurd stamp from feeding
    // llround an out-of-range value (UB).
    const double delta_d =
        ((time_ns - t0_) * rate_ / 1e9) - static_cast<double>(position);
    const double cap = kMaxGapSeconds * rate_;
    if ((delta_d > cap) || (delta_d < -cap)) {
      t0_ = time_ns - Sounder::sampleToNs(position, rate_);
      result.resync = true;
      return result;
    }
    const int64_t delta = static_cast<int64_t>(std::llround(delta_d));
    // |delta| <= tolerance_: timestamp rounding jitter, not a real gap.
    if (delta > tolerance_) {
      result.pad_samples = static_cast<size_t>(delta);
    } else if (delta < -tolerance_) {
      result.backward = true;
    }
    return result;
  }

  inline bool has_t0(void) const { return has_t0_; }
  inline long long t0(void) const { return t0_; }
  inline long long sampleToNs(int64_t sample) const {
    return Sounder::sampleToNs(sample, rate_);
  }

 private:
  double rate_;
  int64_t tolerance_;
  bool has_t0_ = false;
  long long t0_ = 0;
};

}; /* End namespace Sounder */

#endif /* SOUNDER_RX_RECORDER_GRID_H_ */
