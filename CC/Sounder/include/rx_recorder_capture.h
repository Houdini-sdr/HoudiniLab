/*
 Copyright (c) 2018-2026, Rice University
 RENEW OPEN SOURCE LICENSE: http://renew-wireless.org/license

----------------------------------------------------------------------
 Slot-assembly state machine for the rx-recorder capture loop.

 fillSlot() assembles fixed-size recording slots from a SampleSource —
 the abstraction over HOW samples arrive from the driver: readStream
 into a staging chunk, or the zero-copy acquire/release direct-buffer
 API (one acquire = one wire packet = the atomic unit of time
 continuity). The grid bookkeeping is shared and source-independent:
 placeholder zeros owed from detected gaps are emitted first, then
 buffered source samples, then fresh fetches, so one stream gap can
 never time-shift the rest of the file (see rx_recorder_grid.h and
 docs/RX_GAP_AWARENESS.md).

 Deliberately SoapySDR-free: the radio-free unit tests drive fillSlot
 with a scripted SampleSource (tests/rx-recorder/test_capture_paths.cc);
 the device-facing sources live in rx_recorder_main.cc.
---------------------------------------------------------------------
*/
#ifndef SOUNDER_RX_RECORDER_CAPTURE_H_
#define SOUNDER_RX_RECORDER_CAPTURE_H_

#include <algorithm>
#include <atomic>
#include <cstdint>
#include <cstring>
#include <functional>
#include <vector>

#include "logger.h"
#include "rx_recorder_grid.h"

namespace Sounder {

// Outcome of SampleSource::fetch(), mapped from the driver's return
// convention by each source (kept SoapySDR-free for the tests).
enum class FetchStatus {
  kData,      // a new span is available via data()/pending()/time
  kTimeout,   // no data within the source's timeout (or a 0-sample return)
  kOverflow,  // driver reported overflow; no data this call
  kFatal,     // unrecoverable error, already logged by the source
};

// One span of contiguous CS16 samples from the driver, plus the stamp of
// its first sample. pending() == 0 means a new fetch() is needed;
// consume() advances within the span. A source with borrow semantics
// (direct buffers) releases the underlying ring slot once its span is
// fully consumed, so at most one buffer is ever held.
class SampleSource {
 public:
  virtual ~SampleSource() = default;
  // Advance to the next span. Precondition: pending() == 0.
  virtual FetchStatus fetch(void) = 0;
  // Samples remaining in the current span (0 = fetch needed).
  virtual size_t pending(void) const = 0;
  // The next unconsumed sample (interleaved I,Q shorts). Valid while
  // pending() > 0; invalidated by consume() reaching 0 and by fetch().
  virtual const short* data(void) const = 0;
  virtual void consume(size_t samples) = 0;
  // Stamp of the current span's FIRST sample; valid after fetch()==kData.
  virtual bool has_time(void) const = 0;
  virtual long long time_ns(void) const = 0;
};

struct CaptureStats {
  size_t slots_recorded = 0;
  size_t slots_dropped = 0;  // ring full: HDF5 writer could not keep up
  size_t read_timeouts = 0;
  size_t overflow_returns = 0;     // OVERFLOW returned by the source itself
  size_t gap_events = 0;           // stream gaps detected via timestamps
  size_t backward_time_jumps = 0;  // timeNs went backward (quantization)
  size_t time_resyncs = 0;         // time-base jumps: grid re-anchored
  double est_lost_samples = 0.0;   // sum of inserted placeholder samples
  size_t status_overflows = 0;     // OVERFLOW events from readStreamStatus
  size_t status_other = 0;
  long long first_sample_time_ns = 0;  // grid time of file sample 0
  bool has_first_sample_time = false;
};

// Capture-thread bookkeeping threaded through fillSlot: the sample-time
// grid, the untrusted-extent list, and the pad machinery that lets
// placeholder zeros be inserted BEFORE a late-arriving span's samples.
struct CaptureState {
  explicit CaptureState(double rate) : grid(rate) {}

  CaptureStats stats;
  TimeGridTracker grid;
  std::vector<GapExtent> extents;
  int64_t grid_pos = 0;        // absolute samples emitted onto the grid
  size_t pad_remaining = 0;    // placeholder zeros still owed to the grid
  size_t last_fetch_samps = 0;  // uncertainty span for widened extents
  // Every source span is time-contiguous (driver breaks reads at gaps,
  // or one acquire = one packet), so a detected gap lies exactly at the
  // span boundary. Without it a mid-span gap is located only to the
  // enclosing fetch, so extents widen by last_fetch_samps. See
  // docs/RX_GAP_AWARENESS.md.
  bool exact_gaps = false;
};

// Abort thresholds for a source that stops making progress. Overflow
// returns are instant (event pops, no wait): a driver wedged in a
// permanent overflow state would otherwise spin fillSlot forever.
constexpr size_t kMaxConsecutiveTimeouts = 10;
constexpr size_t kMaxConsecutiveOverflows = 10000;

// Checks a stamped span against the grid: anchors t0, schedules placeholder
// zeros for forward gaps, records untrusted extents.
inline void onStampedChunk(long long time_ns, CaptureState& state) {
  const bool first = (state.grid.has_t0() == false);
  const Sounder::GridCheck check = state.grid.onStamp(time_ns, state.grid_pos);
  if (first == true) {
    state.stats.first_sample_time_ns = state.grid.t0();
    state.stats.has_first_sample_time = true;
    return;
  }
  if (check.resync == true) {
    // Hardware time-base changed under us: absolute time alignment is
    // broken from here on (the marker records where); the grid continues
    // relative to the new anchor instead of zero-filling the capture.
    state.stats.time_resyncs++;
    state.extents.push_back({state.grid_pos, 0, Sounder::kGapResync});
    return;
  }
  if (check.pad_samples > 0) {
    state.stats.gap_events++;
    state.stats.est_lost_samples += static_cast<double>(check.pad_samples);
    int64_t start = state.grid_pos;
    if (state.exact_gaps == false) {
      // Gap position is only known to the enclosing fetch: widen the
      // untrusted extent backward over the previous span's samples.
      start = std::max<int64_t>(
          0, state.grid_pos - static_cast<int64_t>(state.last_fetch_samps));
    }
    const int64_t length =
        static_cast<int64_t>(check.pad_samples) + (state.grid_pos - start);
    state.extents.push_back({start, length, Sounder::kGapTimeJump});
    state.pad_remaining = check.pad_samples;
  } else if (check.backward == true) {
    state.stats.backward_time_jumps++;
    state.extents.push_back({state.grid_pos, 0, Sounder::kGapBackward});
  }
}

// Fills dest with exactly slot_samps grid samples: placeholder zeros
// owed from detected gaps, the source's buffered span, then fresh
// fetches. `interrupted` is polled every iteration (signal handler).
// Returns false when the capture should stop (fatal error, shutdown,
// or interruption); fatal errors also latch running = false.
inline bool fillSlot(SampleSource& src, short* dest, size_t slot_samps,
                     CaptureState& state, std::atomic<bool>& running,
                     const std::function<bool(void)>& interrupted) {
  size_t filled = 0;
  size_t consecutive_timeouts = 0;
  size_t consecutive_overflows = 0;

  while ((filled < slot_samps) && (running.load() == true) &&
         (interrupted() == false)) {
    // 1) Placeholder zeros owed to the grid from a detected gap.
    if (state.pad_remaining > 0) {
      const size_t n = std::min(state.pad_remaining, slot_samps - filled);
      std::memset(dest + (2 * filled), 0, n * 2 * sizeof(short));
      filled += n;
      state.grid_pos += static_cast<int64_t>(n);
      state.pad_remaining -= n;
      continue;
    }
    // 2) Buffered samples from the source's current span.
    if (src.pending() > 0) {
      const size_t n = std::min(src.pending(), slot_samps - filled);
      std::memcpy(dest + (2 * filled), src.data(), n * 2 * sizeof(short));
      src.consume(n);
      filled += n;
      state.grid_pos += static_cast<int64_t>(n);
      continue;
    }
    // 3) Fetch a fresh span.
    const FetchStatus status = src.fetch();
    if (status == FetchStatus::kTimeout) {
      state.stats.read_timeouts++;
      if (++consecutive_timeouts >= kMaxConsecutiveTimeouts) {
        MLPD_ERROR("capture: %zu consecutive timeouts, aborting capture\n",
                   consecutive_timeouts);
        running = false;
        return false;
      }
      continue;
    }
    if (status == FetchStatus::kOverflow) {
      state.stats.overflow_returns++;
      consecutive_timeouts = 0;  // the stream is alive, just lossy
      if (++consecutive_overflows >= kMaxConsecutiveOverflows) {
        MLPD_ERROR(
            "capture: %zu consecutive overflows with no data, aborting "
            "capture\n",
            consecutive_overflows);
        running = false;
        return false;
      }
      continue;
    }
    if (status == FetchStatus::kFatal) {
      running = false;
      return false;
    }
    consecutive_timeouts = 0;
    consecutive_overflows = 0;
    if (src.has_time() == true) {
      // May schedule pads: the fetched span then emits AFTER them, which
      // places its first sample exactly where the timestamp says it belongs.
      onStampedChunk(src.time_ns(), state);
    }
    state.last_fetch_samps = src.pending();
  }
  return (filled == slot_samps);
}

}; /* End namespace Sounder */

#endif /* SOUNDER_RX_RECORDER_CAPTURE_H_ */
