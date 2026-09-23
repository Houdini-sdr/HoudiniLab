/**
 * @file houdini/resync_window.h
 * @brief Where the UE places a read window so a due re-sync can see the beacon (AP-80).
 *
 * The anchored UE loop drains the RX backlog and reads a fresh one-slot window
 * each iteration, and a targeted re-sync is attempted only when the grid's next
 * predicted beacon END lands inside that window with the search slice's lead
 * and tail around it (receiver.cc: kLead <= off <= request - kTail). That
 * relied on the window's phase against the frame being random. Measured on
 * silicon (AP-79 F0 and the AP-80 run, 2026-09-23, DEMO_VERIFICATION 9.16): the
 * loop period locks to exactly two frames for 15-25 s at a time, the window
 * then holds a phase that misses the beacon, 0 windows are searched, and the UE
 * schedule free-runs until the lock breaks (the BS pilot seat walked to +901).
 *
 * The fix places the window instead: when a re-sync is due, the radio reads up
 * to a start computed from the stream's own head tick, so the predicted beacon
 * end sits `want` samples into the window whatever the loop period is.
 */
#pragma once

#include <cmath>

namespace houdini {
namespace sync {

/// The window start tick at or after `head` that puts the next predicted beacon
/// END `want` samples into the window. The grid is the loop's own:
/// frame n starts at pilot_ref + llround(n * period), and the beacon end sits
/// beacon_end ticks after a frame start. The result is never before `head`
/// (an integer x = head + want - pilot_ref - beacon_end <= n * period, and
/// llround only rounds to the nearest integer, so llround(n * period) >= x) and
/// less than one period after it. For 1 <= want <= period - 1, the loop's `off`
/// for a window stamped at the returned tick is exactly `want`.
inline long long placedWindowStart(long long head, long long pilot_ref, double period,
                                   long long beacon_end, long long want) {
  const double n = std::ceil(static_cast<double>(head + want - pilot_ref - beacon_end) / period);
  return pilot_ref + std::llround(n * period) + beacon_end - want;
}

/// The `want` the loop places at: the middle of the offsets the targeted check
/// accepts (kLead <= off and off + kTail <= request), so a few ticks of rounding
/// or a gap-shifted read cannot push the beacon out of the slice.
inline long long placedWant(long long lead, long long tail, long long request) {
  return lead + (request - lead - tail) / 2;
}

}  // namespace sync
}  // namespace houdini
