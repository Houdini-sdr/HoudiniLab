/**
 * @file houdini/pilot_ladder.h
 * @brief Where the UE's seated pilot-burst ladder resumes on each call (AP-79).
 *
 * The UE schedules one burst per frame at txTime + i * frame_d, for integer i,
 * up to a horizon, and remembers the last burst it queued (the cursor). Each
 * call re-derives txTime from the tracked beacon grid, so the new txTime is NOT
 * on the old call's lattice: it lands a fraction of a sample either side.
 *
 * The rule this replaces resumed at the first i with cur >= cursor + frame
 * (ceil). When txTime lands early by any fraction, that ceil rounds past the
 * next frame and SKIPS it. Measured on silicon (AP-79 R0 and R1, 2026-09-23):
 * the UE covered only 74-75 % of the frames, and the BS logged every missing
 * frame as "no UE burst". The fix resumes at the first i whose burst starts more
 * than HALF a frame past the cursor, which is the next frame for any grid
 * jitter under half a frame, and can never queue the cursor's own frame twice.
 */
#pragma once

#include <cmath>

namespace houdini {
namespace ladder {

/// First ladder index i (cur = tx_time + i * frame_d) to schedule after the
/// burst queued at `cursor`; 0 when nothing queued lies ahead of tx_time.
inline long long resumeIndex(long long cursor, long long tx_time, double frame_d) {
  const double next = static_cast<double>(cursor) + 0.5 * frame_d;
  if (next <= static_cast<double>(tx_time)) return 0;
  return static_cast<long long>(std::floor((next - static_cast<double>(tx_time)) / frame_d)) + 1;
}

}  // namespace ladder
}  // namespace houdini
