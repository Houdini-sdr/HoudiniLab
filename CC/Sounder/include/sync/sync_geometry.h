/**
 * @file sync/sync_geometry.h
 * @brief The UE beacon-sync geometry, derived in ONE place so it can be tested
 *        without a radio: the targeted-slice geometry and the resync schedule.
 *
 * A pure function of the config numbers, so what the geometry comes out as
 * at any sample rate is visible in a table (`sync_geometry_test` prints and
 * asserts it with no hardware in the loop) instead of only by running a UE
 * against silicon. Two failure classes are arithmetic of exactly that kind,
 * invisible at the one rate a bench runs:
 *
 *   - the acquisition and tracking tolerances must both be TIMES; one held in
 *     samples scales apart from the other at every other rate
 *     (DEMO_VERIFICATION 8.59);
 *   - the clamp that keeps the tracking tolerance inside the slot must leave
 *     room for the accept window: clamping against the WHOLE slot collapses
 *     the window to a single admissible read phase at every rate above
 *     ~226 MSPS, exactly the rates a time-based tolerance exists to serve
 *     (8.65).
 *
 * Two objects, because they answer two questions: SliceGeometry is what the
 * detector's targeted slice needs (lead, tail, the accept window), sized from
 * the tolerances and the REPLICA LENGTH of the configured shape (a 64-tap
 * shape needs a shorter run-up than the 128-tap gold); ResyncSchedule is when
 * to look (the cadence in frames, seconds, and the Iris/UHD frame count).
 * SyncGeometry is both, for callers that want one.
 *
 * Header-only and dependency-free on purpose: the test must not have to link
 * the sounder (and therefore SoapySDR, HDF5 and muFFT) to check arithmetic.
 */
#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>

namespace houdini {
namespace sync {

/// Reserve for the accept window, as a fraction of the slot. The clamp must
/// leave the gate a usable width, not merely a non-negative one (8.65).
constexpr long long kSyncWindowReserveDiv = 4;  // a quarter of the slot
/// The Iris/UHD frame-count cadence assumes at most this clock error, in
/// parts per billion.
constexpr double kIrisResyncMaxCfoPpb = 100.0;

/// What the targeted resync slice needs. Sample counts at the given rate.
struct SliceGeometry {
  long long scatter_tol = 0;       ///< tracking gate, samples
  long long confirm_tol = 0;       ///< acquisition gate, samples
  long long slot_cap = 0;          ///< ceiling the scatter tolerance is clamped to
  bool scatter_clamped = false;    ///< true when the tolerance hit that ceiling
  bool confirm_clamped = false;    ///< true when acquisition was pulled in to match
  long long lead = 0;              ///< samples the targeted slice needs ahead
  long long tail = 0;              ///< samples it needs behind
  long long accept_window = 0;     ///< read phases that can produce an attempt
  double accept_window_frac = 0.0; ///< that, as a fraction of the slot
  bool usable = false;             ///< false when the UE could never attempt a resync
};

/// When to look for the beacon.
struct ResyncSchedule {
  double resync_frames = 0.0;         ///< cadence in REAL frames
  double resync_interval_s = 0.0;     ///< the same cadence in seconds (what we time on)
  long long resync_period_iters = 0;  ///< the Iris/UHD frame-count cadence
};

struct SyncGeometry : SliceGeometry, ResyncSchedule {};

/// The slice geometry from the config numbers and the shape's replica length.
/// No I/O, no globals, no device. The correlator's run-up (2 replica lengths)
/// and the tail (half a replica) are genuine SAMPLE counts: the matched filter
/// needs its run-up regardless of how fast we sample, so unlike the
/// tolerances they do NOT scale with the rate.
inline SliceGeometry computeSliceGeometry(double rate_hz, long long samps_per_slot,
                                          long long samps_per_frame,
                                          long long replica_len, long long replica_tail,
                                          double scatter_tol_us,
                                          double confirm_tol_us) {
  SliceGeometry g{};
  const double rate = (rate_hz > 0.0)
                          ? rate_hz
                          : static_cast<double>(samps_per_frame) * 1e3;
  const long long slot = std::max<long long>(1, samps_per_slot);
  const long long L = std::max<long long>(1, replica_len);
  const long long tail = std::max<long long>(0, replica_tail);
  // The slice is placed by the beacon END; the matched field ends `tail`
  // samples before it, and the correlator needs two replicas of run-up
  // before THAT (a leading replica, nr_pss, is 144 samples before the end).
  const long long corr_context = tail + 2 * L;

  const long long geom = corr_context + L / 2;
  g.slot_cap = std::max<long long>(
      1, (slot - geom - slot / kSyncWindowReserveDiv) / 2);

  const long long want = std::llround(scatter_tol_us * 1e-6 * rate);
  g.scatter_clamped = want > g.slot_cap;
  g.scatter_tol = std::max<long long>(1, g.scatter_clamped ? g.slot_cap : want);
  // ACQUISITION MUST NEVER BE LOOSER THAN TRACKING. Both are times, so both
  // scale with the rate, but the tracking gate is additionally CLAMPED by the
  // slot geometry, which does not scale. Without the same bound on the
  // acquisition gate the two can invert: at 491.52 MSPS with an 8.33 us
  // tracking tolerance, the 5.21 us confirm gate is 2560 samples against a
  // tracking gate clamped to 1376, so acquisition would hand back an anchor
  // that the very first tracking check rejects as off-grid -- a lock that
  // escalates immediately, forever (AP-56).
  const long long confirm_want =
      std::max<long long>(1, std::llround(confirm_tol_us * 1e-6 * rate));
  g.confirm_tol = std::min(confirm_want, g.scatter_tol);
  g.confirm_clamped = confirm_want > g.scatter_tol;

  g.lead = g.scatter_tol + corr_context;
  g.tail = g.scatter_tol + L / 2;
  g.accept_window = slot - g.lead - g.tail;
  g.accept_window_frac =
      static_cast<double>(g.accept_window) / static_cast<double>(slot);
  g.usable = g.accept_window > 0;
  return g;
}

/// The cadence from the drift budget: look again before `sync_tol_samples` of
/// drift can accumulate at `sync_residual_ppm`.
inline ResyncSchedule computeResyncSchedule(double rate_hz, long long samps_per_frame,
                                            double sync_tol_samples,
                                            double sync_residual_ppm) {
  ResyncSchedule s{};
  const double rate = (rate_hz > 0.0)
                          ? rate_hz
                          : static_cast<double>(samps_per_frame) * 1e3;
  const long long frame = std::max<long long>(1, samps_per_frame);
  s.resync_frames =
      sync_tol_samples /
      (sync_residual_ppm * 1e-6 * static_cast<double>(frame));
  s.resync_interval_s = std::min(
      1e3, std::max(1e-3, s.resync_frames * static_cast<double>(frame) / rate));
  s.resync_period_iters = static_cast<long long>(std::max(
      1.0, std::floor(1e9 / (kIrisResyncMaxCfoPpb * static_cast<double>(frame)))));
  return s;
}

/// Both, for a caller that wants the whole table.
inline SyncGeometry computeSyncGeometry(double rate_hz, long long samps_per_slot,
                                        long long samps_per_frame,
                                        long long replica_len, long long replica_tail,
                                        double scatter_tol_us,
                                        double confirm_tol_us,
                                        double sync_tol_samples,
                                        double sync_residual_ppm) {
  SyncGeometry g{};
  static_cast<SliceGeometry&>(g) = computeSliceGeometry(
      rate_hz, samps_per_slot, samps_per_frame, replica_len, replica_tail,
      scatter_tol_us, confirm_tol_us);
  static_cast<ResyncSchedule&>(g) = computeResyncSchedule(
      rate_hz, samps_per_frame, sync_tol_samples, sync_residual_ppm);
  return g;
}

}  // namespace sync
}  // namespace houdini
