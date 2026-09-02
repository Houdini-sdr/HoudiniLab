/**
 * @file sync_geometry.h
 * @brief The UE beacon-sync geometry, derived in ONE place so it can be tested
 *        without a radio.
 *
 * Every constant here used to be computed inline inside clientSyncTxRx, from
 * three formulas spread across two hundred lines, and the only way to see what
 * they came out as at a given sample rate was to run a UE against live silicon.
 * Two defects lived in that gap and neither was reachable from the bench:
 *
 *   - the acquisition tolerance stayed a fixed 640 SAMPLES while the tracking
 *     tolerance had been converted to TIME, so the two gates scaled apart at
 *     every rate but the one we happened to run (DEMO_VERIFICATION 8.59);
 *   - the clamp that keeps the tracking tolerance inside the slot used the
 *     WHOLE slot, so the moment it fired the accept window collapsed to a
 *     single admissible read phase -- at every rate above ~226 MSPS, i.e. at
 *     exactly the rates the time-based tolerance exists to serve (8.65).
 *
 * Both are arithmetic. Both are visible at a glance in a table of rates. So the
 * derivation is a pure function of the config numbers, and `sync_geometry_test`
 * prints and asserts that table with no hardware in the loop.
 *
 * Header-only and dependency-free on purpose: the test must not have to link
 * the sounder (and therefore SoapySDR, HDF5 and muFFT) to check arithmetic.
 */
#ifndef SYNC_GEOMETRY_H_
#define SYNC_GEOMETRY_H_

#include <algorithm>
#include <cmath>
#include <cstddef>

namespace Sounder {

// The gold sequence is 128 taps. Both of these are genuine SAMPLE counts: the
// matched filter needs its run-up regardless of how fast we sample, so unlike
// the tolerances they do NOT scale with the rate.
constexpr int kSyncGoldLen = 128;
constexpr long long kSyncCorrContext = 2 * kSyncGoldLen;  // correlator run-up
// Reserve for the accept window, as a fraction of the slot. The clamp must
// leave the gate a usable width, not merely a non-negative one (8.65).
constexpr long long kSyncWindowReserveDiv = 4;  // a quarter of the slot

struct SyncGeometry {
  long long scatter_tol;       ///< tracking gate, samples
  long long confirm_tol;       ///< acquisition gate, samples
  long long slot_cap;          ///< ceiling the scatter tolerance is clamped to
  bool scatter_clamped;        ///< true when the tolerance hit that ceiling
  bool confirm_clamped;        ///< true when acquisition was pulled in to match
  long long lead;              ///< samples the targeted slice needs ahead
  long long tail;              ///< samples it needs behind
  long long accept_window;     ///< read phases that can produce an attempt
  double accept_window_frac;   ///< that, as a fraction of the slot
  double resync_frames;        ///< cadence in REAL frames
  double resync_interval_s;    ///< the same cadence in seconds (what we time on)
  long long resync_period_iters;  ///< the Iris/UHD frame-count cadence
  bool usable;                 ///< false when the UE could never attempt a resync
};

/// All of it from the config numbers. No I/O, no globals, no device.
inline SyncGeometry computeSyncGeometry(double rate_hz, long long samps_per_slot,
                                        long long samps_per_frame,
                                        double scatter_tol_us,
                                        double confirm_tol_us,
                                        double sync_tol_samples,
                                        double sync_residual_ppm) {
  SyncGeometry g{};
  const double rate = (rate_hz > 0.0)
                          ? rate_hz
                          : static_cast<double>(samps_per_frame) * 1e3;
  const long long slot = std::max<long long>(1, samps_per_slot);
  const long long frame = std::max<long long>(1, samps_per_frame);

  const long long geom =
      static_cast<long long>(2 * kSyncGoldLen + kSyncGoldLen / 2);
  g.slot_cap = std::max<long long>(
      1, (slot - geom - slot / kSyncWindowReserveDiv) / 2);

  const long long want = std::llround(scatter_tol_us * 1e-6 * rate);
  g.scatter_clamped = want > g.slot_cap;
  g.scatter_tol = std::max<long long>(1, g.scatter_clamped ? g.slot_cap : want);
  // ACQUISITION MUST NEVER BE LOOSER THAN TRACKING. Both are times now, so
  // both scale with the rate -- but the tracking gate is additionally CLAMPED
  // by the slot geometry, which does not scale, and the acquisition gate was
  // not. Above ~450 MSPS that inverted them: at 491.52 the confirm gate is
  // 2560 samples against a clamped tracking gate of 1376, so acquisition would
  // hand back an anchor that the very first tracking check rejects as off-grid
  // -- a lock that escalates immediately, forever. Found by this file's own
  // test on its first run, which is the entire argument for having it (AP-56).
  const long long confirm_want =
      std::max<long long>(1, std::llround(confirm_tol_us * 1e-6 * rate));
  g.confirm_tol = std::min(confirm_want, g.scatter_tol);
  g.confirm_clamped = confirm_want > g.scatter_tol;

  g.lead = g.scatter_tol + kSyncCorrContext;
  g.tail = g.scatter_tol + kSyncGoldLen / 2;
  g.accept_window = slot - g.lead - g.tail;
  g.accept_window_frac =
      static_cast<double>(g.accept_window) / static_cast<double>(slot);
  g.usable = g.accept_window > 0;

  g.resync_frames =
      sync_tol_samples /
      (sync_residual_ppm * 1e-6 * static_cast<double>(frame));
  g.resync_interval_s = std::min(
      1e3, std::max(1e-3, g.resync_frames * static_cast<double>(frame) / rate));
  g.resync_period_iters = static_cast<long long>(std::max(
      1.0, std::floor(1e9 / (100.0 * static_cast<double>(frame)))));
  return g;
}

}  // namespace Sounder

#endif  // SYNC_GEOMETRY_H_
