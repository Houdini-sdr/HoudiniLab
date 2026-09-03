/**
 * @file sync_geometry_test.cc
 * @brief The UE beacon-sync geometry across the whole rate ladder. NO hardware.
 *
 * Two defects shipped because these constants could only be seen by running a
 * UE against silicon at one sample rate: the acquisition gate stayed a fixed
 * sample count beside a tracking gate that had been converted to time
 * (DEMO_VERIFICATION 8.59), and the clamp keeping the tracking gate inside the
 * slot left an accept window one read-phase wide at every rate above ~226 MSPS
 * (8.65). Both are arithmetic, and both are obvious in a printed table.
 *
 * So this walks the rate ladder we actually care about and asserts the
 * properties that must hold at ALL of them, rather than the values that happen
 * to hold at one. It prints the table either way, because the table is the
 * thing a reader wants when a gate misbehaves.
 *
 * Build: CMake target sync_geometry_test. Run: ./sync_geometry_test (or ctest).
 */
#include <cinttypes>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "sync/sync_geometry.h"

namespace {

int g_fail = 0;

void check(bool ok, const std::string& what) {
  std::printf("%s  %s\n", ok ? "PASS" : "FAIL", what.c_str());
  if (!ok) ++g_fail;
}

struct Rung {
  const char* name;
  double rate_hz;
  long long samps_per_slot;
  long long samps_per_frame;
};

// The shipped configuration, and the rungs AP-3 aims the recorder at. The slot
// is OFDM geometry (48 x (64 + 16) + 128 + 128 = 4096) and does NOT scale with
// the rate; the frame is 30 slots. That non-scaling is precisely what makes the
// high rungs interesting here.
const std::vector<Rung> kLadder = {
    {"122.88 MSPS (shipped)", 122.88e6, 4096, 122880},
    {"245.76 MSPS", 245.76e6, 4096, 122880},
    {"491.52 MSPS", 491.52e6, 4096, 122880},
    {"983.04 MSPS", 983.04e6, 4096, 122880},
    {"1966.08 MSPS (max)", 1966.08e6, 4096, 122880},
};

// Shipped defaults, from receiver.cc's envDouble fallbacks.
constexpr double kScatterUs = 2.0;         // was 8.3333 until 2026-09-02
constexpr double kConfirmUs = 5.2083;
constexpr double kSyncTolSamples = 32.0;   // ofdm_tx_zero_prefix 128 / 4
constexpr double kResidualPpm = 0.1;       // was 1.0 until 2026-09-02

}  // namespace

// The gold replica the whole table was derived with. A shorter replica needs
// less run-up: dot11 and nr (64 taps) get 128 fewer lead and 32 fewer tail.
constexpr long long kReplicaLen = 128;

int main() {
  std::printf(
      "%-24s %8s %8s %8s %9s %8s %9s %10s\n", "rate", "scatter", "confirm",
      "cap", "clamped", "window", "window %", "resync ms");
  std::vector<houdini::sync::SyncGeometry> geo;
  for (const auto& r : kLadder) {
    const auto g = houdini::sync::computeSyncGeometry(r.rate_hz, r.samps_per_slot, r.samps_per_frame, kReplicaLen, 0, kScatterUs, kConfirmUs,
        kSyncTolSamples, kResidualPpm);
    geo.push_back(g);
    std::printf("%-24s %8lld %8lld %8lld %9s %8lld %8.1f%% %10.1f\n", r.name,
                g.scatter_tol, g.confirm_tol, g.slot_cap,
                g.scatter_clamped ? "yes" : "no", g.accept_window,
                100.0 * g.accept_window_frac, g.resync_interval_s * 1e3);
  }
  std::printf("\n");

  // ---- properties that must hold at EVERY rate -------------------------
  for (size_t i = 0; i < kLadder.size(); ++i) {
    const auto& g = geo[i];
    const std::string at = std::string(" at ") + kLadder[i].name;

    // The one that shipped broken. A gate with no width is a UE that never
    // looks at the beacon, counts no miss, and therefore never escalates:
    // silence, which is the worst failure available here.
    check(g.usable && g.accept_window > 0, "the accept window is non-empty" + at);
    // And "non-empty" is not enough: one admissible read phase out of a frame
    // is functionally the same failure. Demand a real fraction of the slot.
    check(g.accept_window_frac >= 0.20,
          "the accept window is at least 20% of the slot" + at);

    // The slice has to FIT the read, or the gate can never be satisfied.
    check(g.lead + g.tail <= kLadder[i].samps_per_slot,
          "the targeted slice fits inside one slot" + at);

    // Acquisition must not be looser than tracking: a confirm accepted outside
    // the tracking gate hands the tracker an anchor it will immediately reject.
    check(g.confirm_tol <= g.scatter_tol,
          "the acquisition gate is no looser than the tracking gate" + at);

    // Both gates are TIMES. Their ratio is a property of the two microsecond
    // constants and must therefore be the same at every rate: that is the whole
    // point of expressing them in time, and it is exactly what 8.59 violated.
    const double ratio = static_cast<double>(g.confirm_tol) /
                         static_cast<double>(g.scatter_tol);
    const double ratio0 = static_cast<double>(geo[0].confirm_tol) /
                          static_cast<double>(geo[0].scatter_tol);
    // Only meaningful while the scatter tolerance is unclamped; once clamped it
    // is the slot geometry, not the constant, that sets the tracking gate.
    if (!g.scatter_clamped) {
      check(ratio > ratio0 * 0.98 && ratio < ratio0 * 1.02,
            "the two gates keep their ratio (both are times)" + at);
    }

    // The cadence is frame geometry, and the frame is defined in samples, so it
    // is rate-invariant in FRAMES and scales in seconds exactly as the frame
    // duration does.
    check(g.resync_period_iters == geo[0].resync_period_iters,
          "the Iris/UHD frame-count cadence is unchanged by the rate" + at);
  }

  // ---- the shipped defaults, and what they were when 8.51 gated ---------
  // These CHANGED on 2026-09-02 [user], so the assertions below pin the NEW
  // values and the comment records the old ones. The gate at 8.51/8.79 was
  // taken on the old defaults and does NOT cover these; it has to re-run.
  //   was: scatter 1024, confirm 640, window 1728 (42.2 %), cadence 260.4 ms
  //   now: scatter  246, confirm 246, window 3284 (80.2 %), cadence 2604 ms
  check(geo[0].scatter_tol == 246, "shipped: tracking gate is 246 samples (2.0 us)");
  check(geo[0].confirm_tol == 246,
        "shipped: acquisition gate is PULLED IN to the tracking gate");
  check(geo[0].confirm_clamped,
        "shipped: the confirm tolerance is clamped by the tracking gate now");
  check(!geo[0].scatter_clamped, "shipped rate: the tolerance is not slot-clamped");
  check(geo[0].accept_window == 3284,
        "shipped: the accept window is 3284 samples (80.2 % of the slot)");
  check(geo[0].accept_window_frac > 0.75,
        "shipped: tightening the gate WIDENED the window past 75 %");
  check(geo[0].resync_interval_s > 2.604 - 1e-3 &&
            geo[0].resync_interval_s < 2.604 + 1e-3,
        "shipped: the resync cadence is 2.604 s (was 260.4 ms)");
  check(geo[0].resync_period_iters == 81,
        "the Iris/UHD cadence is UNCHANGED at 81 frames by both edits");

  // ---- the clamp must engage where the slot really does run out --------
  // The clamp must not bite at the shipped rate, and must bite SOMEWHERE on the
  // ladder -- otherwise it is untested. Which rung it first bites at is a
  // consequence of the tolerance default and moved when that default changed
  // (8.3333 us clamped at 2x, 2.0 us does not clamp until ~688 MSPS), so
  // pinning a rung would have been pinning the wrong thing.
  check(!geo[0].scatter_clamped,
        "the clamp does NOT bite at the shipped rate");
  {
    bool any = false;
    size_t first = 0;
    for (size_t i = 0; i < geo.size(); ++i) {
      if (geo[i].scatter_clamped) { any = true; first = i; break; }
    }
    check(any, "the clamp bites somewhere on the ladder, so it is exercised");
    if (any)
      std::printf("  clamp first engages at %s\n", kLadder[first].name);
  }

  // ---- THE DOCUMENTED SWEEP, which is where this broke ------------------
  // HOUDINI_SCATTER_TOL_US is a sweep knob (walkthrough 7.1) and session-plan
  // leg 9 sweeps it. The acquisition gate is CLAMPED by the tracking one, so
  // any caller deriving the two from different scatter tolerances inverts
  // them. That is exactly what happened: houdiniAcquireAnchor re-derived its
  // own copy with the tolerance hardcoded to the default, and at
  // HOUDINI_SCATTER_TOL_US=4 the tracking gate was 492 against an acquisition
  // gate of 640. There is one derivation now, threaded through; this walks the
  // sweep so the invariant is checked across it rather than at the default.
  std::printf("=== HOUDINI_SCATTER_TOL_US swept, shipped rate ===\n");
  for (double tol_us : {1.0, 2.0, 4.0, 8.3333, 12.0, 20.0, 60.0}) {
    const auto g = houdini::sync::computeSyncGeometry(122.88e6, 4096, 122880, kReplicaLen, 0, tol_us,
                                                kConfirmUs, kSyncTolSamples,
                                                kResidualPpm);
    std::printf("  %6.2f us -> scatter %5lld  confirm %5lld%s  window %5lld "
                "(%.1f%%)\n", tol_us, g.scatter_tol, g.confirm_tol,
                g.confirm_clamped ? " (pulled in)" : "            ",
                g.accept_window, 100.0 * g.accept_window_frac);
    check(g.confirm_tol <= g.scatter_tol,
          "swept: acquisition stays no looser than tracking at " +
              std::to_string(tol_us) + " us");
    check(g.usable && g.accept_window_frac >= 0.20,
          "swept: the accept window stays usable at " +
              std::to_string(tol_us) + " us");
  }
  std::printf("\n");

  // ---- the replica length sizes the slice ------------------------------
  {
    const auto g128 = houdini::sync::computeSliceGeometry(122.88e6, 4096, 122880, 128, 0, kScatterUs, kConfirmUs);
    const auto g64 = houdini::sync::computeSliceGeometry(122.88e6, 4096, 122880, 64, 0, kScatterUs, kConfirmUs);
    check(g128.lead - g64.lead == 128 && g128.tail - g64.tail == 32 &&
              g64.accept_window == g128.accept_window + 160,
          "a 64-tap replica needs 128 less lead and 32 less tail than the 128-tap gold");
    // A LEADING replica (nr_pss: 128 taps, 144 samples before the end) needs
    // the tail in front of the run-up: lead 646 against 502.
    const auto gpss = houdini::sync::computeSliceGeometry(122.88e6, 4096, 122880, 128, 144, kScatterUs, kConfirmUs);
    check(gpss.lead == g128.lead + 144 && gpss.tail == g128.tail && gpss.usable,
          "nr_pss: the lead covers the replica tail (" + std::to_string(gpss.lead) + " = 502 + 144)");
    const auto whole = houdini::sync::computeSyncGeometry(122.88e6, 4096, 122880, 128, 0, kScatterUs, kConfirmUs, kSyncTolSamples, kResidualPpm);
    const auto sched = houdini::sync::computeResyncSchedule(122.88e6, 122880, kSyncTolSamples, kResidualPpm);
    check(whole.lead == g128.lead && whole.resync_interval_s == sched.resync_interval_s &&
              whole.resync_period_iters == static_cast<long long>(std::floor(1e9 / (100.0 * 122880.0))),
          "computeSyncGeometry composes the slice and the schedule; the Iris cadence is the 100 ppb rule");
  }

  // ---- degenerate inputs must not produce a silent open loop -----------
  {
    // A tolerance far too large for any slot: the clamp, not the caller, has to
    // keep the gate usable.
    const auto g = houdini::sync::computeSyncGeometry(122.88e6, 4096, 122880, kReplicaLen, 0, 1000.0,
                                                kConfirmUs, kSyncTolSamples,
                                                kResidualPpm);
    check(g.scatter_clamped && g.usable && g.accept_window_frac >= 0.20,
          "an absurd HOUDINI_SCATTER_TOL_US is clamped to a usable window");
  }
  {
    // A zero rate would divide by zero in the cadence; the fallback keeps the
    // frame at 1 ms rather than producing an infinity the size_t cast turns
    // into a huge value that disables beacon checking entirely.
    const auto g = houdini::sync::computeSyncGeometry(0.0, 4096, 122880, kReplicaLen, 0, kScatterUs,
                                                kConfirmUs, kSyncTolSamples,
                                                kResidualPpm);
    check(g.resync_interval_s > 0.0 && g.resync_interval_s <= 1e3,
          "a zero sample rate still yields a finite, bounded cadence");
    check(g.scatter_tol >= 1 && g.confirm_tol >= 1,
          "a zero sample rate still yields positive gates");
  }

  std::printf("\nRESULT: %s (%d failure(s))\n", g_fail ? "FAIL" : "PASS", g_fail);
  return g_fail ? 1 : 0;
}
