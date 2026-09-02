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
#include <cstdio>
#include <string>
#include <vector>

#include "include/sync_geometry.h"

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
constexpr double kScatterUs = 8.3333;
constexpr double kConfirmUs = 5.2083;
constexpr double kSyncTolSamples = 32.0;   // ofdm_tx_zero_prefix 128 / 4
constexpr double kResidualPpm = 1.0;

}  // namespace

int main() {
  std::printf(
      "%-24s %8s %8s %8s %9s %8s %9s %10s\n", "rate", "scatter", "confirm",
      "cap", "clamped", "window", "window %", "resync ms");
  std::vector<Sounder::SyncGeometry> geo;
  for (const auto& r : kLadder) {
    const auto g = Sounder::computeSyncGeometry(
        r.rate_hz, r.samps_per_slot, r.samps_per_frame, kScatterUs, kConfirmUs,
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

  // ---- the shipped rate must be bit-identical to what was gated --------
  // 640 samples was the acquisition tolerance measured at 122.88 MSPS and 1024
  // the tracking one. Expressing them in time was meant to change the SCALING
  // and nothing else, so if either moves here, the gate evidence taken at this
  // rate no longer describes this build.
  check(geo[0].scatter_tol == 1024, "shipped rate: tracking gate is still 1024 samples");
  check(geo[0].confirm_tol == 640, "shipped rate: acquisition gate is still 640 samples");
  check(!geo[0].scatter_clamped, "shipped rate: the tolerance is not clamped");
  check(geo[0].accept_window == 1728, "shipped rate: the accept window is still 1728 samples");
  check(geo[0].resync_interval_s > 0.2604 - 1e-4 &&
            geo[0].resync_interval_s < 0.2604 + 1e-4,
        "shipped rate: the resync cadence is 260.4 ms");
  check(geo[0].resync_period_iters == 81,
        "the Iris/UHD cadence is back to its original 81 frames");

  // ---- the clamp must engage where the slot really does run out --------
  check(!geo[0].scatter_clamped && geo[1].scatter_clamped,
        "the clamp engages between the shipped rate and 2x, as the slot fills");

  // ---- degenerate inputs must not produce a silent open loop -----------
  {
    // A tolerance far too large for any slot: the clamp, not the caller, has to
    // keep the gate usable.
    const auto g = Sounder::computeSyncGeometry(122.88e6, 4096, 122880, 1000.0,
                                                kConfirmUs, kSyncTolSamples,
                                                kResidualPpm);
    check(g.scatter_clamped && g.usable && g.accept_window_frac >= 0.20,
          "an absurd HOUDINI_SCATTER_TOL_US is clamped to a usable window");
  }
  {
    // A zero rate would divide by zero in the cadence; the fallback keeps the
    // frame at 1 ms rather than producing an infinity the size_t cast turns
    // into a huge value that disables beacon checking entirely.
    const auto g = Sounder::computeSyncGeometry(0.0, 4096, 122880, kScatterUs,
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
