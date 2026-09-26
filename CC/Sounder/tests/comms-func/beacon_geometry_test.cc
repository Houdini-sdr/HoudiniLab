/**
 * @file beacon_geometry_test.cc
 * @brief Where the detector's index lands, for each candidate beacon, as a
 *        function of received level, channel and fractional delay. NO hardware.
 *
 * THE INVARIANT the whole timing chain rests on:
 *
 *     sync_index == houdiniBeaconEnd() == strobe + beacon_size
 *
 * If the returned index moves, `beaconSnrDb()` measures a window of
 * pre-beacon noise and the SNR floor rejects every resync detection, while
 * acquisition still works: the link looks healthy with its liveness path dead
 * (AP-34(a): an index moved -274 samples read 10.5 dB against a true 48.3).
 *
 * THE MECHANISM. A rule that picks the EARLIEST threshold crossing in the
 * search window is level-dependent. The legacy STS preamble is 16-periodic and
 * 16 divides the 128-sample correlator lag, so the STS field is perfectly
 * lag-128 self-coherent and manufactures crossings a few hundred samples before
 * the true peak. Whether those crossings win is a function of received level,
 * because the power-ratio test `corr_scale * |gc[i]|^2 |gc[i-L]|^2 > sum |gc|^2`
 * compares a 4th-order quantity to a 2nd-order one and is NOT scale invariant.
 * A guard field in front of the gold does not cause the fault; it lowers the
 * level at which it appears, from ~3200 counts peak to ~400.
 *
 * So the level sweep below is a two-sided regression test:
 *   - with kFirstCrossing EVERY candidate beacon false-locks somewhere in the
 *     level range a real link spans;
 *   - with kTargetedArgmax -- legal only because the resync slice is ~812
 *     samples against a beacon copy spacing of one full frame -- every
 *     candidate lands on the beacon end at every level.
 * If a change reintroduces a level-dependent index, this fails. Sections
 * marked "reported, not gated" print the tables behind the DEMO_VERIFICATION
 * rows they cite rather than asserting a requirement.
 *
 * WHAT THIS TEST DOES NOT ESTABLISH. The bursts are synthesised from
 * beacon_shapes.h, not captured from the TX RAM, so they carry neither the
 * conjugation and x8 band-limited upsampling of the transmit path nor a channel.
 * The ABSOLUTE count levels are therefore not bench levels. What transfers is
 * the ORDERING and the mechanism: which rule is level-dependent, and by how much
 * the candidates differ in processing gain.
 *
 * Build: CMake target beacon_geometry_test. Run: ./beacon_geometry_test.
 */
#include <cinttypes>
#include <cmath>
#include <complex>
#include <cstdio>
#include <map>
#include <random>
#include <string>
#include <utility>
#include <vector>

#include "sync/beacon_shapes.h"
#include "sync/numerology.h"
#include "sync/sync_config.h"
#include "sync/sim/channel.h"
#include "comms-lib.h"
#include "utils.h"

namespace {

using houdini::sync::shapes::cf;
using houdini::sync::shapes::Desc;
using houdini::sync::shapes::Shape;
using Pick = CommsLib::BeaconPick;
using Thr = CommsLib::BeaconThresh;

int g_fail = 0;

void check(bool ok, const std::string& what) {
  std::printf("%s  %s\n", ok ? "PASS" : "FAIL", what.c_str());
  if (!ok) ++g_fail;
}

/// The residual functions below place `core` so its END sits at `end` in a
/// noise buffer and ask the real detector, not a re-implementation of it (one
/// would agree with itself and prove nothing). They return the detector's
/// index MINUS the true beacon end, or kMiss.
constexpr long long kMiss = -1000000;

constexpr double kRate = houdini::sync::Numerology::houdiniDefault().rate_hz;

/// A channel: taps at given delays, plus a carrier offset.
///
/// OVER THE AIR THE TRUTH IS THE FIRST PATH, NOT THE STRONGEST. A frame grid
/// wants a timing reference that is physically meaningful and, more
/// importantly, STABLE. The strongest path changes as the channel fades; the
/// first path does not. So `residual` below is measured against the DIRECT
/// path's beacon end even when a later tap is stronger, and a rule that returns
/// the late tap scores as biased, which is what it is.
struct Channel {
  std::vector<std::pair<int, double>> taps{{0, 1.0}};  // (delay, amplitude)
  double cfo_hz = 0.0;
  const char* name = "1 path";
};

/// Residual with a channel and a carrier offset applied.
long long residualCh(const Desc& b, double peak_counts, double snr_db,
                     long long lead, long long tail, float corr_scale, Pick pick,
                     unsigned seed, Thr thresh_form, const Channel& ch,
                     int guard = 0, double frac = 0.0) {
  const long long len = static_cast<long long>(b.core.size());
  houdini::sync::sim::Channel sc;
  sc.taps.clear();
  for (const auto& tp : ch.taps) sc.taps.push_back({tp.first, {tp.second, 0.0}});
  sc.cfo_hz = ch.cfo_hz;
  sc.frac_delay = frac;
  sc.rate_hz = kRate;
  sc.snr_db = snr_db;
  sc.peak_counts = peak_counts;
  const long long pos = 4000, end = pos + len, s0 = end - lead, n = lead + tail;
  std::vector<std::complex<int16_t>> buf = sc.receive(b.core, pos, s0, n, seed);
  // A single-copy replica (nr_pss) has no repeat to check: the receiver forces
  // the plain matched filter for it (syncSearch), and so does this test, so
  // every column below measures the form the shape would actually run with.
  if (b.replica_reps < 2) thresh_form = Thr::kCoherence;
  const ssize_t idx = CommsLib::find_beacon_avx(
      buf.data(), b.replica, n, corr_scale, pick, thresh_form,
      static_cast<int>(b.replica.size() / 2),
      CommsLib::kDefaultFirstPathFloorDb, guard);
  // The detector reports the last sample of the MATCHED field; the beacon end
  // is replica_tail() later (0 for every shape but nr_pss), exactly as
  // syncSearch applies it.
  const long long rep_tail = static_cast<long long>(b.replica_tail());
  return idx < 0 ? kMiss : s0 + idx + rep_tail - end;  // vs the DIRECT path's end
}
using houdini::sync::sim::fracDelay;

long long residual(const Desc& b, double peak_counts, double snr_db,
                   long long lead, long long tail, float corr_scale, Pick pick,
                   unsigned seed, Thr thresh_form = Thr::kPowerRatio,
                   double frac = 0.0) {
  const long long len = static_cast<long long>(b.core.size());
  houdini::sync::sim::Channel sc;
  sc.snr_db = snr_db;
  sc.peak_counts = peak_counts;
  sc.frac_delay = frac;
  const long long pos = 4000, end = pos + len, s0 = end - lead, n = lead + tail;
  std::vector<std::complex<int16_t>> buf = sc.receive(b.core, pos, s0, n, seed);
  if (b.replica_reps < 2) thresh_form = Thr::kCoherence;  // see residualCh
  const ssize_t idx = CommsLib::find_beacon_avx(buf.data(), b.replica, n,
                                               corr_scale, pick, thresh_form);
  const long long rep_tail = static_cast<long long>(b.replica_tail());
  return idx < 0 ? kMiss : s0 + idx + rep_tail - end;
}

// The shipped resync slice at 122.88 MSPS (scatter_tol 246 samples, the 2.0 us
// default): lead = 246 + 256, tail = 246 + 64. sync_geometry.h owns the
// derivation; these are the values it produces, restated so this test says what
// geometry it is testing rather than pulling in the whole header.
constexpr long long kLead = 502, kTail = 310;
constexpr float kResyncCorrScale = 100.0f;  // corr_scale of the legacy-beacon configs (houdini-ul, -1u, -r0)
constexpr double kSnrDb = 45.0;             // measured in-window beacon SNR
// The detector reports the last sample of the matched field, so the true beacon
// end lands one sample later than the returned index.
constexpr long long kEndConvention = -1;

const double kLevels[] = {200, 400, 800, 1600, 3200, 6400, 12800};

struct Row {
  long long lo = 1LL << 40, hi = -(1LL << 40);
  int miss = 0;
};

Row sweepAt(const Desc& b, double level, Pick pick,
            Thr tf = Thr::kPowerRatio) {
  Row r;
  for (unsigned s = 1; s <= 8; ++s) {
    const long long v =
        residual(b, level, kSnrDb, kLead, kTail, kResyncCorrScale, pick, s, tf);
    if (v == kMiss) {
      ++r.miss;
      continue;
    }
    r.lo = std::min(r.lo, v);
    r.hi = std::max(r.hi, v);
  }
  return r;
}

void cell(const Row& r) {
  char buf[32];
  if (r.miss == 8) std::snprintf(buf, sizeof buf, "MISS");
  else if (r.miss) std::snprintf(buf, sizeof buf, "%+lld/%dmiss", r.lo, r.miss);
  else if (r.lo == r.hi) std::snprintf(buf, sizeof buf, "%+lld", r.lo);
  else std::snprintf(buf, sizeof buf, "%+lld..%+lld", r.lo, r.hi);
  std::printf(" %13s", buf);
}

/// The first-path back-scan window SyncConfig::resolve() derives for a shape:
/// half its replica (sync_config.cc), 64 for a 128-tap replica and 32 for a
/// 64-tap one. One fixed width would read dot11 as 64 samples biased: an error
/// of the measurement, not of the detector.
int shippedWindow(const Desc& b) {
  return static_cast<int>(b.replica.size() / 2);
}

/// One single-path run at fractional delay `frac`: the integer residual, or
/// kMiss when nothing was found. `first_path_window` 0 disables the back-scan,
/// which makes the pick the argmax: with window 0 the back-scan loop runs
/// zero times and the rule returns `best`, the same value kTargetedArgmax
/// computes from the same code, so this is identity rather than agreement.
long long runAt(const Desc& b, double frac, unsigned seed,
                int first_path_window, double snr_db, int guard) {
  const long long len = static_cast<long long>(b.core.size());
  houdini::sync::sim::Channel sc;
  sc.snr_db = snr_db;
  sc.peak_counts = 1600.0;
  sc.frac_delay = frac;
  const long long pos = 4000, end = pos + len, s0 = end - kLead,
                  n = kLead + kTail;
  const std::vector<std::complex<int16_t>> buf =
      sc.receive(b.core, pos, s0, n, seed);
  const Thr tf = b.replica_reps < 2 ? Thr::kCoherence : Thr::kNormalizedXCorr;
  const CommsLib::BeaconResult r = CommsLib::find_beacon_ex(
      buf.data(), b.replica, static_cast<size_t>(n), kResyncCorrScale,
      Pick::kFirstPath, tf, first_path_window,
      CommsLib::kDefaultFirstPathFloorDb, guard);
  const long long rep_tail = static_cast<long long>(b.replica_tail());
  return r.index < 0 ? kMiss : s0 + r.index + rep_tail - end;
}

/// One noise-only hunt window: 12288 samples of complex Gaussian noise, 20
/// counts per component, from raw mt19937 output (bit-specified, unlike the
/// std distributions) seeded with `seed`.
std::vector<std::complex<int16_t>> noiseWindow(unsigned seed) {
  std::mt19937 g(seed);
  auto u01 = [&g]() { return (static_cast<double>(g()) + 0.5) / 4294967296.0; };
  std::vector<std::complex<int16_t>> buf(12288);
  for (auto& v : buf) {
    const double a = std::sqrt(-2.0 * std::log(u01()));
    const double ph = 2.0 * M_PI * u01();
    v = std::complex<int16_t>(static_cast<int16_t>(20.0 * a * std::cos(ph)),
                              static_cast<int16_t>(20.0 * a * std::sin(ph)));
  }
  return buf;
}

}  // namespace

int main() {
  const Desc ds[] = {houdini::sync::shapes::make(Shape::kLegacy),
                     houdini::sync::shapes::make(Shape::kLegacyGuard),
                     houdini::sync::shapes::make(Shape::kDot11),
                     houdini::sync::shapes::make(Shape::kNr),
                     houdini::sync::shapes::make(Shape::kNrPss)};

  std::printf("Candidate beacons, and where the detector says they are.\n");
  std::printf("Resync slice [end-%lld, end+%lld), corr_scale %.0f, SNR %.0f dB.\n",
              kLead, kTail, kResyncCorrScale, kSnrDb);
  std::printf("Cells are (returned index - true beacon end) over 8 noise draws;\n");
  std::printf("%+lld is correct.\n", kEndConvention);

  // The processing gain is that of the REPLICA field -- the fine field for
  // four shapes, the PSS for nr_pss -- because that is what the matched filter
  // integrates over. `tail` is how far the beacon end sits past the index the
  // detector returns; syncSearch adds it, and so does residual() below.
  std::printf("\n%-14s %6s %8s %8s %5s %8s %10s\n", "shape", "core", "rep_off",
              "rep_len", "reps", "PAPR dB", "proc gain");
  for (const auto& b : ds) {
    double pk = 0.0, fp = 0.0;
    const size_t rl = b.replica.size();
    for (const auto& v : b.core) pk = std::max(pk, static_cast<double>(std::norm(v)));
    for (size_t i = b.replica_off; i < b.replica_off + rl; ++i)
      fp += std::norm(b.core[i]);
    fp /= static_cast<double>(rl);
    std::printf("%-14s %6zu %8zu %8zu %5zu %8.2f %7.1f dB  (tail %zu)\n",
                b.name.c_str(), b.core.size(), b.replica_off, rl,
                b.replica_reps, b.papr_db(),
                20.0 * std::log10(rl * std::sqrt(fp / pk)), b.replica_tail());
  }

  for (const auto pick : {Pick::kFirstCrossing, Pick::kTargetedArgmax}) {
    const bool argmax = pick == Pick::kTargetedArgmax;
    // Both rows use the POWER-RATIO threshold, not the shipped form: they
    // compare PICK RULES with the threshold held fixed. The shipped
    // combination, xcorr + first-path, is exercised by the matrix further down.
    std::printf("\n=== %s (power-ratio threshold) ===\n",
                argmax ? "kTargetedArgmax (resync)" : "kFirstCrossing (resync)");
    std::printf("%-12s", "peak counts");
    for (const auto& b : ds) std::printf(" %13s", b.name.c_str());
    std::printf("\n");
    int level_dependent = 0;
    for (const double lv : kLevels) {
      std::printf("%-12.0f", lv);
      for (const auto& b : ds) {
        const Row r = sweepAt(b, lv, pick);
        cell(r);
        if (r.miss == 0 && (r.lo != kEndConvention || r.hi != kEndConvention))
          ++level_dependent;
      }
      std::printf("\n");
    }
    if (argmax) {
      check(level_dependent == 0,
            "  every candidate lands on the beacon end at EVERY level");
    } else {
      // Not a defect being tolerated: this asserts the first-crossing rule
      // really is level-dependent, so the argmax row measures a fix rather
      // than a no-op. If this ever passes cleanly, the mechanism changed and
      // both rows need re-deriving before the argmax result can be trusted.
      check(level_dependent > 0,
            "  first-crossing DOES false-lock somewhere (the argmax row is not a no-op)");
    }
  }

  // Sensitivity floor: every candidate must still be detectable at a level well
  // below where the bench runs, or a shape wins the index test by being
  // undetectable. Checked with the rule we actually ship.
  std::printf("\n");
  for (const auto& b : ds) {
    // Sensitivity floor under the SHIPPED combination, not the power-ratio one.
    const Row r = sweepAt(b, 200.0, Pick::kFirstPath, Thr::kNormalizedXCorr);
    check(r.miss == 0 && r.lo == kEndConvention && r.hi == kEndConvention,
          "  " + b.name + ": detected at 200 counts peak, index exact");
  }

  // `legacy` MUST STAY THE ORIGINAL BEACON. Config::genPilots builds the beacon
  // from beacon_shapes.h and the bench probes read its dumped waveforms, so the
  // header is the one definition, and every result taken on the legacy beacon
  // rests on it being sample-identical to the original recipe: 15 x STS(16)
  // then 2 x gold(128), each through Utils::float_to_cint16. Rebuild that
  // recipe here and require sample equality (AP-34(a) is what a bench and a
  // build disagreeing about a beacon costs).
  {
    auto sts_ci16 = Utils::float_to_cint16(CommsLib::getSequence(CommsLib::STS_SEQ));
    auto gold_ci16 = Utils::float_to_cint16(CommsLib::getSequence(CommsLib::GOLD_IFFT));
    std::vector<std::complex<int16_t>> want;
    for (int i = 0; i < 15; ++i)
      want.insert(want.end(), sts_ci16.begin(), sts_ci16.end());
    for (int i = 0; i < 2; ++i)
      want.insert(want.end(), gold_ci16.begin(), gold_ci16.end());

    const auto legacy = houdini::sync::shapes::make(Shape::kLegacy);
    std::vector<std::complex<int16_t>> got;
    {
      std::vector<std::complex<float>> f(legacy.core.begin(), legacy.core.end());
      got = Utils::cfloat_to_cint16(f);
    }
    bool same = want.size() == got.size();
    size_t first_diff = 0;
    if (same) {
      for (size_t i = 0; i < want.size(); ++i) {
        if (want[i] != got[i]) { same = false; first_diff = i; break; }
      }
    }
    if (!same && want.size() == got.size())
      std::printf("      first difference at sample %zu: config (%d,%d) vs "
                  "beacon_shapes (%d,%d)\n", first_diff,
                  want[first_diff].real(), want[first_diff].imag(),
                  got[first_diff].real(), got[first_diff].imag());
    check(same,
          "  beacon_shapes 'legacy' is sample-identical to the original genPilots recipe");
  }

  // NO TWO TONES ON ONE BIN. The NR fields are built in the frequency domain; a
  // DC-landing tone parked on a bin already occupied doubles one tone and drops
  // another, and nothing else fails: transmit and correlator share the same
  // malformed symbol, so it detects fine and measures a beacon nobody designed.
  // Check the property directly -- every field must occupy as many distinct
  // non-DC bins as it has tones.
  {
    const auto nr = houdini::sync::shapes::make(Shape::kNr);
    // The tracking symbol is the last fine_len samples of the core.
    std::vector<cf> sym(nr.core.end() - nr.fine_len, nr.core.end());
    auto spec = CommsLib::FFT(sym, static_cast<int>(nr.fine_len), false);
    double tot = 0.0;
    for (const auto& v : spec) tot += std::norm(v);
    int occupied = 0;
    for (size_t i = 0; i < spec.size(); ++i) {
      if (std::norm(spec[i]) > tot / (200.0 * spec.size())) ++occupied;
    }
    // 64 tones into a 64-point IFFT leaves 63 usable bins once DC is nulled.
    check(occupied == static_cast<int>(nr.fine_len) - 1,
          "  NR tracking symbol occupies every non-DC bin exactly once (" +
              std::to_string(occupied) + " of " +
              std::to_string(nr.fine_len - 1) + ")");
    check(std::norm(spec[0]) <= tot / (200.0 * spec.size()),
          "  NR tracking symbol nulls DC");
  }

  // ---------------------------------------------------------------------
  // THE THRESHOLD FORM, WHICH IS THE STRUCTURAL HALF OF THE SAME DEFECT.
  //
  // kTargetedArgmax above fixes WHICH crossing is returned. It does not fix
  // what the threshold MEANS: the power-ratio statistic is 4th order in
  // received amplitude over 2nd, so `corr_scale` is a different test at every
  // level (at the true peak it spans 4136 = 64^2 across a 64x level sweep).
  // Normalised -- divided by the energy term squared, Schmidl & Cox 1997 -- it
  // is level-invariant.
  //
  // The question the matrix answers (the xcorr + FIRST-crossing line is
  // reported, not gated): with the normalised statistic the preamble plateau
  // sits at 1/L^2, level-independent and far below any sensible bar, so the
  // earliest-crossing rule might land on the beacon end at every level too. If
  // it does, the normalisation subsumes the selection fix; if not, both are
  // needed.
  std::printf("\n=== threshold form x pick rule, over the level sweep ===\n");
  std::printf("cells: levels (of %zu) whose index is exact / levels that MISS\n",
              sizeof(kLevels) / sizeof(*kLevels));
  std::printf("%-13s %11s %11s %11s %11s %11s %11s %11s %11s\n", "shape",
              "pow+first", "pow+argmx", "pow+1stpth", "xc+first",
              "xc+argmx", "xc+1stpth", "nolag+frst", "nolag+1stp");
  int norm_first_bad = 0, norm_firstpath_bad = 0, power_first_bad = 0;
  int nolag_bad = 0, nrpss_bad = 0;
  for (const auto& b : ds) {
    std::printf("%-14s", b.name.c_str());
    struct Combo { Thr tf; Pick pk; };
    const Combo combos[] = {{Thr::kPowerRatio, Pick::kFirstCrossing},
                            {Thr::kPowerRatio, Pick::kTargetedArgmax},
                            {Thr::kPowerRatio, Pick::kFirstPath},
                            {Thr::kNormalizedXCorr, Pick::kFirstCrossing},
                            {Thr::kNormalizedXCorr, Pick::kTargetedArgmax},
                            {Thr::kNormalizedXCorr, Pick::kFirstPath},
                            {Thr::kCoherence, Pick::kFirstCrossing},
                            {Thr::kCoherence, Pick::kFirstPath}};
    for (const auto& cb : combos) {
      {
        const Thr tf = cb.tf; const Pick pk = cb.pk;
        int exact = 0, miss = 0;
        for (const double lv : kLevels) {
          bool ok = true, any_miss = false;
          for (unsigned sd = 1; sd <= 8; ++sd) {
            const long long v = residual(b, lv, kSnrDb, kLead, kTail,
                                         kResyncCorrScale, pk, sd, tf);
            if (v == kMiss) { any_miss = true; ok = false; }
            else if (v != kEndConvention) ok = false;
          }
          if (ok) ++exact;
          if (any_miss) ++miss;
        }
        const int nlev = static_cast<int>(sizeof(kLevels) / sizeof(*kLevels));
        if (tf == Thr::kNormalizedXCorr && pk == Pick::kFirstCrossing)
          norm_first_bad += nlev - exact;
        if (tf == Thr::kNormalizedXCorr && pk == Pick::kFirstPath)
          norm_firstpath_bad += nlev - exact;
        if (tf == Thr::kPowerRatio && pk == Pick::kFirstCrossing)
          power_first_bad += nlev - exact;
        // nr_pss runs nolag in EVERY column (residual() forces it), so it must
        // not be counted as evidence about the repeat check on the others.
        if (tf == Thr::kCoherence && pk == Pick::kFirstPath) {
          if (b.replica_reps < 2) nrpss_bad += nlev - exact;
          else nolag_bad += nlev - exact;
        }
        char c[32];
        std::snprintf(c, sizeof c, "%dex/%dms", exact, miss);
        std::printf(" %11s", c);
      }
    }
    std::printf("\n");
  }
  check(power_first_bad > 0,
        "  power+first still fails somewhere (the comparison is not a no-op)");
  check(norm_firstpath_bad == 0,
        "  xcorr + FIRST-PATH is exact at every level, every shape");
  // THE NR-STYLE (NO-LAG) DETECTOR IS WORSE ON A REPEATED REPLICA, AND THAT IS
  // THE RESULT. Dropping the lag product removes the repeat check, which is
  // what rejects a lone noise spike or sidelobe (the nolag+1stp column above
  // shows the draws it loses). NR uses a plain matched filter because its PSS
  // does NOT repeat; a beacon with a repeated field keeps the 802.11-style
  // detector the waveform supports, and follows NR's ARCHITECTURE (acquisition
  // field, then pilots for fine tracking).
  check(nolag_bad > 0,
        "  no-lag on a REPEATED replica is worse: the repeat check is load-bearing");
  // AP-66, THE OTHER HALF OF NR. On a replica that appears twice the no-lag
  // detector has a rep1/rep2 ambiguity (-129 = one fine_len on legacy_guard).
  // NR's PSS appears once, so with the PSS as the replica the plain matched
  // filter has nothing to be ambiguous about and must be exact at every level;
  // if it is not, the failure is in the code, not in the architecture.
  check(nrpss_bad == 0,
        "  nr_pss: the PSS matched filter (no repeat check) is exact at every level");
  // Reported, not gated, because it is the claim under test rather than a
  // requirement: if normalising alone were enough, the selection rule would be
  // belt-and-braces rather than load-bearing.
  std::printf("\n  xcorr + FIRST-crossing: %d level(s) not exact -- %s\n",
              norm_first_bad,
              norm_first_bad == 0
                  ? "normalisation ALONE fixes the index too"
                  : "normalisation is NOT sufficient; the pick rule is still needed");

  // ---------------------------------------------------------------------
  // DOES THE KNOB NAME A FIXED THING? The whole point of normalising is that
  // `corr_scale` should mean the same test at every received level. Measured
  // through the PUBLIC API only, no internals exposed: for each level, the
  // SMALLEST corr_scale that still returns the exact index. If the statistic is
  // level-invariant that number is constant; if it is 4th-order-over-2nd it
  // must fall as 1/level^2.
  std::printf("\n=== smallest corr_scale that still detects exactly ===\n");
  std::printf("%-14s %-10s", "shape", "form");
  for (const double lv : kLevels) std::printf(" %8.0f", lv);
  std::printf("   spread\n");
  for (const auto& b : ds) {
    for (const auto tf : {Thr::kPowerRatio, Thr::kNormalizedXCorr,
                          Thr::kCoherence}) {
      // A single-copy replica runs nolag whatever is asked (residual() forces
      // it, as syncSearch does), so its other two rows would be duplicates
      // printed under the wrong name.
      if (b.replica_reps < 2 && tf != Thr::kCoherence) continue;
      std::printf("%-14s %-10s", b.name.c_str(),
                  tf == Thr::kPowerRatio ? "power"
                      : tf == Thr::kNormalizedXCorr ? "xcorr" : "nolag");
      double lo = 1e300, hi = 0.0;
      for (const double lv : kLevels) {
        // Walk corr_scale down in half-decades until the index stops being
        // exact; the last value that worked is the sensitivity edge.
        double edge = 0.0;
        for (double cs = 1e7; cs >= 1e-4; cs /= 3.1623) {
          bool ok = true;
          for (unsigned sd = 1; sd <= 4 && ok; ++sd) {
            const long long v = residual(b, lv, kSnrDb, kLead, kTail,
                                         static_cast<float>(cs),
                                         Pick::kFirstPath, sd, tf);
            if (v != kEndConvention) ok = false;
          }
          if (ok) edge = cs; else if (edge > 0.0) break;
        }
        std::printf(" %8.3g", edge);
        if (edge > 0.0) { lo = std::min(lo, edge); hi = std::max(hi, edge); }
      }
      std::printf("   %6.0fx\n", (lo < 1e299 && lo > 0) ? hi / lo : 0.0);
    }
  }
  std::printf("\nA constant row means one threshold works at every level.\n");
  std::printf("A row falling as 1/level^2 means the knob is a different test\n");
  std::printf("at every level, which is what the power-ratio form does.\n");

  // ---------------------------------------------------------------------
  // OVER THE AIR: MULTIPATH AND A CARRIER OFFSET.
  //
  // The bench is one cabled path with a sub-ppm clock pair. Neither holds over
  // the air on free-running clocks, and the two departures pull in OPPOSITE
  // directions on the pick rule: multipath is the case where kTargetedArgmax
  // returns the wrong path, and it is the case kFirstCrossing was originally
  // written for. Measured against the DIRECT path's beacon end, so returning a
  // stronger later tap scores as the bias it is.
  //
  // CFO here is 4.25 kHz = 8.5 ppm of 500 MHz, the measured free-running offset
  // between these two boards on internal clocks.
  // Run for the legacy beacon under the shipped xcorr threshold, then for
  // nr_pss, whose single-copy replica forces the plain matched filter. Both
  // must show the same thing, first-path exact on every channel and argmax
  // biased on the stronger echoes: the pick rule is a property of the channel,
  // not of the replica.
  struct Ota { Shape shape; Thr tf; const char* label; };
  const Ota otas[] = {{Shape::kLegacy, Thr::kNormalizedXCorr, "legacy beacon, xcorr threshold"},
                      {Shape::kNrPss, Thr::kCoherence, "nr_pss beacon, nolag threshold"}};
  for (const auto& ota : otas) {
    std::printf("\n=== over-the-air channels, %s ===\n", ota.label);
    const auto b = houdini::sync::shapes::make(ota.shape);
    const Channel chans[] = {
        {{{0, 1.0}}, 0.0, "1 path, no CFO"},
        {{{0, 1.0}}, 4250.0, "1 path, 8.5 ppm CFO"},
        {{{0, 1.0}, {8, 0.7}}, 4250.0, "echo +8 samp, -3 dB"},
        {{{0, 1.0}, {8, 1.4}}, 4250.0, "echo +8 samp, STRONGER"},
        {{{0, 1.0}, {24, 1.4}}, 4250.0, "echo +24 samp, STRONGER"},
        {{{0, 0.5}, {40, 1.4}}, 4250.0, "weak direct, echo +40 STRONGER"},
    };
    const bool nolag_only = ota.tf == Thr::kCoherence;
    std::printf("%-30s %13s %13s %13s %13s\n", "channel",
                nolag_only ? "nolag+first" : "xc+first",
                nolag_only ? "nolag+argmax" : "xc+argmax",
                nolag_only ? "nolag+1stpath" : "xc+1stpath",
                nolag_only ? "(same)" : "nolag+1stpath");
    int argmax_bias = 0, firstpath_bias = 0;
    for (const auto& ch : chans) {
      std::printf("%-30s", ch.name);
      struct MC { Thr tf; Pick pk; };
      const MC mcs[] = {{ota.tf, Pick::kFirstCrossing},
                        {ota.tf, Pick::kTargetedArgmax},
                        {ota.tf, Pick::kFirstPath},
                        {Thr::kCoherence, Pick::kFirstPath}};
      for (const auto& mc : mcs) {
        const Pick pk = mc.pk;
        long long lo = 1LL << 40, hi = -(1LL << 40);
        int miss = 0;
        for (unsigned sd = 1; sd <= 6; ++sd) {
          const long long v =
              residualCh(b, 1600.0, kSnrDb, kLead, kTail, kResyncCorrScale, pk,
                         sd, mc.tf, ch);
          if (v == kMiss) { ++miss; continue; }
          lo = std::min(lo, v); hi = std::max(hi, v);
        }
        char c[32];
        if (miss == 6) std::snprintf(c, sizeof c, "MISS");
        else if (lo == hi) std::snprintf(c, sizeof c, "%+lld", lo);
        else std::snprintf(c, sizeof c, "%+lld..%+lld", lo, hi);
        std::printf(" %13s", c);
        const long long worst = std::max(std::llabs(lo - kEndConvention),
                                         std::llabs(hi - kEndConvention));
        // COUNT THE SHAPE'S OWN THRESHOLD FORM ONLY. Keyed on the pick rule
        // alone, the extra nolag+1stpath column would be counted too, and the
        // checks below would silently be statements about the no-lag rule.
        if (miss < 6 && worst > 4 && mc.tf == ota.tf) {
          if (pk == Pick::kTargetedArgmax) ++argmax_bias;
          if (pk == Pick::kFirstPath) ++firstpath_bias;
        }
      }
      std::printf("\n");
    }
    std::printf("\n  channels where the rule is >4 samples off the DIRECT path:"
                "  argmax %d, first-path %d\n", argmax_bias, firstpath_bias);
    check(argmax_bias > 0,
          std::string("  ") + b.name +
              ": argmax IS biased on multipath (the comparison is not a no-op)");
    check(firstpath_bias == 0,
          std::string("  ") + b.name +
              ": first-path is exact on every channel");
  }

  // ---------------------------------------------------------------------
  // TIMING JITTER AGAINST SNR (reported, not gated; DEMO_VERIFICATION 8.160).
  // Tests a mechanism offered for nr_pss's higher silicon jitter at reduced
  // level: the first-path walk-back applies its fraction to a 2nd-order
  // statistic whose near-peak skirt is wider than the lag product's 4th-order
  // one, so at lower SNR it would land a sample EARLY more often. If so,
  // nr_pss's residual spread grows faster than legacy's as SNR falls AND is
  // biased negative; a spread that stays symmetric about the true end is plain
  // matched-filter timing noise. 8.160 records the outcome: every shape exact
  // down to 10 dB, the mechanism withdrawn.
  std::printf("\n=== residual spread against SNR (1600 counts, first-path, 16 draws) ===\n");
  std::printf("%-8s", "SNR dB");
  for (const auto& b : ds) std::printf(" %22s", b.name.c_str());
  std::printf("\n%-8s", "");
  for (size_t i = 0; i < sizeof(ds) / sizeof(*ds); ++i)
    std::printf(" %22s", "min..max  mean  sd");
  std::printf("\n");
  for (const double snr : {45.0, 25.0, 15.0, 10.0}) {
    std::printf("%-8.0f", snr);
    for (const auto& b : ds) {
      long long lo = 1LL << 40, hi = -(1LL << 40);
      double sum = 0.0, sum2 = 0.0;
      int n = 0, miss = 0;
      for (unsigned sd = 1; sd <= 16; ++sd) {
        const long long v = residual(b, 1600.0, snr, kLead, kTail,
                                     kResyncCorrScale, Pick::kFirstPath, sd,
                                     Thr::kNormalizedXCorr);
        if (v == kMiss) { ++miss; continue; }
        const long long e = v - kEndConvention;  // 0 is exact
        lo = std::min(lo, e); hi = std::max(hi, e);
        sum += static_cast<double>(e); sum2 += static_cast<double>(e * e);
        ++n;
      }
      char c[48];
      if (n == 0) {
        std::snprintf(c, sizeof c, "MISS x%d", miss);
      } else {
        const double mean = sum / n;
        const double var = n > 1 ? (sum2 - n * mean * mean) / (n - 1) : 0.0;
        std::snprintf(c, sizeof c, "%+lld..%+lld %+5.2f %4.2f%s", lo, hi, mean,
                      std::sqrt(std::max(0.0, var)), miss ? "*" : "");
      }
      std::printf(" %22s", c);
    }
    std::printf("\n");
  }
  std::printf("(* = some draws missed; nr_pss runs nolag in every column)\n");

  // FRACTIONAL TIMING (reported, not gated; 8.160). A real link's beacon
  // arrives BETWEEN samples, and a free-running clock walks it through every
  // phase. Every shape's integer index flips between two adjacent values
  // somewhere in tau = 0..1 (that is what rounding is); a detector-side cause
  // for a shape's extra jitter would show as a flip at a DIFFERENT tau than
  // legacy's, a three-value spread, or a flip that depends on the noise draw
  // over a wide band of tau (dither). The same flip point and two clean values
  // for all shapes put the difference outside the detector.
  std::printf("\n=== returned index against fractional delay (1600 counts, "
              "first-path, 8 draws; cell = residual min..max) ===\n");
  for (const double snr : {45.0, 27.0}) {
    std::printf("SNR %.0f dB\n%-6s", snr, "tau");
    for (const auto& b : ds) std::printf(" %13s", b.name.c_str());
    std::printf("\n");
    for (int t = 0; t <= 10; ++t) {
      const double tau = 0.1 * t;
      std::printf("%-6.1f", tau);
      for (const auto& b : ds) {
        long long lo = 1LL << 40, hi = -(1LL << 40);
        int miss = 0;
        for (unsigned sd = 1; sd <= 8; ++sd) {
          const long long v = residual(b, 1600.0, snr, kLead, kTail,
                                       kResyncCorrScale, Pick::kFirstPath, sd,
                                       Thr::kNormalizedXCorr, tau);
          if (v == kMiss) { ++miss; continue; }
          const long long e = v - kEndConvention;
          lo = std::min(lo, e); hi = std::max(hi, e);
        }
        char c[32];
        if (miss == 8) std::snprintf(c, sizeof c, "MISS");
        else if (lo == hi) std::snprintf(c, sizeof c, "%+lld", lo);
        else std::snprintf(c, sizeof c, "%+lld..%+lld", lo, hi);
        std::printf(" %13s", c);
      }
      std::printf("\n");
    }
  }

  // ---------------------------------------------------------------------
  // AP-72: WHAT THE FIRST-PATH RULE COSTS ON A LINK WITH NO MULTIPATH, AND
  // WHAT A ONE-SAMPLE GUARD BUYS BACK.
  //
  // A beacon between samples splits its matched-filter peak over two adjacent
  // taps. The back-scan admits the earlier one whenever it clears the -9 dB
  // floor, so on a clean link the rule reports a path one sample before the
  // arrival for a band of tau. `first_path_guard` skips exactly that tap.
  //
  // THREE WAYS TO MISMEASURE THIS (DEMO_VERIFICATION 8aj):
  //   - a "lobe reach" table that varies the back-scan window measures the
  //     -9 dB FLOOR, not any lobe: one more dB of floor, or 20 dB SNR, moves
  //     the reach from 1 to 2 on four of five shapes;
  //   - an adjacent-difference "jitter along tau" is 1/sqrt(N-1) of the
  //     sweep's own grid, not a jitter, and not the statistic the rig's
  //     frame-to-frame figure is;
  //   - counting DISTINCT indices on a tau grid of 0.1 lands exactly on the
  //     argmax's undecided point (tau = 0.5) and steps over the rule's own
  //     (tau = 0.74 for legacy), which makes the rule look like a stabiliser.
  //     It is not.
  // What follows uses the spread of the returned index across draws, on a
  // grid fine enough to find either rule's transition, and reports the RMS
  // against the true arrival beside it.
  std::printf("\n=== AP-72: first-path rule against the argmax, single path, "
              "and what guard 1 recovers ===\n");
  std::printf("%-14s %5s %7s %7s %7s %14s %14s %14s\n", "shape", "SNR",
              "argmax", "guard0", "guard1", "argmax", "guard0", "guard1");
  std::printf("%-14s %5s %-23s %s\n", "", "", "  RMS vs true arrival",
              "  worst minority share @ phase (resolution 1/48 = 0.021)");
  for (const double snr : {45.0, 30.0}) {
    for (const auto& b : ds) {
      const int w = shippedWindow(b);
      double sq[3] = {0.0, 0.0, 0.0}, minority[3] = {0.0, 0.0, 0.0};
      double at[3] = {0.0, 0.0, 0.0};
      int nq[3] = {0, 0, 0};
      int g0_vs_g1_diff = 0, g1_at_am_minus_1 = 0, g0_at_am_minus_1 = 0;
      for (int t = 0; t < 100; ++t) {
        const double tau = 0.01 * t;
        const double truth = static_cast<double>(kEndConvention) + tau;
        long long idx[3] = {kMiss, kMiss, kMiss};
        for (int rule = 0; rule < 3; ++rule) {
          const int win = rule == 0 ? 0 : w;
          const int guard = rule == 2 ? 1 : 0;
          // ONE PASS AT 48 DRAWS, NOT A SCREEN AND A REFINEMENT. A screen
          // misses a real minority share of 0.008 most of the time and then
          // prints 0.000, the value a decided rule prints. One pass has a
          // floor instead of a blind spot: the resolution is 1/48 = 0.021 and
          // anything under that reads as 0, which the header says.
          //
          // The statistic is the fraction of draws NOT returning the modal
          // index. Unlike a count of distinct values or an sd on integers it
          // tells one flip in twenty from a coin toss.
          std::map<long long, int> hist;
          int kept = 0;
          for (unsigned sd = 1; sd <= 48; ++sd) {
            const long long r = runAt(b, tau, sd, win, snr, guard);
            if (r == kMiss) continue;
            const double e = static_cast<double>(r) - truth;
            sq[rule] += e * e;
            ++nq[rule];
            ++hist[r];
            ++kept;
          }
          if (kept == 0) continue;
          // The modal index (the lowest one on a tie) and its count.
          int best_n = 0;
          for (const auto& kv : hist)
            if (kv.second > best_n) { best_n = kv.second; idx[rule] = kv.first; }
          const double m = 1.0 - static_cast<double>(best_n) / static_cast<double>(kept);
          if (m > minority[rule]) { minority[rule] = m; at[rule] = tau; }
        }
        // Per-phase comparisons on the modal index of each rule.
        if (idx[0] != kMiss && idx[2] != kMiss && idx[2] == idx[0] - 1)
          ++g1_at_am_minus_1;
        if (idx[0] != kMiss && idx[1] != kMiss && idx[1] == idx[0] - 1)
          ++g0_at_am_minus_1;
        if (idx[1] != kMiss && idx[2] != kMiss && idx[1] != idx[2]) ++g0_vs_g1_diff;
      }
      std::printf("%-14s %5.0f", b.name.c_str(), snr);
      for (int r = 0; r < 3; ++r)
        std::printf(" %7.3f", nq[r] ? std::sqrt(sq[r] / nq[r]) : 999.0);
      // THE PHASE IS PRINTED BESIDE THE SHARE, because the share alone means
      // opposite things depending on where it sits (the error DEMO_VERIFICATION
      // 8ah was retracted for). At phase 0.50 the truth is exactly between two
      // samples and both answers are equally right; at a rule's own transition
      // one answer is much further from the truth than the other. Guard 0's
      // worst share mostly sits at its transition, but not always (`nr` at
      // 30 dB puts it at 0.49), so the phase is printed for the reader rather
      // than reduced to a rule.
      for (int r = 0; r < 3; ++r)
        std::printf(" %8.3f@%.2f", minority[r], at[r]);
      std::printf("\n");
      // EVERY POINT MUST HAVE BEEN DETECTED, not merely one of them: a build
      // dropping half the detections leaves the RMS columns unmoved, because
      // the survivors are a biased subset.
      const int expect = 100 * 48;
      // Fails under: any dropped detection, biased or not.
      check(nq[0] == expect && nq[1] == expect && nq[2] == expect,
            std::string("every phase and draw detected on ") + b.name + " (" +
                std::to_string(nq[0]) + "/" + std::to_string(nq[1]) + "/" +
                std::to_string(nq[2]) + " of " + std::to_string(expect) + ")");
      // GUARD 1 AGAINST THE ARGMAX, AS AN EXACT PER-POINT CLAIM. An RMS
      // comparison would be true by construction on four of the five shapes:
      // with the split partner excluded there is usually no other candidate
      // above the floor, so guard 1 simply IS the argmax.
      // THE GUARD'S CONTRACT, ASSERTED LITERALLY: it must never return the tap
      // IMMEDIATELY before the argmax, because that tap is the split partner
      // of the same arrival. Guard 0 does return it, often, which is the whole
      // finding. This is exact, it is not true by construction, and it fails
      // for a guard that is off, inverted, or applied to the wrong index.
      //
      // It deliberately does NOT bound how far guard 1 may sit from the
      // argmax: `nr` has a second tap above the floor on about 1 % of phases
      // and legitimately lands TWO samples early there.
      // Fails under: the guard off (10 rows), or inverted (10 rows).
      check(g1_at_am_minus_1 == 0,
            std::string("guard 1 never returns the tap immediately before the "
                        "argmax: ") + b.name + " " +
                std::to_string(g1_at_am_minus_1) + " of " + std::to_string(100));
      // Fails under: the guard applied unconditionally, ignoring the knob.
      check(g0_at_am_minus_1 > 0,
            std::string("guard 0 DOES return that tap, so the guard has "
                        "something to prevent: ") + b.name + " " +
                std::to_string(g0_at_am_minus_1) + " of " + std::to_string(100));
      // POSITIVE CONTROL, ON REACHABILITY RATHER THAN ON QUALITY. Asserting
      // guard 0 measurably WORSE than the argmax would tie the suite to the
      // unguarded rule's quality, and a legitimate improvement to it would
      // break ten rows. What must be true for the knob to mean anything is
      // only that the two settings differ somewhere.
      // Fails under: the knob never reaching the correlator.
      check(g0_vs_g1_diff > 0,
            std::string("the two guard settings differ somewhere on ") + b.name +
                " (0 would mean the knob never reaches the correlator)");
    }
  }

  // AND THE GUARD MUST NOT COST WHAT THE RULE EXISTS FOR. The first-path rule
  // is there for MULTIPATH: over the air the stable reference is the direct
  // arrival, not the strongest one. A guard that bought single-path accuracy
  // by blinding the rule to real echoes would be a bad trade.
  //
  // SWEPT OVER FRACTIONAL DELAY, NOT ONLY AT ZERO. At tau = 0 no split peak
  // exists, so the guarded tap is never the one the rule wants and both
  // columns read the same value: the comparison could not fail. With tau
  // swept the guarded tap is live. All five shapes, including dot11, whose
  // lobe is the widest.
  std::printf("\n=== AP-72: the guard against genuine multipath (phase grid "
              "0.02, 4 draws; err = worst |residual - truth|) ===\n");
  std::printf("%-32s %-14s %8s %8s %9s %9s %9s\n", "channel", "shape",
              "err g0", "err g1", "differed", "g1 worse", "g1 better");
  {
    // The direct path sits 8.9 dB under the echo in the "weak direct" case,
    // against a -9.0 dB floor whose own comment records the case flipping
    // between -8.8 and -8.0 dB. It is kept because it is the hardest case, and
    // named here so a result there reads as the margin it is.
    const Channel mp[] = {
        {{{0, 1.0}, {8, 1.4}}, 0.0, "echo +8 samp, STRONGER"},
        {{{0, 1.0}, {24, 1.4}}, 0.0, "echo +24 samp, STRONGER"},
        {{{0, 0.5}, {40, 1.4}}, 0.0, "weak direct -8.9 dB, echo +40"},
        {{{0, 1.0}, {2, 1.4}}, 0.0, "echo +2 samp, STRONGER"},
        {{{0, 1.0}, {1, 1.4}}, 0.0, "echo +1 samp (UNRESOLVABLE)"},
    };
    int control_differed = 0;
    for (const auto& ch : mp) {
      for (const auto& b : ds) {
        std::printf("%-32s %-14s", ch.name, b.name.c_str());
        double worst[2] = {0.0, 0.0};
        int differed = 0, g1_worse = 0, g1_better = 0, points = 0, found = 0;
        // A GRID OF 0.02, NOT 0.25 AND NOT 0.05. On the +2 echo the settings
        // differ over a band of roughly [0.34, 0.43], where guard 1 is BETTER:
        // a quarter-sample grid steps over it and reads "no point differs",
        // and a 0.05 grid catches it by a single point, one small shift from
        // vacuous.
        // TWO SNRs (45 and 30 dB on alternate draws), because the claim in
        // comms-lib.h and 8aj is that the guard changes nothing over a sweep
        // of arrival phase, noise draw and SNR. The reported-only channels
        // stay at one SNR to bound the runtime.
        const bool asserted = std::string(ch.name).find("UNRESOLVABLE") == std::string::npos &&
                              std::string(ch.name).find("weak direct") == std::string::npos;
        for (int t = 0; t < 50; ++t) {
          const double tau = 0.02 * t;
          const double truth = static_cast<double>(kEndConvention) + tau;
          for (unsigned sd = 1; sd <= 4u; ++sd) {
            long long v[2];
            double err[2];
            const double snr = (asserted && (sd % 2u) == 0u) ? 30.0 : kSnrDb;
            for (int g = 0; g < 2; ++g) {
              v[g] = residualCh(b, 1600.0, snr, kLead, kTail, kResyncCorrScale,
                                Pick::kFirstPath, sd,
                                b.replica_reps < 2 ? Thr::kCoherence
                                                   : Thr::kNormalizedXCorr,
                                ch, g, tau);
              err[g] = v[g] == kMiss ? 1000.0
                                     : std::fabs(static_cast<double>(v[g]) - truth);
              worst[g] = std::max(worst[g], err[g]);
            }
            ++points;
            if (v[0] != kMiss && v[1] != kMiss) ++found;
            if (v[0] != v[1]) ++differed;
            if (err[1] > err[0] + 1e-9) ++g1_worse;
            if (err[1] < err[0] - 1e-9) ++g1_better;
          }
        }
        std::printf(" %8.2f %8.2f %9d %9d %9d\n", worst[0], worst[1], differed,
                    g1_worse, g1_better);
        control_differed += differed;
        // A DETECTION FLOOR FIRST. Every statistic here reads perfectly when
        // nothing is detected: `differed` counts agreement, so two settings
        // that both find nothing agree on everything.
        // Fails under: a correlator returning kMiss on any phase.
        check(found == points,
              std::string("both settings detect at every point on ") + ch.name +
                  ", " + b.name + " (" + std::to_string(found) + " of " +
                  std::to_string(points) + ")");
        // AND THE CLAIM IS "NEVER WORSE", NOT "NEVER DIFFERENT". On a
        // resolvable channel the guard may legitimately change the answer, and
        // where it does on the +2 echo it IMPROVES it. This is not true by
        // construction the way the single-path version was: guard 1 falls back
        // to the argmax, which on multipath is the ECHO, so a guard that
        // reached too far would fail here.
        // `asserted` above exempts the two reported-only channels by name: the
        // one-sample echo, which no rule can separate from a split peak in one
        // window (comms-lib.h), and the weak direct path at the floor's
        // margin. Renaming either channel changes what is asserted.
        if (asserted)
          // Fails under: guard forced to 2 (5 shapes fail on this channel).
          check(g1_worse == 0 && g1_better == differed,
                std::string("on ") + ch.name + ", " + b.name +
                    ": every point where guard 1 differs is an IMPROVEMENT (" +
                    std::to_string(g1_better) + " better, " +
                    std::to_string(g1_worse) + " worse, of " +
                    std::to_string(differed) + " differing)");
      }
    }
    // POSITIVE CONTROL. Without this the whole block passes on a build that
    // ignores the knob and applies the guard unconditionally, its table
    // reading as an improvement. The +1 echo is where the two settings must
    // differ.
    // Fails under: the knob ignored, so both settings behave identically.
    check(control_differed > 0,
          "the two guard settings differ somewhere: " +
              std::to_string(control_differed) +
              " points (0 would mean the knob is not reaching the correlator)");
  }

  // WHAT A 60 s SILICON LEG WOULD SEE, SIMULATED, BECAUSE THE GATE RESTS ON IT.
  // DEMO_VERIFICATION 8ak's gate design rests on its PASS statistics being
  // blind to the guard: the guard moves the rounding threshold in arrival
  // phase rather than adding a toggle, so both settings step once per cycle
  // and an adjacent-difference jitter differences it away. This block
  // regenerates that claim.
  //
  // The model, and why it is the right one: the shipped resync cadence is
  // 2604 ms, and at the measured clock rate the arrival phase advances tens of
  // samples between accepts, so successive detections see INDEPENDENT phase,
  // not a slow walk. Each leg draws 23 detections at uniform phase, which is
  // what a 60 s leg collects. `jitter` here is the adjacent-difference sd
  // divided by sqrt(2), the same statistic shape_campaign_summary.py reports,
  // so the number is comparable with the rig's.
  std::printf("\n=== AP-72: a simulated 60 s leg, both guard settings "
              "(40 legs x 23 detections, legacy, 45 dB) ===\n");
  {
    const auto& b = ds[0];
    const int w = shippedWindow(b);
    double jit[2] = {0.0, 0.0}, sd[2] = {0.0, 0.0};
    int legs = 0, arms_differed = 0;
    // One master seed is one realisation, and the difference between the two
    // settings changes sign between seeds (+0.018, -0.007, +0.003 over three),
    // so the check bounds its magnitude, not its sign.
    std::mt19937 phase_rng(20260904u);
    for (int leg = 0; leg < 40; ++leg) {
      std::vector<double> res[2];
      for (int k = 0; k < 23; ++k) {
        const double tau =
            (static_cast<double>(phase_rng()) + 0.5) / 4294967296.0;
        const unsigned seed = static_cast<unsigned>(leg * 23 + k + 1);
        long long got[2] = {kMiss, kMiss};
        for (int g = 0; g < 2; ++g) {
          got[g] = runAt(b, tau, seed, w, 45.0, g);
          if (got[g] != kMiss)
            res[g].push_back(static_cast<double>(got[g]) -
                             (static_cast<double>(kEndConvention) + tau));
        }
        if (got[0] != kMiss && got[1] != kMiss && got[0] != got[1]) ++arms_differed;
      }
      // EVERY DETECTION, NOT MERELY TWO. Without this floor a build that drops
      // most detections still reports 39 of 40 legs and prints a jitter four
      // times off the number the gate rests on.
      // Fails under: a correlator that returns kMiss on any phase.
      if (res[0].size() != 23 || res[1].size() != 23) continue;
      ++legs;
      for (int g = 0; g < 2; ++g) {
        // EXACTLY WHAT shape_campaign_summary.py REPORTS, so the number is
        // comparable with the rig's: the SD of the adjacent differences (mean
        // removed, divided by n-1 of the DIFFERENCES), then divided by root
        // two. The RMS about zero over the count of SAMPLES reads 2.3 % low.
        std::vector<double> d;
        for (size_t i = 1; i < res[g].size(); ++i) d.push_back(res[g][i] - res[g][i - 1]);
        double dm = 0.0;
        for (const double x : d) dm += x;
        dm /= static_cast<double>(d.size());
        double q = 0.0;
        for (const double x : d) q += (x - dm) * (x - dm);
        jit[g] += std::sqrt(q / static_cast<double>(d.size() - 1)) / std::sqrt(2.0);
        double m = 0.0, v = 0.0;
        for (const double x : res[g]) m += x;
        m /= static_cast<double>(res[g].size());
        for (const double x : res[g]) v += (x - m) * (x - m);
        sd[g] += std::sqrt(v / static_cast<double>(res[g].size() - 1));
      }
    }
    const double n = legs > 0 ? static_cast<double>(legs) : 1.0;
    std::printf("%-10s %14s %14s\n", "setting", "mean jitter", "mean sd");
    for (int g = 0; g < 2; ++g)
      std::printf("guard %-4d %14.3f %14.3f\n", g, jit[g] / n, sd[g] / n);
    // The premise the gate rests on: the two settings are indistinguishable
    // in these statistics. A tenth of a sample is far inside the 1.3 the gate
    // allows and inside the leg-to-leg spread of the bench itself.
    // POSITIVE CONTROL FIRST, because that claim is an EQUALITY, and an
    // equality passes vacuously when both arms are the same configuration
    // (each guard mutation named below makes them so). On a clean single path
    // guard 0 returns the earlier tap on about a quarter of phases, so the two
    // arms' residual sequences must differ even though their jitter does not.
    // Fails under: the knob ignored, the guard off, or the guard applied
    // unconditionally.
    check(arms_differed > 0,
          "the two simulated arms are different configurations: " +
              std::to_string(arms_differed) + " of 920 detections differ");
    // Fails under: nothing in the guard; this is the gate's own prediction and
    // it is insensitive to the guard by design, which is the point 8ak makes.
    check(std::fabs(jit[0] - jit[1]) / n < 0.1,
          "a simulated leg cannot tell the two guard settings apart by jitter: " +
              std::to_string(jit[0] / n) + " against " + std::to_string(jit[1] / n));
    check(legs == 40,
          "every simulated leg detected on every one of its 23 phases: " +
              std::to_string(legs) + " of 40");
  }

  // THE SHAPE OF THE CORRELATION LOBE, THE SPECIFICATION FOR AP-75's
  // sub-sample estimator (DEMO_VERIFICATION 8al records the withdrawn one).
  // Three-point estimators fail on it: a parabola assumes a smooth lobe and
  // the lobe is nearly a delta; a ratio of the bracketing pair assumes the
  // neighbours are SYMMETRIC at zero delay and they are not (the table prints
  // them at the detected index), so near a whole-sample arrival the fixed
  // asymmetry outweighs the delay's and the estimate takes the wrong sign. An
  // estimator has to use the replica's own autocorrelation, which is what
  // this table measures.
  std::printf("\n=== AP-72: the correlation lobe at zero fractional delay "
              "(amplitude relative to the peak) ===\n");
  std::printf("%-14s %9s %9s %9s %9s\n", "shape", "peak-2", "peak-1", "peak+1",
              "peak+2");
  for (const auto& b : ds) {
    houdini::sync::sim::Channel sc;
    sc.snr_db = 60.0;
    sc.peak_counts = 1600.0;
    const long long len = static_cast<long long>(b.core.size());
    const long long pos = 4000, end = pos + len, s0 = end - kLead,
                    n = kLead + kTail;
    const std::vector<std::complex<int16_t>> buf =
        sc.receive(b.core, pos, s0, n, 1);
    const std::vector<std::complex<float>> raw =
        CommsLib::toCorrelatorScale(buf.data(), static_cast<size_t>(n));
    const std::vector<std::complex<float>> corr =
        CommsLib::correlate_mt(raw, b.replica);
    // AT THE INDEX THE DETECTOR REPORTS, NOT AT THE GLOBAL ARGMAX. A repeated
    // preamble has several equal correlation peaks, so the argmax over the
    // whole window lands on a different copy from seed to seed and the printed
    // neighbours then belong to whichever copy happened to win. The detector's
    // own index is the one every other number here is about.
    const CommsLib::BeaconResult det = CommsLib::find_beacon_ex(
        buf.data(), b.replica, static_cast<size_t>(n), kResyncCorrScale,
        Pick::kTargetedArgmax,
        b.replica_reps < 2 ? Thr::kCoherence : Thr::kNormalizedXCorr,
        shippedWindow(b), CommsLib::kDefaultFirstPathFloorDb, 0);
    if (det.index < 1 || static_cast<size_t>(det.index) + 2 >= corr.size()) {
      std::printf("%-14s   (no detection)\n", b.name.c_str());
      continue;
    }
    const size_t top = static_cast<size_t>(det.index);
    const double best = std::abs(corr[top]);
    std::printf("%-14s", b.name.c_str());
    for (const int d : {-2, -1, 1, 2}) {
      const long long k = static_cast<long long>(top) + d;
      const double a = (k >= 0 && k < static_cast<long long>(corr.size()))
                           ? std::abs(corr[static_cast<size_t>(k)]) / best
                           : 0.0;
      std::printf(" %9.4f", a);
    }
    std::printf("\n");
    // Informational, deliberately not a threshold: the asymmetry of peak-1
    // and peak+1 at zero delay is the point (see the block's header).
  }

  // ---------------------------------------------------------------------
  // THE NO-LAG FALSE-CROSSING RATE ON NOISE, AGAINST ITS MODEL (reported;
  // DEMO_VERIFICATION 8.155, 8.159: one rejected noise-window crossing per
  // acquisition hunt on nr_pss). The coherence of a pure-noise window against
  // an L-tap replica is Beta(1, L-1), so P(coh > bar) per index is
  // (1 - bar)^(L-1) = 0.9^127 = 1.5e-6 at bar 0.1, and the lag product's
  // (coh1 * coh2) is far rarer. Over 16 noise-only hunt windows of 12288
  // samples (196608 indices) the model gives about 0.3 no-lag crossings in
  // total at bar 0.1 and about 0 for xcorr; at bar 0.01 (corr_scale 100, the
  // resync retry ladder's reach) no-lag crosses in nearly EVERY window
  // (0.99^127 = 0.28 per index) and xcorr in almost none. A no-lag rate an
  // order of magnitude off either way means 8.155's mechanism is wrong.
  std::printf("\n=== noise-only hunt windows: how often each form crosses ===\n");
  std::printf("%-10s %-8s %10s %10s\n", "corr_scale", "form", "windows", "crossed");
  {
    const auto pss = houdini::sync::shapes::make(Shape::kNrPss);
    const auto leg = houdini::sync::shapes::make(Shape::kLegacy);
    for (const float cs : {10.0f, 100.0f}) {
      for (int form = 0; form < 2; ++form) {
        const bool nolag = form == 0;
        int crossed = 0;
        const int nwin = 16;
        for (int w = 0; w < nwin; ++w) {
          const std::vector<std::complex<int16_t>> buf = noiseWindow(9000u + w);
          const ssize_t idx = CommsLib::find_beacon_avx(
              buf.data(), nolag ? pss.replica : leg.replica, buf.size(), cs,
              Pick::kFirstClusterRefined,
              nolag ? Thr::kCoherence : Thr::kNormalizedXCorr);
          if (idx >= 0) ++crossed;
        }
        std::printf("%-10.0f %-8s %10d %10d\n", cs, nolag ? "nolag" : "xcorr",
                    nwin, crossed);
      }
    }
    // P3: the same noise windows at the bar a per-window probability implies.
    // At pfa 0.1 over 12288 samples the coherence bar is 0.088 for 128 taps
    // (1 - (0.1/12288)^(1/127)), and the expected crossings over 16 windows
    // are ~1.6; the corr_scale 100 bar (0.01) above crosses on every window.
    // Asserted: crossed <= 4 (a 2.5x allowance on a Poisson mean of 1.6).
    {
      const double pfa = 0.1;
      const double bar = houdini::sync::ThresholdPolicy::coherenceBar(pss.replica.size(), pfa, 12288);
      int crossed = 0;
      for (int w = 0; w < 16; ++w) {
        const std::vector<std::complex<int16_t>> buf = noiseWindow(9000u + w);
        const ssize_t idx = CommsLib::find_beacon_avx(buf.data(), pss.replica, buf.size(),
                                                      static_cast<float>(1.0 / bar),
                                                      Pick::kFirstClusterRefined, Thr::kCoherence);
        if (idx >= 0) ++crossed;
      }
      std::printf("pfa %.2f  bar %.4f  windows 16  crossed %d (expected ~1.6)\n", pfa, bar, crossed);
      check(crossed <= 4, "P3: the pfa-derived coherence bar crosses noise windows at about the stated rate (" +
                              std::to_string(crossed) + " of 16 at pfa 0.1)");
    }
  }

  // ---------------------------------------------------------------------
  // THE BEACON CFO ESTIMATOR AGAINST FRACTIONAL DELAY (reported, not gated;
  // DEMO_VERIFICATION 8.114 the silicon scatter, 8.164 what this table shows).
  // On a real link the beacon sits BETWEEN samples, so the samples after the
  // last repetition are not zero but the interpolation tail of the beacon's
  // edge, and any estimator window that touches that edge multiplies real
  // beacon samples by that tail.
  //
  // Window placements, all Receiver::estimateCFO's arithmetic (rep2 against
  // rep1 at lag fine_len) rebuilt on the shape geometry:
  //   guard 0   windows exactly on the fine field, index from the detector
  //   guard +8  the shipped placement (sync.cfo.index_guard 8, AP-39): both
  //             windows slid 8 samples LATER, derived on an integer-delay model
  //             where the trailing samples are exactly zero
  //   margin N  windows shrunk N samples at BOTH ends, so neither touches an
  //             edge; the NR/802.11 way of using a cyclic field
  // Eight edge samples are 1/8 of the sum for the short-field shapes (nr,
  // dot11: L 64), so "guard +8" is worst there. Mean and sd over 8 noise draws
  // so a bias can be told from the floor.
  std::printf("\n=== beacon CFO error at true CFO 0 vs fractional delay "
              "(45 dB, 8 draws, mean/sd Hz) ===\n");
  // Two interpolation kernels: a 33-tap Hann-windowed sinc (smooth edges, the
  // gentlest case) and a 129-tap one (a sharper band edge whose ringing
  // reaches further, closer to what the RFDC decimation filter does to a
  // hard-edged burst).
  struct Win { const char* name; int shift; int margin; int R; };
  const Win wins[] = {{"guard 0, 33-tap", 0, 0, 16}, {"guard +8, 33-tap", 8, 0, 16},
                      {"margin 4, 33-tap", 0, 4, 16}, {"guard +8, 129-tap", 8, 0, 64},
                      {"margin 4, 129-tap", 0, 4, 64}, {"margin 12, 129-tap", 0, 12, 64}};
  for (const auto& w : wins) {
    std::printf("-- %s\n%-6s", w.name, "tau");
    for (const auto& b : ds) std::printf(" %15s", b.name.c_str());
    std::printf("\n");
    for (int t = 0; t <= 10; t += 2) {
      const double tau = 0.1 * t;
      std::printf("%-6.1f", tau);
      for (const auto& b : ds) {
        const std::vector<cf> core = tau != 0.0 ? fracDelay(b.core, tau, w.R) : b.core;
        double pk = 0.0, mean_p = 0.0;
        for (const auto& v : b.core) {
          pk = std::max(pk, static_cast<double>(std::norm(v)));
          mean_p += std::norm(v);
        }
        mean_p /= static_cast<double>(b.core.size());
        const double scale = 1600.0 / std::sqrt(pk);
        const double ns = std::sqrt((mean_p / std::pow(10.0, 45.0 / 10.0)) / 2.0);
        const long long pos = 4000, n = 6000;
        double sum = 0.0, sum2 = 0.0;
        int cnt = 0;
        for (unsigned seed = 1; seed <= 8; ++seed) {
          std::mt19937 g(700u + seed);
          auto u01 = [&g]() { return (static_cast<double>(g()) + 0.5) / 4294967296.0; };
          auto gauss = [&]() { return std::sqrt(-2.0 * std::log(u01())) * std::cos(2.0 * M_PI * u01()); };
          std::vector<std::complex<int16_t>> buf(n);
          for (long long i = 0; i < n; ++i) {
            cf v(0.f, 0.f);
            if (i >= pos && i < pos + static_cast<long long>(core.size())) v = core[i - pos];
            buf[i] = std::complex<int16_t>(
                static_cast<int16_t>(v.real() * scale + gauss() * ns * scale),
                static_cast<int16_t>(v.imag() * scale + gauss() * ns * scale));
          }
          const Thr tf = b.replica_reps < 2 ? Thr::kCoherence : Thr::kNormalizedXCorr;
          const ssize_t idx = CommsLib::find_beacon_avx(buf.data(), b.replica, n,
                                                        kResyncCorrScale,
                                                        Pick::kFirstPath, tf);
          if (idx < 0) continue;
          const long long end = idx + static_cast<long long>(b.replica_tail()) + 1;
          const long long start = end - static_cast<long long>(b.core.size());
          const long long L = static_cast<long long>(b.fine_len);
          const long long g1 = start + static_cast<long long>(b.fine_off) + w.shift + w.margin;
          const long long g2 = g1 + L;
          std::complex<double> r(0.0, 0.0);
          for (long long i = 0; i < L - 2 * w.margin; ++i) {
            const std::complex<double> a(buf[g1 + i].real(), buf[g1 + i].imag());
            const std::complex<double> c(buf[g2 + i].real(), buf[g2 + i].imag());
            r += std::conj(a) * c;
          }
          const double hz = std::arg(r) / (2.0 * M_PI * L) * kRate;
          sum += hz; sum2 += hz * hz; ++cnt;
        }
        if (cnt == 0) { std::printf(" %15s", "MISS"); continue; }
        const double mean = sum / cnt;
        const double var = cnt > 1 ? (sum2 - cnt * mean * mean) / (cnt - 1) : 0.0;
        char c[32];
        std::snprintf(c, sizeof c, "%+6.0f/%4.0f", mean, std::sqrt(std::max(0.0, var)));
        std::printf(" %15s", c);
      }
      std::printf("\n");
    }
  }

  std::printf("\nRESULT: %s (%d failure(s))\n", g_fail ? "FAIL" : "PASS", g_fail);
  return g_fail ? 1 : 0;
}
