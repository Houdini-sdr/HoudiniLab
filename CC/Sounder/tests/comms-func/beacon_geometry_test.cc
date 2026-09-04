/**
 * @file beacon_geometry_test.cc
 * @brief Where the detector's index lands, for each candidate beacon, as a
 *        function of RECEIVED LEVEL. NO hardware.
 *
 * AP-34(a) added an 802.11 GI2 guard to the beacon, shipped it to silicon, and
 * had to revert: the guard moved `find_beacon`'s returned index by a measured
 * -274 samples, which broke the invariant the entire timing chain rests on --
 *
 *     sync_index == houdiniBeaconEnd() == strobe + beacon_size
 *
 * -- so `beaconSnrDb()` measured a window of pre-beacon noise, reported 10.5 dB
 * against a true 48.3, and the 30 dB floor rejected every resync detection.
 * Acquisition still worked, so the demo came up and looked healthy while the
 * liveness path was dead. That cost a bench session.
 *
 * THE GUARD WAS NOT THE CAUSE. This test found the real one, offline, in
 * minutes. The resync path selected the EARLIEST threshold crossing in its
 * search window. The beacon's own STS preamble is 16-periodic and 16 divides the
 * 128-sample correlator lag, so the STS field is perfectly lag-128 self-coherent
 * and manufactures crossings a few hundred samples before the true peak. Whether
 * those crossings win is a function of received level, because the threshold
 * test `corr_scale * |gc[i]|^2 |gc[i-L]|^2 > sum |gc|^2` compares a 4th-order
 * quantity to a 2nd-order one and is therefore NOT scale invariant. The guard
 * did not introduce the fault; it lowered the level at which the fault appears,
 * from ~3200 counts peak to ~400.
 *
 * So the sweep below is the actual regression test, and it is a two-sided one:
 *   - with the shipped kFirstCrossing rule EVERY candidate beacon false-locks
 *     somewhere in the level range a real link spans;
 *   - with kTargetedArgmax -- legal only because the resync slice is ~812
 *     samples against a beacon copy spacing of one full frame -- every candidate
 *     lands
 *     on the beacon end at every level.
 * If a future change reintroduces a level-dependent index, this fails.
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
#include <random>
#include <set>
#include <utility>
#include <string>
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

/// Place `core` so its END sits at `end`, in a noise buffer, and ask the real
/// detector -- not a replica. A replica would agree with itself and prove
/// nothing, which is the whole reason AP-34(a) needed silicon to find its bug.
/// Returns the returned index MINUS the true beacon end, or kMiss.
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
                     unsigned seed, Thr thresh_form, const Channel& ch) {
  const long long len = static_cast<long long>(b.core.size());
  houdini::sync::sim::Channel sc;
  sc.taps.clear();
  for (const auto& tp : ch.taps) sc.taps.push_back({tp.first, {tp.second, 0.0}});
  sc.cfo_hz = ch.cfo_hz;
  sc.rate_hz = kRate;
  sc.snr_db = snr_db;
  sc.peak_counts = peak_counts;
  const long long pos = 4000, end = pos + len, s0 = end - lead, n = lead + tail;
  std::vector<std::complex<int16_t>> buf = sc.receive(b.core, pos, s0, n, seed);
  // A single-copy replica (nr_pss) has no repeat to check: the receiver forces
  // the plain matched filter for it (syncSearch), and so does this test, so
  // every column below measures the form the shape would actually run with.
  if (b.replica_reps < 2) thresh_form = Thr::kCoherence;
  const ssize_t idx =
      CommsLib::find_beacon_avx(buf.data(), b.replica, n, corr_scale, pick,
                                thresh_form);
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

/// Residual on a SINGLE-PATH channel at fractional delay `frac`, taken with an
/// explicit first-path back-scan window. Window 0 disables the back-scan, so
/// the returned index is the argmax; that is the reference the AP-72 section
/// measures every other window against.
long long residualPick(const Desc& b, double frac, unsigned seed,
                       int first_path_window, double snr_db);
/// The same run, reporting the sub-sample estimate: `.first` is the integer
/// residual (kMiss when nothing was found) and `.second` the detector's
/// frac_offset at that index.
std::pair<long long, double> residualFrac(const Desc& b, double frac,
                                          unsigned seed, double snr_db);
/// The first-path back-scan window SyncConfig::resolve() derives for a shape:
/// half its replica (sync_config.cc), 64 for a 128-tap replica and 32 for a
/// 64-tap one. Measuring at one fixed width instead read dot11 as 64 samples
/// biased, which is this test's error and not the detector's.
int shippedWindow(const Desc& b);

// The shipped resync slice at 122.88 MSPS with the 2026-09-02 defaults
// (scatter_tol 246): lead = 246 + 256, tail = 246 + 64. sync_geometry.h owns the
// derivation; these are the values it produces, restated so this test says what
// geometry it is testing rather than pulling in the whole header.
constexpr long long kLead = 502, kTail = 310;
// What SyncConfig::resolve() derives for the shipped Houdini config: half a
// replica, capped (8.176 records 64, and 32 for dot11 whose replica is short).
// The AP-72 sweep uses the one value for every shape so the columns compare.
constexpr int kShippedFirstPathWindow = 64;
constexpr float kResyncCorrScale = 100.0f;  // files/houdini-*.json corr_scale
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


int shippedWindow(const Desc& b) {
  return static_cast<int>(b.replica.size() / 2);
}

std::pair<long long, double> residualFrac(const Desc& b, double frac,
                                          unsigned seed, double snr_db) {
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
      Pick::kFirstPath, tf, shippedWindow(b),
      CommsLib::kDefaultFirstPathFloorDb);
  const long long rep_tail = static_cast<long long>(b.replica_tail());
  if (r.index < 0) return {kMiss, 0.0};
  return {s0 + r.index + rep_tail - end, r.frac_offset};
}

long long residualPick(const Desc& b, double frac, unsigned seed,
                       int first_path_window, double snr_db) {
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
      CommsLib::kDefaultFirstPathFloorDb);
  const long long rep_tail = static_cast<long long>(b.replica_tail());
  return r.index < 0 ? kMiss : s0 + r.index + rep_tail - end;
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
    // NB both rows below use the POWER-RATIO threshold, which is no longer the
    // shipped one -- they compare PICK RULES with the threshold held fixed at
    // the historical form, which is what makes the pre/post comparison honest.
    // The shipped combination is xcorr + first-path and it is exercised by the
    // matrix further down, not here.
    std::printf("\n=== %s (power-ratio threshold) ===\n",
                argmax ? "kTargetedArgmax (resync, 2026-09-02 morning)"
                       : "kFirstCrossing (resync, before 2026-09-02)");
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
      // Not a defect being tolerated: this asserts the OLD rule really is
      // broken, so the test above is measuring a fix rather than a no-op. If
      // this ever passes cleanly, the mechanism changed and both branches need
      // re-deriving before the argmax result can be trusted.
      check(level_dependent > 0,
            "  the old rule DOES false-lock somewhere (the fix is not a no-op)");
    }
  }

  // Sensitivity floor: every candidate must still be detectable at a level well
  // below where the bench runs, or a shape wins the index test by being
  // undetectable. Checked with the rule we actually ship.
  std::printf("\n");
  for (const auto& b : ds) {
    // Sensitivity floor under the SHIPPED combination, not the historical one.
    const Row r = sweepAt(b, 200.0, Pick::kFirstPath, Thr::kNormalizedXCorr);
    check(r.miss == 0 && r.lo == kEndConvention && r.hi == kEndConvention,
          "  " + b.name + ": detected at 200 counts peak, index exact");
  }

  // THE CHECK THAT WOULD HAVE PREVENTED AP-34(a). beacon_shapes.h is about to
  // become the thing Config::genPilots builds from, and the bench probes already
  // read its dumped waveforms. If the header's `legacy` ever stops being the
  // beacon config actually transmits, the bench measures one waveform and the
  // build ships another -- which is exactly how a guard variant reached silicon
  // with nobody having derived its index convention. So rebuild config.cc's
  // beacon here, by its own recipe (genPilots: 15 x STS(16) then 2 x gold(128),
  // each through Utils::float_to_cint16), and require sample equality.
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
          "  beacon_shapes 'legacy' is sample-identical to Config::genPilots");
  }

  // NO TWO TONES ON ONE BIN. The NR fields are built in the frequency domain,
  // and the first version of toneIfft "parked" a DC-landing tone at a bin that
  // was already occupied, so the tracking symbol shipped with one tone doubled
  // and one missing. Nothing failed: transmit and correlator shared the same
  // malformed symbol, so it detected fine and merely measured a beacon nobody
  // designed. Check the property directly -- every field must occupy as many
  // distinct non-DC bins as it has tones.
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
  // what the threshold MEANS: the shipped statistic is 4th order in received
  // amplitude over 2nd, so `corr_scale` is a different test at every level.
  // Measured separately: the statistic at the true peak runs 0.0777 to 321.4
  // across a 64x level sweep, a spread of 4136 (= 64^2). Normalised -- divide
  // by the energy term squared, Schmidl & Cox 1997 -- it runs 0.9845 to 0.9843.
  //
  // The prediction under test, stated before the numbers: with the normalised
  // statistic the preamble plateau sits at 1/L^2, which is level-INDEPENDENT
  // and far below any sensible bar, so EVEN THE OLD earliest-crossing rule
  // should land on the beacon end at every level. If that holds, the
  // normalisation subsumes the selection fix rather than merely complementing
  // it. If it does not, the two are independent and both are needed.
  std::printf("\n=== threshold form x pick rule, over the level sweep ===\n");
  std::printf("cells: levels (of %zu) whose index is exact / levels that MISS\n",
              sizeof(kLevels) / sizeof(*kLevels));
  std::printf("%-13s %11s %11s %11s %11s %11s %11s %11s %11s\n", "shape",
              "pow+first", "pow+argmx", "pow+1stpth", "xc+first",
              "xc+argmx", "xc+1stpth", "nolag+frst", "nolag+1stp");
  int norm_first_bad = 0, norm_argmax_bad = 0, power_first_bad = 0;
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
          norm_argmax_bad += nlev - exact;
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
  check(norm_argmax_bad == 0,
        "  xcorr + FIRST-PATH is exact at every level, every shape");
  // THE NR-STYLE DETECTOR IS MEASURABLY WORSE HERE, AND THAT IS THE RESULT.
  // Dropping the lag product removes the repeat check, which is what rejects a
  // lone noise spike or sidelobe. Measured at corr_scale 100, 8 noise draws:
  // legacy and legacy_guard each lose one draw (-190 and -129 samples), dot11
  // loses six of eight, nr four of eight; xcorr+first-path is exact on all 32.
  // NR uses a plain matched filter because PSS does NOT repeat. Our beacon DOES
  // have a repeated field, so using it buys real robustness -- follow NR's
  // ARCHITECTURE (acquisition field, then pilots for fine tracking) and keep the
  // 802.11-style detector that the waveform actually supports.
  check(nolag_bad > 0,
        "  no-lag on a REPEATED replica is worse: the repeat check is load-bearing");
  // AP-66, THE OTHER HALF OF NR, STATED BEFORE THE NUMBERS. The rows above
  // measured NR's detector on a replica that appears twice and found the
  // rep1/rep2 ambiguity (-129 = one fine_len on legacy_guard). NR's PSS
  // appears once. PREDICTION: with the PSS as the replica the plain matched
  // filter has nothing to be ambiguous about and is exact at every level; if
  // it is not, the failure is in the code and not in the architecture.
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
  std::printf("at every level, which is what the shipped form does.\n");

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
  // Run for the shipped beacon under the shipped threshold, then for nr_pss,
  // whose threshold is necessarily the plain matched filter. PREDICTION for
  // nr_pss: first-path exact on every channel, argmax biased on the stronger
  // echoes just as it is for legacy -- the pick rule is a property of the
  // channel, not of the replica.
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
        // COUNT THE SHAPE'S OWN THRESHOLD FORM ONLY. Keying on the pick rule
        // alone lumped xcorr+first-path together with nolag+first-path, and
        // since the shipped column is exact on every channel it contributed
        // nothing -- so the gate below was silently a statement about the
        // NO-LAG rule.
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
  // TIMING JITTER AGAINST SNR, FOR THE 8.160 OBSERVATION. On silicon at
  // reduced transmit level nr_pss's adjacent-difference jitter read 1.3-2.2x
  // legacy's while at full level the two were indistinguishable. The mechanism
  // offered there, BEFORE this ran: the first-path walk-back applies its
  // fraction to a 2nd-order statistic whose near-peak skirt is wider than the
  // lag product's 4th-order one, so at lower SNR it lands a sample EARLY more
  // often. PREDICTION, stated first: if that is the mechanism, nr_pss's
  // residual spread grows faster than legacy's as SNR falls AND its errors are
  // biased negative (early). If the spread grows but stays symmetric about the
  // true end, it is plain matched-filter timing noise and the story is wrong.
  // Reported, not gated: this section exists to test a mechanism, not a
  // requirement.
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

  // FRACTIONAL TIMING. The sweep above came back exact for every shape down to
  // 10 dB, so detector noise is NOT the source of the silicon jitter and the
  // 8.160 mechanism is wrong as stated. What that sweep never exercised is a
  // beacon that arrives BETWEEN samples, which a real link always does and
  // which the free-running clock walks through continuously. PREDICTION,
  // stated first: every shape's integer index must flip between two adjacent
  // values somewhere in tau = 0..1 (that is what rounding is); the mechanism
  // that would explain nr_pss reading more jitter on silicon is a flip that
  // happens at a DIFFERENT tau than legacy's, or a three-value spread, or a
  // flip that depends on the noise draw over a wide band of tau (dither).
  // Same flip point and two clean values for all shapes means the silicon
  // difference is not in the detector either.
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
  // AP-72: HOW WIDE IS THE CORRELATION LOBE, AND HOW OFTEN DOES THE FIRST-PATH
  // RULE PICK INSIDE IT? 8.177 found the pick one sample before the argmax on
  // 7 of 24 golden windows, with the earlier neighbour's statistic 3.6 to
  // 14.9 dB down. The proposed mechanism: a beacon between samples splits the
  // matched-filter peak over two adjacent taps, and the -9 dB first-path floor
  // admits the earlier one whenever the fractional timing puts enough energy
  // there. This measures the mechanism instead of assuming it, on a SINGLE
  // PATH channel where the only correct answer is the sample nearest the true
  // end, using the shipped detector through its window argument:
  //   window 0 -> no back-scan at all, so the pick IS the argmax;
  //   window k -> the back-scan may reach k samples before the argmax.
  // The first window at which the returned index stops moving is the lobe's
  // half-width in samples: how far the same physical path reaches. Anything
  // the rule picks inside that reach is the peak's own skirt, not an earlier
  // path, and it toggles with tau rather than describing the channel.
  std::printf("\n=== AP-72: first-path reach into its own peak, single path "
              "(cells = index - argmax over tau = 0.05..0.95, 8 draws) ===\n");
  std::printf("%-14s", "shape");
  for (const int w : {1, 2, 3, 4}) std::printf("  window %-6d", w);
  std::printf("  %-13s\n", "shipped");
  for (const auto& b : ds) {
    std::printf("%-14s", b.name.c_str());
    for (const int w : {1, 2, 3, 4, shippedWindow(b)}) {
      long long lo = 1LL << 40, hi = -(1LL << 40);
      for (int t = 1; t <= 19; ++t) {
        const double tau = 0.05 * t;
        for (unsigned sd = 1; sd <= 8; ++sd) {
          const long long a = residualPick(b, tau, sd, 0, kSnrDb);
          const long long v = residualPick(b, tau, sd, w, kSnrDb);
          if (a == kMiss || v == kMiss) continue;
          lo = std::min(lo, v - a);
          hi = std::max(hi, v - a);
        }
      }
      char c[32];
      if (lo > hi) std::snprintf(c, sizeof c, "MISS");
      else if (lo == hi) std::snprintf(c, sizeof c, "%+lld", lo);
      else std::snprintf(c, sizeof c, "%+lld..%+lld", lo, hi);
      std::printf("  %-13s", c);
    }
    std::printf("\n");
  }

  // And the share of the jitter it explains. Over a fine tau sweep the correct
  // index steps ONCE (at tau = 0.5, where the nearest sample changes); every
  // other change of the returned index between adjacent tau is the rule
  // moving inside its own peak. `toggles` counts the (tau, seed) points where
  // the pick differs from the argmax at all; the two jitter columns are the
  // adjacent-difference sd of each series, per seed and averaged, which is the
  // statistic the silicon campaigns report (0.7 to 1.7 samples there).
  std::printf("\n=== AP-72: pick against argmax over tau = 0..1 in 0.02, 8 "
              "draws, shipped window ===\n");
  std::printf("%-14s %5s %10s %12s %12s %12s\n", "shape", "SNR", "toggles",
              "of points", "argmax jit", "pick jit");
  for (const double snr : {45.0, 30.0}) {
    for (const auto& b : ds) {
      const int w = shippedWindow(b);
      int toggles = 0, points = 0;
      double jam = 0.0, jpk = 0.0;
      int seeds = 0;
      for (unsigned sd = 1; sd <= 8; ++sd) {
        std::vector<long long> am, pk;
        for (int t = 0; t <= 50; ++t) {
          const double tau = 0.02 * t;
          const long long a = residualPick(b, tau, sd, 0, snr);
          const long long v = residualPick(b, tau, sd, w, snr);
          if (a == kMiss || v == kMiss) continue;
          ++points;
          if (a != v) ++toggles;
          am.push_back(a);
          pk.push_back(v);
        }
        const auto jitter = [](const std::vector<long long>& x) {
          if (x.size() < 2) return 0.0;
          double q = 0.0;
          for (size_t i = 1; i < x.size(); ++i) {
            const double d = static_cast<double>(x[i] - x[i - 1]);
            q += d * d;
          }
          return std::sqrt(q / static_cast<double>(x.size() - 1));
        };
        if (am.size() > 1) {
          jam += jitter(am);
          jpk += jitter(pk);
          ++seeds;
        }
      }
      const double n = seeds > 0 ? static_cast<double>(seeds) : 1.0;
      std::printf("%-14s %5.0f %10d %12d %12.2f %12.2f\n", b.name.c_str(), snr,
                  toggles, points, jam / n, jpk / n);
    }
  }

  // THE SWEEP ABOVE MOVES TAU AND THE NOISE TOGETHER, WHICH IS NOT THE SILICON
  // CASE. On the rig the clock walks tau slowly while a fresh noise draw
  // arrives every frame, so what the campaigns measure as adjacent-difference
  // jitter is the spread AT FIXED TAU across draws. That is the only condition
  // under which the split-peak rule can add jitter rather than merely move
  // where the one correct step happens: near the tau where the earlier
  // neighbour's statistic sits ON the floor, noise decides the pick frame by
  // frame. Below: 32 draws at each tau, the number of DISTINCT indices the
  // pick returns (1 = decided, 2 = dithering), for the argmax and for the
  // shipped first-path rule.
  std::printf("\n=== AP-72: dither at FIXED tau across 32 draws (values = "
              "distinct indices returned; 1 is decided) ===\n");
  std::printf("%-14s %5s", "shape", "SNR");
  for (int t = 0; t <= 9; ++t) std::printf(" %4.1f", 0.1 * t);
  std::printf("   worst\n");
  for (const double snr : {45.0, 30.0}) {
    for (const auto& b : ds) {
      const int w = shippedWindow(b);
      std::printf("%-14s %5.0f", b.name.c_str(), snr);
      int worst_am = 1, worst_pk = 1;
      for (int t = 0; t <= 9; ++t) {
        const double tau = 0.1 * t;
        std::set<long long> am, pk;
        for (unsigned sd = 1; sd <= 32; ++sd) {
          const long long a = residualPick(b, tau, sd, 0, snr);
          const long long v = residualPick(b, tau, sd, w, snr);
          if (a != kMiss) am.insert(a);
          if (v != kMiss) pk.insert(v);
        }
        worst_am = std::max(worst_am, static_cast<int>(am.size()));
        worst_pk = std::max(worst_pk, static_cast<int>(pk.size()));
        std::printf(" %d/%d", static_cast<int>(am.size()),
                    static_cast<int>(pk.size()));
      }
      std::printf("   argmax %d, pick %d\n", worst_am, worst_pk);
    }
  }

  // WHY THE ESTIMATOR IS A RATIO AND NOT A PARABOLA, MEASURED RATHER THAN
  // ASSERTED (8ai). comms-lib-portable.cc says the beacon's autocorrelation is
  // a delta, so the three samples around the top trace the fractional-delay
  // kernel and not the beacon; that claim is a number, so here it is. At zero
  // fractional delay a delta-like lobe leaves its neighbours near zero, and a
  // parabola through three points of such a lobe is at its worst.
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
    size_t top = 0;
    double best = -1.0;
    for (size_t k = 0; k < corr.size(); ++k) {
      const double a = std::abs(corr[k]);
      if (a > best) { best = a; top = k; }
    }
    std::printf("%-14s", b.name.c_str());
    for (const int d : {-2, -1, 1, 2}) {
      const long long k = static_cast<long long>(top) + d;
      const double a = (k >= 0 && k < static_cast<long long>(corr.size()))
                           ? std::abs(corr[static_cast<size_t>(k)]) / best
                           : 0.0;
      std::printf(" %9.4f", a);
    }
    std::printf("\n");
    // Informational, deliberately not a threshold. The first version of this
    // asserted "delta-like" for every shape at 0.05 and dot11 FAILED it at
    // 0.18: dot11's replica is a band-limited training field, not a full-rate
    // pseudorandom sequence, so its main lobe is genuinely wider and the
    // review's "essentially a delta" was measured on legacy and generalised.
    // The estimator's premise survives per shape rather than in general, and
    // the consequence is visible in the RMS table below, where dot11 is the
    // worst column (0.095) and legacy the best (0.018). What is asserted is
    // that outcome, which is pre-registered, not a threshold invented here.
  }

  // AP-72's OTHER NAMED FIX, MEASURED AGAINST THE INTEGER IT REFINES (8ah).
  // `frac_offset` is a three-point parabolic fit on the correlator amplitude
  // at the lobe the returned index sits on, so `index + frac_offset` should
  // estimate the true fractional end. The bar was set before the run: it has
  // to beat the quantisation it replaces, sd 1/sqrt(12) = 0.289 samples, or it
  // is not worth shipping. The `integer` column is that quantisation as this
  // sweep measures it; the `fitted` column is what the fit achieves.
  std::printf("\n=== AP-72: sub-sample fit against the true end (RMS samples, "
              "tau = 0..1 in 0.02, 8 draws) ===\n");
  std::printf("%-14s %5s %12s %12s %8s %10s\n", "shape", "SNR", "integer RMS",
              "fitted RMS", "no-refine", "verdict");
  for (const double snr : {45.0, 30.0}) {
    for (const auto& b : ds) {
      double si = 0.0, sf = 0.0;
      int n = 0, none = 0;
      for (unsigned sd = 1; sd <= 8; ++sd) {
        for (int t = 0; t <= 50; ++t) {
          const double tau = 0.02 * t;
          const auto rf = residualFrac(b, tau, sd, snr);
          if (rf.first == kMiss) continue;
          const double truth = static_cast<double>(kEndConvention) + tau;
          const double ei = static_cast<double>(rf.first) - truth;
          // NaN means the estimator reported no refinement, which leaves the
          // consumer with the integer: scored as the integer, and counted, so
          // a column cannot look good by declining to answer.
          if (std::isnan(rf.second)) ++none;
          const double off = std::isnan(rf.second) ? 0.0 : rf.second;
          const double ef = static_cast<double>(rf.first) + off - truth;
          si += ei * ei;
          sf += ef * ef;
          ++n;
        }
      }
      const double ri = n ? std::sqrt(si / n) : 0.0;
      const double rf2 = n ? std::sqrt(sf / n) : 0.0;
      std::printf("%-14s %5.0f %12.3f %12.3f %8d %10s\n", b.name.c_str(), snr,
                  ri, rf2, none, rf2 < 0.289 ? "PASS" : "FAIL");
      check(rf2 < 0.289, std::string("sub-sample fit beats rounding: ") +
                             b.name + " at " + std::to_string(int(snr)) + " dB");
    }
  }

  // And it must be DECIDED where the integer dithers: at fixed tau across 32
  // draws the estimate's own spread has to stay under 0.1 samples (8ah).
  std::printf("\n=== AP-72: sub-sample fit spread at FIXED tau, 32 draws "
              "(sd in samples, 45 dB) ===\n");
  std::printf("%-14s", "shape");
  for (int t = 0; t <= 9; ++t) std::printf(" %5.1f", 0.1 * t);
  std::printf("   worst\n");
  for (const auto& b : ds) {
    std::printf("%-14s", b.name.c_str());
    double worst = 0.0;
    for (int t = 0; t <= 9; ++t) {
      const double tau = 0.1 * t;
      std::vector<double> v;
      for (unsigned sd = 1; sd <= 32; ++sd) {
        const auto rf = residualFrac(b, tau, sd, 45.0);
        if (rf.first != kMiss && !std::isnan(rf.second))
          v.push_back(static_cast<double>(rf.first) + rf.second);
      }
      double m = 0.0, q = 0.0;
      for (const double x : v) m += x;
      if (!v.empty()) m /= static_cast<double>(v.size());
      for (const double x : v) q += (x - m) * (x - m);
      const double sd = v.size() > 1 ? std::sqrt(q / (v.size() - 1)) : 0.0;
      worst = std::max(worst, sd);
      std::printf(" %5.3f", sd);
    }
    std::printf("   %.3f\n", worst);
    check(worst < 0.1, std::string("sub-sample fit is decided: ") + b.name);
  }

  // ---------------------------------------------------------------------
  // THE NO-LAG FALSE-CROSSING RATE ON NOISE, MEASURED AGAINST ITS PREDICTION.
  // 8.155/8.159 saw one rejected noise-window crossing per acquisition hunt on
  // nr_pss and predicted it from the statistic: the coherence of a pure-noise
  // window against an L-tap replica is Beta(1, L-1), so P(coh > bar) per index
  // is (1 - bar)^(L-1) = 0.9^127 = 1.5e-6 at bar 0.1, and the lag product's
  // (coh1 * coh2) is far rarer. PREDICTION, stated first: over 16 noise-only
  // hunt windows of 12288 samples (196608 indices) at bar 0.1 the no-lag
  // detector crosses about 0.3 times in total and the xcorr form about 0;
  // at bar 0.01 (corr_scale 100, the resync retry ladder's reach) no-lag
  // crosses in nearly EVERY window (0.99^127 = 0.28 per index) and xcorr in
  // almost none. A no-lag rate an order of magnitude off either way means
  // the mechanism in 8.155 is wrong and the row must be corrected.
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
          std::mt19937 g(9000u + w);
          auto u01 = [&g]() { return (static_cast<double>(g()) + 0.5) / 4294967296.0; };
          std::vector<std::complex<int16_t>> buf(12288);
          for (auto& v : buf) {
            const double a = std::sqrt(-2.0 * std::log(u01()));
            const double ph = 2.0 * M_PI * u01();
            v = std::complex<int16_t>(static_cast<int16_t>(20.0 * a * std::cos(ph)),
                                      static_cast<int16_t>(20.0 * a * std::sin(ph)));
          }
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
    // PREDICTION stated first: crossed <= 4 (a 2.5x allowance on a Poisson
    // mean of 1.6). Measured 3 on the first run.
    {
      const double pfa = 0.1;
      const double bar = houdini::sync::ThresholdPolicy::coherenceBar(pss.replica.size(), pfa, 12288);
      int crossed = 0;
      for (int w = 0; w < 16; ++w) {
        std::mt19937 g(9000u + w);
        auto u01 = [&g]() { return (static_cast<double>(g()) + 0.5) / 4294967296.0; };
        std::vector<std::complex<int16_t>> buf(12288);
        for (auto& v : buf) {
          const double a = std::sqrt(-2.0 * std::log(u01()));
          const double ph = 2.0 * M_PI * u01();
          v = std::complex<int16_t>(static_cast<int16_t>(20.0 * a * std::cos(ph)),
                                    static_cast<int16_t>(20.0 * a * std::sin(ph)));
        }
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
  // THE BEACON CFO ESTIMATOR AGAINST FRACTIONAL DELAY, FOR 8.114. The
  // per-detection beacon CFO scatter on silicon is 2-10x its thermal floor and
  // its ordering across shapes is unexplained. One thing the estimator sees on
  // a real link that the offline model never gave it: a beacon that sits
  // BETWEEN samples. Then the samples after the last repetition are not zero
  // but the interpolation tail of the beacon's edge, and any estimator window
  // that touches that edge multiplies real beacon samples by that tail.
  //
  // Three window placements, all Receiver::estimateCFO's arithmetic (rep2
  // against rep1 at lag fine_len) rebuilt on the shape geometry:
  //   guard 0   windows exactly on the fine field, index from the detector
  //   guard +8  the shipped placement (AP-39, HOUDINI_CFO_INDEX_GUARD): both
  //             windows slid 8 samples LATER, derived on an integer-delay model
  //             where the trailing samples were exactly zero
  //   margin 4  windows shrunk 4 samples at BOTH ends, so neither touches an
  //             edge; the NR/802.11 way of using a cyclic field
  // PREDICTIONS, stated first. (1) At true CFO 0 the error under "guard +8"
  // grows with tau and is largest for the short-field shapes (nr, dot11: L 64),
  // because eight edge samples out of 64 is 1/8 of the sum. (2) "margin 4" is
  // within the thermal floor at every tau for every shape. (3) If instead all
  // three read alike, fractional timing is not the 8.114 mechanism. Mean and
  // sd over 8 noise draws so a bias can be told from the floor.
  std::printf("\n=== beacon CFO error at true CFO 0 vs fractional delay "
              "(45 dB, 8 draws, mean/sd Hz) ===\n");
  // Two interpolation kernels: a 33-tap Hann-windowed sinc (smooth edges, the
  // gentlest case) and a 129-tap one (a sharper band edge whose ringing
  // reaches further, closer to what the RFDC decimation filter does to a
  // hard-edged burst). PREDICTION for the second kernel: the errors grow for
  // every shape, most for the placements whose windows touch the edge.
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
