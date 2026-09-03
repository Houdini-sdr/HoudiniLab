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
#include <string>
#include <vector>

#include "include/beacon_shapes.h"
#include "include/comms-lib.h"
#include "include/utils.h"

namespace {

using beacon_shapes::cf;
using beacon_shapes::Desc;
using beacon_shapes::Shape;
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
  std::mt19937 g(seed);
  auto u01 = [&g]() { return (static_cast<double>(g()) + 0.5) / 4294967296.0; };
  auto gauss = [&]() {
    return std::sqrt(-2.0 * std::log(u01())) * std::cos(2.0 * M_PI * u01());
  };
  double mean_p = 0.0, peak_p = 0.0;
  for (const auto& v : b.core) {
    mean_p += std::norm(v);
    peak_p = std::max(peak_p, static_cast<double>(std::norm(v)));
  }
  mean_p /= static_cast<double>(len);
  const double scale = peak_counts / std::sqrt(peak_p);
  const double ns = std::sqrt((mean_p / std::pow(10.0, snr_db / 10.0)) / 2.0);

  const long long pos = 4000, end = pos + len, s0 = end - lead, n = lead + tail;
  std::vector<std::complex<double>> sig(n, {0.0, 0.0});
  for (const auto& tp : ch.taps) {
    for (long long k = 0; k < len; ++k) {
      const long long a = pos + tp.first + k - s0;
      if (a < 0 || a >= n) continue;
      const double ph = 2.0 * M_PI * ch.cfo_hz * static_cast<double>(k) / 122.88e6;
      sig[a] += tp.second * std::complex<double>(b.core[k].real(), b.core[k].imag()) *
                std::exp(std::complex<double>(0.0, ph));
    }
  }
  std::vector<std::complex<int16_t>> buf(n);
  for (long long i = 0; i < n; ++i) {
    const double re = sig[i].real() * scale + gauss() * ns * scale;
    const double im = sig[i].imag() * scale + gauss() * ns * scale;
    buf[i] = std::complex<int16_t>(
        static_cast<int16_t>(std::max(-32000.0, std::min(32000.0, re))),
        static_cast<int16_t>(std::max(-32000.0, std::min(32000.0, im))));
  }
  // A single-copy replica (nr_pss) has no repeat to check: the receiver forces
  // the plain matched filter for it (syncSearch), and so does this test, so
  // every column below measures the form the shape would actually run with.
  if (b.replica_reps < 2) thresh_form = Thr::kXCorrNoLag;
  const ssize_t idx =
      CommsLib::find_beacon_avx(buf.data(), b.replica, n, corr_scale, pick,
                                thresh_form);
  // The detector reports the last sample of the MATCHED field; the beacon end
  // is replica_tail() later (0 for every shape but nr_pss), exactly as
  // syncSearch applies it.
  const long long rep_tail = static_cast<long long>(b.replica_tail());
  return idx < 0 ? kMiss : s0 + idx + rep_tail - end;  // vs the DIRECT path's end
}
/// Delay a waveform by a fractional sample with a windowed sinc (33 taps).
/// Returns len+1 samples so the tail is kept. The bench's link has an
/// arbitrary fractional timing; the integer placement everywhere else in this
/// test cannot show what a detector does BETWEEN samples.
std::vector<cf> fracDelay(const std::vector<cf>& x, double tau) {
  const int R = 16;
  std::vector<cf> y(x.size() + 1, cf(0.f, 0.f));
  for (size_t k = 0; k < y.size(); ++k) {
    std::complex<double> acc(0.0, 0.0);
    for (int m = static_cast<int>(k) - R; m <= static_cast<int>(k) + R; ++m) {
      if (m < 0 || m >= static_cast<int>(x.size())) continue;
      const double t = static_cast<double>(k) - static_cast<double>(m) - tau;
      const double sinc = (std::fabs(t) < 1e-9) ? 1.0
                          : std::sin(M_PI * t) / (M_PI * t);
      const double w = 0.5 + 0.5 * std::cos(M_PI * t / (R + 1));  // Hann
      acc += std::complex<double>(x[m].real(), x[m].imag()) * (sinc * w);
    }
    y[k] = cf(static_cast<float>(acc.real()), static_cast<float>(acc.imag()));
  }
  return y;
}

long long residual(const Desc& b, double peak_counts, double snr_db,
                   long long lead, long long tail, float corr_scale, Pick pick,
                   unsigned seed, Thr thresh_form = Thr::kPowerRatio,
                   double frac = 0.0) {
  const std::vector<cf> core = frac != 0.0 ? fracDelay(b.core, frac) : b.core;
  const long long len = static_cast<long long>(b.core.size());
  const long long clen = static_cast<long long>(core.size());
  std::mt19937 g(seed);
  auto u01 = [&g]() { return (static_cast<double>(g()) + 0.5) / 4294967296.0; };
  auto gauss = [&]() {
    return std::sqrt(-2.0 * std::log(u01())) * std::cos(2.0 * M_PI * u01());
  };
  double mean_p = 0.0, peak_p = 0.0;
  for (const auto& v : b.core) {
    mean_p += std::norm(v);
    peak_p = std::max(peak_p, static_cast<double>(std::norm(v)));
  }
  mean_p /= static_cast<double>(len);
  // Scale by PEAK, because that is what the transmit path constrains: the core
  // goes out at a fixed fraction of full scale. Normalising by rms instead would
  // hand every candidate the same average power and hide the PAPR cost.
  const double scale = peak_counts / std::sqrt(peak_p);
  const double ns = std::sqrt((mean_p / std::pow(10.0, snr_db / 10.0)) / 2.0);

  const long long pos = 4000, end = pos + len, s0 = end - lead, n = lead + tail;
  std::vector<std::complex<int16_t>> buf(n);
  for (long long i = 0; i < n; ++i) {
    const long long a = s0 + i;
    cf v(0.f, 0.f);
    if (a >= pos && a < pos + clen) v = core[a - pos];
    const double re = v.real() * scale + gauss() * ns * scale;
    const double im = v.imag() * scale + gauss() * ns * scale;
    buf[i] = std::complex<int16_t>(
        static_cast<int16_t>(std::max(-32000.0, std::min(32000.0, re))),
        static_cast<int16_t>(std::max(-32000.0, std::min(32000.0, im))));
  }
  if (b.replica_reps < 2) thresh_form = Thr::kXCorrNoLag;  // see residualCh
  const ssize_t idx = CommsLib::find_beacon_avx(buf.data(), b.replica, n,
                                               corr_scale, pick, thresh_form);
  const long long rep_tail = static_cast<long long>(b.replica_tail());
  return idx < 0 ? kMiss : s0 + idx + rep_tail - end;
}

// The shipped resync slice at 122.88 MSPS with the 2026-09-02 defaults
// (scatter_tol 246): lead = 246 + 256, tail = 246 + 64. sync_geometry.h owns the
// derivation; these are the values it produces, restated so this test says what
// geometry it is testing rather than pulling in the whole header.
constexpr long long kLead = 502, kTail = 310;
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

}  // namespace

int main() {
  const Desc ds[] = {beacon_shapes::make(Shape::kLegacy),
                     beacon_shapes::make(Shape::kLegacyGuard),
                     beacon_shapes::make(Shape::kDot11),
                     beacon_shapes::make(Shape::kNr),
                     beacon_shapes::make(Shape::kNrPss)};

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

    const auto legacy = beacon_shapes::make(Shape::kLegacy);
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
    const auto nr = beacon_shapes::make(Shape::kNr);
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
                            {Thr::kXCorrNoLag, Pick::kFirstCrossing},
                            {Thr::kXCorrNoLag, Pick::kFirstPath}};
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
        if (tf == Thr::kXCorrNoLag && pk == Pick::kFirstPath) {
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
                          Thr::kXCorrNoLag}) {
      // A single-copy replica runs nolag whatever is asked (residual() forces
      // it, as syncSearch does), so its other two rows would be duplicates
      // printed under the wrong name.
      if (b.replica_reps < 2 && tf != Thr::kXCorrNoLag) continue;
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
                      {Shape::kNrPss, Thr::kXCorrNoLag, "nr_pss beacon, nolag threshold"}};
  for (const auto& ota : otas) {
    std::printf("\n=== over-the-air channels, %s ===\n", ota.label);
    const auto b = beacon_shapes::make(ota.shape);
    const Channel chans[] = {
        {{{0, 1.0}}, 0.0, "1 path, no CFO"},
        {{{0, 1.0}}, 4250.0, "1 path, 8.5 ppm CFO"},
        {{{0, 1.0}, {8, 0.7}}, 4250.0, "echo +8 samp, -3 dB"},
        {{{0, 1.0}, {8, 1.4}}, 4250.0, "echo +8 samp, STRONGER"},
        {{{0, 1.0}, {24, 1.4}}, 4250.0, "echo +24 samp, STRONGER"},
        {{{0, 0.5}, {40, 1.4}}, 4250.0, "weak direct, echo +40 STRONGER"},
    };
    const bool nolag_only = ota.tf == Thr::kXCorrNoLag;
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
                        {Thr::kXCorrNoLag, Pick::kFirstPath}};
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

  std::printf("\nRESULT: %s (%d failure(s))\n", g_fail ? "FAIL" : "PASS", g_fail);
  return g_fail ? 1 : 0;
}
