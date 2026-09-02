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
 *     samples against a 4096-sample beacon copy spacing -- every candidate lands
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
long long residual(const Desc& b, double peak_counts, double snr_db,
                   long long lead, long long tail, float corr_scale, Pick pick,
                   unsigned seed) {
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
    if (a >= pos && a < pos + len) v = b.core[a - pos];
    const double re = v.real() * scale + gauss() * ns * scale;
    const double im = v.imag() * scale + gauss() * ns * scale;
    buf[i] = std::complex<int16_t>(
        static_cast<int16_t>(std::max(-32000.0, std::min(32000.0, re))),
        static_cast<int16_t>(std::max(-32000.0, std::min(32000.0, im))));
  }
  const ssize_t idx =
      CommsLib::find_beacon_avx(buf.data(), b.replica, n, corr_scale, pick);
  return idx < 0 ? kMiss : s0 + idx - end;
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

Row sweepAt(const Desc& b, double level, Pick pick) {
  Row r;
  for (unsigned s = 1; s <= 8; ++s) {
    const long long v =
        residual(b, level, kSnrDb, kLead, kTail, kResyncCorrScale, pick, s);
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
                     beacon_shapes::make(Shape::kNr)};

  std::printf("Candidate beacons, and where the detector says they are.\n");
  std::printf("Resync slice [end-%lld, end+%lld), corr_scale %.0f, SNR %.0f dB.\n",
              kLead, kTail, kResyncCorrScale, kSnrDb);
  std::printf("Cells are (returned index - true beacon end) over 8 noise draws;\n");
  std::printf("%+lld is correct.\n", kEndConvention);

  std::printf("\n%-14s %6s %8s %8s %8s %10s\n", "shape", "core", "fine_off",
              "fine_len", "PAPR dB", "proc gain");
  for (const auto& b : ds) {
    double pk = 0.0, fp = 0.0;
    for (const auto& v : b.core) pk = std::max(pk, static_cast<double>(std::norm(v)));
    for (size_t i = b.fine_off; i < b.fine_off + b.fine_len; ++i)
      fp += std::norm(b.core[i]);
    fp /= static_cast<double>(b.fine_len);
    std::printf("%-14s %6zu %8zu %8zu %8.2f %7.1f dB\n", b.name.c_str(),
                b.core.size(), b.fine_off, b.fine_len, b.papr_db(),
                20.0 * std::log10(b.fine_len * std::sqrt(fp / pk)));
  }

  for (const auto pick : {Pick::kFirstCrossing, Pick::kTargetedArgmax}) {
    const bool argmax = pick == Pick::kTargetedArgmax;
    std::printf("\n=== %s ===\n",
                argmax ? "kTargetedArgmax (what resync uses)"
                       : "kFirstCrossing (what resync used before 2026-09-02)");
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
    const Row r = sweepAt(b, 200.0, Pick::kTargetedArgmax);
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

  std::printf("\nRESULT: %s (%d failure(s))\n", g_fail ? "FAIL" : "PASS", g_fail);
  return g_fail ? 1 : 0;
}
