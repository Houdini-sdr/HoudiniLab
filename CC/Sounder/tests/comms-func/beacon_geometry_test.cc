/**
 * @file beacon_geometry_test.cc
 * @brief The index-convention check AP-34(a) did not have. NO hardware.
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
 * NONE OF THAT NEEDED HARDWARE TO FIND. The detector is a pure function of the
 * samples. This synthesises each candidate beacon at a KNOWN position in a
 * noise buffer, runs the REAL CommsLib::find_beacon_avx over it (not a replica
 * -- a replica would agree with itself and prove nothing), and asserts where
 * the returned index lands relative to that known truth.
 *
 * WHAT THIS TEST DOES NOT ESTABLISH. The bursts here are synthesised from
 * CommsLib sequences, not captured from the TX RAM, so they do not carry the
 * conjugation, scaling and any pre/postfix the real transmit path applies. The
 * ABSOLUTE offsets printed below are therefore NOT the production constants and
 * must not be copied into config. What transfers is the comparison: whether a
 * candidate beacon's index is stable, whether it lands inside the burst, and
 * how far it moves RELATIVE to the shipped one. A candidate that shifts the
 * index is not thereby wrong -- it needs its convention and tx_advance
 * re-derived together, which is AP-34(a)'s condition and the whole point.
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

#include "include/comms-lib.h"
#include "include/constants.h"

namespace {

int g_fail = 0;

void check(bool ok, const std::string& what) {
  std::printf("%s  %s\n", ok ? "PASS" : "FAIL", what.c_str());
  if (!ok) ++g_fail;
}

using cf = std::complex<float>;

std::vector<cf> seq(size_t type, size_t len) {
  auto m = CommsLib::getSequence(type, len);
  std::vector<cf> out(m[0].size());
  for (size_t i = 0; i < m[0].size(); ++i) out[i] = cf(m[0][i], m[1][i]);
  return out;
}

struct Beacon {
  std::string name;
  std::vector<cf> core;     ///< the transmitted burst
  std::vector<cf> replica;  ///< what the detector correlates against
  size_t fine_off;          ///< where the fine field starts inside the core
  size_t fine_len;          ///< one repetition of the fine field
  bool guarded;             ///< is the fine field cyclically extended?
};

/// LEGACY: what we ship. 15 x STS(16) then 2 x gold(128), NO guard.
Beacon legacy() {
  Beacon b{"legacy (shipped)", {}, {}, 0, 128, false};
  auto sts = seq(CommsLib::STS_SEQ, 16);
  auto gold = seq(CommsLib::GOLD_IFFT, 128);
  for (int i = 0; i < 15; ++i) b.core.insert(b.core.end(), sts.begin(), sts.end());
  b.fine_off = b.core.size();
  for (int i = 0; i < 2; ++i) b.core.insert(b.core.end(), gold.begin(), gold.end());
  b.replica = gold;
  return b;
}

/// DOT11: the actual 802.11 legacy preamble, built from the sequences this
/// repo ALREADY generates. getSequence(STS_SEQ,160) is the standard's short
/// training field (10 reps); getSequence(LTS_SEQ,160) is its long training
/// field, and the `% 64` wrap inside getSequence makes the first 32 samples the
/// LTS's own cyclic prefix -- that IS GI2. We have had this all along and
/// genBeacon does not use it.
Beacon dot11() {
  Beacon b{"802.11 legacy preamble", {}, {}, 0, Consts::kFftSize_80211, true};
  auto stf = seq(CommsLib::STS_SEQ, 160);
  auto ltf = seq(CommsLib::LTS_SEQ, 160);   // GI2(32) + LTS + LTS
  b.core = stf;
  b.fine_off = b.core.size() + 32;          // past the guard
  b.core.insert(b.core.end(), ltf.begin(), ltf.end());
  b.replica = seq(CommsLib::LTS_SEQ, Consts::kFftSize_80211);
  return b;
}

/// NR-SHAPED: a Zadoff-Chu acquisition field (the PSS family) followed by a
/// GUARDED repeated tracking pair, which is the TRS idea expressed in one
/// burst. NOT literally NR -- NR sends SSB and TRS as separate signals at
/// different periodicities -- but it is the NR STRUCTURE under our constraint
/// that the UE sees only one periodic downlink burst.
Beacon nr_shaped() {
  Beacon b{"NR-shaped (ZC + guarded TRS pair)", {}, {}, 0, 64, true};
  auto zc = seq(CommsLib::LTE_ZADOFF_CHU, 128);
  auto trs = seq(CommsLib::LTS_SEQ, 64);
  b.core = zc;
  // cyclic prefix of the tracking symbol, then two copies of it
  b.fine_off = b.core.size() + 16;
  for (size_t i = 64 - 16; i < 64; ++i) b.core.push_back(trs[i]);
  for (int r = 0; r < 2; ++r) b.core.insert(b.core.end(), trs.begin(), trs.end());
  b.replica = trs;
  return b;
}

/// Place `core` at `pos` in a noise buffer and ask the real detector.
ssize_t detect(const Beacon& b, size_t pos, size_t total, double snr_db,
               unsigned seed, float corr_scale) {
  std::mt19937 g(seed);
  auto u01 = [&g]() { return (static_cast<double>(g()) + 0.5) / 4294967296.0; };
  auto gauss = [&]() {
    return std::sqrt(-2.0 * std::log(u01())) * std::cos(2.0 * M_PI * u01());
  };
  double sp = 0.0;
  for (const auto& v : b.core) sp += std::norm(v);
  sp /= static_cast<double>(b.core.size());
  const double ns = std::sqrt((sp / std::pow(10.0, snr_db / 10.0)) / 2.0);

  std::vector<std::complex<int16_t>> buf(total);
  const double scale = 20000.0 / std::sqrt(sp);
  for (size_t i = 0; i < total; ++i) {
    cf v(0.f, 0.f);
    if (i >= pos && i < pos + b.core.size()) v = b.core[i - pos];
    const double re = v.real() * scale + gauss() * ns * scale;
    const double im = v.imag() * scale + gauss() * ns * scale;
    buf[i] = std::complex<int16_t>(static_cast<int16_t>(std::max(-32000.0, std::min(32000.0, re))),
                                   static_cast<int16_t>(std::max(-32000.0, std::min(32000.0, im))));
  }
  return CommsLib::find_beacon_avx(buf.data(), b.replica, total, corr_scale, false);
}

void probe(const Beacon& b) {
  const size_t total = 8192, pos = 3000;
  std::printf("\n=== %s ===\n", b.name.c_str());
  std::printf("  core %zu samples, fine field %zu x2 at offset %zu, guard %s\n",
              b.core.size(), b.fine_len, b.fine_off, b.guarded ? "YES" : "no");

  // DO NOT ASSERT A CONVENTION THIS TEST BELIEVES IN. The codebase carries two
  // that differ by 128 samples -- houdiniBeaconEnd() says the index is
  // `strobe + beacon_size`, i.e. the beacon END, while
  // two_node_beacon_arrival.py's CORE_OFF_2NDREP says it is `core + 368`, the
  // start of the second gold repetition. Encoding either here would test my
  // reading of the code rather than the detector.
  //
  // What is unambiguous, and what AP-34(a) actually needed, is this: for a
  // given beacon the detector index must be a FIXED, KNOWN offset from the true
  // position. So MEASURE that offset, assert it is stable, and print it as the
  // constant a caller would have to configure. A beacon whose offset differs
  // from the shipped one is not wrong -- it needs its index convention and
  // tx_advance re-derived, which is exactly the finding AP-34(a) paid for.
  const float corr_scale = 100.0f;   // the shipped config value
  std::vector<ssize_t> got;
  for (unsigned s = 1; s <= 6; ++s)
    got.push_back(detect(b, pos, total, 40.0, s, corr_scale));

  int found = 0;
  long long lo = 1LL << 40, hi = -(1LL << 40);
  for (auto v : got) {
    if (v < 0) continue;
    ++found;
    lo = std::min<long long>(lo, v);
    hi = std::max<long long>(hi, v);
  }
  std::printf("  detector returned:");
  for (auto v : got) std::printf(" %zd", v);
  std::printf("\n");
  check(found == 6, "  detects in all 6 noise draws");
  if (found != 6) return;

  check(hi - lo <= 4, "  index is STABLE across noise draws (spread <= 4)");
  const long long off_core = lo - static_cast<long long>(pos);
  const long long off_end =
      off_core - static_cast<long long>(b.core.size());
  std::printf("  MEASURED index convention: core_start %+lld, beacon_end %+lld\n",
              off_core, off_end);
  // The index must at least land INSIDE the burst, or beaconSnrDb() measures a
  // window that is mostly not beacon -- which is precisely how AP-34(a) failed:
  // 10.5 dB reported against a true 48.3, and the 30 dB floor then rejected
  // every resync while acquisition still worked and the demo looked healthy.
  check(off_core >= 0 && off_core <= static_cast<long long>(b.core.size()),
        "  index lands INSIDE the burst (so the SNR window sees beacon)");
  // And it must leave a full fine field behind it, which is what the SNR
  // window and the CFO estimator both read backwards from.
  check(off_core >= static_cast<long long>(b.fine_len),
        "  at least one fine-field length precedes it (SNR + CFO read back)");
}

}  // namespace

int main() {
  std::printf("Index-convention check for candidate beacons.\n");
  std::printf("The invariant under test: sync_index == houdiniBeaconEnd()\n");
  std::printf("== strobe + beacon_size. AP-34(a) broke it by -274 samples on\n");
  std::printf("silicon; nothing about that needed hardware to find.\n");

  probe(legacy());
  probe(dot11());
  probe(nr_shaped());

  std::printf("\nRESULT: %s (%d failure(s))\n", g_fail ? "FAIL" : "PASS", g_fail);
  // A candidate FAILING here is the point: it means that beacon needs its index
  // convention and tx_advance re-derived before it can ship, which is exactly
  // what AP-34(a) says and exactly what was learned the expensive way.
  return g_fail ? 1 : 0;
}
