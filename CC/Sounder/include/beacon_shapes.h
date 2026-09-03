/**
 * @file beacon_shapes.h
 * @brief The candidate beacon waveforms, defined ONCE.
 *
 * [user 2026-09-02] "instead of being 802.11 LTS / NR TRS and 802.11 'like',
 * what about going straight 802.11 and NR beacons (we can have it as a
 * parameter (config)), selectable between the 3 and see which works the best".
 *
 * WHY A SHARED HEADER AND NOT FOUR COPIES. The offline geometry test, the
 * waveform dumper the bench probes read, and Config::genPilots all have to agree
 * on the sample-exact core, or the bench measures one beacon and the shipped
 * build transmits another. That is not hypothetical: AP-34(a) shipped a guard
 * variant to silicon whose index convention nobody had derived, and the cost was
 * a bench session. One definition, three consumers.
 *
 * EVERY SHAPE IS BUILT FROM SEQUENCES THIS REPO ALREADY GENERATES, except the
 * two NR fields, which are generated here from the 38.211 definitions rather
 * than approximated -- the user asked for a standard implementation and an
 * approximation is the thing that would make the comparison meaningless.
 *
 * Header-only, like sync_geometry.h and grid_tracker.h: no link dependency
 * beyond CommsLib itself, so a probe can pull it in without the sounder.
 */
#pragma once

#include <cmath>
#include <complex>
#include <cstdint>
#include <string>
#include <vector>

#include "comms-lib.h"

namespace beacon_shapes {

using cf = std::complex<float>;

enum class Shape {
  kLegacy,       ///< 15 x STS(16) + 2 x gold(128). WHAT WE SHIP.
  kLegacyGuard,  ///< the same, with an 802.11-style cyclic guard before gold.
  kDot11,        ///< the actual 802.11a/g/n legacy preamble: STF + LTF.
  kNr,           ///< NR PSS (38.211 7.4.2.2) + CP + a CSI-RS tracking pair.
  kNrPss,        ///< the kNr core, matched-filtered on its PSS: NR's ARCHITECTURE.
};

/// Everything a consumer needs: the waveform, what to correlate against, and
/// where the coarse and fine estimator fields sit inside the core.
struct Desc {
  std::string name;
  Shape shape;
  std::vector<cf> core;     ///< transmitted burst, unit-ish scale
  std::vector<cf> replica;  ///< the matched-filter reference (one symbol)

  // WHERE THE REPLICA SITS IN THE CORE. `replica_off` is the offset of its
  // first copy and `replica_reps` how many back-to-back copies the core
  // carries. Two facts every consumer needs follow from these and nothing
  // else:
  //   - the detector reports the LAST sample of the last matched copy, so the
  //     beacon END -- the convention sync_index, the SNR window and the CFO
  //     index all rest on -- is replica_tail() samples later. That is 0 for
  //     every shape whose replica is its trailing fine field, and 144 for
  //     kNrPss, whose replica is the LEADING PSS.
  //   - replica_reps == 1 is a NON-REPEATING reference. The lag-product
  //     threshold forms multiply the peak by the correlation one replica
  //     length earlier, which for such a reference is the silence before the
  //     beacon, so they score zero at the right index. Only the plain matched
  //     filter (BeaconThresh::kCoherence) describes it, and consumers select
  //     that form from this field rather than from the environment.
  size_t replica_off = 0, replica_reps = 0;
  size_t replica_tail() const {
    return core.size() - (replica_off + replica.size() * replica_reps);
  }

  // COARSE field: `coarse_reps` back-to-back copies of a `coarse_len` symbol
  // starting at `coarse_off`. Used for the wide-range CFO stage (ambiguity
  // rate/coarse_len). coarse_reps < 2 means the shape has no coarse field --
  // which is not disqualifying here: the fine stage alone is unambiguous to
  // +-rate/(2*fine_len), 480 kHz at fine_len 128, against a link CFO that is at
  // most 8.5 ppm of 500 MHz = 4.25 kHz.
  size_t coarse_off = 0, coarse_len = 0, coarse_reps = 0;

  // FINE field: `fine_reps` copies of a `fine_len` symbol at `fine_off`. This
  // pair is what find_beacon's lag-`fine_len` autocorrelation locks onto and
  // what the CFO estimator reads. `guard_len` samples of cyclic prefix sit
  // immediately BEFORE fine_off when non-zero.
  size_t fine_off = 0, fine_len = 0, fine_reps = 0, guard_len = 0;

  /// Peak-to-average power ratio in dB. Not cosmetic: the transmit path scales
  /// the core to a fixed fraction of full scale by PEAK, so a higher PAPR
  /// delivers less average power and costs detection margin directly.
  double papr_db() const {
    double sum = 0.0, peak = 0.0;
    for (const auto& v : core) {
      const double p = std::norm(v);
      sum += p;
      if (p > peak) peak = p;
    }
    if (sum <= 0.0) return 0.0;
    return 10.0 * std::log10(peak / (sum / static_cast<double>(core.size())));
  }
};

namespace detail {

inline std::vector<cf> seq(size_t type, size_t len) {
  auto m = CommsLib::getSequence(type, len);
  std::vector<cf> out(m[0].size());
  for (size_t i = 0; i < m[0].size(); ++i) out[i] = cf(m[0][i], m[1][i]);
  return out;
}

/// 3GPP TS 38.211 7.4.2.2.1: the NR PSS is a length-127 m-sequence,
/// x(i+7) = (x(i+4) + x(i)) mod 2 with x(0..6) = 0 1 1 0 1 1 1, mapped
/// d(n) = 1 - 2 x((n + 43 N_ID2) mod 127). N_ID2 = 0 here (cell 0).
inline std::vector<float> nrPssMSeq(int n_id2 = 0) {
  int x[127 + 7];
  const int init[7] = {0, 1, 1, 0, 1, 1, 1};
  for (int i = 0; i < 7; ++i) x[i] = init[i];
  for (int i = 0; i < 127; ++i) x[i + 7] = (x[i + 4] + x[i]) % 2;
  std::vector<float> d(127);
  for (int n = 0; n < 127; ++n)
    d[n] = 1.0f - 2.0f * static_cast<float>(x[(n + 43 * n_id2) % 127]);
  return d;
}

/// 3GPP TS 38.211 5.2.1: the pseudo-random (Gold) sequence, two length-31
/// m-sequences with Nc = 1600. CSI-RS -- and therefore a TRS -- is QPSK over
/// this, which is what makes the NR tracking symbol full-band and noise-like
/// rather than a repeat of a shorter pattern.
inline std::vector<int> gold38211(size_t len, uint32_t c_init) {
  const size_t Nc = 1600;
  std::vector<int> x1(Nc + len + 31, 0), x2(Nc + len + 31, 0);
  x1[0] = 1;
  for (int i = 0; i < 31; ++i) x2[i] = (c_init >> i) & 1;
  for (size_t n = 0; n + 31 < x1.size(); ++n) {
    x1[n + 31] = (x1[n + 3] + x1[n]) % 2;
    x2[n + 31] = (x2[n + 3] + x2[n + 2] + x2[n + 1] + x2[n]) % 2;
  }
  std::vector<int> c(len);
  for (size_t n = 0; n < len; ++n) c[n] = (x1[n + Nc] + x2[n + Nc]) % 2;
  return c;
}

/// Map `tones` frequency-domain values onto an `n`-point IFFT, DC nulled,
/// centred, and return the n time samples normalised to unit mean power.
///
/// THE DC SKIP MUST NOT COLLIDE, AND THE FIRST VERSION OF THIS DID. It centred
/// the tones on DC and, when a tone landed on bin 0, "parked" it at k/2 + 1.
/// For the NR tracking symbol that is 64 tones into a 64-point IFFT, and bin
/// k/2+1 = 33 is ALREADY OCCUPIED by the tone from bin -31. The assignment
/// silently overwrote it, so the symbol went out with one tone doubled and one
/// missing. Transmit and correlator shared the same malformed symbol, so
/// detection still worked and nothing failed -- it simply measured a beacon
/// nobody designed, and the NR row's anomalies (4x the CFO scatter of the
/// equal-length dot11 field, and 4.5 dB of unexplained SNR) were partly mine.
/// Fixed by walking the non-DC bins in order instead: bins -floor/.../+ceil
/// skipping zero, taking as many as there is room for.
inline std::vector<cf> toneIfft(const std::vector<cf>& tones, size_t n) {
  std::vector<cf> f(n, cf(0.f, 0.f));
  const size_t k = std::min(tones.size(), n - 1);  // n-1 usable bins, DC nulled
  // Bin positions, centred and skipping DC, generated in order so no two tones
  // can ever land on the same bin.
  std::vector<long long> bins;
  bins.reserve(k);
  for (long long b = -static_cast<long long>((k + 1) / 2);
       bins.size() < k && b <= static_cast<long long>(n / 2); ++b) {
    if (b != 0) bins.push_back(b);
  }
  for (size_t i = 0; i < bins.size(); ++i) {
    const size_t idx = static_cast<size_t>(
        (bins[i] + static_cast<long long>(n)) % static_cast<long long>(n));
    f[idx] = tones[i];
  }
  auto t = CommsLib::IFFT(f, static_cast<int>(n), 1.0f, false, false);
  double p = 0.0;
  for (const auto& v : t) p += std::norm(v);
  p = std::sqrt(p / static_cast<double>(t.size()));
  if (p > 0.0)
    for (auto& v : t) v /= static_cast<float>(p);
  return t;
}

/// Append the last `g` samples of `sym` (its cyclic prefix) then `reps` copies.
inline void appendGuardedReps(std::vector<cf>& core, const std::vector<cf>& sym,
                              size_t g, size_t reps) {
  for (size_t i = sym.size() - g; i < sym.size(); ++i) core.push_back(sym[i]);
  for (size_t r = 0; r < reps; ++r)
    core.insert(core.end(), sym.begin(), sym.end());
}

/// Scale to unit mean power.
///
/// THE SEQUENCES THIS REPO GENERATES ARE NOT MUTUALLY NORMALISED, and composing
/// them naively is a trap I walked into. Measured: getSequence returns STS at
/// rms 0.786 and GOLD at 0.600 -- 2.3 dB apart, which is why the shipped beacon
/// is fine -- but LTS at 0.113 and LTE_ZADOFF_CHU at 0.088, which is 17 dB below
/// the STS. A straight concatenation therefore transmits the 802.11 long
/// training field 17 dB under its own short training field, and the first
/// version of this header did exactly that. The detector then missed the dot11
/// beacon at every level a real link runs at, and the honest-looking conclusion
/// "802.11 is 20 dB less sensitive" would have been a property of MY
/// concatenation, not of 802.11 -- which specifies the two fields at equal
/// power. Normalise each field, then scale the core to the shipped beacon's
/// peak so every candidate presents the DAC with the same constraint.
inline void unitPower(std::vector<cf>& v) {
  double p = 0.0;
  for (const auto& x : v) p += std::norm(x);
  if (p <= 0.0) return;
  const auto g = static_cast<float>(std::sqrt(static_cast<double>(v.size()) / p));
  for (auto& x : v) x *= g;
}

/// Scale the whole core so its PEAK is `peak`, matching what the transmit path
/// constrains (the core goes out at a fixed fraction of full scale).
inline void scaleToPeak(std::vector<cf>& v, double peak) {
  double pk = 0.0;
  for (const auto& x : v) pk = std::max(pk, static_cast<double>(std::norm(x)));
  if (pk <= 0.0) return;
  const auto g = static_cast<float>(peak / std::sqrt(pk));
  for (auto& x : v) x *= g;
}

/// The NR core: PSS, cyclic guard, two copies of a TRS symbol. Shared by kNr
/// and kNrPss, which transmit the SAME burst and differ only in what the
/// detector correlates against. Returns the PSS symbol length.
inline size_t buildNr(Desc& d) {
    // NR sends SSB and TRS as separate signals at different periodicities.
    // Under this link's constraint -- the UE sees ONE periodic downlink burst
    // and gets no pilots of its own -- the NR structure collapses to: an
    // acquisition field that is a full-band non-repeating sequence, then a
    // guarded repeated tracking symbol. Both fields are the standard's own
    // sequences, not stand-ins.
    std::vector<cf> pss_tones;
    for (float v : nrPssMSeq(0)) pss_tones.push_back(cf(v, 0.f));
    auto pss = toneIfft(pss_tones, 128);
    unitPower(pss);
    // TRS symbol: QPSK over a 38.211 Gold sequence, full band. c_init is
    // arbitrary but must be FIXED, or TX and the correlator disagree.
    const size_t kTrsLen = 64;
    const auto c = gold38211(2 * kTrsLen, 0x1u);
    std::vector<cf> trs_tones(kTrsLen);
    const float r = static_cast<float>(1.0 / std::sqrt(2.0));
    for (size_t i = 0; i < kTrsLen; ++i)
      trs_tones[i] = cf(r * (1.f - 2.f * c[2 * i]), r * (1.f - 2.f * c[2 * i + 1]));
    auto trs = toneIfft(trs_tones, kTrsLen);
    unitPower(trs);
    d.core = pss;
    // The PSS is one symbol, so it gives no repeat pair: no coarse stage.
    d.coarse_reps = 0;
    d.guard_len = 16;
    d.fine_off = d.core.size() + d.guard_len;
    d.fine_len = kTrsLen; d.fine_reps = 2;
    appendGuardedReps(d.core, trs, d.guard_len, 2);
    scaleToPeak(d.core, 1.0);
    d.replica.assign(d.core.begin() + d.fine_off,
                     d.core.begin() + d.fine_off + d.fine_len);
  return 128;  // the PSS symbol: 127 tones in a 128-point IFFT
}

}  // namespace detail

inline Desc make(Shape s) {
  using namespace detail;
  Desc d;
  d.shape = s;
  switch (s) {
    case Shape::kLegacy: {
      // BIT-IDENTICAL to what Config::genPilots builds today. Deliberately NOT
      // renormalised: this is the reference every measurement on this bench was
      // taken against, and a "tidier" scaling would silently invalidate them.
      d.name = "legacy";
      const auto sts = seq(CommsLib::STS_SEQ, 16);
      const auto gold = seq(CommsLib::GOLD_IFFT, 128);
      for (int i = 0; i < 15; ++i)
        d.core.insert(d.core.end(), sts.begin(), sts.end());
      d.coarse_off = 0; d.coarse_len = 16; d.coarse_reps = 15;
      d.fine_off = d.core.size(); d.fine_len = 128; d.fine_reps = 2;
      d.guard_len = 0;
      for (int i = 0; i < 2; ++i)
        d.core.insert(d.core.end(), gold.begin(), gold.end());
      d.replica = gold;
      break;
    }
    case Shape::kLegacyGuard: {
      d.name = "legacy_guard";
      const auto sts = seq(CommsLib::STS_SEQ, 16);
      const auto gold = seq(CommsLib::GOLD_IFFT, 128);
      for (int i = 0; i < 15; ++i)
        d.core.insert(d.core.end(), sts.begin(), sts.end());
      d.coarse_off = 0; d.coarse_len = 16; d.coarse_reps = 15;
      d.guard_len = 32;
      d.fine_off = d.core.size() + d.guard_len;
      d.fine_len = 128; d.fine_reps = 2;
      appendGuardedReps(d.core, gold, d.guard_len, 2);
      d.replica = gold;
      break;
    }
    case Shape::kDot11: {
      // getSequence(STS_SEQ,160) IS the standard's short training field (10
      // reps of the 16-sample symbol); getSequence(LTS_SEQ,160) is its long
      // training field, and the `% 64` wrap inside getSequence already makes
      // its first 32 samples the LTS's own cyclic prefix -- that is GI2. Both
      // have been in this repo the whole time and genBeacon never used them.
      d.name = "dot11";
      auto stf = seq(CommsLib::STS_SEQ, 160);
      auto ltf = seq(CommsLib::LTS_SEQ, 160);
      unitPower(stf);  // 802.11 sends STF and LTF at EQUAL power; getSequence
      unitPower(ltf);  // does not, by 17 dB. See unitPower.
      d.core = stf;
      d.coarse_off = 0; d.coarse_len = 16; d.coarse_reps = 10;
      d.guard_len = 32;
      d.fine_off = d.core.size() + d.guard_len;
      d.fine_len = 64; d.fine_reps = 2;
      d.core.insert(d.core.end(), ltf.begin(), ltf.end());
      scaleToPeak(d.core, 1.0);
      // The replica only has to MATCH the transmitted symbol in shape; the
      // detector's threshold is a ratio, so its scale cancels. Take it from the
      // core so it can never drift from what is actually sent.
      d.replica.assign(d.core.begin() + d.fine_off,
                       d.core.begin() + d.fine_off + d.fine_len);
      break;
    }
    case Shape::kNr: {
      d.name = "nr";
      buildNr(d);
      break;
    }
    case Shape::kNrPss: {
      // THE SAME BURST, DETECTED THE WAY NR DETECTS IT. kNr transmits a PSS and
      // hands the detector the TRACKING pair, so it runs an NR waveform through
      // an 802.11-shaped detector, and DEMO_VERIFICATION 8.134/8.144 measured
      // that as the worst of the four on detection margin. NR itself never
      // correlates on a repeated field: the UE matched-filters the PSS -- a
      // 127-tone m-sequence that appears ONCE in the burst -- and the peak is
      // the peak (38.213 4.1; srsRAN's ssb_pss_find is an FFT-domain matched
      // filter over exactly this sequence, MATLAB's NR cell search the same
      // across half-subcarrier CFO hypotheses). This shape is the
      // transmit-identical control that isolates that architectural
      // difference: same core, same TRS pair for the CFO estimator, replica =
      // the PSS, and the detector's repeat check necessarily off. AP-66.
      d.name = "nr_pss";
      const size_t pss_len = buildNr(d);
      d.replica.assign(d.core.begin(), d.core.begin() + pss_len);
      d.replica_off = 0;
      d.replica_reps = 1;
      break;
    }
  }
  if (d.replica_reps == 0) {
    // Every shape whose replica is its fine field: the detector matches the
    // trailing repeated symbol, so the returned index IS the beacon end.
    d.replica_off = d.fine_off;
    d.replica_reps = d.fine_reps;
  }
  return d;
}

/// Config spelling -> shape. Returns false on an unknown name rather than
/// silently falling back, because a typo that quietly ships the old beacon is
/// exactly the failure this parameter exists to make visible.
inline bool parse(const std::string& s, Shape* out) {
  if (s == "legacy") { *out = Shape::kLegacy; return true; }
  if (s == "legacy_guard") { *out = Shape::kLegacyGuard; return true; }
  if (s == "dot11" || s == "802.11") { *out = Shape::kDot11; return true; }
  if (s == "nr" || s == "5gnr") { *out = Shape::kNr; return true; }
  if (s == "nr_pss" || s == "5gnr_pss") { *out = Shape::kNrPss; return true; }
  return false;
}

inline const char* kAllNames[] = {"legacy", "legacy_guard", "dot11", "nr",
                                  "nr_pss"};

}  // namespace beacon_shapes
