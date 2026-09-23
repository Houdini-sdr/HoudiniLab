/**
 * @file beacon_bandlimit_test.cc
 * @brief The AP-79 band-limited beacon (nr_pss_bl), NO hardware: it stays
 *        inside the +-24 MHz sub-6 channel, it fits the replay RAM at TX 245.76
 *        with the interpolator's lead, and the REAL detector finds its end
 *        through the UE's actual receive chain (the channel filter, int16).
 *
 * THE BAND CHECK HAS NATURAL MUTANTS: every shipped shape is wideband by
 * construction (gold and STS fields fill the output; nr_pss's PSS sits at the
 * 960 kHz default and its TRS pair on every bin), so the same measurement run
 * on them must FAIL. A band check none of them fails would be measuring
 * nothing.
 *
 * DETECTION is a positive control on the shipped detector, not a threshold
 * study: seeded noise at a modest SNR, the uncalibrated pair's 8.52 ppm carrier
 * offset at 2425 MHz (20.7 kHz, the worst case the calibrated hold improves
 * on), and a 0 dBc tone at +45 MHz standing in for the channel's mirror, all
 * through dsp::ChannelFilter as the UE's sub-6 lane applies it. The end must
 * be found at the sample it was placed.
 *
 * Build: CMake target beacon_bandlimit_test. Run: ./beacon_bandlimit_test (or ctest).
 */
#include <cmath>
#include <complex>
#include <cstdint>
#include <cstdio>
#include <random>
#include <string>
#include <vector>

#include "dsp/band_filters.h"
#include "houdini/tx_rx_boundary.h"
#include "sync/beacon_shape.h"
#include "sync/beacon_shapes.h"
#include "sync/detector.h"

namespace {

using houdini::sync::BeaconShape;
using houdini::sync::Detector;
using houdini::sync::DetectorConfig;
using houdini::sync::Numerology;
using houdini::sync::Platform;

int g_fail = 0;
void check(bool ok, const std::string& what) {
  std::printf("%s  %s\n", ok ? "PASS" : "FAIL", what.c_str());
  if (!ok) ++g_fail;
}

constexpr double kRate = 122.88e6;

// Fraction of the core's energy outside +-edge_hz, in dB, from a zero-padded
// DFT. The burst's own edges are part of what is transmitted, so they count.
double outOfBandDb(const std::vector<std::complex<float>>& core, double edge_hz) {
  const size_t n = 4096;
  double in = 0.0, out = 0.0;
  for (size_t k = 0; k < n; ++k) {
    std::complex<double> acc(0, 0);
    for (size_t t = 0; t < core.size(); ++t) {
      const double ph = -2.0 * M_PI * static_cast<double>(k * t % n) / static_cast<double>(n);
      acc += std::complex<double>(core[t].real(), core[t].imag()) * std::complex<double>(std::cos(ph), std::sin(ph));
    }
    const double f = (k < n / 2 ? static_cast<double>(k) : static_cast<double>(k) - n) * kRate / n;
    (std::fabs(f) <= edge_hz ? in : out) += std::norm(acc);
  }
  return 10.0 * std::log10(out / (in + out));
}

// A normal deviate from mt19937 by Box-Muller: mt19937 is bit-specified, the
// std distributions are not, so this is the same on every library.
struct Gauss {
  std::mt19937 g;
  explicit Gauss(uint32_t seed) : g(seed) {}
  double operator()() {
    const double u1 = (g() + 1.0) / 4294967297.0, u2 = (g() + 1.0) / 4294967297.0;
    return std::sqrt(-2.0 * std::log(u1)) * std::cos(2.0 * M_PI * u2);
  }
};

// The UE's sub-6 lane: beacon at `place` (amplitude `amp` peak), CFO, noise,
// an out-of-band 0 dBc tone, then the channel filter and int16. Returns the
// detector's end index.
long detectThroughLane(const BeaconShape& shape, size_t place, double cfo_hz, double snr_db, uint32_t seed,
                       houdini::sync::PickRule pick) {
  const size_t n = 16384;
  const auto& core = shape.core();
  const double amp = 8000.0;
  double p_sig = 0.0;
  for (const auto& v : core) p_sig += std::norm(v) * amp * amp;
  p_sig /= static_cast<double>(core.size());
  const double sigma = std::sqrt(p_sig / std::pow(10.0, snr_db / 10.0) / 2.0);
  Gauss gn(seed);
  std::vector<std::complex<float>> x(n);
  for (size_t k = 0; k < n; ++k) {
    std::complex<double> v(sigma * gn(), sigma * gn());
    if (k >= place && k < place + core.size()) {
      const auto c = core[k - place];
      v += amp * std::complex<double>(c.real(), c.imag());
    }
    const double a_int = std::sqrt(p_sig);  // 0 dBc against the beacon's power
    const double ph_i = 2.0 * M_PI * 45.0e6 / kRate * static_cast<double>(k);
    v += a_int * std::complex<double>(std::cos(ph_i), std::sin(ph_i));
    const double ph = 2.0 * M_PI * cfo_hz / kRate * static_cast<double>(k);
    v *= std::complex<double>(std::cos(ph), std::sin(ph));
    x[k] = std::complex<float>(static_cast<float>(v.real()), static_cast<float>(v.imag()));
  }
  std::vector<std::complex<float>> y(n);
  houdini::dsp::ChannelFilter().run(x.data(), n, y.data());
  std::vector<std::complex<int16_t>> q(n);
  houdini::dsp::HalfbandInterp2::quantize(y.data(), n, q.data());
  DetectorConfig dc;
  dc.pick = pick;
  Detector det(shape, dc);
  return static_cast<long>(det.run(q.data(), n, 10.0f).end_index);
}

}  // namespace

int main() {
  const Numerology num = Numerology::houdiniDefault();
  const auto bl = BeaconShape::make("nr_pss_bl", Platform::kHoudini, num);

  // ---- the shape itself ----------------------------------------------------
  std::printf("nr_pss_bl: core %zu samples, replica %zu, replica tail %zu, PAPR %.1f dB\n",
              bl.coreLen(), bl.replicaLen(), bl.replicaTail(), bl.paprDb());
  check(bl.numerologyHeld(), "the PSS holds 240 kHz at 122.88 MSPS (a whole 512-point symbol)");
  check(bl.replicaLen() == 512 && bl.singleCopy(), "the replica is the one 512-point PSS, matched once");
  check(bl.coreLen() == 36 + 512 + 16 + 2 * 256, "core = CP 36 + PSS 512 + guard 16 + 2 x TRS 256 = 1076 samples");
  // Replay RAM at TX 245.76: 4096 TX samples = 2048 ticks, less the lead and
  // tail the prefiltered interpolator needs, derived the way HoudiniFramer
  // derives them (lead rounded up to the 4-tick grid).
  const size_t lead = (houdini::boundary::TxBurstInterpolator::prefilterLead() + 3) / 4 * 4;
  const size_t tail = houdini::boundary::TxBurstInterpolator::prefilterTail();
  std::printf("replay image: lead %zu + core %zu + tail %zu = %zu of 2048 ticks\n", lead, bl.coreLen(), tail,
              lead + bl.coreLen() + tail);
  check(lead + bl.coreLen() + tail <= 2048, "it fits the replay RAM at TX 245.76 with the prefiltered interpolator's lead and tail");

  // ---- the band check, and the wideband shapes it must reject -------------
  const double oob = outOfBandDb(bl.core(), 24e6);
  std::printf("nr_pss_bl energy outside +-24 MHz: %.1f dB\n", oob);
  check(oob <= -20.0, "nr_pss_bl keeps its energy inside +-24 MHz (outside <= -20 dB of the raw core; "
                      "the rest is edge splatter the TX prefilter removes, tx_rx_boundary_test)");
  std::printf("-- mutation matrix: every shipped shape is wideband and must FAIL the same band check --\n");
  for (const char* name : {"legacy", "legacy_guard", "dot11", "nr", "nr_pss"}) {
    const auto s = BeaconShape::make(name, Platform::kHoudini, num);
    const double o = outOfBandDb(s.core(), 24e6);
    check(o > -20.0, std::string("wideband ") + name + " fails the band check (" + std::to_string(o).substr(0, 5) + " dB outside)");
  }

  // ---- detection through the UE's sub-6 lane -------------------------------
  const size_t place = 5000;
  const long want = static_cast<long>(place + bl.coreLen() - 1);
  int found = 0, trials = 0;
  for (uint32_t seed = 1; seed <= 8; ++seed) {
    const long got = detectThroughLane(bl, place, 20.7e3, 10.0, seed, houdini::sync::PickRule::kArgmax);
    ++trials;
    if (got == want) ++found;
    if (seed == 1) std::printf("detector end %ld, placed end %ld\n", got, want);
  }
  check(found == trials, "8/8 seeds: the detector with the dual-band config's argmax pick finds the core end "
                         "exactly, through the channel filter, at 10 dB SNR, 20.7 kHz CFO and a 0 dBc out-of-band tone "
                         "[mutation: the default first_path pick]");
  // The mutant is the shipped Houdini default: first_path takes the earliest
  // sample within -9 dB of the peak, and this band-limited PSS's lobe is about
  // 8 samples wide, so it lands early on the lobe's own skirt (measured -2 on a
  // clean, unfiltered burst: the rule, not noise). Hence the config's pick.
  const long fp = detectThroughLane(bl, place, 20.7e3, 10.0, 1, houdini::sync::PickRule::kFirstPath);
  std::printf("first_path pick: end %ld (placed %ld)\n", fp, want);
  check(fp != want, "mutant first_path pick misplaces the end on the band-limited lobe");

  std::printf("%s: %d failure(s)\n", g_fail == 0 ? "ALL PASS" : "FAILED", g_fail);
  return g_fail == 0 ? 0 : 1;
}
