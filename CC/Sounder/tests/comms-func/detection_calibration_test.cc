/**
 * @file detection_calibration_test.cc
 * @brief The acquisition thresholds for the band-limited beacon nr_pss_bl,
 *        derived on a simulated link instead of carried from legacy (AP-79),
 *        NO hardware. Prints the values the dual-band configs use.
 *
 * WHAT WAS CARRIED, AND WHY IT IS WRONG FOR THIS BEACON. The dual-band configs
 * started from the last demo's corr_scale (100 resync, 10 acquisition) and
 * the 30 dB in-window SNR floor, all set on the wideband legacy beacon. For
 * nr_pss_bl, whose replica is one 512-sample PSS, the detector resolves to the
 * coherence form, whose bar is 1 / corr_scale. MEASURED here through the
 * filtered lane: the noise-only maximum coherence over a 16384-sample window
 * is 0.038 median, 0.057 at the 99th percentile, 0.080 worst of 400; a
 * bench-level beacon reads 0.975 and one 30 dB weaker still 0.90. So the bar
 * is 0.2 (corr_scale 5 for acquisition and resync): 2.5x above the worst
 * noise, 4.5x under a beacon 30 dB down. The carried resync bar (0.01) sits
 * INSIDE the noise, and so does the false-alarm-probability bar
 * (detector.pfa_per_window 1e-3 -> 0.032): its white-noise model counts the
 * replica's 512 samples as independent, but a band-limited PSS has about 127
 * degrees of freedom (its tones; 30.5 MHz x 4.17 us), so noise correlates with
 * it far more than the model predicts. Both are this test's mutants.
 *
 * The SNR floor (confirm.snr_floor_db) is a property of the link AND the
 * waveform (confirm.h, 8.157). It was set at 30 dB against legacy's measured
 * 45 to 48 dB on this bench. So the noise here is FIXED by reproducing that
 * measurement (legacy, 0.6 peak, unfiltered: 46.5 dB in-window), and
 * nr_pss_bl runs through the SAME noise at its planned transmit scale (0.34
 * peak, the plan's -16 dBFS RMS back-off at its 6.6 dB PAPR) and through the
 * RX channel filter the UE's sub-6 lane applies. The shift between the two is
 * the floor's shift: the new floor keeps legacy's 16.5 dB margin.
 *
 * The checks: every beacon at the bench level is detected at its exact end
 * and clears the new floor (20 seeds, the uncalibrated 20.7 kHz CFO); no
 * noise-only window crosses the bar or is accepted (400 seeds); and
 * the carried bar (corr_scale 100 -> 0.01) is the mutant that false-alarms on
 * noise.
 *
 * Build: CMake target detection_calibration_test. Run: ./detection_calibration_test.
 */
#include <algorithm>
#include <cmath>
#include <random>
#include <complex>
#include <cstdio>
#include <string>
#include <vector>

#include "dsp/band_filters.h"
#include "sync/beacon_shape.h"
#include "sync/confirm.h"
#include "sync/detector.h"
#include "sync/sim/channel.h"
#include "sync/sync_config.h"

using namespace houdini::sync;

namespace {
int g_fail = 0;
void check(bool ok, const std::string& what) {
  std::printf("%s  %s\n", ok ? "PASS" : "FAIL", what.c_str());
  if (!ok) ++g_fail;
}

constexpr double kLegacyScale = 0.6;   // legacy's shipped tx_full_scale
constexpr double kBlScale = 0.34;      // nr_pss_bl at -16 dBFS RMS (6.6 dB PAPR)
constexpr double kCountsPerFs = 3200.0 / kLegacyScale;  // sim peak counts at full scale
constexpr double kBenchLegacySnr = 46.5;                // measured 45 to 48 (confirm.h)
constexpr double kLegacyFloor = 30.0;
constexpr size_t kWin = 16384;
constexpr size_t kPlace = 6000;

double meanPowerOverPeak(const BeaconShape& s) {
  double sum = 0.0, peak = 0.0;
  for (const auto& v : s.core()) {
    sum += std::norm(v);
    peak = std::max(peak, static_cast<double>(std::norm(v)));
  }
  return (sum / static_cast<double>(s.coreLen())) / peak;
}

// Per-component noise variance, in counts^2, that gives legacy the bench's
// in-window SNR (the sim's snr_db is relative to each core's own mean power,
// so an ABSOLUTE noise is held by converting it per shape).
double benchNoisePower(const BeaconShape& legacy) {
  const double peak = kLegacyScale * kCountsPerFs;
  const double mean_p = peak * peak * meanPowerOverPeak(legacy);
  return mean_p / std::pow(10.0, kBenchLegacySnr / 10.0);
}

std::vector<std::complex<int16_t>> window(const BeaconShape& s, double scale, double noise_p, double cfo_hz,
                                          unsigned seed, bool beacon, bool filter) {
  sim::Channel ch;
  ch.cfo_hz = cfo_hz;
  ch.peak_counts = scale * kCountsPerFs;
  const double mean_p = ch.peak_counts * ch.peak_counts * meanPowerOverPeak(s);
  ch.snr_db = 10.0 * std::log10(mean_p / noise_p);
  std::vector<std::complex<int16_t>> w;
  if (beacon) {
    w = ch.receive(s.core(), static_cast<long long>(kPlace), 0, static_cast<long long>(kWin), seed);
  } else {
    // Noise alone at the same ABSOLUTE power (the sim scales its noise to a
    // core, so a silent window is drawn directly).
    std::mt19937 g(seed);
    auto u01 = [&g]() { return (static_cast<double>(g()) + 0.5) / 4294967296.0; };
    const double sd = std::sqrt(noise_p / 2.0);
    w.resize(kWin);
    for (auto& v : w) {
      const double a = std::sqrt(-2.0 * std::log(u01())) * std::cos(2.0 * M_PI * u01());
      const double b = std::sqrt(-2.0 * std::log(u01())) * std::cos(2.0 * M_PI * u01());
      // Truncated toward zero, as sim::Channel::receive quantizes (review: a
      // rounding here made noise-only windows 0.45 dB louder than the
      // beacon windows' noise).
      v = std::complex<int16_t>(static_cast<int16_t>(sd * a), static_cast<int16_t>(sd * b));
    }
  }
  if (filter) {
    std::vector<std::complex<float>> x(w.size()), y(w.size());
    for (size_t i = 0; i < w.size(); ++i) x[i] = std::complex<float>(w[i].real(), w[i].imag());
    houdini::dsp::ChannelFilter().run(x.data(), x.size(), y.data());
    houdini::dsp::HalfbandInterp2::quantize(y.data(), y.size(), w.data());
  }
  return w;
}

double inWindowSnr(const BeaconShape& s, const std::vector<std::complex<int16_t>>& w, ssize_t end) {
  return SnrWindowGuard(s.coreLen(), 0.0, SnrWindowGuard::guardFor(0)).snrDb(w.data(), w.size(), end);
}
}  // namespace

int main() {
  const Numerology num = Numerology::houdiniDefault();
  const auto legacy = BeaconShape::make("legacy", Platform::kHoudini, num);
  const auto bl = BeaconShape::make("nr_pss_bl", Platform::kHoudini, num);
  const double noise_p = benchNoisePower(legacy);

  // ---- the floor's shift, measured -----------------------------------------
  auto meanSnr = [&](const BeaconShape& s, double scale, bool filter) {
    double acc = 0.0;
    int n = 0;
    for (unsigned seed = 1; seed <= 20; ++seed) {
      const auto w = window(s, scale, noise_p, 0.0, seed, true, filter);
      acc += inWindowSnr(s, w, static_cast<ssize_t>(kPlace + s.coreLen()));
      ++n;
    }
    return acc / n;
  };
  const double snr_legacy = meanSnr(legacy, kLegacyScale, false);
  const double snr_bl = meanSnr(bl, kBlScale, true);
  const double shift = snr_bl - snr_legacy;
  const double floor_bl = std::round(kLegacyFloor + shift);
  std::printf("in-window SNR: legacy %.1f dB (bench-calibrated), nr_pss_bl %.1f dB (0.34 peak, filtered lane)\n",
              snr_legacy, snr_bl);
  std::printf("=> confirm.snr_floor_db for nr_pss_bl: %.0f dB (legacy's 30 dB shifted by %+.1f dB)\n", floor_bl, shift);
  // A harness figure, not an assertion: the noise was CHOSEN from 46.5 dB, so
  // this only shows the sim's estimator agrees with that choice (review).
  std::printf("harness: legacy reads %.1f dB against the 46.5 dB the noise was set from\n", snr_legacy);

  // ---- the detector, configured as the dual-band configs are --------------
  auto cfg = SyncConfig::loadFromText(R"({"sync": {"detector": {"pick": "argmax"}}})");
  cfg.resolve({bl.replicaLen(), 32.0, Platform::kHoudini, true, 1});
  const Detector det(bl, cfg.detector);
  const SnrWindowGuard guard(bl.coreLen(), floor_bl, SnrWindowGuard::guardFor(0));
  constexpr float kScale = 5.0f;  // the chosen bar 0.2; see the header

  int found = 0;
  double min_margin = 1e9;
  for (unsigned seed = 1; seed <= 20; ++seed) {
    const auto w = window(bl, kBlScale, noise_p, 20.7e3, 100 + seed, true, true);
    const auto d = det.run(w.data(), w.size(), kScale);
    const ssize_t want = static_cast<ssize_t>(kPlace + bl.coreLen());
    const double snr = inWindowSnr(bl, w, d.end_index + 1);
    // Detector end_index is the LAST sample of the core; the guard takes the
    // exclusive end.
    if (d.end_index + 1 == want && guard.accept(snr)) ++found;
    min_margin = std::min(min_margin, snr - floor_bl);
  }
  std::printf("bench-level beacons: %d/20 detected at the exact end and accepted; worst margin over the floor %.1f dB\n",
              found, min_margin);
  check(found == 20, "20/20 bench-level beacons (20.7 kHz CFO, filtered lane) detected exactly and above the floor");
  check(min_margin >= 10.0, "the floor sits at least 10 dB under the weakest bench-level beacon");

  int bar_cross = 0, accepted = 0;
  double worst_noise_snr = -99.0;
  for (unsigned seed = 1; seed <= 400; ++seed) {
    const auto w = window(bl, kBlScale, noise_p, 0.0, 5000 + seed, false, true);
    const auto d = det.run(w.data(), w.size(), kScale);
    if (d.found()) {
      ++bar_cross;
      const double snr = inWindowSnr(bl, w, d.end_index + 1);
      worst_noise_snr = std::max(worst_noise_snr, snr);
      if (guard.accept(snr)) ++accepted;
    }
  }
  std::printf("noise-only windows: %d/400 crossed the 0.2 bar, %d accepted by the floor\n", bar_cross, accepted);
  check(bar_cross == 0 && accepted == 0, "no noise-only window crosses the 0.2 bar (0 of 400), none is accepted");

  // The header's figures, asserted (review). The noise-only maximum
  // coherence, probed with a vanishing bar so every window reports its max.
  {
    std::vector<double> nmax;
    for (unsigned seed = 1; seed <= 400; ++seed) {
      const auto w = window(bl, kBlScale, noise_p, 0.0, 9000 + seed, false, true);
      nmax.push_back(det.run(w.data(), w.size(), 1e6f).statistic);
    }
    std::sort(nmax.begin(), nmax.end());
    std::printf("noise-only max coherence over 400 windows: median %.4f, p99 %.4f, worst %.4f\n", nmax[200], nmax[396],
                nmax.back());
    check(nmax.back() < 0.1, "every noise-only window's max coherence is under 0.1 (so under the 0.2 bar and the 0.1 floor bar)");
    int low_found = 0;
    for (unsigned seed = 1; seed <= 20; ++seed) {
      const auto w = window(bl, kBlScale * std::pow(10.0, -30.0 / 20.0), noise_p, 20.7e3, 300 + seed, true, true);
      const auto d = det.run(w.data(), w.size(), kScale);
      if (d.end_index + 1 == static_cast<ssize_t>(kPlace + bl.coreLen())) ++low_found;
    }
    check(low_found == 20, "20/20 beacons 30 dB under bench level still detected at their exact end with the 0.2 bar");
  }
  // The resync ladder: +1 on corr_scale per retry, stopped at min_bar 0.1.
  {
    ThresholdPolicy b;
    b.corr_scale = kScale;
    b.min_bar = 0.1;
    const float worst = static_cast<float>(b.relaxed(100));  // the default retry_max
    int crosses = 0;
    for (unsigned seed = 1; seed <= 400; ++seed) {
      const auto w = window(bl, kBlScale, noise_p, 0.0, 5000 + seed, false, true);
      if (det.run(w.data(), w.size(), worst).found()) ++crosses;
    }
    std::printf("resync retry 100 with min_bar 0.1: corr_scale %.1f (bar %.3f), %d/400 noise-only windows cross\n", worst,
                1.0 / worst, crosses);
    check(crosses == 0, "at retry 100 the relaxed bar (held at 0.1 by min_bar) is crossed by no noise-only window");
    ThresholdPolicy u;
    u.corr_scale = kScale;
    int ucross = 0;
    for (unsigned seed = 1; seed <= 200; ++seed) {
      const auto w = window(bl, kBlScale, noise_p, 0.0, 5000 + seed, false, true);
      if (det.run(w.data(), w.size(), static_cast<float>(u.relaxed(20))).found()) ++ucross;
    }
    std::printf("mutant: no min_bar, retry 20 -> bar %.3f: %d/200 cross\n", 1.0 / u.relaxed(20), ucross);
    check(ucross > 20, "mutant: without min_bar the +1-per-retry ladder walks the bar into the noise by retry 20");
  }

  std::printf("-- mutation matrix --\n");
  auto noiseCrossings = [&](const Detector& d, float scale) {
    int crosses = 0;
    for (unsigned seed = 1; seed <= 200; ++seed) {
      const auto w = window(bl, kBlScale, noise_p, 0.0, 5000 + seed, false, true);
      if (d.run(w.data(), w.size(), scale).found()) ++crosses;
    }
    return crosses;
  };
  {
    const int c = noiseCrossings(det, 100.0f);  // the carried resync corr_scale: bar 0.01
    std::printf("carried corr_scale 100 (bar 0.01): %d/200 noise-only windows cross\n", c);
    check(c > 20, "mutant: the carried corr_scale 100 bar false-alarms on noise");
  }
  {
    auto pcfg = SyncConfig::loadFromText(R"({"sync": {"detector": {"pick": "argmax", "pfa_per_window": 1e-3}}})");
    pcfg.resolve({bl.replicaLen(), 32.0, Platform::kHoudini, true, 1});
    const Detector pdet(bl, pcfg.detector);
    const int c = noiseCrossings(pdet, 100.0f);
    std::printf("pfa 1e-3 (white-noise bar %.4f): %d/200 noise-only windows cross\n",
                1.0 / pdet.effectiveScale(100.0f, kWin), c);
    check(c > 20, "mutant: the white-noise pfa bar false-alarms on a band-limited replica (127, not 512, degrees of freedom)");
  }
  {
    // The carried floor would still accept every bench beacon (it has margin),
    // but it is the wrong number to calibrate against; the floor's job is also
    // to reject what crosses the bar on noise, which it does at any sane value.
    // So the floor's mutant is a floor ABOVE the bench beacons: it must reject.
    const SnrWindowGuard high(bl.coreLen(), min_margin + floor_bl + 1.0, SnrWindowGuard::guardFor(0));
    const auto w = window(bl, kBlScale, noise_p, 20.7e3, 101, true, true);
    const auto d = det.run(w.data(), w.size(), kScale);
    check(!high.accept(inWindowSnr(bl, w, d.end_index + 1)), "mutant: a floor above the bench level rejects the beacon");
  }
  std::printf("%s: %d failure(s)\n", g_fail == 0 ? "ALL PASS" : "FAILED", g_fail);
  return g_fail == 0 ? 0 : 1;
}
