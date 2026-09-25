// The Spectrum tab's Welch average (houdini/spectrum.h): known answers for the
// dBFS normalisation, the DC-centred order and the bin reduction, at the
// sounder's own sizes (2048-point segments, 512 bins, a 61440-sample slot).
// Each assertion names its mutation.
#include <cmath>
#include <complex>
#include <cstdio>
#include <random>
#include <stdexcept>
#include <vector>

#include "houdini/spectrum.h"

static int failures = 0;
static void check(bool ok, const char* what) {
  std::printf("%s  %s\n", ok ? "PASS" : "FAIL", what);
  if (!ok) ++failures;
}

static const int kSeg = 2048, kBins = 512, kR = kSeg / kBins, kSlot = 61440;
static const double kFs = 32767.0;

// A complex tone of amplitude a at `cyc` cycles per segment, as CS16.
static std::vector<short> tone(double a, double cyc, int n) {
  std::vector<short> d(2 * static_cast<size_t>(n));
  for (int i = 0; i < n; ++i) {
    const double ph = 2 * M_PI * cyc * i / kSeg;
    d[2 * i] = static_cast<short>(std::lround(a * std::cos(ph)));
    d[2 * i + 1] = static_cast<short>(std::lround(a * std::sin(ph)));
  }
  return d;
}
static int argmax(const std::vector<float>& v) {
  int k = 0;
  for (int i = 1; i < static_cast<int>(v.size()); ++i)
    if (v[i] > v[k]) k = i;
  return k;
}
static double lin(float db) { return std::pow(10.0, db / 10.0); }

int main() {
  houdini::WelchSpectrum w(kSeg, kBins);

  // 1. A full-scale complex tone centred in display bin 400 (fine bins
  // 1600..1603, so fine position 1601.5, i.e. 1601.5 - 1024 cycles a segment).
  {
    const int i0 = 400;
    const auto d = tone(kFs, i0 * kR + 1.5 - kSeg / 2, kSlot);
    const auto s = w.run(d.data(), kSlot, false, kFs);
    check(static_cast<int>(s.size()) == kBins, "one value per display bin");
    check(argmax(s) == i0, "the tone lands in its own bin, 400 (mutation: no DC-centring rotation moves it to 144)");
    std::printf("      tone bin %.3f dBFS\n", s[i0]);
    check(std::fabs(s[i0]) < 0.1,
          "a full-scale complex tone reads 0 dBFS within 0.1 dB (mutation: normalise by sum(w)^2, the "
          "peak-bin convention, reads +1.8 dB; average the r fine bins instead of summing reads -6.0 dB)");
    // Parseval: the bins sum to the signal's power, whatever the leakage.
    double tot = 0.0;
    for (float v : s) tot += lin(v);
    check(std::fabs(10 * std::log10(tot)) < 0.05, "the bins sum to the tone's power, 0 dBFS (mutation: drop the 1/nseg, +14.8 dB)");
    // The R2C inversion: conj puts the same tone at the mirror frequency.
    const auto c = w.run(d.data(), kSlot, true, kFs);
    // Mirror of fine position 1601.5 about DC (1024) is 446.5, display bin 111.
    check(argmax(c) == 111, "conj mirrors the tone about DC to bin 111 (mutation: ignore conj, it stays at 400)");
  }

  // 2. A tone ON a display-bin edge splits between the two bins, but the pair
  // holds all of its power: no part of the band is blind.
  {
    const auto d = tone(kFs, 300 * kR - 0.5 - kSeg / 2, kSlot);  // between fine bins 1199 and 1200
    const auto s = w.run(d.data(), kSlot, false, kFs);
    std::printf("      edge tone bins 299/300: %.2f / %.2f dBFS\n", s[299], s[300]);
    check(std::fabs(10 * std::log10(lin(s[299]) + lin(s[300]))) < 0.1,
          "an edge tone's two bins sum to 0 dBFS (mutation: a max over the r fine bins instead of the sum)");
    check(std::fabs(s[299] - s[300]) < 0.2, "and it splits evenly, -3 dB each (mutation: group fine bins [i r + 1, (i+1) r + 1))");
  }

  // 3. DC-centred order: a constant (DC) reads in bin nbins/2; a tone one fine
  // bin above -rate/2 reads in bin 0.
  {
    std::vector<short> dc(2 * static_cast<size_t>(kSlot));
    for (int i = 0; i < kSlot; ++i) { dc[2 * i] = 8000; dc[2 * i + 1] = 0; }
    check(argmax(w.run(dc.data(), kSlot, false, kFs)) == kBins / 2, "DC reads in bin nbins/2 (mutation: no rotation, bin 0; the off-by-one grouping, bin 255)");
    const auto lo = tone(kFs / 4, 1 - kSeg / 2, kSlot);
    check(argmax(w.run(lo.data(), kSlot, false, kFs)) == 0, "-rate/2 + one fine bin reads in bin 0 (mutation: no rotation, bin 256)");
  }

  // 4. White noise of known power P reads P - 10 log10(nbins) in every bin.
  {
    std::mt19937 rng(7);
    const double sigma = 1000.0;  // per component, counts
    std::normal_distribution<double> g(0.0, sigma);
    std::vector<short> d(2 * static_cast<size_t>(kSlot));
    double p = 0.0;  // the samples' actual mean power, fs^2 units (after rounding)
    for (int i = 0; i < 2 * kSlot; ++i) {
      d[i] = static_cast<short>(std::lround(g(rng)));
      p += static_cast<double>(d[i]) * d[i];
    }
    p /= kSlot * kFs * kFs;
    const double want = 10 * std::log10(p) - 10 * std::log10(static_cast<double>(kBins));
    const auto s = w.run(d.data(), kSlot, false, kFs);
    double mean = 0.0, worst = 0.0;
    for (float v : s) { mean += lin(v); worst = std::max(worst, std::fabs(v - want)); }
    mean = 10 * std::log10(mean / kBins);
    std::printf("      noise: expect %.2f dBFS a bin, mean %.2f, worst bin off by %.2f dB\n", want, mean, worst);
    check(std::fabs(mean - want) < 0.2,
          "white noise of power P reads P - 10 log10(512) a bin on average (mutation: average the r fine bins, -6.0 dB)");
    check(worst < 2.5, "every bin within 2.5 dB of it (30 segments x 4 fine bins; mutation: one segment only, the spread doubles)");
  }

  // 5. The reduction: each display bin is the linear sum of its r fine bins.
  {
    std::mt19937 rng(11);
    std::normal_distribution<double> g(0.0, 300.0);
    auto d = tone(12000.0, 200.25, kSlot);
    for (auto& v : d) v = static_cast<short>(v + std::lround(g(rng)));
    houdini::WelchSpectrum fine(kSeg, kSeg);
    const auto f = fine.run(d.data(), kSlot, false, kFs);
    const auto s = w.run(d.data(), kSlot, false, kFs);
    double worst = 0.0;
    for (int i = 0; i < kBins; ++i) {
      double sum = 0.0;
      for (int j = i * kR; j < (i + 1) * kR; ++j) sum += lin(f[j]);
      worst = std::max(worst, std::fabs(10 * std::log10(sum) - s[i]));
    }
    check(worst < 0.01, "bin i is the sum of fine bins [4i, 4i+4) (mutation: a max, or a mean, of them)");
  }

  // 6. Edges: a short slot gives nothing, zeros stay finite, bad sizes throw.
  {
    std::vector<short> z(2 * static_cast<size_t>(kSlot), 0);
    check(w.run(z.data(), kSeg - 1, false, kFs).empty(), "a slot shorter than one segment gives no spectrum");
    const auto s = w.run(z.data(), kSlot, false, kFs);
    check(std::isfinite(s[0]) && s[0] <= -199.9f, "silence reads the finite -200 dB floor (mutation: drop the floor, -inf)");
    bool threw = false;
    try { houdini::WelchSpectrum bad(kSeg, 500); } catch (const std::invalid_argument&) { threw = true; }
    check(threw, "nbins that does not divide the segment throws");
  }

  std::printf("%s: %d failure(s)\n", failures ? "FAIL" : "PASS", failures);
  return failures ? 1 : 0;
}
