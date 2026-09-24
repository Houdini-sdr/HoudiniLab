/**
 * @file band_filters_test.cc
 * @brief The AP-79 filters measured on their implementation, NO hardware: the
 *        x2 TX interpolator and the sub-6 RX channel filter
 *        (include/dsp/band_filters.h).
 *
 * WHAT IS MEASURED. Every number comes from tones pushed through run(), not
 * from a formula evaluated on the taps, so a defect in the polyphase indexing
 * or the edge handling shows up here even when the taps are right. Tones sit
 * on exact bins of the measurement window, so a projection over the window
 * reads a tone's complex amplitude with no leakage and a -90 dB line is
 * measurable.
 *
 * THE REQUIREMENTS, and where each comes from:
 *   - interpolator images >= 60 dB down over the whole +-25 MHz waveform band
 *     (software lane, AP-79 (l): the RFDC's interpolation passband reaches
 *     about 98 MHz and there is no analog filter after the DAC);
 *   - channel filter stopband >= 40 dB down from +-40.2 MHz (the alias arrives
 *     at 0 dBc, AP-79 (m)), passband flat over +-24 MHz (96 subcarriers at
 *     480 kHz occupy +-23.28 MHz);
 *   - both ZERO PHASE, so neither adds a timing offset BS and UE would have to
 *     calibrate out.
 * The designs carry a margin over these; the checks assert the requirement,
 * and print the measured value so the margin is visible.
 *
 * EVERY ASSERTION NAMES THE MUTATION THAT BREAKS IT, AND THAT MUTATION IS
 * BUILT (the lane rule from the AP-72 review). The mutation matrix at the end
 * runs each named mutant through the same measurement and requires it to
 * FAIL; a check that no mutant can fail is not a check.
 *
 * Build: CMake target band_filters_test. Run: ./band_filters_test (or ctest).
 */
#include <cmath>
#include <complex>
#include <cstdint>
#include <cstdio>
#include <functional>
#include <string>
#include <vector>

#include "dsp/band_filters.h"

namespace {

using cf = std::complex<float>;
using cd = std::complex<double>;
using houdini::dsp::ChannelFilter;
using houdini::dsp::HalfbandInterp2;

int g_fail = 0;

void check(bool ok, const std::string& what) {
  std::printf("%s  %s\n", ok ? "PASS" : "FAIL", what.c_str());
  if (!ok) ++g_fail;
}

constexpr double kRx = 122.88e6;  // input / RX rate = the tick
constexpr double kTx = 245.76e6;  // interpolated TX rate

// Complex amplitude of the tone at `f` in y[n0 .. n0+m), rate fs. Exact when
// f completes a whole number of cycles in m samples.
cd project(const std::vector<cf>& y, size_t n0, size_t m, double f, double fs) {
  cd acc(0.0, 0.0);
  for (size_t n = 0; n < m; ++n) {
    const double ph = -2.0 * M_PI * f * static_cast<double>(n0 + n) / fs;
    acc += cd(y[n0 + n].real(), y[n0 + n].imag()) * cd(std::cos(ph), std::sin(ph));
  }
  return acc / static_cast<double>(m);
}

std::vector<cf> tone(size_t n, double f, double fs, double amp = 1.0) {
  std::vector<cf> x(n);
  for (size_t k = 0; k < n; ++k) {
    const double ph = 2.0 * M_PI * f * static_cast<double>(k) / fs;
    x[k] = cf(static_cast<float>(amp * std::cos(ph)), static_cast<float>(amp * std::sin(ph)));
  }
  return x;
}

double db(double x) { return 20.0 * std::log10(std::max(x, 1e-15)); }

// ---------------------------------------------------------------------------
// Interpolator measurements. `interp` maps an N-sample periodic input to its
// 2N-sample output; the real one is runCircular, the mutants wrap it.
using Interp = std::function<std::vector<cf>(const std::vector<cf>&)>;

constexpr size_t kN = 4096;                 // input samples, one period
constexpr double kBinIn = kRx / kN;         // 30 kHz: tones on this grid are exact

struct InterpResult {
  double worst_image_db = -999.0;  // image relative to the wanted tone, worst over the band
  double ripple_db = 0.0;          // max - min wanted-tone gain over the band
  double worst_phase_rad = 0.0;    // wanted-tone phase error, worst over the band
};

InterpResult measureInterp(const Interp& interp) {
  InterpResult r;
  double gmin = 1e9, gmax = -1e9;
  // +-25 MHz in 0.99 MHz steps, both signs, plus the band edges.
  std::vector<long> bins;
  for (long b = -833; b <= 833; b += 33) bins.push_back(b);
  bins.push_back(-833); bins.push_back(833);
  for (long b : bins) {
    const double f = static_cast<double>(b) * kBinIn;
    const auto y = interp(tone(kN, f, kRx));
    // The ideal output is the same tone sampled at 245.76 with no delay:
    // amplitude 1, phase 0 at n = 0. Its image sits 122.88 MHz away.
    const cd want = project(y, 0, 2 * kN, f, kTx);
    const double f_img = (f >= 0.0) ? f - kRx : f + kRx;
    const cd img = project(y, 0, 2 * kN, f_img, kTx);
    const double g = db(std::abs(want));
    gmin = std::min(gmin, g); gmax = std::max(gmax, g);
    r.worst_image_db = std::max(r.worst_image_db, db(std::abs(img)) - g);
    r.worst_phase_rad = std::max(r.worst_phase_rad, std::fabs(std::arg(want)));
  }
  r.ripple_db = gmax - gmin;
  return r;
}

// ---------------------------------------------------------------------------
// Channel filter measurements over a long linear buffer; the window is taken
// well inside so the zero-outside edges do not reach it.
using Filt = std::function<std::vector<cf>(const std::vector<cf>&)>;

constexpr size_t kLen = 8192, kWin = 4096, kW0 = 2048;
constexpr double kBinCh = kRx / kWin;  // 30 kHz

struct ChanResult {
  double ripple_db = 0.0;           // over +-24 MHz
  double worst_stop_db = -999.0;    // over +-[40.2, 61.44] MHz
  double worst_phase_rad = 0.0;     // over +-24 MHz
};

ChanResult measureChan(const Filt& filt) {
  ChanResult r;
  double gmin = 1e9, gmax = -1e9;
  for (long b = -800; b <= 800; b += 25) {  // +-24 MHz
    const double f = static_cast<double>(b) * kBinCh;
    const auto y = filt(tone(kLen, f, kRx));
    const cd a = project(y, kW0, kWin, f, kRx);
    const double g = db(std::abs(a));
    gmin = std::min(gmin, g); gmax = std::max(gmax, g);
    r.worst_phase_rad = std::max(r.worst_phase_rad, std::fabs(std::arg(a)));
  }
  r.ripple_db = gmax - gmin;
  for (long b = 1340; b <= 2047; b += 13) {  // 40.2 MHz .. just under Nyquist
    for (int s : {-1, 1}) {
      const double f = s * static_cast<double>(b) * kBinCh;
      const auto y = filt(tone(kLen, f, kRx));
      r.worst_stop_db = std::max(r.worst_stop_db, db(std::abs(project(y, kW0, kWin, f, kRx))));
    }
  }
  return r;
}

// The requirement predicates, shared by the real filters and the mutants so a
// mutant is judged by exactly the rule it must break.
bool imagesOk(const InterpResult& r) { return r.worst_image_db <= -60.0; }
bool interpFlat(const InterpResult& r) { return r.ripple_db <= 0.01; }
bool interpZeroPhase(const InterpResult& r) { return r.worst_phase_rad <= 1e-3; }
bool chanStopOk(const ChanResult& r) { return r.worst_stop_db <= -40.0; }
bool chanFlat(const ChanResult& r) { return r.ripple_db <= 0.1; }
bool chanZeroPhase(const ChanResult& r) { return r.worst_phase_rad <= 1e-3; }

}  // namespace

int main() {
  const HalfbandInterp2 hb;
  const ChannelFilter ch;

  // ---- interpolator --------------------------------------------------------
  const Interp real_interp = [&hb](const std::vector<cf>& x) {
    std::vector<cf> y(2 * x.size());
    hb.runCircular(x.data(), x.size(), y.data());
    return y;
  };
  const auto ir = measureInterp(real_interp);
  std::printf("interp: worst image %.1f dB, ripple %.5f dB, worst phase %.2e rad\n",
              ir.worst_image_db, ir.ripple_db, ir.worst_phase_rad);
  check(imagesOk(ir), "interp images >= 60 dB down over +-25 MHz [mutation: 7-tap halfband]");
  check(interpFlat(ir), "interp passband ripple <= 0.01 dB [mutation: beta 3 halfband]");
  check(interpZeroPhase(ir), "interp zero phase [mutation: output delayed one TX sample]");

  {  // passthrough is bit-exact: out[2k] == in[k]
    const auto x = tone(kN, 7.0 * kBinIn * 101, kRx);
    const auto y = real_interp(x);
    bool exact = true;
    for (size_t k = 0; k < kN; ++k) exact = exact && (y[2 * k] == x[k]);
    check(exact, "interp out[2k] == in[k] bit-exact [mutation: output delayed one TX sample]");
  }

  {  // edge contract: content inside the stated zero margin matches the
     // unbounded filter; one sample closer to the edge and it must not.
    auto edge_matches = [&hb](size_t margin_before, size_t margin_after) {
      const size_t body = 64;
      const auto t = tone(body, 3.3e6, kRx);
      std::vector<cf> tight(margin_before + body + margin_after, cf(0, 0));
      std::vector<cf> wide(tight.size() + 64, cf(0, 0));
      for (size_t k = 0; k < body; ++k) { tight[margin_before + k] = t[k]; wide[32 + margin_before + k] = t[k]; }
      std::vector<cf> yt(2 * tight.size()), yw(2 * wide.size());
      hb.run(tight.data(), tight.size(), yt.data());
      hb.run(wide.data(), wide.size(), yw.data());
      // The tight output must equal the wide one where they overlap AND the
      // wide output must be zero everywhere the tight buffer does not reach:
      // a margin that is too small loses the tail off the end, which an
      // overlap-only comparison cannot see.
      for (size_t k = 0; k < yw.size(); ++k) {
        const bool inside = k >= 64 && k < 64 + yt.size();
        const cf ref = inside ? yt[k - 64] : cf(0, 0);
        if (std::abs(ref - yw[k]) > 1e-6f) return false;
      }
      return true;
    };
    std::printf("interp context: %zu before, %zu after\n", hb.contextBefore(), hb.contextAfter());
    check(edge_matches(hb.contextBefore(), hb.contextAfter()),
          "interp output inside the stated margins equals the unbounded filter");
    check(!edge_matches(hb.contextBefore(), hb.contextAfter() - 1) &&
              !edge_matches(hb.contextBefore() - 1, hb.contextAfter()),
          "interp stated margins are TIGHT: one sample less changes the output "
          "[mutation: a context figure one short would pass the check above]");
    check(hb.contextAfter() <= 128 && hb.contextBefore() <= 128,
          "interp margins fit inside the 128-tick zero prefix/postfix of a TDD slot");
  }

  {  // CS16 path: saturation is counted, and the demo's level never saturates
    const HalfbandInterp2 hbq;
    auto to16 = [](const std::vector<cf>& x, double scale) {
      std::vector<std::complex<int16_t>> o(x.size());
      for (size_t k = 0; k < x.size(); ++k)
        o[k] = {static_cast<int16_t>(std::lround(scale * x[k].real())),
                static_cast<int16_t>(std::lround(scale * x[k].imag()))};
      return o;
    };
    // A two-tone signal at peak 0.4 FS (the demo back-off is about 0.3 to 0.4)
    std::vector<cf> two(kN);
    const auto a = tone(kN, 5.0e6, kRx), b = tone(kN, -17.01e6, kRx);
    for (size_t k = 0; k < kN; ++k) two[k] = 0.5f * (a[k] + b[k]);
    std::vector<std::complex<int16_t>> out(2 * kN);
    float pk = 0.0f;
    // The radio's CS16 path (interpolate, then quantize) on the looped signal.
    auto cs16Circular = [&](const std::vector<std::complex<int16_t>>& in, float* peak) {
      std::vector<cf> fin(in.size()), fout(2 * in.size());
      for (size_t k = 0; k < in.size(); ++k) fin[k] = cf(in[k].real(), in[k].imag());
      hbq.runCircular(fin.data(), fin.size(), fout.data());
      return HalfbandInterp2::quantize(fout.data(), fout.size(), out.data(), peak);
    };
    const auto in_lo = to16(two, 0.4 * 32767.0);
    const size_t sat_lo = cs16Circular(in_lo, &pk);
    std::printf("interp CS16 at 0.4 FS: peak %.0f (%.3f FS), saturated %zu\n", pk, pk / 32767.0, sat_lo);
    check(sat_lo == 0, "interp CS16 at 0.4 FS peak: no saturation");
    const auto in_hi = to16(two, 1.0 * 32767.0);
    const size_t sat_hi = cs16Circular(in_hi, &pk);
    check(sat_hi > 0, "interp CS16 saturation counter fires at full scale "
                      "[mutation: a counter that never increments would pass the check above]");
  }

  // ---- channel filter ------------------------------------------------------
  const Filt real_chan = [&ch](const std::vector<cf>& x) {
    std::vector<cf> y(x.size());
    ch.run(x.data(), x.size(), y.data());
    return y;
  };
  const auto cr = measureChan(real_chan);
  std::printf("channel: ripple %.4f dB over +-24 MHz, worst stop %.1f dB from +-40.2 MHz, worst phase %.2e rad\n",
              cr.ripple_db, cr.worst_stop_db, cr.worst_phase_rad);
  check(chanStopOk(cr), "channel stopband >= 40 dB down from +-40.2 MHz [mutation: 15-tap filter; cutoff moved to 38 MHz]");
  check(chanFlat(cr), "channel passband ripple <= 0.1 dB over +-24 MHz [mutation: cutoff moved to 24 MHz]");
  check(chanZeroPhase(cr), "channel zero phase [mutation: output delayed one sample]");

  {  // the scenario the filter exists for: a 0 dBc alias beside the signal
    std::vector<cf> x(kLen);
    const auto sig = tone(kLen, 10.02e6, kRx), alias = tone(kLen, 45.0e6, kRx);
    for (size_t k = 0; k < kLen; ++k) x[k] = sig[k] + alias[k];
    const auto y = real_chan(x);
    const double rel = db(std::abs(project(y, kW0, kWin, 45.0e6, kRx))) -
                       db(std::abs(project(y, kW0, kWin, 10.02e6, kRx)));
    std::printf("channel: 0 dBc alias at +45 MHz leaves at %.1f dBc\n", rel);
    check(rel <= -40.0, "channel removes a 0 dBc alias at +45 MHz by >= 40 dB");
  }

  // ---- mutation matrix: each named mutant must FAIL its check --------------
  std::printf("-- mutation matrix (each line must read PASS: the mutant was caught) --\n");
  {
    const HalfbandInterp2 short_hb(7, 9.0);
    const Interp m = [&short_hb](const std::vector<cf>& x) {
      std::vector<cf> y(2 * x.size()); short_hb.runCircular(x.data(), x.size(), y.data()); return y; };
    check(!imagesOk(measureInterp(m)), "mutant 7-tap halfband fails the image check");
  }
  {
    const HalfbandInterp2 soft_hb(HalfbandInterp2::kTaps, 3.0);
    check(!interpFlat(measureInterp([&soft_hb](const std::vector<cf>& x) {
            std::vector<cf> y(2 * x.size()); soft_hb.runCircular(x.data(), x.size(), y.data()); return y; })),
          "mutant beta-3 halfband fails the ripple check");
  }
  {
    const Interp delayed = [&real_interp](const std::vector<cf>& x) {
      auto y = real_interp(x);
      std::vector<cf> d(y.size());
      for (size_t k = 0; k < y.size(); ++k) d[k] = y[(k + y.size() - 1) % y.size()];
      return d;
    };
    check(!interpZeroPhase(measureInterp(delayed)), "mutant one-sample-delayed interp fails the phase check");
    const auto x = tone(kN, 7.0 * kBinIn * 101, kRx);
    const auto y = delayed(x);
    bool exact = true;
    for (size_t k = 0; k < kN; ++k) exact = exact && (y[2 * k] == x[k]);
    check(!exact, "mutant one-sample-delayed interp fails the passthrough check");
  }
  {
    const ChannelFilter short_ch(15, ChannelFilter::kBeta);
    check(!chanStopOk(measureChan([&short_ch](const std::vector<cf>& x) {
            std::vector<cf> y(x.size()); short_ch.run(x.data(), x.size(), y.data()); return y; })),
          "mutant 15-tap channel filter fails the stopband check");
    const ChannelFilter wide_ch(ChannelFilter::kTaps, ChannelFilter::kBeta, 38.0e6);
    check(!chanStopOk(measureChan([&wide_ch](const std::vector<cf>& x) {
            std::vector<cf> y(x.size()); wide_ch.run(x.data(), x.size(), y.data()); return y; })),
          "mutant 38 MHz-cutoff channel filter fails the stopband check");
    const ChannelFilter narrow_ch(ChannelFilter::kTaps, ChannelFilter::kBeta, 24.0e6);
    check(!chanFlat(measureChan([&narrow_ch](const std::vector<cf>& x) {
            std::vector<cf> y(x.size()); narrow_ch.run(x.data(), x.size(), y.data()); return y; })),
          "mutant 24 MHz-cutoff channel filter fails the ripple check");
    const Filt delayed = [&real_chan](const std::vector<cf>& x) {
      auto y = real_chan(x);
      std::vector<cf> d(y.size(), cf(0, 0));
      for (size_t k = 1; k < y.size(); ++k) d[k] = y[k - 1];
      return d;
    };
    check(!chanZeroPhase(measureChan(delayed)), "mutant one-sample-delayed channel filter fails the phase check");
  }

  std::printf("%s: %d failure(s)\n", g_fail == 0 ? "ALL PASS" : "FAILED", g_fail);
  return g_fail == 0 ? 0 : 1;
}
