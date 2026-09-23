/**
 * @file tx_rx_boundary_test.cc
 * @brief houdini/tx_rx_boundary.h, NO hardware: the TX burst interpolator the
 *        radio boundary uses (beat padding, the content-keyed cache, saturation
 *        reporting) and the per-lane RX channel filter.
 *
 * The interpolator's own spectral quality is band_filters_test's job; this
 * checks the PLUMBING around it, where the failure modes are a burst of the
 * wrong length, a stale cached burst, a clipped burst reported as clean, and a
 * filter on the wrong lane.
 *
 * Build: CMake target tx_rx_boundary_test. Run: ./tx_rx_boundary_test (or ctest).
 */
#include <algorithm>
#include <cmath>
#include <complex>
#include <cstdio>
#include <string>
#include <vector>

#include "houdini/tx_rx_boundary.h"

namespace {

using houdini::boundary::cs16;
using houdini::boundary::RxLaneFilters;
using houdini::boundary::TxBurstInterpolator;

int g_fail = 0;
void check(bool ok, const std::string& what) {
  std::printf("%s  %s\n", ok ? "PASS" : "FAIL", what.c_str());
  if (!ok) ++g_fail;
}

// A burst shaped like the UE's: zeros, a tone "slot", zeros.
std::vector<cs16> burst(size_t n, double f_frac, double amp, size_t lead = 32, size_t tail = 32) {
  std::vector<cs16> b(n, cs16(0, 0));
  for (size_t k = lead; k + tail < n; ++k) {
    const double ph = 2.0 * M_PI * f_frac * static_cast<double>(k);
    b[k] = cs16(static_cast<int16_t>(std::lround(amp * std::cos(ph))),
                static_cast<int16_t>(std::lround(amp * std::sin(ph))));
  }
  return b;
}

std::vector<cs16> direct(const std::vector<cs16>& in) {
  const houdini::dsp::HalfbandInterp2 hb;
  std::vector<cs16> p(in);
  p.resize(houdini::boundary::beatPaddedInput(in.size()), cs16(0, 0));
  std::vector<cs16> out(2 * p.size());
  hb.runCs16(p.data(), p.size(), out.data());
  return out;
}

bool equal(const void* a, const std::vector<cs16>& b) {
  const auto* p = static_cast<const cs16*>(a);
  for (size_t k = 0; k < b.size(); ++k)
    if (p[k] != b[k]) return false;
  return true;
}

// The cache scenario the receiver produces: one buffer, rewritten IN PLACE with
// new content (a new pad), then sent again. Returns whether the second output
// is the interpolation of the NEW content.
bool rewriteInPlaceIsFresh(TxBurstInterpolator::CacheKey key) {
  TxBurstInterpolator ti(false, key);
  auto buf = burst(4096, 0.05, 9000.0);
  const void* p[1] = {buf.data()};
  ti.run(p, 1, buf.size());
  const auto fresh = burst(4096, 0.11, 9000.0, 40);
  std::copy(fresh.begin(), fresh.end(), buf.begin());  // same buffer, new content
  const auto o = ti.run(p, 1, buf.size());
  return equal(o.buffs[0], direct(buf));
}

}  // namespace

int main() {
  {  // lengths: x2 and a whole 8-sample beat
    TxBurstInterpolator ti;
    auto b = burst(4097, 0.03, 8000.0);
    const void* p[1] = {b.data()};
    const auto o = ti.run(p, 1, b.size());
    std::printf("4097 input samples -> %zu TX samples\n", o.samples);
    check(o.samples == 2 * 4100 && o.samples % 8 == 0,
          "a 4097-sample burst leaves as 8200 TX samples, a whole number of 8-sample beats");
    check(equal(o.buffs[0], direct(b)), "the output is exactly the halfband interpolation of the zero-padded burst");
  }
  {  // two channels, independent content
    TxBurstInterpolator ti;
    auto a = burst(2048, 0.02, 7000.0), b = burst(2048, -0.07, 7000.0);
    const void* p[2] = {a.data(), b.data()};
    const auto o = ti.run(p, 2, a.size());
    check(o.buffs.size() == 2 && equal(o.buffs[0], direct(a)) && equal(o.buffs[1], direct(b)),
          "two channels are interpolated independently, each from its own buffer");
  }
  {  // TX spectral shaping: energy 40.2 to 90.2 MHz from the NCO folds back
     // onto the sub-6 channel at the far ADC, so the prefiltered output must
     // hold it >= 60 dB down, and the unshaped output (the mutant) must not
    auto splatter = [](const TxBurstInterpolator::Out& o, size_t nch0) {
      // energy of channel 0's TX output in |f| 40.2..90.2 MHz vs total, 245.76 MSPS
      const auto* p = static_cast<const cs16*>(o.buffs[nch0]);
      const size_t n = o.samples;
      double in = 0.0, out = 0.0;
      for (size_t k = 0; k < n; k += 3) {  // every 3rd bin is plenty for a band ratio
        std::complex<double> acc(0, 0);
        for (size_t t = 0; t < n; ++t) {
          const double ph = -2.0 * M_PI * static_cast<double>((k * t) % n) / static_cast<double>(n);
          acc += std::complex<double>(p[t].real(), p[t].imag()) * std::complex<double>(std::cos(ph), std::sin(ph));
        }
        const double f = std::fabs((k < n / 2 ? static_cast<double>(k) : static_cast<double>(k) - n) * 245.76e6 / n);
        // The whole folding band: TX energy +40.2 to +90.2 MHz from the NCO
        // lands on the channel at the far ADC (65.2 - f); checked on both
        // sides. Above 61.44 it sits in the halfband's transition band.
        (f >= 40.2e6 && f <= 90.2e6 ? out : in) += std::norm(acc);
      }
      return 10.0 * std::log10(out / in);
    };
    // A burst with hard edges and a symbol-boundary jump: the splatter source.
    std::vector<cs16> b(1024, cs16(0, 0));
    for (size_t k = 64; k < 960; ++k) {
      const double f = (k < 512) ? 0.07 : -0.13;  // two "symbols", in band (8.6 and -16 MHz)
      const double ph = 2.0 * M_PI * f * static_cast<double>(k);
      b[k] = cs16(static_cast<int16_t>(std::lround(8000 * std::cos(ph))), static_cast<int16_t>(std::lround(8000 * std::sin(ph))));
    }
    const void* p[1] = {b.data()};
    TxBurstInterpolator shaped(true), raw(false);
    const double s_shaped = splatter(shaped.run(p, 1, b.size()), 0);
    const double s_raw = splatter(raw.run(p, 1, b.size()), 0);
    std::printf("TX splatter 40.2-90.2 MHz: shaped %.1f dB, unshaped %.1f dB\n", s_shaped, s_raw);
    check(s_shaped <= -60.0, "prefiltered TX holds the folding band 40.2 to 90.2 MHz >= 60 dB down [mutation: no prefilter]");
    check(s_raw > -60.0, "mutant without the prefilter leaves the folding band above -60 dB");
    // margins: content prefilterLead() zeros in reproduces the long-padded
    // result; one zero fewer must not
    auto matches = [](size_t lead, size_t tail) {
      const size_t body = 96;
      std::vector<cs16> tight(lead + body + tail, cs16(0, 0)), wide(tight.size() + 128, cs16(0, 0));
      for (size_t k = 0; k < body; ++k) {
        const cs16 v(static_cast<int16_t>(3000 + 37 * k), static_cast<int16_t>(-2000 + 11 * k));
        tight[lead + k] = v;
        wide[64 + lead + k] = v;
      }
      TxBurstInterpolator a(true), w(true);
      const void* pt[1] = {tight.data()};
      const void* pw[1] = {wide.data()};
      const auto ot = a.run(pt, 1, tight.size());
      const auto ow = w.run(pw, 1, wide.size());
      const auto* yt = static_cast<const cs16*>(ot.buffs[0]);
      const auto* yw = static_cast<const cs16*>(ow.buffs[0]);
      for (size_t k = 0; k < ow.samples; ++k) {
        const bool inside = k >= 128 && k < 128 + 2 * tight.size();
        const cs16 ref = inside ? yt[k - 128] : cs16(0, 0);
        if (std::abs(ref.real() - yw[k].real()) > 1 || std::abs(ref.imag() - yw[k].imag()) > 1) return false;
      }
      return true;
    };
    const size_t L = TxBurstInterpolator::prefilterLead(), T = TxBurstInterpolator::prefilterTail();
    std::printf("prefiltered burst margins: %zu before, %zu after\n", L, T);
    // The stated margins are the filters' exact support. At int16 the outer
    // taps of the Kaiser-windowed channel filter sit under one LSB, so a cut
    // becomes VISIBLE only from about 8 samples in (measured: 7 cut still
    // matches, 9 does not); the check therefore proves sufficiency at the stated
    // margins and that it can fail at 9, not bit-tightness at 1.
    check(matches(L, T) && !matches(L - 9, T) && !matches(L, T - 9),
          "prefiltered margins reproduce the unbounded burst, and a 9-sample cut visibly truncates it");
    check(L <= 32 && T <= 32, "prefiltered margins fit the 32-tick slot prefix/postfix");
  }
  {  // a null channel (the beacon load's non-beacon streams) passes through
    TxBurstInterpolator ti;
    auto b = burst(2048, 0.04, 7000.0);
    const void* p[2] = {nullptr, b.data()};
    const auto o = ti.run(p, 2, b.size());
    check(o.buffs.size() == 2 && o.buffs[0] == nullptr && equal(o.buffs[1], direct(b)),
          "a null channel stays null (not read, not zero-filled); the other is interpolated");
  }
  {  // the cache
    TxBurstInterpolator ti;
    auto b = burst(4096, 0.05, 9000.0);
    const void* p[1] = {b.data()};
    ti.run(p, 1, b.size());
    ti.run(p, 1, b.size());
    ti.run(p, 1, b.size());
    check(ti.misses() == 1 && ti.hits() == 2, "an unchanged burst is interpolated once and reused");
    check(rewriteInPlaceIsFresh(TxBurstInterpolator::CacheKey::kContent),
          "a burst rewritten IN PLACE with new content is re-interpolated "
          "[mutation: a cache keyed on the buffer address]");
  }
  {  // saturation is reported, and the demo level does not saturate
    TxBurstInterpolator ti;
    auto lo = burst(4096, 0.05, 0.4 * 32767.0);
    const void* p[1] = {lo.data()};
    const auto o = ti.run(p, 1, lo.size());
    check(o.saturated == 0, "a 0.4 FS burst interpolates with no saturation");
    TxBurstInterpolator t2;
    std::vector<cs16> hi(4096, cs16(0, 0));
    // A full-scale square wave with long flats: the halfband rings past full
    // scale beside each edge (Gibbs), which is what the counter must catch.
    for (size_t k = 32; k < 4064; ++k) hi[k] = ((k / 64) % 2) ? cs16(32767, 32767) : cs16(-32767, -32767);
    const void* q[1] = {hi.data()};
    check(t2.run(q, 1, hi.size()).saturated > 0,
          "a full-scale square-wave burst reports saturation (the halfband overshoots its edges)");
  }
  {  // RX lanes: the BS opens {0, 2}; only ch0 (sub-6) is filtered
    const auto flags = houdini::boundary::laneFlags(std::vector<size_t>{0, 2}, [](size_t ch) { return ch == 0; });
    check(flags.size() == 2 && flags[0] && !flags[1], "BS lanes {ch0, ch2}: lane 0 filtered, lane 1 not");
    const auto rev = houdini::boundary::laneFlags(std::vector<size_t>{2, 0}, [](size_t ch) { return ch == 0; });
    check(!rev[0] && rev[1], "lane order follows rx_channels, not the channel number");
    RxLaneFilters f(flags);
    // lane 0: an in-band tone + a 0 dBc alias at +45 MHz; lane 1: the same, untouched
    const size_t n = 8192;
    std::vector<cs16> l0(n), l1(n);
    for (size_t k = 0; k < n; ++k) {
      const double a = 2 * M_PI * (10.02e6 / 122.88e6) * k, b = 2 * M_PI * (45.0e6 / 122.88e6) * k;
      l0[k] = cs16(static_cast<int16_t>(std::lround(6000 * (std::cos(a) + std::cos(b)))),
                   static_cast<int16_t>(std::lround(6000 * (std::sin(a) + std::sin(b)))));
    }
    l1 = l0;
    const std::vector<cs16> orig = l0;
    void* bufs[2] = {l0.data(), l1.data()};
    f.apply(bufs, 2, n);
    // alias power left on lane 0, over the steady middle, by projection
    auto proj = [&](const std::vector<cs16>& v, double fhz) {
      std::complex<double> acc(0, 0);
      for (size_t k = 2048; k < 6144; ++k) {
        const double ph = -2 * M_PI * (fhz / 122.88e6) * k;
        acc += std::complex<double>(v[k].real(), v[k].imag()) * std::complex<double>(std::cos(ph), std::sin(ph));
      }
      return std::abs(acc) / 4096.0;
    };
    const double rel = 20 * std::log10(proj(l0, 45.0e6) / proj(l0, 10.02e6));
    std::printf("RX lane 0: alias at +45 MHz leaves at %.1f dBc\n", rel);
    check(rel <= -40.0, "the filtered lane removes a 0 dBc alias by >= 40 dB");
    check(l1 == orig, "the unfiltered lane (X-IF) is left bit-exact");
  }

  {  // the BS slice path: a slot filtered with the capture as context is
     // bit-identical to filtering the whole capture and cutting the slot out
    const size_t cap = 20000, n = 4096;
    std::vector<cs16> capture(cap);
    for (size_t k = 0; k < cap; ++k)
      capture[k] = cs16(static_cast<int16_t>((k * 7919) % 20001 - 10000), static_cast<int16_t>((k * 104729) % 16001 - 8000));
    std::vector<cs16> whole(capture);
    void* wb[1] = {whole.data()};
    RxLaneFilters all({true});
    all.apply(wb, 1, cap);
    RxLaneFilters sl({true});
    bool exact = true;
    for (size_t st : {size_t{0}, size_t{5}, size_t{17}, size_t{3000}, cap / 2, cap - n - 17, cap - n}) {
      std::vector<cs16> out(n);
      sl.filterSlice(capture.data(), cap, st, n, out.data());
      for (size_t k = 0; k < n; ++k) exact = exact && out[k] == whole[st + k];
    }
    check(exact, "a slot filtered from the capture (starts 0, 5, 17, 3000, mid, end-17, end) equals the whole-capture filter, bit for bit "
                 "[mutation: slice without context below]");
    // The mutant: filter the slot on its own, zeros outside it (the old
    // per-window behaviour); it differs within 17 samples of each edge.
    std::vector<cs16> iso(capture.begin() + 3000, capture.begin() + 3000 + n);
    void* ib[1] = {iso.data()};
    RxLaneFilters one({true});
    one.apply(ib, 1, n);
    bool differs = false;
    for (size_t k = 0; k < n; ++k) differs = differs || iso[k] != whole[3000 + k];
    check(differs, "mutant: the slot filtered without its context differs at its edges");
  }

  {
    // AP-79 #6, PLACEMENT: an R3-like burst (32 zeros, noise-like content, 32
    // zeros) behind every pad 0..383, as the UE's ladder sends it. The
    // reference is the full computation on the zero-EXTENDED burst (64 zeros
    // appended, so no output of it takes the filter's edge path), cut at the
    // burst's own padded length: the filter on the unbounded signal. A burst
    // with fewer leading zeros than placeLead() takes the full path, and is
    // compared with the full path on itself.
    std::vector<cs16> content(6000, cs16(0, 0));
    uint32_t lcg = 12345u;
    for (size_t k = 32; k + 32 < content.size(); ++k) {
      lcg = lcg * 1664525u + 1013904223u;
      const int16_t re = static_cast<int16_t>(static_cast<int>((lcg >> 16) % 12001) - 6000);
      lcg = lcg * 1664525u + 1013904223u;
      const int16_t im = static_cast<int16_t>(static_cast<int>((lcg >> 16) % 12001) - 6000);
      content[k] = cs16(re, im);
    }
    for (bool pre : {true, false}) {
      TxBurstInterpolator placed(pre);
      TxBurstInterpolator full(pre, TxBurstInterpolator::CacheKey::kContent, false);
      bool exact = true;
      int first_bad = -1, fallbacks = 0;
      for (int pad = 0; pad < 384; ++pad) {
        std::vector<cs16> in(static_cast<size_t>(pad), cs16(0, 0));
        in.insert(in.end(), content.begin(), content.end());
        const size_t np = houdini::boundary::beatPaddedInput(in.size());
        const bool fallback = static_cast<size_t>(pad) + 32 < placed.placeLead();
        fallbacks += fallback ? 1 : 0;
        std::vector<cs16> ref_in(in);
        if (!fallback) ref_in.resize(in.size() + 64, cs16(0, 0));
        const void* rb[1] = {ref_in.data()};
        const auto ro = full.run(rb, 1, ref_in.size());
        std::vector<cs16> ref(static_cast<const cs16*>(ro.buffs[0]), static_cast<const cs16*>(ro.buffs[0]) + 2 * np);
        const void* ib[1] = {in.data()};
        const auto po = placed.run(ib, 1, in.size());
        if (po.samples != 2 * np || !equal(po.buffs[0], ref)) {
          exact = false;
          if (first_bad < 0) first_bad = pad;
        }
      }
      std::printf("      placement (%s): misses %zu over 384 pads, %d fell back, first mismatch %d\n",
                  pre ? "prefilter" : "halfband only", placed.misses(), fallbacks, first_bad);
      check(exact, std::string("placement is bit-exact against the zero-extended full computation for every pad (") +
                       (pre ? "prefilter" : "halfband only") +
                       ") (mutation: place the core at the input offset instead of twice it)");
      check(placed.misses() <= static_cast<size_t>(1 + fallbacks),
            std::string("one interpolation serves every placeable pad (") + (pre ? "prefilter" : "halfband only") +
                ") (mutation: key the cache on the burst with its pad, a miss per pad)");
    }
  }

  std::printf("-- mutation matrix (each line must read PASS: the mutant was caught) --\n");
  check(!rewriteInPlaceIsFresh(TxBurstInterpolator::CacheKey::kAddress),
        "mutant address-keyed cache transmits the STALE burst after an in-place rewrite");
  std::printf("%s: %d failure(s)\n", g_fail == 0 ? "ALL PASS" : "FAILED", g_fail);
  return g_fail == 0 ? 0 : 1;
}
