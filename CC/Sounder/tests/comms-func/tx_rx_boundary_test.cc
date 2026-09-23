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
  TxBurstInterpolator ti(key);
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

  std::printf("-- mutation matrix (each line must read PASS: the mutant was caught) --\n");
  check(!rewriteInPlaceIsFresh(TxBurstInterpolator::CacheKey::kAddress),
        "mutant address-keyed cache transmits the STALE burst after an in-place rewrite");
  std::printf("%s: %d failure(s)\n", g_fail == 0 ? "ALL PASS" : "FAILED", g_fail);
  return g_fail == 0 ? 0 : 1;
}
