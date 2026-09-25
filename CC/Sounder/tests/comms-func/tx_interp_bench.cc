/**
 * @file tx_interp_bench.cc
 * @brief AP-85: the host CPU cost of the UE's x2 TX interpolation per lane
 *        path, NO hardware. Standalone (not in ctest): run it on the host that
 *        drives the UE, `./build/tx_interp_bench [rounds]`.
 *
 * The burst is the demo's: a pad, the P slot and the adjacent U slot (two
 * 61440-sample slots at fft 4096, CP 288, 14 symbols, 32-sample zero prefix
 * and postfix each), noise-like content. Three costs, per lane:
 *   - prefilter + 23-tap halfband: today's path, the sub-6 lane and, until
 *     AP-85, the X-band lane;
 *   - 47-tap wide halfband alone: the X-band lane at 270 RB;
 *   - 23-tap halfband alone: the reference the wide design is compared with;
 *   - the placement hit: the steady state of either lane. The UE re-sends
 *     the same content every frame and the interpolator shifts one stored
 *     output, so this, not the filter, is the per-burst cost once running.
 * The full computations run once per lane at the first burst (and on any
 * content change). The variants are INTERLEAVED round by round, so a clock
 * or load change on the host moves them alike, and the medians are
 * printed with their ratios.
 */
#include <algorithm>
#include <chrono>
#include <complex>
#include <cstdio>
#include <cstdlib>
#include <vector>

#include "houdini/tx_rx_boundary.h"

namespace {
using houdini::boundary::cs16;
using houdini::boundary::TxBurstInterpolator;
using Clock = std::chrono::steady_clock;

double median(std::vector<double> v) {
  std::sort(v.begin(), v.end());
  return v.empty() ? 0.0 : v[v.size() / 2];
}
}  // namespace

int main(int argc, char** argv) {
  const int rounds = argc > 1 ? std::max(1, std::atoi(argv[1])) : 15;
  constexpr size_t kSlot = 61440, kPad = 100, kZero = 32;
  const size_t n = kPad + 2 * kSlot;
  std::vector<cs16> burst(n, cs16(0, 0));
  uint32_t lcg = 12345u;
  for (size_t k = kPad + kZero; k + kZero < kPad + 2 * kSlot; ++k) {
    lcg = lcg * 1664525u + 1013904223u;
    const int16_t re = static_cast<int16_t>(static_cast<int>((lcg >> 16) % 8001) - 4000);
    lcg = lcg * 1664525u + 1013904223u;
    const int16_t im = static_cast<int16_t>(static_cast<int>((lcg >> 16) % 8001) - 4000);
    burst[k] = cs16(re, im);
  }
  // The same content one sample later: a pad change, the placement path.
  std::vector<cs16> shifted(n + 1, cs16(0, 0));
  std::copy(burst.begin(), burst.end(), shifted.begin() + 1);

  auto ms = [](Clock::time_point a, Clock::time_point b) {
    return std::chrono::duration<double, std::milli>(b - a).count();
  };
  const double sub6 = 1596 * 30e3 / 2.0, xband = 3240 * 30e3 / 2.0;
  auto run_full = [&](TxBurstInterpolator ti) {
    const void* b[1] = {burst.data()};
    const auto t0 = Clock::now();
    const auto o = ti.run(b, 1, n);
    const auto t1 = Clock::now();
    if (o.samples == 0) std::abort();
    return ms(t0, t1);
  };
  // A fresh interpolator each time: the full computation, as at a lane's first burst.
  auto full = [&](double half_bw) { return run_full(TxBurstInterpolator::forBand(half_bw)); };
  auto hb23 = [&]() { return run_full(TxBurstInterpolator(false)); };
  auto warm = TxBurstInterpolator::forBand(xband);
  {
    const void* b[1] = {burst.data()};
    warm.run(b, 1, n);
  }
  bool flip = false;
  auto hit = [&]() {
    const void* b[1] = {flip ? shifted.data() : burst.data()};
    const size_t len = flip ? n + 1 : n;
    flip = !flip;
    const auto t0 = Clock::now();
    const auto o = warm.run(b, 1, len);
    const auto t1 = Clock::now();
    if (o.samples == 0) std::abort();
    return ms(t0, t1);
  };
  std::vector<double> t_pre, t_wide, t_hb23, t_hit;
  full(sub6);  // warm the caches and the allocator once, not timed
  full(xband);
  for (int r = 0; r < rounds; ++r) {
    t_pre.push_back(full(sub6));
    t_wide.push_back(full(xband));
    t_hb23.push_back(hb23());
    t_hit.push_back(hit());
  }
  const double pre = median(t_pre), wide = median(t_wide), narrow = median(t_hb23), h = median(t_hit);
  std::printf("tx_interp_bench: one lane, burst of %zu input samples (pad + P + U at fft 4096), %d rounds interleaved\n",
              n, rounds);
  std::printf("  prefilter + 23-tap halfband (sub-6; the X-band before AP-85): %8.2f ms median\n", pre);
  std::printf("  47-tap wide halfband alone (the X-band at 270 RB):              %8.2f ms median\n", wide);
  std::printf("  23-tap halfband alone (reference):                             %8.2f ms median\n", narrow);
  std::printf("  placement hit (steady state, either lane):                      %8.3f ms median\n", h);
  std::printf("  wide / prefiltered: %.2f, wide / 23-tap alone: %.2f\n", pre > 0.0 ? wide / pre : 0.0,
              narrow > 0.0 ? wide / narrow : 0.0);
  std::printf("  (the full computations run once per lane at its first burst; the hit is the per-burst cost)\n");
  return 0;
}
