/**
 * @file houdini/cns_sample.h
 * @brief The stride the constellation view samples its (symbol, tone) pairs by.
 *
 * sendConstellation keeps a pair when (symbol * tones + tone) % stride == 0. A
 * stride that shares a factor with the tone count lands on the same few tones in
 * every symbol and every frame: at 270 RB (2970 data tones, stride 60) it sampled
 * 99 tones, one of them the refclk image's tone, and the dashboard's MER read about
 * 6 dB low (DEMO_VERIFICATION 9.56). A stride coprime with the tone count moves the
 * sampled residue with each symbol, so one frame spans the band.
 */
#pragma once

#include <cstddef>
#include <numeric>

namespace houdini {

/// The smallest stride >= ceil(pairs / max_pts) that is coprime with `tones`.
inline size_t cnsStride(size_t pairs, size_t max_pts, size_t tones) {
  size_t s = (max_pts == 0) ? 1 : (pairs + max_pts - 1) / max_pts;
  if (s < 1) s = 1;
  if (tones > 1)
    while (std::gcd(s, tones) != 1) ++s;
  return s;
}

}  // namespace houdini
