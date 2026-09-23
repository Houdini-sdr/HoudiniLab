/**
 * @file houdini/slot_align.h
 * @brief Where the UE's pilot slot starts in a BS capture, aligned on the
 *        WHOLE burst (AP-79).
 *
 * The UE sends its pilot and every uplink-data slot as ONE burst, each at an
 * exact whole-slot offset from the pilot (receiver.cc composes it). The BS
 * framer used to centroid-align each slot separately, over a window 1.25 slots
 * wide starting n/8 before the slot. That dated from when each slot was its own
 * burst snapped to the TDD grid. With P and U in ADJACENT slots (the AP-79
 * schedule) each window takes in its neighbour's energy: P's centroid is pulled
 * late by U's head and U's early by P's tail. Measured on silicon (R1c,
 * 2026-09-23): U extracted 264-317 samples before P + n, and the data would
 * not decode (5 dB) where the same capture, correctly windowed, decodes at
 * 39.7 dB.
 *
 * The fix places the pilot once, by its leading edge (see burstPilotStart),
 * and every other slot at a whole number of slots from it, as sent.
 */
#pragma once

#include <algorithm>
#include <cmath>
#include <vector>

namespace houdini {
namespace slotalign {

/// Mean index of the samples whose 128-sample energy (from the cumulative
/// energy `cse`, cse[i] = sum of |x|^2 over [0, i)) exceeds 15 % of the
/// window's peak, over [w0, w1). Returns false when nothing qualifies.
inline bool countCentroid(const std::vector<double>& cse, long long w0, long long w1, double* centroid) {
  const long long cg = static_cast<long long>(cse.size()) - 1;
  w0 = std::max(0LL, w0);
  w1 = std::min(cg, w1);
  double peak = 0.0;
  for (long long i = w0 + 64; i + 64 <= w1; ++i) peak = std::max(peak, cse[i + 64] - cse[i - 64]);
  const double thr = 0.15 * peak;
  long long cnt = 0;
  double isum = 0.0;
  for (long long i = w0 + 64; i + 64 <= w1; ++i)
    if (cse[i + 64] - cse[i - 64] > thr) {
      ++cnt;
      isum += static_cast<double>(i);
    }
  if (cnt == 0) return false;
  *centroid = isum / static_cast<double>(cnt);
  return true;
}

/// The pilot slot's start in the capture, from the pilot's LEADING EDGE.
///
/// A centroid over the whole burst (the first fix tried) is level-sensitive:
/// it counts samples above a fraction of the PEAK, so a data slot quieter than
/// the pilot is under-counted and one more than ~8 dB down drops out entirely.
/// The edge is not: the slot before the pilot is silent (a guard), and the
/// 128-sample energy window centred on i is exactly half full at the first
/// content sample, so the first crossing of half the pilot's plateau IS the
/// content start, whatever follows the pilot and at whatever level. The pilot
/// slot starts `prefix` samples before it; every other slot of the burst sits
/// a whole number of slots from it, as sent.
///
/// `guess` is the pilot slot's coarse start (within n/4). Falls back to
/// `guess` when no plateau is found. Clamped into the capture.
inline long long burstPilotStart(const std::vector<double>& cse, long long guess, long long n, int prefix) {
  const long long cg = static_cast<long long>(cse.size()) - 1;
  auto m = [&](long long i) { return cse[i + 64] - cse[i - 64]; };
  // The pilot's plateau: the median windowed energy over its middle half.
  std::vector<double> mid;
  for (long long i = guess + n / 4; i < guess + 3 * n / 4; i += 16)
    if (i - 64 >= 0 && i + 64 <= cg) mid.push_back(m(i));
  long long st = guess;
  if (!mid.empty()) {
    std::nth_element(mid.begin(), mid.begin() + static_cast<long>(mid.size() / 2), mid.end());
    const double half = 0.5 * mid[mid.size() / 2];
    if (half > 0.0) {
      for (long long i = std::max(64LL, guess - n / 4); i + 64 <= cg && i < guess + n / 2; ++i)
        if (m(i) >= half) {
          st = i - prefix;
          break;
        }
    }
  }
  return std::max(0LL, std::min(st, cg - n));
}

}  // namespace slotalign
}  // namespace houdini
