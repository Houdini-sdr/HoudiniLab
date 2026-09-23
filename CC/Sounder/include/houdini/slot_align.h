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
/// `guess` is the pilot slot's coarse start (within about n/2; the slot
/// before the pilot must be quiet). Falls back to `guess` when no plateau is
/// found. Clamped into the capture.
inline long long burstPilotStart(const std::vector<double>& cse, long long guess, long long n, int prefix) {
  const long long cg = static_cast<long long>(cse.size()) - 1;
  auto m = [&](long long i) { return cse[i + 64] - cse[i - 64]; };
  // The coarse guess can be off by up to about n/2: when the data slot is
  // within ~0.5 dB of the pilot, the densest-window search that produces it
  // is nearly flat across P+U (an Opus review, L1). So the level is the 75th
  // percentile of the windowed energy over [guess - n/2, guess + 3n/2], which
  // holds the whole pilot either way, and the edge search starts at guess -
  // n/2, in the silent guard slot before the pilot.
  std::vector<double> span;
  for (long long i = guess - n / 2; i < guess + 3 * n / 2; i += 16)
    if (i - 64 >= 0 && i + 64 <= cg) span.push_back(m(i));
  long long st = guess;
  if (span.empty()) return std::max(0LL, std::min(st, cg - n));
  const size_t q = span.size() * 3 / 4;
  std::nth_element(span.begin(), span.begin() + static_cast<long>(q), span.end());
  const double p75 = span[q];
  if (!(p75 > 0.0)) return std::max(0LL, std::min(st, cg - n));
  // Stage 1, coarse: the first crossing of 10 % of that level. Whatever the
  // data slot's level, the pilot (which comes first) crosses it.
  const long long lo = std::max(64LL, guess - n / 2), hi_end = guess + 3 * n / 4;
  long long coarse = -1;
  for (long long i = lo; i + 64 <= cg && i < hi_end; ++i)
    if (m(i) >= 0.1 * p75) {
      coarse = i;
      break;
    }
  if (coarse < 0) return std::max(0LL, std::min(st, cg - n));
  // Stage 2: the PILOT's own plateau, the median over a quarter slot just
  // inside it, and the first crossing of half of that, from before the
  // coarse edge. Independent of the data slot's level (a louder U no longer
  // sets the threshold).
  std::vector<double> own;
  for (long long i = coarse + 192; i < coarse + 192 + n / 4; i += 8)
    if (i + 64 <= cg) own.push_back(m(i));
  if (own.empty()) return std::max(0LL, std::min(st, cg - n));
  std::nth_element(own.begin(), own.begin() + static_cast<long>(own.size() / 2), own.end());
  const double half = 0.5 * own[own.size() / 2];
  for (long long i = std::max(64LL, coarse - 128); i + 64 <= cg && i <= coarse + 192; ++i)
    if (m(i) >= half) {
      st = i - prefix;
      break;
    }
  return std::max(0LL, std::min(st, cg - n));
}

}  // namespace slotalign
}  // namespace houdini
