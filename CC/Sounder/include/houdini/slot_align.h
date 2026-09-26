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
#include <utility>
#include <vector>

namespace houdini {
namespace slotalign {

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

/// Where the pilot slot must start in a BS capture, from the BS's own
/// schedule: the capture's first-sample stamp and the armed schedule's epoch
/// (both in ticks, one tick per sample), the pilot slot index, the slot length
/// n and the frame length fr. The earliest copy, in [0, fr).
inline long long expectedPilotStart(long long stamp_ticks, long long epoch, long long pilot_slot, long long n,
                                    long long fr) {
  return (((epoch + pilot_slot * n - stamp_ticks) % fr) + fr) % fr;
}

/// Which copy of the scheduled pilot to search around. The next frame's copy
/// when the pilot sits in the read's first n/2 samples (it may start before
/// sample 0, where the edge search finds no silent guard before it) AND that
/// copy fits the read with the window and the rx span (`span_n`, as the framer
/// counts it); otherwise `expect`. A short read (a gap) keeps the head copy:
/// shifting there left the window empty and fell back to the whole read.
inline long long chooseExpect(long long expect, long long n, long long fr, long long span_n, long long cg) {
  if (expect < n / 2 && expect + fr + n / 4 + span_n <= cg) return expect + fr;
  return expect;
}

/// The densest n-sample window whose start lies within +-tol of `expect`,
/// stepping `step`, over the cumulative energy cse: {start, energy}, or
/// {-1, 0} when no window fits the capture. The host half of the user's
/// contract that a TDD node receives only its RX slots (SH-347): the BS keeps
/// its rx gate open all frame (a gate close abandons the continuous capture),
/// so over the air its own beacon slot and the guards carry whatever is on
/// the air, and a whole-frame search takes the loudest of it for the UE.
/// Searching only the scheduled pilot position cannot.
inline std::pair<long long, double> densestNear(const std::vector<double>& cse, long long expect, long long tol,
                                                long long n, long long step) {
  const long long cg = static_cast<long long>(cse.size()) - 1;
  const long long lo = std::max(0LL, expect - tol), hi = std::min(cg - n, expect + tol);
  std::pair<long long, double> best{-1, 0.0};
  for (long long t = lo; t <= hi; t += step) {
    const double e = cse[static_cast<size_t>(t + n)] - cse[static_cast<size_t>(t)];
    if (best.first < 0 || e > best.second) best = {t, e};
  }
  return best;
}

/// The LTS check: the pilot is identical repeated symbols, so its
/// self-similarity at lag cp + fft is high; data and noise score low. Below
/// this a pilot slot is not trusted.
constexpr double kLtsMinSelfsim = 0.4;

/// The BS presence gate on one lane: a UE burst is there when the densest
/// window's rms clears the absolute bar and four times the read's quietest
/// slot-length window. In slots mode (`floorless`) the read has no noise-only
/// window: the guards are the device's cut, exact zeros, so the floor reads 0
/// and the relative bar could never fire. There the pilot's own LTS check
/// (self-similarity `ss` at least kLtsMinSelfsim) stands in for it, so a silent UE with
/// interference above the absolute bar is a quiet frame, not a delivered one.
inline bool lanePresent(double pilot_rms, double floor_rms, double ss, bool floorless) {
  if (pilot_rms < 120.0) return false;
  return floorless ? ss >= kLtsMinSelfsim : pilot_rms >= 4.0 * floor_rms;
}

/// The placed pilot start against its scheduled slot boundary, folded into
/// [-fr/2, fr/2]: HOUDINI_BS_RX's pilot_grid_off. `stamp_ticks` is the read's
/// first-sample tick, `p_start` the placed start in the read.
inline long long pilotGridOff(long long stamp_ticks, long long p_start, long long epoch, long long pilot_slot,
                              long long n, long long fr) {
  long long rel = ((stamp_ticks + p_start - epoch) % fr + fr) % fr - pilot_slot * n;
  if (rel > fr / 2) rel -= fr;
  if (rel < -fr / 2) rel += fr;
  return rel;
}

/// Slots mode: whether the burst's content starts at (or within 2 samples of)
/// the pilot slot's edge. The device cuts there, so a burst more than its
/// prefix early loses its head, the edge search locks to the cut, and the
/// placed start pins at -prefix; nothing else says so.
inline bool headAtSlotEdge(long long grid_off, int prefix) { return grid_off <= -(prefix - 2); }

/// Whether a candidate lane takes the slot cut from the current reference.
/// Lane 0 starts as the reference. A lane that fails the presence gate never
/// takes it; a passing lane takes it from a failing reference, or from a
/// passing one whose pilot self-similarity it beats by more than 0.05. So a
/// frame is skipped only when every lane fails, and a weak lane can neither
/// skip a frame another lane carries nor place its cut.
inline bool laneTakesCut(bool cand_present, double cand_ss, bool ref_present, double ref_ss) {
  if (!cand_present) return false;
  return !ref_present || cand_ss > ref_ss + 0.05;
}

}  // namespace slotalign
}  // namespace houdini
