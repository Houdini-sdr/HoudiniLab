/**
 * @file houdini/replay_strobe.h
 * @brief The beacon replay strobe's offset and length (TDD_REPLAY_STROBE
 *        offs / len), as one pure function so the units are tested (AP-79).
 *
 * UNITS, from the driver and fpga lanes (2026-09-22): offs is in 122.88 MHz
 * ticks whatever the TX rate, at least 256, and the RTL quantizes the stamp up
 * to the beat grid; len counts units of 2 TX SAMPLES (so one unit is one tick
 * at TX = 2 x the tick rate, half a tick at 1 x), 8..2048, in whole 8-unit
 * beats; a burst streams from replay RAM address 0 at window_open + offs, and
 * one that overlaps the gate close is a GATED_DROPS case.
 *
 * `k_tx` is TX samples per tick (1 or 2); `n_load_ticks` the image loaded
 * (built at the tick rate, x k_tx on the wire); the image holds `lead_ticks`
 * zeros, the `beacon_ticks` core, and needs `tail_ticks` after it (the
 * prefiltered interpolator's margins; 0 at 1 x). The strobe is pulled in by
 * the lead so the core still plays at `grid_offs` (384).
 *
 * RENEW OPEN SOURCE LICENSE: http://renew-wireless.org/license
 */
#pragma once

#include <algorithm>
#include <cstddef>
#include <stdexcept>
#include <string>

namespace houdini {

struct ReplayStrobe {
  long long offs = 0;    ///< ticks
  size_t len_units = 0;  ///< 2-TX-sample units, whole 8-unit beats
};

/// The inputs by NAME: six integers in a row could be swapped at the call
/// site (lead and tail are both small ints) and still compile (review).
struct ReplayStrobeInputs {
  int k_tx = 1;                ///< TX samples per tick (1 or 2)
  long long symbol_ticks = 0;  ///< the replay window (the TDD symbol = the sounder slot)
  size_t n_load_ticks = 0;     ///< the image loaded, at the tick rate
  int beacon_ticks = 0;        ///< the beacon core
  int lead_ticks = 0;          ///< zeros ahead of the core in the image
  int tail_ticks = 0;          ///< room the interpolator needs after the core
  long long grid_offs = 384;   ///< where the core must play, ticks after window open
};

inline ReplayStrobe replayStrobe(const ReplayStrobeInputs& in) {
  const int k_tx = in.k_tx, beacon_ticks = in.beacon_ticks, lead_ticks = in.lead_ticks,
            tail_ticks = in.tail_ticks;
  const long long symbol_ticks = in.symbol_ticks, grid_offs = in.grid_offs;
  const size_t n_load_ticks = in.n_load_ticks;
  if (k_tx != 1 && k_tx != 2) throw std::invalid_argument("replayStrobe: k_tx must be 1 or 2");
  ReplayStrobe s;
  s.offs = grid_offs - lead_ticks;
  if (s.offs < 256) throw std::invalid_argument("replayStrobe: offs below the driver's 256-tick floor");
  if (symbol_ticks <= s.offs) throw std::invalid_argument("replayStrobe: the window ends before the strobe");
  const size_t k = static_cast<size_t>(k_tx);
  const size_t image_units = k * n_load_ticks / 2;
  const size_t span_units = static_cast<size_t>(symbol_ticks - s.offs) * k / 2;
  size_t len = std::max<size_t>((k * static_cast<size_t>(lead_ticks + beacon_ticks) + 1) / 2,
                                std::min(image_units, span_units));
  const size_t cap = std::min(image_units, span_units) / 8 * 8;
  const size_t need = (k * static_cast<size_t>(lead_ticks + beacon_ticks + tail_ticks) + 1) / 2;
  if (need > cap) {
    throw std::invalid_argument("Houdini beacon: " + std::to_string(need) +
                                " replay units do not fit the window (" + std::to_string(cap) +
                                " whole-beat units between the strobe offset and the slot end)");
  }
  s.len_units = std::min(((len + 7) / 8) * 8, cap);
  return s;
}

}  // namespace houdini
