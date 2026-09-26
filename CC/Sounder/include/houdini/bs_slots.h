/**
 * @file houdini/bs_slots.h
 * @brief The BS receiving only its RX slots (AP-87, the fix for O1): the TDD
 *        pattern the schedule means, where a tick sits on the slot grid, and how
 *        much of a gap in the read falls in the RX slots.
 *
 * With `bs_rx_slots` the BS arms the pattern its schedule means (the beacon
 * symbol replay strobe only, the P/U/R/N slots rx, every other slot a guard)
 * and the device's slots mode (SH-347) cuts every packet outside the rx slots
 * from the continuous stream. The receive path zero-pads a timestamp gap so the
 * samples after it stay on their true offsets, so the BS's read comes back as
 * the frame's timeline with the guards and the beacon slot as zeros: the UE's
 * P and U where they always were, and nothing else. A gap in a guard is the
 * schedule, not a loss; only the part of a gap inside an rx slot is.
 */
#pragma once

#include <algorithm>
#include <string>

namespace houdini {
namespace bsslots {

/// The TDD pattern armed for a BS schedule string (the config's 'B', 'P', 'U',
/// 'R', 'N', 'G' per slot), one hex nibble per slot {bit2 replay strobe, bit1
/// rx_gate, bit0 tx_gate}. rx_slots_only: the beacon slot '4' (strobe, no rx),
/// P/U/R/N '2', everything else '0' (a guard). Otherwise the all-rx form the
/// continuous capture needed before slots mode: '6' on the beacon, '2' elsewhere.
/// The slots form puts a guard ('0', the last slot) directly before the beacon
/// ('4'). The device refuses that arm (rule 6b, the power board's warm return)
/// only while the schedule drives TX_SEL; highz and AP-86's static source do
/// not, so it arms. A per-symbol T/R switch from the schedule would be refused,
/// loudly, at TDD_ARM.
inline std::string tddPattern(const std::string& sched, bool rx_slots_only) {
  std::string t(sched.size(), rx_slots_only ? '0' : '2');
  for (size_t s = 0; s < sched.size(); ++s) {
    const char c = sched[s];
    if (c == 'B') t[s] = rx_slots_only ? '4' : '6';
    else if (rx_slots_only && (c == 'P' || c == 'U' || c == 'R' || c == 'N')) t[s] = '2';
  }
  if (!rx_slots_only && sched.find('B') == std::string::npos && !t.empty()) t[0] = '6';  // the framer's default beacon slot
  return t;
}

/// The rx bit per slot of a pattern, as the device reports it in
/// TDD_RX_SLOTS (`rx=0101...`): '1' where bit1 is set.
inline std::string rxBits(const std::string& pattern) {
  std::string r(pattern.size(), '0');
  for (size_t s = 0; s < pattern.size(); ++s) {
    const char c = pattern[s];
    const int v = (c >= '0' && c <= '9') ? c - '0' : (c >= 'a' && c <= 'f') ? c - 'a' + 10 : (c >= 'A' && c <= 'F') ? c - 'A' + 10 : 0;
    if (v & 2) r[s] = '1';
  }
  return r;
}

/// Where a tick (one per sample) sits on the grid of an armed schedule: the
/// absolute frame since the epoch (floor, so a tick before the epoch is frame
/// -1), the slot in that frame and the offset into it.
struct SlotPos {
  long long frame = 0;
  long long slot = 0;
  long long off = 0;
};
inline SlotPos slotPos(long long tick, long long epoch, long long n, long long fr) {
  const long long d = tick - epoch;
  long long f = d / fr;
  if (d % fr < 0) --f;
  const long long in = d - f * fr;
  return {f, in / n, in % n};
}

/// How many of the `len` samples starting at `tick` fall inside an rx slot of
/// the map `rx` (one '0'/'1' per slot, slots of n ticks, frames of fr ticks from
/// `epoch`): the part of a gap that is a real loss. The rest is the schedule's
/// own gap (a guard or the beacon slot, cut by design).
inline long long rxOverlap(long long tick, long long len, long long epoch, long long n, long long fr,
                           const std::string& rx) {
  if (len <= 0 || n <= 0 || rx.empty()) return 0;
  long long got = 0, t = tick;
  const long long end = tick + len;
  while (t < end) {
    const SlotPos p = slotPos(t, epoch, n, fr);
    const long long run = std::min(end - t, n - p.off);  // to this slot's end
    const size_t s = static_cast<size_t>(p.slot);
    if (s < rx.size() && rx[s] == '1') got += run;
    t += run;
  }
  return got;
}

/// Whether the rest of a read window (`samples - got` samples on from tick
/// `start + got`) lies wholly outside the rx slots. In slots mode nothing more
/// is delivered for it, so a reader that waited would block until the next rx
/// slot (the next frame's pilot) and then keep none of it. False without a slot
/// map, so a reader outside slots mode never stops early.
inline bool restIsCut(long long start, long long got, long long samples, long long epoch, long long n, long long fr,
                      const std::string& rx) {
  if (rx.empty() || got >= samples) return false;
  return rxOverlap(start + got, samples - got, epoch, n, fr, rx) == 0;
}

}  // namespace bsslots
}  // namespace houdini
