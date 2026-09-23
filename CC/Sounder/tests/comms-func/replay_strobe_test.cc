/**
 * @file replay_strobe_test.cc
 * @brief houdini::replayStrobe, the beacon replay strobe's offs and len in the
 *        driver's units (AP-79), NO hardware. The values the device receives,
 *        pinned for the three configurations that exist, and the units slips a
 *        change could make, each required to change the answer.
 *
 * Expected values, by hand (driver/fpga contract in replay_strobe.h):
 *   R1/R2/R3 at TX 245.76 (k=2): image 2048 ticks = 4096 TX samples = 2048
 *     units; lead 24, core 1076, tail 22; offs = 384 - 24 = 360; the window
 *     span (4096 - 360) = 3736 units at R1, 61080 at R3; len = the image,
 *     2048 units (whole beats), so the core plays at 360 + 24 = 384.
 *   The pre-mode-V single-band path (k=1): image 4096 ticks = 2048 units, core
 *     496 (legacy), no lead/tail; offs 384; span (4096 - 384) / 2 = 1856; len
 *     1856, as before AP-79 (232 whole beats).
 *
 * Build: CMake target replay_strobe_test. Run: ./replay_strobe_test (or ctest).
 */
#include <cstdio>
#include <functional>
#include <stdexcept>
#include <string>

#include "houdini/replay_strobe.h"

namespace {
int g_fail = 0;
void check(bool ok, const std::string& what) {
  std::printf("%s  %s\n", ok ? "PASS" : "FAIL", what.c_str());
  if (!ok) ++g_fail;
}
bool throws(const std::function<void()>& f) {
  try { f(); } catch (const std::invalid_argument&) { return true; }
  return false;
}
bool is(const houdini::ReplayStrobe& s, long long offs, size_t len) { return s.offs == offs && s.len_units == len; }
houdini::ReplayStrobe strobe(int k, long long sym, size_t load, int beacon, int lead, int tail) {
  houdini::ReplayStrobeInputs in;
  in.k_tx = k;
  in.symbol_ticks = sym;
  in.n_load_ticks = load;
  in.beacon_ticks = beacon;
  in.lead_ticks = lead;
  in.tail_ticks = tail;
  return houdini::replayStrobe(in);
}
}  // namespace

int main() {
  const auto r1 = strobe(2, 4096, 2048, 1076, 24, 22);
  const auto r3 = strobe(2, 61440, 2048, 1076, 24, 22);
  const auto sb = strobe(1, 4096, 4096, 496, 0, 0);
  std::printf("R1 offs %lld len %zu; R3 offs %lld len %zu; single-band offs %lld len %zu\n", r1.offs, r1.len_units,
              r3.offs, r3.len_units, sb.offs, sb.len_units);
  check(is(r1, 360, 2048), "R1/R2 (TX 245.76, 4096-tick slot): offs 360, len 2048 units = the whole 4096-sample image");
  check(is(r3, 360, 2048), "R3 (TX 245.76, 61440-tick slot): offs 360, len 2048 units");
  check(is(sb, 384, 1856), "single-band (TX 122.88): offs 384, len 1856, unchanged from before AP-79");
  check(r1.len_units % 8 == 0 && r3.len_units % 8 == 0 && sb.len_units % 8 == 0, "every len is whole 8-unit beats");
  check(r1.offs + 24 == 384, "the core plays at +384 ticks (offs + lead), where the UE's beacon geometry expects it");
  check(throws([] { strobe(2, 1000, 2048, 1076, 24, 22); }),
        "a window too short for lead + core + tail is refused, not cut at the gate close");
  check(throws([] { strobe(2, 4096, 2048, 1076, 200, 22); }), "an offs below the driver's 256 floor is refused");

  // INPUT SENSITIVITY, not code mutants (review): each units slip a caller
  // could make changes the answer, so the pinned values above would catch it.
  // A slip INSIDE the function is caught by the pinned values themselves
  // (dropping k turns R1's len into 1024); the named inputs struct stops the
  // call site from swapping lead and tail.
  std::printf("-- input sensitivity: each units slip at the call site changes what reaches the device --\n");
  check(!is(strobe(1, 4096, 2048, 1076, 24, 22), 360, 2048),
        "slip: len not doubled at TX 245.76 (k taken as 1) changes R1's len");
  check(!is(strobe(2, 4096, 2048, 1076, 0, 22), 360, 2048),
        "slip: offs not pulled in by the lead (core would play at +408) changes R1's offs");
  check(strobe(2, 4096, 4096, 1076, 24, 22).len_units != 2048,
        "slip: the image counted in TX samples (4096) instead of ticks changes R1's len (it would play past the image)");
  std::printf("%s: %d failure(s)\n", g_fail == 0 ? "ALL PASS" : "FAILED", g_fail);
  return g_fail == 0 ? 0 : 1;
}
