// AP-87: houdini/bs_slots.h, the BS receiving only its RX slots. Each assertion
// names the mutation that breaks it.
#include <cstdio>
#include <string>

#include "houdini/bs_slots.h"

static int failures = 0;
static void check(bool ok, const char* what) {
  std::printf("%s  %s\n", ok ? "PASS" : "FAIL", what);
  if (!ok) ++failures;
}

int main() {
  namespace bs = houdini::bsslots;
  check(bs::tddPattern("BGPUGGGGGGGGGGGGGGGG", true) == "40220000000000000000",
        "slots only: the beacon '4' (strobe, no rx), P/U '2', guards '0' (mutation: the rx bit left on the beacon)");
  check(bs::tddPattern("BGPUGGGGGGGGGGGGGGGG", false) == "62222222222222222222",
        "the legacy pattern is byte-identical to the one armed today (mutation: the default path changed)");
  check(bs::tddPattern("GBGPRUNG", true) == "04022220",
        "R and N slots receive too, the beacon anywhere in the frame (mutation: only P/U, or slot 0 assumed)");
  check(bs::rxBits("40220000000000000000") == "00110000000000000000" && bs::rxBits("6222") == "1111",
        "rxBits reads bit1 per nibble, as TDD_RX_SLOTS reports it (mutation: bit2 or any non-zero)");

  const long long n = 61440, fr = 20 * n, E = 1234567;
  auto same = [](bs::SlotPos a, long long f, long long s, long long o) { return a.frame == f && a.slot == s && a.off == o; };
  check(same(bs::slotPos(E + 2 * n, E, n, fr), 0, 2, 0) && same(bs::slotPos(E + 3 * n + 5, E, n, fr), 0, 3, 5) &&
            same(bs::slotPos(E + fr + 2 * n, E, n, fr), 1, 2, 0),
        "slotPos: frame, slot and offset from the tick and the epoch (mutation: the epoch not subtracted)");
  check(same(bs::slotPos(E - n, E, n, fr), -1, 19, 0),
        "a tick before the epoch is the previous frame's last slot (mutation: truncating division)");

  const std::string rx = bs::rxBits(bs::tddPattern("BGPUGGGGGGGGGGGGGGGG", true));  // P = 2, U = 3
  check(bs::rxOverlap(E + 4 * n, 16 * n, E, n, fr, rx) == 0,
        "a gap over the guards and the beacon slot is the schedule, not a loss (mutation: every pad counted)");
  check(bs::rxOverlap(E + 3 * n + 1000, 2 * n, E, n, fr, rx) == n - 1000,
        "a gap from inside U into the guard counts only U's part (mutation: whole-slot rounding)");
  check(bs::rxOverlap(E + 19 * n, 5 * n, E, n, fr, rx) == 2 * n,
        "a gap across the frame wrap counts the next frame's P and U (mutation: the slot not folded per frame)");
  check(bs::rxOverlap(E + 2 * n + 7, 100, E, n, fr, rx) == 100 && bs::rxOverlap(E, 0, E, n, fr, rx) == 0,
        "a loss inside P counts whole, an empty gap nothing (mutation: an off-by-one at the run's start)");
  if (failures) std::printf("FAILED: %d failure(s)\n", failures);
  else std::printf("ALL PASS\n");
  return failures ? 1 : 0;
}
