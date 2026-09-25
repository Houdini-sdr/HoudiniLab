// houdini/rx_packet.h: the RX packet that tiles a slot. Each assertion names
// the mutation that breaks it.
#include <cstdio>

#include "houdini/rx_packet.h"

static int failures = 0;
static void check(bool ok, const char* what) {
  std::printf("%s  %s\n", ok ? "PASS" : "FAIL", what);
  if (!ok) ++failures;
}

int main() {
  namespace rp = houdini::rxpkt;
  check(rp::deviceSamples(8192) == 2032,
        "the device's derivation reproduces today's 2032 at MTU 8192 (mutation: overhead or beat rounding wrong)");
  const size_t s = rp::samplesForSlot(61440);
  check(s == 1920 && 61440 % s == 0 && 61440 / s == 32,
        "the demo slot (61440) tiles as 32 packets of 1920 (mutation: a non-divisor, e.g. 2032)");
  check(122880 % s == 0 && 122880 / s == 64,
        "the same packet tiles the TX slot at the 2x TX rate (64 per 122880) (mutation: a divisor of the RX slot only)");
  check(rp::mtuFor(1920) == 7738 && rp::deviceSamples(rp::mtuFor(1920)) == 1920,
        "MTU 7738 makes the device derive exactly 1920 (mutation: mtuFor off by the header)");
  check(rp::samplesForSlot(4096) == 1024 && rp::samplesForSlot(0) == 0 && rp::samplesForSlot(61441) == 0,
        "a power-of-two slot gets 1024, since 2048 exceeds the MTU (mutation: the MTU cap dropped); no divisor gives 0 (mutation: no divisibility check)");
  check(rp::tiledPacketOrDefault(61440) == 1920 && rp::tiledPacketOrDefault(4096) == 0,
        "a small tiling packet (4096 -> 1024) keeps the default instead of doubling the packet rate (mutation: no 3/4 floor)");
  if (failures) std::printf("FAILED: %d failure(s)\n", failures);
  return failures ? 1 : 0;
}
