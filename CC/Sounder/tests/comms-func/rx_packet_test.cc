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
        "the device's derivation reproduces the default 2032 at MTU 8192 (mutation: overhead or beat rounding wrong)");
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
  // SH-488: under the TDD framer the device refuses any packet that does not
  // divide the slot or spans under 128 ticks, so the BS takes the divisor.
  constexpr double k122 = 122.88e6;
  check(rp::framerPacket(4096, k122) == 1024 && rp::framerPacket(61440, k122) == 1920,
        "under the framer a 4096 slot takes 1024, the demo slot 1920 (mutation: the 3/4 floor applied under the framer)");
  check(rp::samplesForSlot(2096) == 16 && rp::framerPacket(2096, k122) == 0 && rp::framerPacket(61441, k122) == 0,
        "a slot whose only divisor is 16 samples, or none, has no framer packet (mutation: no 128-tick floor)");
  // The floor is 128 TICKS of the 122.88 MHz clock: a 112-sample packet
  // (14672 = 112 x 131) is 112 ticks at 122.88 MSPS but 224 at 61.44, and the
  // demo's 1920 is only 120 ticks at 1966.08 MSPS.
  check(rp::samplesForSlot(14672) == 112 && rp::framerPacket(14672, k122) == 0 &&
            rp::framerPacket(14672, 61.44e6) == 112,
        "the floor scales with the rate: 112 samples pass at 61.44 MSPS only (mutation: the floor in samples, not ticks)");
  check(rp::framerPacket(61440, 1966.08e6) == 0 && rp::framerPacket(4096 * 16, 1966.08e6) == 0,
        "at 1966.08 MSPS a packet the MTU allows (at most 2032 samples) spans under 128 ticks (mutation: the rate ignored)");
  check(rp::bsPacket(4096, true, k122) == 1024 && rp::bsPacket(4096, false, k122) == 0 &&
            rp::bsPacket(61440, false, k122) == 1920,
        "the BS takes the framer packet only while its framer is armed (mutation: the framer flag ignored)");
  if (failures) std::printf("FAILED: %d failure(s)\n", failures);
  return failures ? 1 : 0;
}
