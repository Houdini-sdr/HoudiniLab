// include/rx_recorder_grid.h TimeGridTracker: every stamped read against the
// stream's sample count. Each assertion names the mutation that breaks it.
#include <cstdio>

#include "include/rx_recorder_grid.h"

static int failures = 0;
static void check(bool ok, const char* what) {
  std::printf("%s  %s\n", ok ? "PASS" : "FAIL", what);
  if (!ok) ++failures;
}

int main() {
  const double rate = 122.88e6;
  auto ns = [&](long long samples) { return Sounder::sampleToNs(samples, rate); };
  Sounder::TimeGridTracker g(rate);
  const long long t0 = 1000000000LL;
  auto a = g.onStamp(t0, 0);
  check(a.pad_samples == 0 && !a.backward && !a.resync, "the first read anchors the count (mutation: a pad from nothing)");
  auto b = g.onStamp(t0 + ns(1920), 1920);
  check(b.pad_samples == 0 && !b.backward && b.delta == 0, "a read on the count: no pad, delta 0 (mutation: an off-by-one count)");
  auto c = g.onStamp(t0 + ns(3840 + 61440), 3840);
  check(c.pad_samples == 61440 && c.delta == 61440 && !c.backward,
        "a read after a slot's gap: pad it, delta the gap (mutation: the gap not measured)");
  auto d = g.onStamp(t0 + ns(3840 + 61440 - 1920), 3840 + 61440);
  check(d.backward && d.delta == -1920 && d.pad_samples == 0,
        "a read stamped EARLIER than the count is out of order, delta negative, never padded (mutation: backward treated as on the count)");
  auto e = g.onStamp(t0 + ns(3840 + 61440) + 20000000000LL, 3840 + 61440);
  check(e.resync && e.pad_samples == 0, "a jump over the 10 s cap re-anchors and says so (mutation: padding 20 s of zeros)");
  if (failures) std::printf("FAILED: %d failure(s)\n", failures);
  else std::printf("ALL PASS\n");
  return failures ? 1 : 0;
}
