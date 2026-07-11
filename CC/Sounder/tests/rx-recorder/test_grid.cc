// Unit test for TimeGridTracker: t0 anchoring, forward-gap pads, jitter
// tolerance, backward jumps, and no-drift over accumulated rounding.
#include <cstdio>

#include "include/rx_recorder_grid.h"

#define CHECK(cond)                                                        \
  do {                                                                     \
    if (!(cond)) {                                                         \
      std::fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
      return 1;                                                            \
    }                                                                      \
  } while (0)

int main() {
  using Sounder::GridCheck;
  using Sounder::TimeGridTracker;
  const double rate = 30.72e6;  // ladder bottom rung
  const long long t0 = 5'000'000'000LL;
  auto ns_of = [&](int64_t sample) {
    return t0 + static_cast<long long>(std::llround(sample * 1e9 / rate));
  };

  {  // Clean stream: stamps exactly on the grid -> no pads ever.
    TimeGridTracker g(rate);
    GridCheck c = g.onStamp(ns_of(0), 0);
    CHECK(c.pad_samples == 0 && !c.backward);
    CHECK(g.has_t0() && g.t0() == t0);
    int64_t pos = 0;
    for (int i = 1; i < 10000; i++) {
      pos += 16384;
      c = g.onStamp(ns_of(pos), pos);  // rounding accumulates ONLY via ns_of
      CHECK(c.pad_samples == 0 && !c.backward);
    }
  }
  {  // Forward gap: 5000 samples missing before the second read.
    TimeGridTracker g(rate);
    g.onStamp(ns_of(0), 0);
    GridCheck c = g.onStamp(ns_of(16384 + 5000), 16384);
    CHECK(c.pad_samples == 5000);
    // After padding, the grid is repaired: next stamp is clean.
    c = g.onStamp(ns_of(16384 + 5000 + 16384), 16384 + 5000 + 16384);
    CHECK(c.pad_samples == 0 && !c.backward);
  }
  {  // +/-1 sample timestamp jitter is tolerated, not padded.
    TimeGridTracker g(rate);
    g.onStamp(ns_of(0), 0);
    GridCheck c = g.onStamp(ns_of(8192 + 1), 8192);
    CHECK(c.pad_samples == 0 && !c.backward);
    c = g.onStamp(ns_of(16384) - 30, 16384);  // -30ns ~ -0.9 samples
    CHECK(c.pad_samples == 0 && !c.backward);
  }
  {  // Backward jump flagged, never padded.
    TimeGridTracker g(rate);
    g.onStamp(ns_of(0), 0);
    GridCheck c = g.onStamp(ns_of(8192 - 100), 8192);
    CHECK(c.backward == true && c.pad_samples == 0);
  }
  {  // Late anchor: first stamp arrives at position > 0.
    TimeGridTracker g(rate);
    GridCheck c = g.onStamp(ns_of(1000), 1000);
    CHECK(c.pad_samples == 0 && !c.backward);
    CHECK(g.t0() == t0);  // projected back to sample 0
    c = g.onStamp(ns_of(2000), 2000);
    CHECK(c.pad_samples == 0 && !c.backward);
  }
  {  // High-rate quantization: at 1.966 GSPS (0.51 ns/sample) +-2-sample
     // deltas are timeNs rounding, not gaps; a real 5000-sample gap still pads.
    TimeGridTracker g(1.96608e9);
    auto ns_hi = [&](int64_t sample) {
      return t0 +
             static_cast<long long>(std::llround(sample * 1e9 / 1.96608e9));
    };
    g.onStamp(ns_hi(0), 0);
    GridCheck c = g.onStamp(ns_hi(65536) + 1, 65536);  // +1 ns ~ +2 samples
    CHECK(c.pad_samples == 0 && !c.backward);
    c = g.onStamp(ns_hi(131072) - 1, 131072);  // -1 ns ~ -2 samples
    CHECK(c.pad_samples == 0 && !c.backward);
    c = g.onStamp(ns_hi(196608 + 5000), 196608);
    CHECK(c.pad_samples > 4990 && c.pad_samples < 5010);
  }
  {  // Time-base jumps beyond kMaxGapSeconds re-anchor instead of padding.
    TimeGridTracker g(rate);
    g.onStamp(ns_of(0), 0);
    const long long hour = 3'600'000'000'000LL;
    GridCheck c = g.onStamp(ns_of(16384) + hour, 16384);  // +1 h forward
    CHECK(c.resync && c.pad_samples == 0 && !c.backward);
    // Grid continues cleanly from the new anchor...
    c = g.onStamp(ns_of(32768) + hour, 32768);
    CHECK(!c.resync && c.pad_samples == 0 && !c.backward);
    // ...real gaps under the cap still pad after the re-anchor...
    c = g.onStamp(ns_of(49152 + 5000) + hour, 49152);
    CHECK(c.pad_samples == 5000);
    // ...and an absurd backward jump also re-anchors (it would otherwise
    // pin every later stamp as a backward event).
    c = g.onStamp(ns_of(65536) - hour, 65536);
    CHECK(c.resync && c.pad_samples == 0 && !c.backward);
  }
  std::printf("PASS: TimeGridTracker (anchor, gap, jitter, backward, drift)\n");
  return 0;
}
