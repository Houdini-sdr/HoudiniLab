// AP-80: where the UE places a read window for a due re-sync
// (houdini/resync_window.h). Each assertion names the mutation that breaks it.
#include <cmath>
#include <cstdio>
#include <random>

#include "houdini/resync_window.h"

static int failures = 0;
static void check(bool ok, const char* what) {
  std::printf("%s  %s\n", ok ? "PASS" : "FAIL", what);
  if (!ok) ++failures;
}

// The loop's own arithmetic (receiver.cc, the targeted re-sync): where the
// grid's next predicted beacon END lands in a window stamped at rx.
static long long loopOff(long long rx, long long pilot_ref, double period, long long beacon_end) {
  const double n_due = std::ceil(static_cast<double>(rx - pilot_ref - beacon_end) / period);
  return pilot_ref + std::llround(n_due * period) + beacon_end - rx;
}

int main() {
  using houdini::sync::placedWant;
  using houdini::sync::placedWindowStart;

  // R2 and R3 geometries: frame 122880 or 1228800 ticks, one-slot windows of
  // 4096 or 61440, the search slice's lead and tail as the loop derives them.
  struct Geo {
    double frame;
    long long request, lead, tail, beacon_end;
    const char* name;
  };
  const Geo geos[] = {{122880.0, 4096, 1280, 1096, 496, "R2"},
                      {1228800.0, 61440, 1280, 1096, 2208, "R3"}};

  std::mt19937_64 g(79);
  for (const Geo& geo : geos) {
    std::uniform_real_distribution<double> ppm(-5.0, 5.0);
    std::uniform_int_distribution<long long> big(1000000000LL, 20000000000LL);
    long long off_ok = 0, ahead = 0, within = 0, accepted = 0;
    const int kTrials = 200000;
    const long long want = placedWant(geo.lead, geo.tail, geo.request);
    for (int i = 0; i < kTrials; ++i) {
      // the tracked period is off-nominal by a few ppm and not an integer
      const double period = geo.frame * (1.0 + ppm(g) * 1e-6);
      const long long pilot_ref = big(g);
      const long long head = pilot_ref + std::uniform_int_distribution<long long>(-5000000, 50000000)(g);
      const long long w = placedWindowStart(head, pilot_ref, period, geo.beacon_end, want);
      const long long off = loopOff(w, pilot_ref, period, geo.beacon_end);
      off_ok += (off == want);
      ahead += (w >= head);
      within += (w - head < static_cast<long long>(std::ceil(period)));
      accepted += (off >= geo.lead && off + geo.tail <= geo.request);
    }
    char msg[256];
    // Fails under: ceil -> floor in placedWindowStart, or `- want` -> `+ want`.
    std::snprintf(msg, sizeof msg, "%s: the loop's own off is exactly want (%lld) for %lld of %d windows",
                  geo.name, want, off_ok, kTrials);
    check(off_ok == kTrials, msg);
    // Fails under: ceil -> floor (the start then lands up to a period behind the head).
    std::snprintf(msg, sizeof msg, "%s: the start is never behind the stream head (%lld of %d)", geo.name, ahead, kTrials);
    check(ahead == kTrials, msg);
    // Fails under: ceil(...) + 1 (a whole extra frame of reading on every re-sync).
    std::snprintf(msg, sizeof msg, "%s: the start is less than one frame ahead (%lld of %d)", geo.name, within, kTrials);
    check(within == kTrials, msg);
    // Fails under: placedWant returning lead - 1 or request - tail + 1 (out of the accepted band).
    std::snprintf(msg, sizeof msg, "%s: every placed window passes the loop's acceptance check (%lld of %d)",
                  geo.name, accepted, kTrials);
    check(accepted == kTrials, msg);
  }

  // placedWant is the middle of [lead, request - tail]: fails under dropping the /2.
  check(placedWant(1280, 1096, 4096) == 1280 + (4096 - 1280 - 1096) / 2, "placedWant is the middle of the accepted band");

  std::printf("%d failure(s)\n", failures);
  return failures ? 1 : 0;
}
