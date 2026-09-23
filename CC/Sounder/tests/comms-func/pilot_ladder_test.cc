// AP-79: the UE pilot ladder's resume rule (houdini/pilot_ladder.h). Each
// assertion names the mutation that breaks it.
#include <cmath>
#include <cstdio>
#include <random>
#include <set>

#include "houdini/pilot_ladder.h"

static int failures = 0;
static void check(bool ok, const char* what) {
  std::printf("%s  %s\n", ok ? "PASS" : "FAIL", what);
  if (!ok) ++failures;
}

// Simulate the scheduler: each call re-derives txTime on the tracked grid with
// sub-sample jitter, advances 1-3 frames, and queues up to `horizon` frames.
// Returns (frames covered, duplicates) over the run.
struct Cover {
  long long frames = 0, covered = 0, dups = 0;
};
template <class Resume>
static Cover simulate(Resume resume, double frame_d, double jitter, unsigned seed) {
  std::mt19937 g(seed);
  std::uniform_real_distribution<double> j(-jitter, jitter);
  std::uniform_int_distribution<int> step(1, 3);
  const int horizon = 96;
  long long cursor = 0;
  double grid = 1e6;  // the true tracked grid, in samples
  std::set<long long> slots;  // which true frame index each burst landed on
  Cover c;
  for (int call = 0; call < 20000; ++call) {
    grid += step(g) * frame_d;
    const long long tx_time = std::llround(grid + j(g));
    const long long end = tx_time + std::llround(horizon * frame_d);
    for (long long i = resume(cursor, tx_time, frame_d);; ++i) {
      const long long cur = tx_time + std::llround(static_cast<double>(i) * frame_d);
      if (cur > end) break;
      const long long idx = std::llround((static_cast<double>(cur) - 1e6) / frame_d);
      if (!slots.insert(idx).second) ++c.dups;
      cursor = cur;
    }
  }
  const long long first = *slots.begin(), last = *slots.rbegin();
  c.frames = last - first + 1;
  c.covered = static_cast<long long>(slots.size());
  return c;
}

int main() {
  const double fd = 122881.0588;  // a tracked period (not a whole number)
  auto fixed = [](long long cur, long long tx, double f) { return houdini::ladder::resumeIndex(cur, tx, f); };
  // The pre-fix rule, kept here as the mutation the coverage check must catch.
  auto old = [](long long cursor, long long tx, double f) -> long long {
    const long long frame = std::llround(f);
    return cursor + frame > tx ? static_cast<long long>(std::ceil(static_cast<double>(cursor + frame - tx) / f)) : 0;
  };
  const Cover a = simulate(fixed, fd, 0.6, 1);
  check(a.covered == a.frames && a.dups == 0,
        "resume: every frame covered once under +-0.6 sample grid jitter (mutation: the old ceil rule)");
  const Cover b = simulate(old, fd, 0.6, 1);
  std::printf("      old rule: %lld of %lld frames (%.1f%%), %lld dups\n", b.covered, b.frames, 100.0 * b.covered / b.frames,
              b.dups);
  check(b.covered < b.frames, "the old ceil rule skips frames under the same jitter (the defect this test pins)");
  const Cover c = simulate(fixed, fd, 0.2 * fd, 2);
  check(c.covered == c.frames && c.dups == 0,
        "resume: still exact under jitter of 0.2 frame, so calls differ by up to 0.4 (mutation: a quarter-frame threshold)");
  check(houdini::ladder::resumeIndex(0, 1000000, fd) == 0, "nothing queued ahead: start at index 0");
  check(houdini::ladder::resumeIndex(1000000 + 5 * 122881, 1000000, fd) == 6,
        "queued through index 5: resume at 6 (mutation: floor without +1 repeats the cursor's frame)");
  if (failures) std::printf("FAILED: %d failure(s)\n", failures);
  return failures ? 1 : 0;
}
