// Known answers for houdini::cnsStride. Each check names the mutation that breaks it.
#include <cstdio>
#include <set>
#include "houdini/cns_sample.h"

static int fails = 0;
static void check(bool ok, const char* what) { std::printf("%s %s\n", ok ? "PASS" : "FAIL", what); fails += !ok; }

// Distinct tones one frame samples: pairs (si, j), si < syms, j < tones, kept when (si*tones + j) % stride == 0.
static size_t distinct(size_t syms, size_t tones, size_t stride) {
  std::set<size_t> t;
  for (size_t si = 0; si < syms; ++si)
    for (size_t j = 0; j < tones; ++j)
      if ((si * tones + j) % stride == 0) t.insert(j);
  return t.size();
}

int main() {
  const size_t s270 = houdini::cnsStride(12 * 2970, 600, 2970);
  check(s270 == 61, "270 RB: 60 shares a factor with 2970, so the stride moves to 61 (mutation: drop the gcd loop)");
  check(distinct(12, 2970, s270) > 500, "270 RB: one frame spans more than 500 distinct tones (stride 60 gives 99)");
  check((12 * 2970 + s270 - 1) / s270 <= 600, "270 RB: still at most 600 points a frame (mutation: step the stride down)");
  const size_t s133 = houdini::cnsStride(12 * 1463, 600, 1463);
  check(s133 == 30, "133 RB: 30 is already coprime with 1463, so the demo's sampling is unchanged (mutation: always add one)");
  check(houdini::cnsStride(10, 600, 1463) == 1, "fewer pairs than points: stride 1, every pair kept (mutation: a floor above 1)");
  std::printf("%d failure(s)\n", fails);
  return fails ? 1 : 0;
}
