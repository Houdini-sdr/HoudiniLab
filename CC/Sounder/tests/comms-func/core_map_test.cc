// HOUDINI_CORE_MAP parsing (houdini/core_map.h). Each assertion names the
// mutation that breaks it.
#include <cstdio>
#include <string>

#include "houdini/core_map.h"

static int failures = 0;
static void check(bool ok, const char* what) {
  std::printf("%s  %s\n", ok ? "PASS" : "FAIL", what);
  if (!ok) ++failures;
}

int main() {
  using houdini::parseCoreMap;
  std::string err;
  // Fails under: a role written to the wrong slot, or the value parsed from the wrong side of '='.
  const auto a = parseCoreMap("main=15,recorder=18,bsrx=5,ue=6", &err);
  check(err.empty() && a.main == 15 && a.recorder == 18 && a.bsrx == 5 && a.ue == 6, "every role lands on its core");
  // Fails under: defaulting an unnamed role to 0 instead of -1 (it would pin to core 0).
  const auto b = parseCoreMap("main=15", &err);
  check(err.empty() && b.main == 15 && b.recorder == -1 && b.bsrx == -1 && b.ue == -1,
        "a role not named keeps the default layout");
  // Fails under: treating a null or empty spec as an error.
  check(parseCoreMap(nullptr, &err).main == -1 && err.empty() && parseCoreMap("", &err).main == -1 && err.empty(),
        "no spec is the default map, no error");
  // Fails under: dropping the unknown-role refusal (a typo would silently pin nothing).
  check(parseCoreMap("mian=15", &err).main == -1 && err.find("not a role") != std::string::npos,
        "an unknown role is refused");
  // Fails under: accepting a value that is not a core number.
  check(parseCoreMap("main=x", &err).main == -1 && !err.empty() && parseCoreMap("main=", &err).main == -1 &&
            !err.empty() && parseCoreMap("main=-1", &err).main == -1 && !err.empty(),
        "a value that is not a core number is refused");
  // Fails under: letting a repeated role silently overwrite the first.
  check(parseCoreMap("main=15,main=16", &err).main == -1 && err.find("twice") != std::string::npos,
        "a role named twice is refused");
  std::printf("%d failure(s)\n", failures);
  return failures ? 1 : 0;
}
