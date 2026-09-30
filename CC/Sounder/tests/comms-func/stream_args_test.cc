// HOUDINI_TX_STREAM_ARGS parsing (houdini/stream_args.h). Each assertion names
// the mutation that breaks it.
#include <cstdio>
#include <string>

#include "houdini/stream_args.h"

static int failures = 0;
static void check(bool ok, const char* what) {
  std::printf("%s  %s\n", ok ? "PASS" : "FAIL", what);
  if (!ok) ++failures;
}

int main() {
  using houdini::extraStreamArgs;
  std::string err;
  // Fails under: splitting on the first '=' of the whole spec instead of per item.
  const auto a = extraStreamArgs("tx_target_frac=0.75,cpu_affinity=18", &err);
  check(err.empty() && a.size() == 2 && a.at("tx_target_frac") == "0.75" && a.at("cpu_affinity") == "18",
        "two pairs parse, each to its own key");
  // Fails under: treating a null or empty spec as an error.
  check(extraStreamArgs(nullptr, &err).empty() && err.empty() && extraStreamArgs("", &err).empty() && err.empty(),
        "no spec is no pairs and no error");
  // Fails under: dropping the owned-key refusal (tdd=0 would silently turn off the TDD anchor).
  check(extraStreamArgs("tx_target_frac=0.75,tdd=0", &err).empty() && err.find("tdd") != std::string::npos,
        "a key the sounder sets is refused, and nothing is applied");
  // Fails under: accepting an item without '=' (or with an empty key or value).
  check(extraStreamArgs("tx_target_frac", &err).empty() && !err.empty() && extraStreamArgs("=1", &err).empty() &&
            !err.empty() && extraStreamArgs("k=", &err).empty() && !err.empty(),
        "an item that is not key=value is refused");
  // Fails under: treating an empty item (a trailing comma) as malformed.
  check(extraStreamArgs("tx_target_frac=0.75,", &err).size() == 1 && err.empty(), "a trailing comma is harmless");
  std::printf("%d failure(s)\n", failures);
  return failures ? 1 : 0;
}
