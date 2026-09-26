// AP-88: every MLPD_* logging macro is ONE statement. Mutation that breaks it:
// a three-statement form (header; fprintf; fflush) leaves the `else` below
// without its `if`, so this file stops compiling, and an unbraced
// `if (x) MLPD_WARN(...)` would guard only the header and print the rest always.
#include <cstdio>

#include "logger.h"

int main(int argc, char**) {
  int taken = 0;
  const bool quiet = argc > 99;  // false, but not a constant the compiler can fold away
  if (quiet) MLPD_WARN("never printed\n");
  else taken = 1;
  if (quiet) MLPD_INFO("never printed\n");
  else ++taken;
  std::printf("%s  the MLPD_* macros are single statements (an unbraced if/else around them compiles)\n",
              taken == 2 ? "PASS" : "FAIL");
  return taken == 2 ? 0 : 1;
}
