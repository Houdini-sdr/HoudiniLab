// include/node_version.h lockstepMismatch: the driver's release lockstep on one
// node. Each assertion names the mutation that breaks it. NO hardware.
#include <cstdio>
#include <string>

#include "node_version.h"

static int failures = 0;
static void check(bool ok, const char* what) {
  std::printf("%s  %s\n", ok ? "PASS" : "FAIL", what);
  if (!ok) ++failures;
}

int main() {
  using Sounder::lockstepMismatch;
  const SoapySDR::Kwargs same{{"host_version", "0.4.0"}, {"device_version", "0.4.0"},
                              {"host_build", "aaaa"}, {"device_build", "bbbb"}};
  check(lockstepMismatch(same).empty(),
        "one release on both, whatever the builds, is in lockstep (mutation: the builds compared instead)");
  SoapySDR::Kwargs skew = same;
  skew["host_version"] = "0.3.1";
  const std::string m = lockstepMismatch(skew);
  check(m.find("0.3.1") != std::string::npos && m.find("0.4.0") != std::string::npos,
        "a host release that is not the device's is named with both releases (mutation: the check removed)");
  SoapySDR::Kwargs absent = same;
  absent.erase("device_version");
  check(lockstepMismatch(absent).empty(),
        "a node that reports no device_version has nothing to judge (mutation: an absent key read as a mismatch)");
  if (failures) std::printf("FAILED: %d failure(s)\n", failures);
  return failures ? 1 : 0;
}
