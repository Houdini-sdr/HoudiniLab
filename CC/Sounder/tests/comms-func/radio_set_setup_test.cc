/**
 * @file radio_set_setup_test.cc
 * @brief A radio whose channel setup throws fails its radio set instead of
 *        aborting the process, with NO hardware: the real BaseRadioSet and
 *        ClientRadioSet over a fake radio.
 *
 * Radio::setup runs on the sets' init threads, and RadioHoudini::setup reads
 * the device back (getFrequency, which the host plugin throws on when the PLL
 * reads 0 or the zone is unreadable). An exception escaping one of those
 * threads is std::terminate: the run aborts with no teardown and no reason.
 * Caught, the radio is listed as not opened with its reason and the set
 * reports getRadioNotFound(), the path a failed open already takes.
 *
 * This file replaces Radio.cc in the build, so Radio::create returns the fake
 * below, whose setup() throws. A missing catch aborts this binary before it
 * prints ALL PASS, which ctest reports as a failure. Each check names the
 * mutation that breaks it.
 *
 * Run from CC/Sounder (the config's relative path): ctest sets the working
 * directory.
 */
#include <atomic>
#include <cstdio>
#include <memory>
#include <stdexcept>
#include <string>

#include "include/BaseRadioSet.h"
#include "include/ClientRadioSet.h"
#include "include/Radio.h"
#include "include/config.h"

namespace {
int g_fail = 0;
void check(bool ok, const std::string& what) {
  std::printf("%s  %s\n", ok ? "PASS" : "FAIL", what.c_str());
  if (!ok) ++g_fail;
}

std::atomic<int> g_creates{0}, g_setups{0};

// A Houdini radio with nothing behind it whose channel setup throws, as
// RadioHoudini::setup does when the device refuses its readback.
class FakeRadio : public Radio {
 public:
  explicit FakeRadio(const RadioParams& p) : Radio(p) {}
  Type type() const override { return Type::kSoapyHoudini; }
  houdini::sync::Platform platform() const override { return houdini::sync::Platform::kHoudini; }
  void printSettings() const override {}
  bool hasHardwareTrigger() const override { return false; }
  bool hasAgc() const override { return false; }
  long long txTimeNs(long long, double, bool, long long) const override { return 0; }
  void setup(int ch, double, double) override {
    ++g_setups;
    throw std::runtime_error(params_.label + ": getFrequency(RX, " + std::to_string(ch) + "): the PLL reads 0");
  }
  int recv(void* const*, int, long long&) override { return 0; }
  int activateRecv(long long, size_t, int) override { return 0; }
  void deactivateRecv() override {}
  int xmit(const void* const*, int, int, long long&) override { return 0; }
  void activateXmit() override {}
  void deactivateXmit() override {}
  int getTriggers() const override { return 0; }
  void drain_buffers(std::vector<void*>, int) override {}
  void reset_DATA_clk_domain() override {}
};
}  // namespace

// Radio.cc's definitions, replaced in this build.
const char* Radio::name(Type) { return "fake"; }
std::unique_ptr<Radio> Radio::create(Type, const RadioParams& params) {
  ++g_creates;
  return std::make_unique<FakeRadio>(params);
}
void Radio::activateRecvOrThrow() {
  if (activateRecv() != 0) throw std::runtime_error(params_.label + ": activateStream(RX) refused");
}
Radio::Type radioTypeFor(const Config&) { return Radio::Type::kSoapyHoudini; }

namespace {
const char* kConf = "files/houdini-dualband-xw-steer-slots.json";

void baseSet() {
  Config cfg(kConf, "/tmp", true, false, false);
  const size_t requested = cfg.n_bs_sdrs().at(0);
  g_creates = 0;
  g_setups = 0;
  BaseRadioSet set(&cfg, false);
  check(g_creates == static_cast<int>(requested) && g_setups == 1,
        "BS: every radio opened and the first channel setup threw (" + std::to_string(g_creates) + " opened of " +
            std::to_string(requested) + ", " + std::to_string(g_setups) +
            " setups; the throw reached the configure thread)");
  check(set.getRadioNotFound(),
        "BS: the set fails on the throw (catch it and drop the reason, and the set arms a radio that never set up)");
}

void clientSet() {
  Config cfg(kConf, "/tmp", false, true, false);
  const size_t requested = cfg.num_cl_sdrs();
  g_creates = 0;
  g_setups = 0;
  ClientRadioSet set(&cfg);
  check(g_creates == static_cast<int>(requested) && g_setups == 1,
        "UE: every radio opened and the first channel setup threw (" + std::to_string(g_creates) + " opened of " +
            std::to_string(requested) + ", " + std::to_string(g_setups) + " setups)");
  check(set.getRadioNotFound(),
        "UE: the set fails on the throw (catch it and keep the radio, and the set activates it)");
}
}  // namespace

int main() {
  // No catch here: an exception escaping a set's init thread is
  // std::terminate whatever main does, and this binary then dies before
  // ALL PASS (remove the catch in BaseRadioSet::configure or
  // ClientRadioSet::init to see it).
  baseSet();
  clientSet();
  std::printf("%s\n", g_fail == 0 ? "ALL PASS" : "FAILURES");
  return g_fail == 0 ? 0 : 1;
}
