/**
 * @file framer_rearm_test.cc
 * @brief The Houdini TDD framer's arm retry reloads the beacon as a fresh
 *        load, with NO hardware: the real HoudiniFramer over a fake radio and
 *        device.
 *
 * A refused TDD_ARM runs the teardown ladder, whose TDD_CMD tx_clear rewinds
 * the device's replay fill to 0, then re-runs the load/schedule/strobe setup.
 * The reload must start at 0 on the host too: the device refuses an append at
 * a stale fill (past the RAM depth; from the software lane's F6b2, any append
 * at a stale fill), and the bring-up would then fail on the first refused arm
 * instead of retrying. The host plugin rewinds the host fill on tx_clear
 * itself; the framer also deactivates the replay stream before the reload, so
 * the fresh load does not rest on that rewind alone.
 *
 * The clear's verdict is the device's: TDD_CMD tx_clear throws by name when it
 * is refused or a bank does not take it, and the ladder reads nothing after it
 * (the bank counters never clear and no register counts clears).
 *
 * The fake radio models a host whose fill only deactivateXmit rewinds (the
 * case the deactivate covers) and the fake device the device side (its fill,
 * rewound by the TX clear), with F6b2's rule: a load lands at 0 or at the
 * device's fill, nowhere else.
 * Each check names the mutation that breaks it.
 *
 * Run from CC/Sounder (the config's relative path): ctest sets the working
 * directory.
 */
#include <SoapySDR/Device.hpp>
#include <SoapySDR/Errors.h>

#include <algorithm>
#include <cstdio>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "include/HoudiniFramer.h"
#include "include/Radio.h"
#include "include/config.h"

namespace {
int g_fail = 0;
void check(bool ok, const std::string& what) {
  std::printf("%s  %s\n", ok ? "PASS" : "FAIL", what.c_str());
  if (!ok) ++g_fail;
}

// The device side: the settings the framer writes, TDD_ARM refused the first
// `refuse_arms` times, and the replay fill a TX clear rewinds. TDD_CMD tx_clear
// clears, unless the device refuses it or judges a bank not cleared (it throws
// either way, as the device does). TX_BANK_STATUS answers in the device's
// render, which carries no count of clears, and counts its reads.
class FakeDevice : public SoapySDR::Device {
 public:
  int refuse_arms = 0;
  int arm_writes = 0;
  size_t level = 0;  // the device's replay fill
  bool clear_refused = false; // tx_clear throws, nothing written
  bool clear_stuck = false;   // tx_clear's pulse is written, a bank's clear_busy never falls: it throws
  mutable int status_reads = 0;
  std::vector<std::string>* log = nullptr;

  void writeSetting(const std::string& key, const std::string& value) override {
    log->push_back("set " + key + "=" + value);
    if (key == "TDD_ARM" && ++arm_writes <= refuse_arms)
      throw std::runtime_error("TDD_ARM: framer busy (state=running): abort first");
    if (key == "TDD_CMD" && value == "tx_clear") {
      if (clear_refused) throw std::runtime_error("TDD_CMD tx_clear: refused: the TDD framer is still engaged");
      log->push_back("TX_CLEAR_ALL");
      if (clear_stuck) throw std::runtime_error("TDD_CMD tx_clear: TX ch1 clear_busy did not fall within 1 ms");
      level = 0;
    }
  }
  std::string readSetting(const std::string& key) const override {
    if (key == "TX_BANK_STATUS") {
      ++status_reads;
      const std::string bank =
          "acked=0,late=0,under=0,seqerr=0,zerofill=0,fill=0,hwm=0,drops=0,last_seq=0,active=0,cap=64,gated=0,"
          "smiss=0,aclose=0,parked=0,late_seen=0,efault=0,played=0,clear_drops=0,clear_busy=0,tick=100:130,"
          "malformed=0,cancels=0,flushed=0,clock_aborts=0";
      return "ch0:" + bank + ";ch1:" + bank;
    }
    return key == "TDD_ARM" ? "accepted=1 epoch=5000" : "";
  }
};

// The host side: the replay stream's fill, which only a deactivate rewinds here
// (the case the framer's deactivate covers; the host plugin also rewinds it on
// tx_clear).
class FakeRadio : public Radio {
 public:
  FakeDevice dev;
  size_t fill = 0;
  std::vector<std::string> log;
  explicit FakeRadio(const RadioParams& p) : Radio(p) { dev.log = &log; }

  // A replay load: lands at the host's fill, which the device takes at 0 (a
  // fresh load) or at its own fill (an append), and refuses anywhere else.
  int xmit(const void* const*, int samples, int, long long&) override {
    const size_t at = fill;
    if (at != 0 && at != dev.level) {
      log.push_back("load at " + std::to_string(at) + " REFUSED");
      return SOAPY_SDR_STREAM_ERROR;
    }
    log.push_back("load at " + std::to_string(at));
    dev.level = at + static_cast<size_t>(samples);
    fill += static_cast<size_t>(samples);
    return samples;
  }
  void deactivateXmit() override {
    log.push_back("deactivate");
    fill = 0;
  }
  SoapySDR::Device* RawDev() const override { return const_cast<FakeDevice*>(&dev); }

  Type type() const override { return Type::kSoapyHoudini; }
  houdini::sync::Platform platform() const override { return houdini::sync::Platform::kHoudini; }
  void printSettings() const override {}
  bool hasHardwareTrigger() const override { return false; }
  bool hasAgc() const override { return false; }
  long long txTimeNs(long long, double, bool, long long) const override { return 0; }
  void setup(int, double, double) override {}
  int recv(void* const*, int, long long&) override { return 0; }
  int activateRecv(long long, size_t, int) override { return 0; }
  void deactivateRecv() override {}
  void activateXmit() override { log.push_back("activate"); }
  int getTriggers() const override { return 0; }
  void drain_buffers(std::vector<void*>, int) override {}
  void reset_DATA_clk_domain() override {}
};

// The fallback demo config: the hardware framer, no slots mode.
const char* kConf = "files/houdini-dualband-xw-steer.json";

int count(const std::vector<std::string>& log, const std::string& prefix) {
  int n = 0;
  for (const auto& l : log) n += l.rfind(prefix, 0) == 0 ? 1 : 0;
  return n;
}
std::string joined(const std::vector<std::string>& log) {
  std::string s;
  for (const auto& l : log)
    if (l.rfind("set TDD_SCHED", 0) != 0) s += (s.empty() ? "" : " | ") + l.substr(0, 40);
  return s;
}

// What one arm left behind.
struct Armed {
  std::string why;  // the arm's exception text, "" if it armed
  int arm_writes = 0;
  int status_reads = 0;
  std::vector<std::string> log;
};

// Arms the framer over one fake radio whose device refuses the first
// `refuse` TDD_ARM writes, after `setup` has shaped the rest of the device.
Armed armWith(int refuse, const std::function<void(FakeDevice&)>& setup = {}) {
  Config cfg(kConf, "/tmp", true, false, false);
  BeaconFramer::Radios radios(1);
  RadioParams p;
  p.label = "BS fake";
  radios[0].push_back(std::make_unique<FakeRadio>(p));
  auto* r = static_cast<FakeRadio*>(radios[0][0].get());
  r->dev.refuse_arms = refuse;
  if (setup) setup(r->dev);
  Armed a;
  {
    HoudiniFramer f(&cfg, radios);
    try {
      f.arm();
    } catch (const std::exception& e) {
      a.why = e.what();
    }
  }
  a.arm_writes = r->dev.arm_writes;
  a.status_reads = r->dev.status_reads;
  a.log = r->log;
  return a;
}
}  // namespace

int main() {
  try {
    {  // the instrument: the fake refuses what F6b2's device refuses
      FakeRadio r(RadioParams{});
      long long t = 0;
      const int first = r.xmit(nullptr, 2048, 0, t);
      r.dev.writeSetting("TDD_CMD", "tx_clear");
      const int stale = r.xmit(nullptr, 2048, 0, t);
      r.deactivateXmit();
      const int fresh = r.xmit(nullptr, 2048, 0, t);
      check(first == 2048 && stale == SOAPY_SDR_STREAM_ERROR && fresh == 2048,
            "the fake: a load after a TX clear is refused at the stale fill and lands after a deactivate (an "
            "instrument that accepts both proves nothing below)");
    }
    {
      const Armed a = armWith(0);
      check(a.why.empty() && a.arm_writes == 1 && count(a.log, "load at 0") == 1 && count(a.log, "deactivate") == 0,
            "arm accepted first time: one load at 0 and no deactivate, as the verified bring-up (" + joined(a.log) +
                (a.why.empty() ? "" : "; threw: " + a.why) + ")");
      check(a.why.empty() && a.status_reads == 0,
            "the ladder takes the device's verdict on its TX clear and reads no TX_BANK_STATUS (mutation: the "
            "clear-epoch second check restored, which finds no epoch in the device's render and throws at every "
            "bring-up: " + std::to_string(a.status_reads) + " reads" + (a.why.empty() ? "" : "; threw: " + a.why) +
                ")");
    }
    {
      const Armed a = armWith(0);
      const auto at = [&a](const std::string& l) {
        const auto it = std::find(a.log.begin(), a.log.end(), l);
        return it == a.log.end() ? -1 : static_cast<int>(it - a.log.begin());
      };
      const int ab = at("set TDD_CMD=abort"), cl = at("set TDD_CMD=tx_clear"), gr = at("set TDD_CMD=gate_release");
      check(ab >= 0 && ab < cl && cl < gr,
            "the ladder runs abort, then tx_clear, then gate_release (mutation: the gate released before the clear, "
            "which lets a strobe play the uncleared bank: " + joined(a.log) + ")");
    }
    {
      const Armed a = armWith(1);
      check(a.why.empty() && a.arm_writes == 2,
            "first TDD_ARM refused: the retry re-arms and the bring-up completes (mutation: no deactivate before "
            "the retry's reload, which the device refuses over a host fill only a deactivate rewinds: " +
                (a.why.empty() ? std::string("armed") : "threw '" + a.why + "'") + ")");
      check(count(a.log, "load at 0") == 2 && count(a.log, "load at 0 REFUSED") == 0 &&
                count(a.log, "deactivate") == 1,
            "the retry's reload is a fresh load at 0, after one deactivate (" + joined(a.log) + ")");
    }
    {
      const Armed a = armWith(0, [](FakeDevice& d) { d.clear_stuck = true; });
      check(a.why.find("TX clear failed") != std::string::npos && a.why.find("clear_busy did not fall") != std::string::npos &&
                a.arm_writes == 0,
            "a tx_clear the device judges not landed: the bring-up throws with the device's reason and writes no "
            "TDD_ARM (mutation: the clear's throw swallowed, which arms over uncleared banks: " +
                (a.why.empty() ? std::string("armed") : "threw '" + a.why.substr(0, 120) + "'") + ")");
      check(count(a.log, "set TDD_CMD=gate_release") == 1,
            "the failed clear still releases the gates before the throw (mutation: the throw before gate_release, "
            "which leaves a teardown's gates held for the next run: " + joined(a.log) + ")");
    }
    {
      const Armed a = armWith(0, [](FakeDevice& d) { d.clear_refused = true; });
      check(a.why.find("TX clear failed") != std::string::npos && a.why.find("still engaged") != std::string::npos &&
                a.arm_writes == 0 && count(a.log, "TX_CLEAR_ALL") == 0 &&
                count(a.log, "set TDD_CMD=gate_release") == 1,
            "a refused tx_clear: the gates are still released, then the bring-up throws with the device's reason "
            "(mutation: the refusal left to escape the ladder, which skips the gate release: " + joined(a.log) + ")");
    }
  } catch (const std::exception& e) {
    check(false, std::string("unexpected exception: ") + e.what());
  }
  std::printf("%s\n", g_fail == 0 ? "ALL PASS" : "FAILURES");
  return g_fail == 0 ? 0 : 1;
}
