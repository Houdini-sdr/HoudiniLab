/**
 * @file framer_rearm_test.cc
 * @brief The Houdini TDD framer's arm retry reloads the beacon as a fresh
 *        load, with NO hardware: the real HoudiniFramer over a fake radio and
 *        device.
 *
 * A refused TDD_ARM runs the teardown ladder, whose raw TX_CLEAR_ALL rewinds
 * the device's replay fill to 0, then re-runs the load/schedule/strobe setup.
 * The host's replay stream keeps its own fill (the next writeStream's address)
 * until a deactivateStream rewinds it, so a reload without one appends at the
 * pre-clear fill and the device refuses it (past the RAM depth; from the
 * software lane's F6b2, any append at a stale fill). The bring-up then fails
 * on the first refused arm instead of retrying.
 *
 * The fake radio models the host side (its fill, rewound by deactivateXmit)
 * and the fake device the device side (its fill, rewound by TX_CLEAR_ALL),
 * with F6b2's rule: a load lands at 0 or at the device's fill, nowhere else.
 * Each check names the mutation that breaks it.
 *
 * Run from CC/Sounder (the config's relative path): ctest sets the working
 * directory.
 */
#include <SoapySDR/Device.hpp>
#include <SoapySDR/Errors.h>

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
// `refuse_arms` times, and the replay fill a TX clear rewinds. The clear lands
// only at the offset this bitstream maps TX_CLEAR_ALL to (a write anywhere
// else is ignored, as the fabric ignores an unmapped offset), and counts in
// both TX banks' clear epoch, read through TX_BANK_STATUS.
class FakeDevice : public SoapySDR::Device {
 public:
  int refuse_arms = 0;
  int arm_writes = 0;
  size_t level = 0;  // the device's replay fill
  unsigned clear_at = HoudiniFramer::kTxClearAllOffset;
  int epoch = 1;              // the power-up envelope
  bool epoch_counts = true;   // false: a bitstream without CLEAR_EPOCH (reads 0)
  bool status_throws = false; // the TX_BANK_STATUS read fails
  int tear_reads = 0;         // TX_BANK_STATUS reads torn after each clear
  mutable int torn = 0;       // torn reads still to come
  std::vector<std::string>* log = nullptr;

  void writeSetting(const std::string& key, const std::string& value) override {
    log->push_back("set " + key + "=" + value);
    if (key == "TDD_ARM" && ++arm_writes <= refuse_arms)
      throw std::runtime_error("TDD_ARM: framer busy (state=running): abort first");
  }
  std::string readSetting(const std::string& key) const override {
    if (key == "TX_BANK_STATUS") {
      if (status_throws) throw std::runtime_error("TX_BANK_STATUS: SoapyRPCUnpacker type check FAIL");
      // A torn read: the envelope landed between the two epoch reads.
      const std::string e = torn > 0 ? std::to_string(epoch - 1) + ":" + std::to_string(epoch)
                                     : std::to_string(epoch) + ":" + std::to_string(epoch);
      if (torn > 0) --torn;
      return "ch0:acked=0,late=0,epoch=" + e + ";ch1:acked=0,late=0,epoch=" + e;
    }
    return key == "TDD_ARM" ? "accepted=1 epoch=5000" : "";
  }
  void writeRegister(const std::string& name, const unsigned addr, const unsigned value) override {
    if (name == "RFCORE" && addr == clear_at && value == 1) {
      log->push_back("TX_CLEAR_ALL");
      level = 0;
      if (epoch_counts) ++epoch;
      torn = tear_reads;
    }
  }
};

// The host side: the replay stream's fill, which only a deactivate rewinds.
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
      r.dev.writeRegister("RFCORE", HoudiniFramer::kTxClearAllOffset, 1);
      const int stale = r.xmit(nullptr, 2048, 0, t);
      r.deactivateXmit();
      const int fresh = r.xmit(nullptr, 2048, 0, t);
      check(first == 2048 && stale == SOAPY_SDR_STREAM_ERROR && fresh == 2048,
            "the fake: a load after a TX clear is refused at the stale fill and lands after a deactivate (an "
            "instrument that accepts both proves nothing below)");
    }
    // The other register map's TX_CLEAR_ALL offset (v3 moved it from 0x24 to
    // 0x084), where a bitstream of that map takes the clear.
    const unsigned other_map = HoudiniFramer::kTxClearAllOffset == 0x24 ? 0x084 : 0x24;
    {  // the instrument: the epoch counts a clear at the mapped offset only
      FakeRadio r(RadioParams{});
      const std::string at_start = r.dev.readSetting("TX_BANK_STATUS");
      r.dev.writeRegister("RFCORE", other_map, 1);
      const std::string after_other = r.dev.readSetting("TX_BANK_STATUS");
      r.dev.writeRegister("RFCORE", HoudiniFramer::kTxClearAllOffset, 1);
      const std::string after_mapped = r.dev.readSetting("TX_BANK_STATUS");
      check(after_other == at_start && after_mapped != at_start,
            "the fake: a write at an unmapped offset leaves the clear epoch, a clear at the mapped one moves it "
            "(an epoch that moves on any write lets the ladder's check pass a clear that never landed)");
    }
    {
      const Armed a = armWith(0);
      check(a.why.empty() && a.arm_writes == 1 && count(a.log, "load at 0") == 1 && count(a.log, "deactivate") == 0,
            "arm accepted first time: one load at 0 and no deactivate, as the verified bring-up (" + joined(a.log) +
                (a.why.empty() ? "" : "; threw: " + a.why) + ")");
    }
    {
      const Armed a = armWith(1);
      check(a.why.empty() && a.arm_writes == 2,
            "first TDD_ARM refused: the retry re-arms and the bring-up completes (mutation: no deactivate before "
            "the retry's reload, which the device refuses: " +
                (a.why.empty() ? std::string("armed") : "threw '" + a.why + "'") + ")");
      check(count(a.log, "load at 0") == 2 && count(a.log, "load at 0 REFUSED") == 0 &&
                count(a.log, "deactivate") == 1,
            "the retry's reload is a fresh load at 0, after one deactivate (" + joined(a.log) + ")");
    }
    {
      const Armed a = armWith(0, [&](FakeDevice& d) { d.clear_at = other_map; });
      check(a.why.find("TX_CLEAR_ALL") != std::string::npos && a.why.find("did not land") != std::string::npos &&
                a.arm_writes == 0,
            "a bitstream that takes TX_CLEAR_ALL at the other map's offset: the bring-up throws naming the clear "
            "and writes no TDD_ARM (mutation: the ladder's epoch check removed, which arms over uncleared banks: " +
                (a.why.empty() ? std::string("armed") : "threw '" + a.why.substr(0, 120) + "'") + ")");
      check(count(a.log, "set TDD_CMD=gate_release") == 1,
            "the failed clear still releases the gates before the throw (mutation: the throw before gate_release, "
            "which leaves a teardown's gates held for the next run: " + joined(a.log) + ")");
    }
    {
      const Armed a = armWith(0, [](FakeDevice& d) { d.tear_reads = 1; });
      check(a.why.empty() && a.arm_writes == 1,
            "a read torn by the clear is read again, not judged (mutation: one read only, which throws on a clear "
            "that landed: " + (a.why.empty() ? std::string("armed") : "threw '" + a.why.substr(0, 120) + "'") + ")");
    }
    {
      const Armed a = armWith(0, [](FakeDevice& d) { d.tear_reads = 5; });
      check(a.why.find("cannot check") != std::string::npos && a.arm_writes == 0,
            "every read torn: the bring-up throws that the clear cannot be checked (mutation: a torn read taken "
            "as an epoch, which passes on the reads' disagreement alone: " +
                (a.why.empty() ? std::string("armed") : "threw '" + a.why.substr(0, 120) + "'") + ")");
    }
    {
      const Armed a = armWith(0, [](FakeDevice& d) { d.status_throws = true; });
      check(a.why.find("cannot check") != std::string::npos && a.arm_writes == 0 &&
                count(a.log, "TX_CLEAR_ALL") == 1 && count(a.log, "set TDD_CMD=gate_release") == 1,
            "a TX_BANK_STATUS read that throws: the pulse and the gate release still run, then the bring-up throws "
            "that the clear cannot be checked (mutation: the read's throw left to escape, which skips both: " +
                joined(a.log) + ")");
    }
    {
      const Armed a = armWith(0, [](FakeDevice& d) {
        d.epoch = 0;
        d.epoch_counts = false;
      });
      check(a.why.empty() && a.arm_writes == 1,
            "a bitstream without CLEAR_EPOCH (every bank reads 0, before HS-220): the clear is trusted and the "
            "bring-up arms (mutation: a zero epoch judged as a miss, which refuses every bring-up on fpga 1.30: " +
                (a.why.empty() ? std::string("armed") : "threw '" + a.why.substr(0, 120) + "'") + ")");
    }
  } catch (const std::exception& e) {
    check(false, std::string("unexpected exception: ") + e.what());
  }
  std::printf("%s\n", g_fail == 0 ? "ALL PASS" : "FAILURES");
  return g_fail == 0 ? 0 : 1;
}
