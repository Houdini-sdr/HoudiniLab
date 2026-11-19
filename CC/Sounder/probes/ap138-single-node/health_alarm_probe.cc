// The link monitor (houdini/link_health.h) against a KNOWN alarm on one radio:
// a positive control. Every clean session shows only that the monitor stays
// quiet; this shows it speaks. On the probe's own device handle it sets up one
// RX stream (not activated: the self-test needs a set-up stream only, and the
// host faults an active one it silences), takes the monitor's baseline, requires a clean
// check, then writes the driver's RX_FAILSTOP_TEST for that channel (the
// device late-aborts one timed start on it, by design) and requires the next
// check to carry exactly "rx<ch>.aborts +1" and no other counter.
//
//   health_alarm_probe <node address> [<rx channel, default 0>] [<port, default 55132>]
//
// with the release's host prefix in SOAPY_SDR_ROOT and SOAPY_SDR_PLUGIN_PATH
// empty. Build on the rig host, from CC/Sounder, with the venv active (one line):
//   g++ -std=c++17 -O1 -Iinclude -I$VIRTUAL_ENV/include probes/ap138-single-node/health_alarm_probe.cc
//       -L$VIRTUAL_ENV/lib -lSoapySDR -Wl,-rpath,$VIRTUAL_ENV/lib -o <out>
// Exit 0 only when both checks read as required. It opens the radio (a make()
// resets it): never against a radio someone else is using.
#include <SoapySDR/Device.hpp>
#include <SoapySDR/Formats.h>

#include <chrono>
#include <cstdio>
#include <exception>
#include <string>
#include <thread>
#include <vector>

#include "houdini/link_health.h"

int main(int argc, char** argv) {
  if (argc < 2) {
    std::fprintf(stderr, "usage: health_alarm_probe <node address> [<rx channel>] [<port>]\n");
    return 2;
  }
  const std::string ip = argv[1];
  const size_t ch = argc > 2 ? static_cast<size_t>(std::stoul(argv[2])) : 0;
  const std::string port = argc > 3 ? argv[3] : "55132";
  SoapySDR::Kwargs args = {{"driver", "houdinisdr"},
                           {"remote", "tcp://" + ip + ":" + port},
                           {"remote:driver", "houdinisdr-device"},
                           {"remote:type", "houdinisdr"},
                           {"timeout", "3000000"}};
  SoapySDR::Device* dev = nullptr;
  SoapySDR::Stream* rx = nullptr;
  int rc = 1;
  try {
    dev = SoapySDR::Device::make(args);
    rx = dev->setupStream(SOAPY_SDR_RX, SOAPY_SDR_CS16, {ch});
    for (const char* k : {"RX_BANK_STATUS", "TX_BANK_STATUS", "EGRESS_STATUS"})
      std::printf("before  %s = %s\n", k, dev->readSetting(k).c_str());
    houdini::health::LinkHealth h([dev](const std::string& k) { return dev->readSetting(k); }, "probe " + ip);
    std::this_thread::sleep_for(std::chrono::seconds(1));
    const auto quiet = h.check();
    std::printf("check 1 (nothing done): %s\n", quiet.line().c_str());
    // Counter rises only: a preflight item is the bring-up's, not this probe's subject.
    const bool quiet_ok = quiet.increases.empty() && quiet.pl_reload.empty();
    dev->writeSetting("RX_FAILSTOP_TEST", std::to_string(ch));
    std::printf("RX_FAILSTOP_TEST = %s\n", dev->readSetting("RX_FAILSTOP_TEST").c_str());
    std::printf("after   RX_BANK_STATUS = %s\n", dev->readSetting("RX_BANK_STATUS").c_str());
    const auto loud = h.check();
    std::printf("check 2 (one late abort made): %s\n", loud.line().c_str());
    const std::string want = "rx" + std::to_string(ch) + ".aborts";
    const bool loud_ok = loud.increases.size() == 1 && loud.increases.count(want) == 1 &&
                         loud.increases.at(want) == 1 && loud.pl_reload.empty();
    const auto after = h.check();
    std::printf("check 3 (nothing more done): %s\n", after.line().c_str());
    const bool after_ok = after.increases.empty() && after.pl_reload.empty();
    std::printf("%s  check 1 reports no counter (a monitor that alarms on standing counts fails here)\n",
                quiet_ok ? "PASS" : "FAIL");
    std::printf("%s  check 2 reports exactly %s +1 (a monitor that cannot see the device's counters fails here)\n",
                loud_ok ? "PASS" : "FAIL", want.c_str());
    std::printf("%s  check 3 reports no counter (an alarm that repeats on a standing count fails here)\n",
                after_ok ? "PASS" : "FAIL");
    rc = quiet_ok && loud_ok && after_ok ? 0 : 1;
  } catch (const std::exception& e) {
    std::printf("FAIL  the probe threw: %s\n", e.what());
  }
  if (dev != nullptr) {
    try {
      if (rx != nullptr) dev->closeStream(rx);
    } catch (const std::exception& e) {
      std::printf("close: %s\n", e.what());
    }
    SoapySDR::Device::unmake(dev);
  }
  return rc;
}
