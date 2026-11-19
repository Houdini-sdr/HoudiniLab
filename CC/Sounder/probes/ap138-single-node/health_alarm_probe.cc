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
//   health_alarm_probe <node address> tx [<tx channel, default 0>]
//
// The tx form is the same control on a TX bank, after SoapyHoudiniSDR's own
// test_tx_error_malformed (host/tests/hil/test_tx_stream.py): on an active
// live TX stream it sends four raw UDP frames to the radio's data port (a
// timed seq 0 about 50 ms ahead, seq 1 and 2, then seq 3 cut to half the units
// its header claims) and requires the next check to read tx<ch>.malformed +1
// and no seqerr. The frame header here is the driver's 16-byte TX wire header
// of that test's date: when the wire changes, this form goes with it.
//
// with the release's host prefix in SOAPY_SDR_ROOT and SOAPY_SDR_PLUGIN_PATH
// empty. Build on the rig host, from CC/Sounder, with the venv active (one line):
//   g++ -std=c++17 -O1 -Iinclude -I$VIRTUAL_ENV/include probes/ap138-single-node/health_alarm_probe.cc
//       -L$VIRTUAL_ENV/lib -lSoapySDR -Wl,-rpath,$VIRTUAL_ENV/lib -o <out>
// Exit 0 only when both checks read as required. It opens the radio (a make()
// resets it): never against a radio someone else is using.
#include <SoapySDR/Device.hpp>
#include <SoapySDR/Formats.h>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cmath>
#include <cstdint>
#include <cstring>

#include <chrono>
#include <cstdio>
#include <exception>
#include <string>
#include <thread>
#include <vector>

#include "houdini/link_health.h"

namespace {
// The driver's TX wire header (tx::PackTxHeader): hdr0 = countdown[15:0] |
// END_BURST bit 28 | HAS_TIME bit 31 | seq << 32; hdr1 = the start tick.
std::vector<uint8_t> txFrame(unsigned countdown, unsigned seq, bool has_time, uint64_t tick, size_t payload_bytes) {
  uint64_t h[2] = {(countdown & 0xFFFFULL) | (has_time ? 1ULL << 31 : 0) | ((seq & 0xFFFFULL) << 32),
                   has_time ? tick : 0};
  std::vector<uint8_t> f(16 + payload_bytes, 0);
  std::memcpy(f.data(), h, 16);  // little-endian host, as the radio's wire
  return f;
}

int txControl(const std::string& ip, size_t ch) {
  SoapySDR::Kwargs args = {{"driver", "houdinisdr"},
                           {"remote", "tcp://" + ip + ":55132"},
                           {"remote:driver", "houdinisdr-device"},
                           {"remote:type", "houdinisdr"},
                           {"timeout", "3000000"}};
  SoapySDR::Device* dev = nullptr;
  SoapySDR::Stream* tx = nullptr;
  int rc = 1, sock = -1;
  try {
    dev = SoapySDR::Device::make(args);
    const auto hw = dev->getHardwareInfo();
    const std::string data_ip = hw.at("data_iface");
    const double tick_rate = std::stod(hw.at("tick_rate_hz"));
    dev->setSampleRate(SOAPY_SDR_TX, ch, 245.76e6);
    tx = dev->setupStream(SOAPY_SDR_TX, SOAPY_SDR_CS16, {ch}, {{"tx_mode", "stream"}});
    if (dev->activateStream(tx) != 0) throw std::runtime_error("activateStream(TX) refused");
    std::printf("before  TX_BANK_STATUS = %s\n", dev->readSetting("TX_BANK_STATUS").c_str());
    houdini::health::LinkHealth h([dev](const std::string& k) { return dev->readSetting(k); }, "probe " + ip);
    const auto quiet = h.check();
    std::printf("check 1 (nothing sent): %s\n", quiet.line().c_str());
    sockaddr_in to{};
    to.sin_family = AF_INET;
    to.sin_port = htons(static_cast<uint16_t>(10101 + ch));
    if (inet_pton(AF_INET, data_ip.c_str(), &to.sin_addr) != 1) throw std::runtime_error("data_iface " + data_ip);
    sock = socket(AF_INET, SOCK_DGRAM, 0);
    const long long ms = 1000000;
    const long long start_ns = ((dev->getHardwareTime() + 50 * ms + ms - 1) / ms) * ms;  // a whole ms, 50 ms ahead
    const uint64_t tick = static_cast<uint64_t>(std::llround(static_cast<double>(start_ns) * tick_rate / 1e9)) & ~1ULL;
    const unsigned words = 8;
    const size_t bytes = words * 8;
    auto send = [&](const std::vector<uint8_t>& f) {
      if (sendto(sock, f.data(), f.size(), 0, reinterpret_cast<const sockaddr*>(&to), sizeof to) < 0)
        throw std::runtime_error("sendto failed");
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    };
    send(txFrame(words, 0, true, tick, bytes));
    send(txFrame(words, 1, false, 0, bytes));
    send(txFrame(words, 2, false, 0, bytes));
    send(txFrame(words, 3, false, 0, bytes / 2));  // claims 8 units, carries 4
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    std::printf("after   TX_BANK_STATUS = %s\n", dev->readSetting("TX_BANK_STATUS").c_str());
    const auto loud = h.check();
    std::printf("check 2 (one malformed frame sent to %s:%d): %s\n", data_ip.c_str(), 10101 + static_cast<int>(ch),
                loud.line().c_str());
    const std::string pre = "tx" + std::to_string(ch) + ".";
    const bool quiet_ok = quiet.increases.empty() && quiet.pl_reload.empty();
    const bool loud_ok = loud.increases.count(pre + "malformed") == 1 && loud.increases.at(pre + "malformed") == 1 &&
                         loud.increases.count(pre + "seqerr") == 0 && loud.pl_reload.empty();
    std::printf("%s  check 1 reports no counter\n", quiet_ok ? "PASS" : "FAIL");
    std::printf("%s  check 2 reports %smalformed +1 and no seqerr (a monitor that skips the TX banks fails here)\n",
                loud_ok ? "PASS" : "FAIL", pre.c_str());
    rc = quiet_ok && loud_ok ? 0 : 1;
  } catch (const std::exception& e) {
    std::printf("FAIL  the probe threw: %s\n", e.what());
  }
  if (sock >= 0) close(sock);
  if (dev != nullptr) {
    try {
      if (tx != nullptr) {
        dev->deactivateStream(tx);
        dev->closeStream(tx);
      }
    } catch (const std::exception& e) {
      std::printf("close: %s\n", e.what());
    }
    SoapySDR::Device::unmake(dev);
  }
  return rc;
}
}  // namespace

int main(int argc, char** argv) {
  if (argc < 2) {
    std::fprintf(stderr, "usage: health_alarm_probe <node address> [<rx channel>] [<port>] | <node address> tx [<tx channel>]\n");
    return 2;
  }
  const std::string ip = argv[1];
  if (argc > 2 && std::string(argv[2]) == "tx") return txControl(ip, argc > 3 ? std::stoul(argv[3]) : 0);
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
