/**
 * @file radio_houdini_args_test.cc
 * @brief RadioHoudini's device and stream arguments for the demo config, known
 *        answers with NO hardware: the static builders (deviceArgs,
 *        rxStreamArgs, txStreamArgs) take a RadioParams and open nothing.
 *
 * The params are filled from the real Config of
 * files/houdini-dualband-xw-steer-slots.json with the accessors
 * BaseRadioSet::init and ClientRadioSet::init read, field for field; those
 * fills themselves run inside the radio's construction (which opens the
 * device), so they are not reached here. modeVPlan is private and only the
 * device-opening constructor calls it: the Config end of the mode-V plan (the
 * per-channel NCOs) is pinned here, and the bring-up from a Plan is
 * mode_v_bringup_test's.
 *
 * Run from CC/Sounder (the config's relative path): ctest sets the working
 * directory. Each check names the mutation that breaks it.
 */
#include <cstdio>
#include <cstdlib>
#include <map>
#include <stdexcept>
#include <string>

#include "include/RadioHoudini.h"
#include "include/config.h"
#include "include/houdini/rx_packet.h"
#include "include/utils.h"

namespace {
int g_fail = 0;
void check(bool ok, const std::string& what) {
  std::printf("%s  %s\n", ok ? "PASS" : "FAIL", what.c_str());
  if (!ok) ++g_fail;
}
std::string show(const SoapySDR::Kwargs& k) {
  std::string s;
  for (const auto& kv : k) s += (s.empty() ? "" : ",") + kv.first + "=" + kv.second;
  return "{" + s + "}";
}
}  // namespace

int main() {
  const char* kConf = "files/houdini-dualband-xw-steer-slots.json";
  unsetenv("HOUDINI_TX_STREAM_ARGS");  // an operator's export must not leak into the known answers
  try {
    Config cfg(kConf, "/tmp", false, false, false);

    // The node as BaseRadioSet::init describes it (192.0.2.1: a documentation
    // address; the builders only format it).
    RadioParams bs;
    bs.id = "192.0.2.1";
    bs.label = "BS " + bs.id;
    bs.remote_port = cfg.remote_port();
    bs.tx_channels = Utils::strToChannels(cfg.bs_tx_channel());
    bs.rx_channels = Utils::strToChannels(cfg.bs_rx_channel());
    bs.packet_samples = houdini::rxpkt::bsPacket(cfg.samps_per_slot(), cfg.bs_hw_framer(), cfg.rate());
    bs.tx_mode = "replay";
    // ...and as ClientRadioSet::init describes the UE.
    RadioParams ue = bs;
    ue.id = "192.0.2.2";
    ue.label = "UE " + ue.id;
    ue.tx_channels = Utils::strToChannels(cfg.cl_tx_channel());
    ue.rx_channels = Utils::strToChannels(cfg.cl_rx_channel());
    ue.tx_mode = "stream";
    ue.tdd = cfg.ue_tdd_pilot();

    // Packets that tile the 61440-tick slot: 1920 samples, 32 per RX slot, and
    // HOUDINI_MTU 1920 x 4 + 58 = 7738 (the config's _status names 7738).
    check(cfg.samps_per_slot() == 61440 && bs.packet_samples == 1920,
          "the demo slot is 61440 samples and its tiled packet 1920 (mutation: bsPacket falling back to 0)");
    const auto da = RadioHoudini::deviceArgs(bs);
    const SoapySDR::Kwargs want_da = {{"driver", "houdinisdr"},
                                      {"remote", "tcp://192.0.2.1:55132"},
                                      {"remote:driver", "houdinisdr-device"},
                                      {"remote:type", "houdinisdr"},
                                      {"timeout", "3000000"},
                                      {"remote:HOUDINI_MTU", "7738"}};
    check(da == want_da, "deviceArgs for the demo BS: " + show(da) +
                             " (mutation: the HOUDINI_MTU kwarg dropped, so the device keeps 2032-sample packets that "
                             "do not tile the slot; or the SH-442 3 s timeout reverted to 1 s)");
    RadioParams dflt = bs;
    dflt.packet_samples = 0;
    check(RadioHoudini::deviceArgs(dflt).count("remote:HOUDINI_MTU") == 0,
          "no tiled packet: no HOUDINI_MTU, the device's default (mutation: the packet_samples > 0 guard dropped, "
          "asking for a 58-byte MTU)");

    // RX: the BS opens A and C as one combined stream, the UE opens A alone;
    // neither names a port, the driver binds each channel's fixed one (SH-425).
    check(bs.rx_channels == std::vector<size_t>{0, 2} && ue.rx_channels == std::vector<size_t>{0},
          "the demo config opens BS RX A and C, UE RX A (mutation: rx_channel ignored, the BS on its common channel A)");
    const auto brx = RadioHoudini::rxStreamArgs(bs);
    check(brx == SoapySDR::Kwargs{{"rx_gap_break", "1"}, {"mts", "true"}},
          "BS rxStreamArgs " + show(brx) +
              ": break-at-gap and MTS, no local_port on the combined stream (mutation: local_port set for a combined "
              "stream, or rx_gap_break left to the driver's default)");
    const auto urx = RadioHoudini::rxStreamArgs(ue);
    check(urx == SoapySDR::Kwargs{{"rx_gap_break", "1"}, {"mts", "true"}},
          "UE rxStreamArgs " + show(urx) +
              ": break-at-gap and MTS, no local_port on a single-channel stream either (mutation: local_port set "
              "for a single channel)");
    RadioParams c_only = ue;
    c_only.rx_channels = {2};
    c_only.mts = false;
    check(RadioHoudini::rxStreamArgs(c_only) == SoapySDR::Kwargs{{"rx_gap_break", "1"}},
          "channel C alone, and no MTS asked when it is off (mutation: mts written regardless, or local_port set for "
          "a single channel)");

    // TX: the BS plays its beacon from the replay RAM; the UE streams on the TDD
    // tick anchor (ue_tdd_pilot is on in the demo config).
    const auto btx = RadioHoudini::txStreamArgs(bs);
    check(btx == SoapySDR::Kwargs{{"tx_mode", "replay"}, {"mts", "true"}},
          "BS txStreamArgs " + show(btx) + " (mutation: tdd asked on the replay stream, or tx_mode not passed)");
    const auto utx = RadioHoudini::txStreamArgs(ue);
    check(ue.tdd && utx == SoapySDR::Kwargs{{"tx_mode", "stream"}, {"tdd", "1"}, {"mts", "true"}},
          "UE txStreamArgs " + show(utx) + " (mutation: the TDD anchor not asked for)");
    setenv("HOUDINI_TX_STREAM_ARGS", "tx_target_frac=0.75", 1);
    const auto utx2 = RadioHoudini::txStreamArgs(ue);
    const auto btx2 = RadioHoudini::txStreamArgs(bs);
    check(utx2.count("tx_target_frac") == 1 && utx2.at("tx_target_frac") == "0.75" && utx2.at("tdd") == "1" &&
              btx2.count("tx_target_frac") == 0,
          "HOUDINI_TX_STREAM_ARGS reaches the UE's stream and not the BS's replay (mutation: the passthrough "
          "dropped, or applied to the replay stream too)");
    setenv("HOUDINI_TX_STREAM_ARGS", "tdd=0", 1);
    bool refused = false;
    try {
      RadioHoudini::txStreamArgs(ue);
    } catch (const std::invalid_argument&) {
      refused = true;
    }
    unsetenv("HOUDINI_TX_STREAM_ARGS");
    check(refused, "HOUDINI_TX_STREAM_ARGS overriding a key the sounder sets (tdd) is refused (mutation: the parse "
                   "error ignored)");

    // The mode-V plan's inputs: the common NCO on the sub-6 channels and the
    // X-IF override on B (UE TX) and C (BS RX), the converter rates, TX at 2x.
    const std::map<size_t, double> want_nco = {{1, 4380e6}, {2, 4380e6}};
    check(cfg.nco() == 2425e6 && cfg.channel_nco() == want_nco,
          "the demo config's NCOs: 2425 MHz common, 4380 MHz on B and C (mutation: a channel_nco_frequency value "
          "altered on the way in, such as +1 MHz, which the config's own load checks accept)");
    check(cfg.adc_fs_hz() == 4915.2e6 && cfg.dac_fs_hz() == 5898.24e6 && cfg.tx_rate() == 245.76e6 &&
              cfg.rate() == 122.88e6,
          "the demo config's converter plan: ADC 4915.2, DAC 5898.24, TX 245.76, RX 122.88 MHz (mutation: "
          "tx_sample_rate ignored, TX at the RX rate)");
  } catch (const std::exception& e) {
    check(false, std::string(kConf) + ": threw: " + e.what());
  }
  std::printf("%s: %d failure(s)\n", g_fail == 0 ? "ALL PASS" : "FAILED", g_fail);
  return g_fail == 0 ? 0 : 1;
}
