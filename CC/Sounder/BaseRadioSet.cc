/** @file BaseRadioSet.cc
  * @brief Defination file for the BaseRadioSet class.
  *
  * Copyright (c) 2018-2022, Rice University
  * RENEW OPEN SOURCE LICENSE: http://renew-wireless.org/license
  * ----------------------------------------------------------
  *  Initializes and Configures Radios in the massive-MIMO base station 
  * ----------------------------------------------------------
  */
#include "include/BaseRadioSet.h"

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <sstream>
#include <string>
#include <vector>

#include "SoapySDR/Errors.hpp"
#include "SoapySDR/Formats.hpp"
#include "SoapySDR/Time.hpp"
#include "include/Radio.h"
#include "include/comms-lib.h"
#include "include/logger.h"
#include "include/macros.h"
#include "include/node_version.h"
#include "include/utils.h"
#include "nlohmann/json.hpp"

using json = nlohmann::json;
static constexpr int kMaxTOSyncRetry = 10;

BaseRadioSet::BaseRadioSet(Config* cfg, const bool calibrate_proc) : _cfg(cfg) {
  std::vector<size_t> num_bs_antenntas(_cfg->num_cells());
  bsRadios.resize(_cfg->num_cells());
  radioNotFound = false;
  std::vector<std::string> radio_serial_not_found;

  for (size_t c = 0; c < _cfg->num_cells(); c++) {
    size_t num_radios = _cfg->n_bs_sdrs()[c];
    num_bs_antenntas[c] = num_radios * _cfg->bs_channel().length();
    MLPD_TRACE("Setting up radio: %zu, cells: %zu\n", num_radios,
               _cfg->num_cells());

    // TODO: we can handle this better!
    // Single hub could potentially trigger multiple BS or RRHs
    // Leave this for future when multi-cell is enabled
    if ((kUseSoapyUHD == false) && (_cfg->hub_ids().empty() == false) &&
        (_cfg->hub_ids().at(c).empty() == false)) {
      SoapySDR::Kwargs args;
      args["driver"] = "remote";
      args["timeout"] = "1000000";
      args["serial"] = _cfg->hub_ids().at(c);
      SoapySDR::Device* dev = SoapySDR::Device::make(args);
      if (dev == NULL)
        throw std::invalid_argument("error making SoapySDR::Device (hub)\n");
      else
        hubs.push_back(dev);
    }
    bsRadios.at(c).resize(num_radios);
    std::atomic_ulong thread_count = ATOMIC_VAR_INIT(num_radios);

    MLPD_TRACE("Init base radios: %zu\n", num_radios);
    for (size_t i = 0; i < num_radios; i++) {
      BaseRadioContext* context = new BaseRadioContext;
      context->brs = this;
      context->thread_count = &thread_count;
      context->tid = i;
      context->cell = c;
#ifdef THREADED_INIT
      pthread_t init_thread_;

      if (pthread_create(&init_thread_, NULL, BaseRadioSet::init_launch,
                         context) != 0) {
        delete context;
        throw std::runtime_error("BaseRadioSet - init thread create failed");
      }
#else
      init(context);
#endif
    }

    // Wait for init
    while (thread_count.load() > 0) {
    }

    // Strip out broken radios.
    for (size_t i = 0; i < num_radios; i++) {
      if (bsRadios.at(c).at(i) == NULL) {
        radioNotFound = true;
        radio_serial_not_found.push_back(_cfg->bs_sdr_ids().at(c).at(i));
        while (num_radios != 0 && bsRadios.at(c).at(num_radios - 1) == NULL) {
          --num_radios;
          bsRadios.at(c).pop_back();
        }
        if (i < num_radios) {
          bsRadios.at(c).at(i) = bsRadios.at(c).at(--num_radios);
          bsRadios.at(c).pop_back();
        }
      }
    }
    bsRadios.at(c).shrink_to_fit();
    _cfg->n_bs_sdrs().at(c) = num_radios;
    // Zero radios is never success. The line above writes the surviving count
    // back into the SHARED Config, and main() builds Config ONCE and reuses it
    // across its retry loop -- so an attempt that strips the only radio leaves
    // n_bs_sdrs=0 behind, and the NEXT attempt constructs nothing, finds nothing
    // to strip, keeps radioNotFound false and reports "BaseRadioSet done". The
    // run then looks healthy while activateHoudiniRx() iterates an empty vector,
    // no RX is ever started, and the receive loop spins forever on a dead
    // stream. Observed twice on the bench: a transient open failure turned into
    // a permanently silent run that the retry could not recover. Fail loudly
    // instead, so the caller retries in a fresh process with a clean config.
    if (bsRadios.at(c).empty()) {
      radioNotFound = true;
      radio_serial_not_found.push_back(
          "(cell " + std::to_string(c) +
          ": no base station radios were constructed; if an earlier attempt in "
          "this process failed, it zeroed n_bs_sdrs in the shared config -- "
          "retry in a fresh process)");
    }
    if (radioNotFound == true) {
      break;
    }

    // Perform DC Offset & IQ Imbalance Calibration
    if (calibrate_proc && _cfg->imbalance_cal_en() == true) {
      if (_cfg->bs_channel().find('A') != std::string::npos)
        dciqCalibrationProc(0);
      if (_cfg->bs_channel().find('B') != std::string::npos)
        dciqCalibrationProc(1);
      MLPD_INFO("%s done!\n", __func__);
      return;
    }

    thread_count.store(num_radios);
    for (size_t i = 0; i < num_radios; i++) {
      BaseRadioContext* context = new BaseRadioContext;
      context->brs = this;
      context->thread_count = &thread_count;
      context->tid = i;
      context->cell = c;
#ifdef THREADED_INIT
      pthread_t configure_thread_;
      if (pthread_create(&configure_thread_, NULL,
                         BaseRadioSet::configure_launch, context) != 0) {
        delete context;
        throw std::runtime_error(
            "BaseRadioSet - configure thread create failed");
      }
#else
      configure(context);
#endif
    }

    while (thread_count.load() > 0) {
    }

    auto channels = Utils::strToChannels(_cfg->bs_channel());

    for (size_t i = 0; i < bsRadios.at(c).size(); i++) {
      auto* dev = bsRadios.at(c).at(i)->RawDev();
      if (_cfg->is_houdini()) {
        // Houdini has no CBRS/UHF front end or LNA/PGA/TIA gain stages.
        std::cout << _cfg->bs_sdr_ids().at(c).at(i) << ": Houdini RFSoC BS, TX "
                  << (dev->getSampleRate(SOAPY_SDR_TX, channels.at(0)) / 1e6)
                  << " MSPS" << std::endl;
        // Register this node's gateware/firmware/host stack for the cross-node
        // skew check the Receiver runs once every radio set is up.
        Sounder::NodeVersions::instance().add(
            "BS " + _cfg->bs_sdr_ids().at(c).at(i), dev->getHardwareInfo());
        continue;
      }
      std::cout << _cfg->bs_sdr_ids().at(c).at(i) << ": Front end "
                << dev->getHardwareInfo()["frontend"] << std::endl;
      for (auto ch : channels) {
        if (ch < dev->getNumChannels(SOAPY_SDR_RX)) {
          printf("RX Channel %zu\n", ch);
          printf("Actual RX sample rate: %fMSps...\n",
                 (dev->getSampleRate(SOAPY_SDR_RX, ch) / 1e6));
          printf("Actual RX frequency: %fGHz...\n",
                 (dev->getFrequency(SOAPY_SDR_RX, ch) / 1e9));
          printf("Actual RX gain: %f...\n", (dev->getGain(SOAPY_SDR_RX, ch)));
          if (!kUseSoapyUHD) {
            printf("Actual RX LNA gain: %f...\n",
                   (dev->getGain(SOAPY_SDR_RX, ch, "LNA")));
            printf("Actual RX PGA gain: %f...\n",
                   (dev->getGain(SOAPY_SDR_RX, ch, "PGA")));
            printf("Actual RX TIA gain: %f...\n",
                   (dev->getGain(SOAPY_SDR_RX, ch, "TIA")));
            if (dev->getHardwareInfo()["frontend"].find("CBRS") !=
                std::string::npos) {
              printf("Actual RX LNA1 gain: %f...\n",
                     (dev->getGain(SOAPY_SDR_RX, ch, "LNA1")));
              printf("Actual RX LNA2 gain: %f...\n",
                     (dev->getGain(SOAPY_SDR_RX, ch, "LNA2")));
            }
          }
          printf("Actual RX bandwidth: %fM...\n",
                 (dev->getBandwidth(SOAPY_SDR_RX, ch) / 1e6));
          printf("Actual RX antenna: %s...\n",
                 (dev->getAntenna(SOAPY_SDR_RX, ch).c_str()));
        }
      }

      for (auto ch : channels) {
        if (ch < dev->getNumChannels(SOAPY_SDR_TX)) {
          printf("TX Channel %zu\n", ch);
          printf("Actual TX sample rate: %fMSps...\n",
                 (dev->getSampleRate(SOAPY_SDR_TX, ch) / 1e6));
          printf("Actual TX frequency: %fGHz...\n",
                 (dev->getFrequency(SOAPY_SDR_TX, ch) / 1e9));
          printf("Actual TX gain: %f...\n", (dev->getGain(SOAPY_SDR_TX, ch)));
          if (!kUseSoapyUHD) {
            printf("Actual TX PAD gain: %f...\n",
                   (dev->getGain(SOAPY_SDR_TX, ch, "PAD")));
            printf("Actual TX IAMP gain: %f...\n",
                   (dev->getGain(SOAPY_SDR_TX, ch, "IAMP")));
            if (dev->getHardwareInfo()["frontend"].find("CBRS") !=
                std::string::npos) {
              printf("Actual TX PA1 gain: %f...\n",
                     (dev->getGain(SOAPY_SDR_TX, ch, "PA1")));
              printf("Actual TX PA2 gain: %f...\n",
                     (dev->getGain(SOAPY_SDR_TX, ch, "PA2")));
              printf("Actual TX PA3 gain: %f...\n",
                     (dev->getGain(SOAPY_SDR_TX, ch, "PA3")));
            }
          }
          printf("Actual TX bandwidth: %fM...\n",
                 (dev->getBandwidth(SOAPY_SDR_TX, ch) / 1e6));
          printf("Actual TX antenna: %s...\n",
                 (dev->getAntenna(SOAPY_SDR_TX, ch).c_str()));
        }
      }
      std::cout << std::endl;
    }
    // Measure Sync Delays now! (Iris trigger-based; Houdini has no such block)
    if (kUseSoapyUHD == false && _cfg->is_houdini() == false) {
      sync_delays(c);
    }
  }

  if (radioNotFound == true) {
    for (auto st = radio_serial_not_found.begin();
         st != radio_serial_not_found.end(); st++)
      std::cout << "\033[1;31m" << *st << "\033[0m" << std::endl;
    std::cout << "\033[1;31mERROR: the above base station radio(s) could not "
                 "be opened. The reason is logged above each address; note "
                 "that a radio can be discoverable (SoapySDRUtil --find) and "
                 "still fail here, e.g. no route to its data-plane address."
                 "\033[0m"
              << std::endl;
  } else if (_cfg->is_houdini()) {
    // Houdini BS. bs_hw_framer=true -> native TDD framer (beacon replay strobe +
    // rx_gate on the pilot slots; loopRecv's HW-framer true-path receives only
    // those slots, tagged). bs_hw_framer=false -> free-running replay beacon +
    // the software-framer read-every-slot path.
    if (_cfg->bs_hw_framer()) {
      armHoudiniTdd();
      MLPD_INFO("%s done (Houdini native TDD framer)!\n", __func__);
    } else {
      armHoudiniBeacon();
      MLPD_INFO("%s done (Houdini replay beacon)!\n", __func__);
    }
  } else {
    if (calibrate_proc && _cfg->sample_cal_en() == true) {
      this->syncTimeOffset();
      return;
    } else if (_cfg->sample_cal_en() == true) {
      const std::string filename = "files/iris_samp_offsets.dat";
      trigger_offsets_ = Utils::ReadVector(filename, false);
      size_t num_radios = _cfg->n_bs_sdrs()[0];
      if (trigger_offsets_.size() == num_radios) {
        this->adjustDelays();
      } else {
        std::printf(
            "The number of sample offsets in file does not match the number of "
            "radios.\n");
      }
    }

    nlohmann::json tddConf;
    tddConf["tdd_enabled"] = true;
    tddConf["frame_mode"] = "free_running";
    tddConf["max_frame"] = _cfg->max_frame();
    tddConf["symbol_size"] = _cfg->samps_per_slot();

    // write TDD schedule and beacons to FPFA buffers only for Iris
    for (size_t c = 0; c < _cfg->num_cells(); c++) {
      if (!kUseSoapyUHD) {
        size_t ndx = 0;
        for (size_t i = 0; i < bsRadios.at(c).size(); i++) {
          auto* dev = bsRadios.at(c).at(i)->RawDev();
          tddConf["frames"] = json::array();
          if (_cfg->internal_measurement() == true) {
            for (char const& c : _cfg->bs_channel()) {
              std::string tx_ram = "TX_RAM_";
              dev->writeRegisters(tx_ram + c, 0, _cfg->pilot());
            }
            tddConf["frames"].push_back(_cfg->bs_array_frames().at(c).at(i));
            std::cout << "Cell " << c << ", SDR " << i
                      << " calibration schedule : "
                      << _cfg->bs_array_frames().at(c).at(i) << std::endl;

          } else {
            tddConf["frames"] = json::array();

            const size_t frame_size =
                _cfg->bs_array_frames().at(c).at(i).size();
            std::string fw_frame = _cfg->bs_array_frames().at(c).at(i);

            for (size_t s = 0; s < frame_size; s++) {
              char sym_type = fw_frame.at(s);
              if (sym_type == 'P')
                fw_frame.replace(s, 1, "R");  // uplink pilots
              else if (sym_type == 'N')
                fw_frame.replace(s, 1, "R");  // uplink data
              else if (sym_type == 'U')
                fw_frame.replace(s, 1, "R");  // uplink data
              else if (sym_type == 'D')
                fw_frame.replace(s, 1, "T");  // downlink data
            }

            tddConf["frames"].push_back(fw_frame);
            std::cout << "Cell " << c << ", SDR " << i
                      << " Schedule : " << fw_frame << std::endl;
          }
          if (_cfg->internal_measurement() == false ||
              _cfg->num_cl_antennas() > 0) {
            dev->writeRegisters("BEACON_RAM", 0, _cfg->beacon());
            std::string tx_ram_wgt = "BEACON_RAM_WGT_";
            for (char const& ch : _cfg->bs_channel()) {
              bool isBeaconAntenna =
                  !_cfg->beam_sweep() && ndx == _cfg->beacon_ant();
              std::vector<unsigned> beacon_weights(num_bs_antenntas[c],
                                                   isBeaconAntenna ? 1 : 0);
              if (_cfg->beam_sweep()) {
                for (size_t j = 0; j < num_bs_antenntas[c]; j++)
                  beacon_weights[j] = CommsLib::hadamard2(ndx, j);
              }
              dev->writeRegisters(tx_ram_wgt + ch, 0, beacon_weights);
              ++ndx;
            }

            dev->writeSetting("BEACON_START",
                              std::to_string(bsRadios.at(c).size()));
            tddConf["beacon_start"] = _cfg->prefix();
            tddConf["beacon_stop"] = _cfg->prefix() + _cfg->beacon_size();
          }
          std::string tddConfStr = tddConf.dump();
          dev->writeSetting("TDD_CONFIG", tddConfStr);
          dev->writeSetting(
              "TX_SW_DELAY",
              "30");  // experimentally good value for dev front-end
          dev->writeSetting("TDD_MODE", "true");
        }
      }

      if (!kUseSoapyUHD) {
        for (size_t i = 0; i < bsRadios.at(c).size(); i++) {
          auto* dev = bsRadios.at(c).at(i)->RawDev();
          bsRadios.at(c).at(i)->activateRecv();
          bsRadios.at(c).at(i)->activateXmit();
          dev->setHardwareTime(0, "TRIGGER");
        }
      } else {
        // Set freq and time source for multiple USRPs
        for (size_t i = 0; i < bsRadios.at(c).size(); i++) {
          auto* dev = bsRadios.at(c).at(i)->RawDev();
          dev->setClockSource("external");
          dev->setTimeSource("external");
          dev->setHardwareTime(0, "PPS");
        }
        // Wait for pps sync pulse
        std::this_thread::sleep_for(std::chrono::seconds(2));
        // Activate Rx and Tx streamers
        for (size_t i = 0; i < bsRadios.at(c).size(); i++) {
          bsRadios.at(c).at(i)->activateRecv();
          bsRadios.at(c).at(i)->activateXmit();
        }
      }
    }
    MLPD_INFO("%s done!\n", __func__);
  }
}

void BaseRadioSet::buildHoudiniBeacon(std::vector<int16_t>& iq) {
  constexpr int kReplayDepth = 4096;  // Houdini TX replay RAM depth (samples)

  // Rebuild the STS+gold core of config's beacon (indices [prefix, prefix+
  // beacon_size) skip the zero pre/postfix). Conjugate it: the matched-NCO R2C
  // mixer delivers the beacon conjugated and receiver.cc::syncSearch feeds the
  // raw RX to find_beacon, so pre-conjugating the TX cancels it.
  //
  // NO host upsampling: the beacon replays at the APP rate (BaseRadioSet opens
  // the beacon Radio's TX at _cfg->rate(), not the DAC max), so the RFDC's own
  // interpolation carries it to the DAC. Placing the beacon_size (~496) core at
  // the head of the 4096-deep RAM and leaving the rest SILENT makes an ISOLATED
  // beacon that recurs every 4096 samples (~33 us): it fits inside the client's
  // detect window AND keeps find_beacon's trailing-energy threshold low, so the
  // SHARP native-rate 2-rep gold peak clears it at corr_scale=1. (The old x8
  // upsample + DAC-max replay recurred every 512 samples -- dense -- and smeared
  // the gold, forcing corr_scale~100.)
  const auto& bc = _cfg->beacon_ci16();
  const int p = _cfg->prefix();
  const int n = _cfg->beacon_size();
  std::vector<std::complex<float>> loop(kReplayDepth, std::complex<float>(0, 0));
  for (int k = 0; k < n && k < kReplayDepth; ++k) {
    loop[k] = std::conj(std::complex<float>(
        static_cast<float>(bc.at(p + k).real()),
        static_cast<float>(bc.at(p + k).imag())));
  }
  float peak = 1e-30f;
  for (const auto& v : loop) peak = std::max(peak, std::abs(v));
  const size_t n_load = loop.size();
  iq.assign(n_load * 2, 0);
  for (size_t k = 0; k < n_load; ++k) {
    iq[2 * k] =
        static_cast<int16_t>(std::lround(loop[k].real() / peak * 0.6f * 32767));
    iq[2 * k + 1] =
        static_cast<int16_t>(std::lround(loop[k].imag() / peak * 0.6f * 32767));
  }
  if (std::getenv("HOUDINI_DUMP_BEACON")) {
    FILE* f = std::fopen("/tmp/beacon_ram.bin", "wb");
    if (f) {
      std::fwrite(iq.data(), sizeof(int16_t), iq.size(), f);
      std::fclose(f);
      MLPD_INFO("HOUDINI_DUMP_BEACON: wrote %zu int16 to /tmp/beacon_ram.bin\n",
                iq.size());
    }
  }
}

void BaseRadioSet::armHoudiniBeacon(void) {
  std::vector<int16_t> iq;
  buildHoudiniBeacon(iq);
  const size_t n_load = iq.size() / 2;

  // Load the replay RAM + arm free-running on the beacon radio's TX. The TX
  // stream is bound to the BS channel (the wired DAC), so xmit targets it. RX
  // is NOT activated here: it would sit unread (overflowing) until the caller is
  // ready to receive -- activateHoudiniRx() starts it on demand.
  const void* buffs[1] = {iq.data()};
  long long t0 = 0;
  for (size_t c = 0; c < bsRadios.size(); ++c) {
    for (size_t i = 0; i < bsRadios.at(c).size(); ++i) {
      if (i != _cfg->beacon_radio()) continue;
      Radio* r = bsRadios.at(c).at(i);
      r->xmit(buffs, static_cast<int>(n_load), 0, t0);  // load replay RAM
      r->activateXmit();                                // arm free-running loop
      MLPD_INFO(
          "Houdini BS beacon armed: %zu-sample app-rate replay loop (isolated "
          "beacon, period %zu) on cell %zu radio %zu\n",
          n_load, n_load, c, i);
    }
  }
}

void BaseRadioSet::activateHoudiniRx(void) {
  // Start the BS RX streams on demand (the reverse link / UE pilots). Kept
  // separate from beacon arming so the RX stream is not left overflowing while
  // the caller is busy elsewhere (e.g. waiting for the UE to acquire).
  for (size_t c = 0; c < bsRadios.size(); ++c)
    for (size_t i = 0; i < bsRadios.at(c).size(); ++i)
      bsRadios.at(c).at(i)->activateRecv();
}

// ---- Houdini native-TDD framer (bs_hw_framer + radio_type=houdini) ----------
namespace {
constexpr long long kTddGridTicks = 384;       // 3.125 us grid (strobe offs)
constexpr long long kTddArmMargin = 36864000;  // ~300 ms of ticks
}  // namespace

// The full teardown ladder. Abort alone is NOT enough, twice over (measured,
// DEMO_VERIFICATION.md 3.2 + 4.24): aborting a RUNNING framer latches
// gates_held and every later arm is REFUSED until gate_release; and skipping
// TX_CLEAR can leave the TX bank in a state where strobe bursts ack and count
// as played but NO RF leaves the DAC ("a consumed arm parks the source until
// the arm clears or TX_CLEAR").
void BaseRadioSet::houdiniTddLadder(SoapySDR::Device* dev) {
  dev->writeSetting("TDD_CMD", "abort");
  dev->writeRegister("RFCORE", 0x24, 1);  // TX_CLEAR_ALL pulse
  dev->writeRegister("RFCORE", 0x24, 0);
  dev->writeSetting("TDD_CMD", "gate_release");
}

long long BaseRadioSet::houdiniArmTdd(SoapySDR::Device* dev,
                                      long long symbol_ticks,
                                      long long symbols_per_frame) {
  const std::string arm = "symbol_ticks=" + std::to_string(symbol_ticks) +
                          ",symbols_per_frame=" + std::to_string(symbols_per_frame) +
                          ",margin=" + std::to_string(kTddArmMargin);
  long long epoch = 0;
  bool accepted = false;
  for (int attempt = 0; attempt < 4 && !accepted; ++attempt) {
    // On the current stack a refused arm THROWS (SH-333) instead of returning
    // accepted=0. A throwing WRITE must never be trusted via the readback:
    // failures before the device stores its last-arm record leave a STALE
    // string (possibly a previous run's accepted=1), so a throw always
    // re-ladders and retries (Opus review finding 3).
    try {
      dev->writeSetting("TDD_ARM", arm);
    } catch (const std::exception& e) {
      MLPD_WARN("TDD_ARM attempt %d refused: %s\n", attempt, e.what());
      houdiniTddLadder(dev);
      continue;
    }
    std::stringstream ss(dev->readSetting("TDD_ARM"));
    std::string tok;
    while (ss >> tok) {
      const auto eq = tok.find('=');
      if (eq == std::string::npos) continue;
      const std::string k = tok.substr(0, eq), v = tok.substr(eq + 1);
      if (k == "epoch") epoch = std::stoll(v);
      if (k == "accepted") accepted = (v == "1");
    }
    if (!accepted) houdiniTddLadder(dev);
  }
  if (!accepted) throw std::runtime_error("Houdini TDD_ARM rejected");
  return epoch;
}

void BaseRadioSet::armHoudiniTdd(void) {
  // Slot-granular ring: one TDD symbol per sounder slot (symbol_ticks =
  // samps_per_slot, symbols_per_frame = slot_per_frame), '6' on the beacon
  // slot, '2' on every other slot. Verified on silicon 2026-08-30
  // (DEMO_VERIFICATION.md 4.12): the ring arms and the strobe plays exactly
  // one burst per frame. Every non-beacon entry must keep the rx bit set:
  // a gate close ABANDONS a running continuous capture (driver contract,
  // D4 window-pump + overlength-abandon), and per-window host pumping costs
  // ~100 ms RPC per window, unusable at 1 ms frames. True rx-only-in-P/U
  // gating therefore needs a driver capability (hardware-chained windowed
  // RX); until then the wire carries the whole frame and the guards are
  // silent AIR, not absent DATA (DEMO_VERIFICATION.md section 3/4).
  //
  // The strobe plays ONE beacon copy per frame (loops=1, len = beacon core):
  // the old loops=forever filled a 0.5 ms symbol with ~15 copies, which made
  // the UE's frame anchor ambiguous by k x 4096 samples per restart.
  //
  // NB (houdini_beacon_ab.py, .21->.22): the SAME beacon RAM scored by the
  // client's gold correlation gives 44.5 dB via the framer strobe vs only 10.2 dB
  // via a continuous activateXmit replay -- the strobe is SHARPER, not distorted
  // (an earlier "strobe distorted ~11 dB" note was the continuous mode mislabeled).
  // And a continuous replay can't coexist with the framer anyway: arming the framer
  // silences activateXmit (-33 dB) and any tx_gate schedule is arm-rejected without
  // a strobe. So the strobe is the only way to get beacon + rx_gate on one board.
  htdd_symbol_ticks_ = static_cast<long long>(_cfg->samps_per_slot());
  htdd_tick_rate_ = _cfg->rate();

  std::vector<int16_t> iq;
  buildHoudiniBeacon(iq);
  const size_t n_load = iq.size() / 2;
  const void* buffs[1] = {iq.data()};

  for (size_t c = 0; c < bsRadios.size(); ++c) {
    for (size_t i = 0; i < bsRadios.at(c).size(); ++i) {
      if (i != _cfg->beacon_radio()) continue;
      Radio* r = bsRadios.at(c).at(i);
      auto* dev = r->RawDev();
      houdiniTddLadder(dev);  // full ladder, never abort alone (3.2 + 4.24)

      // The sounder pilot/uplink slots to receive (tag with these indices).
      const std::string& sched = _cfg->bs_array_frames().at(c).at(i);
      htdd_rx_slots_.clear();
      for (size_t s = 0; s < sched.size(); ++s) {
        const char ch = sched.at(s);
        if (ch == 'P' || ch == 'R' || ch == 'U' || ch == 'N')
          htdd_rx_slots_.push_back(s);
        if (ch == 'P') htdd_pilot_slot_ = s;  // CSI reference slot
      }
      // TDD frame must EQUAL the sounder frame: the beacon fires once per TDD
      // frame, so a longer TDD frame makes the beacon period differ from the UE's
      // (sounder) frame and the pilot/data walk relative to the beacon -> noisy CSI.
      // One symbol per sounder slot, beacon strobe on the schedule's B slot,
      // rx bit on EVERY entry (see the function comment: a closed gate kills
      // the continuous capture; the rx slots are still extracted from the
      // continuous read in houdiniTddRx).
      const size_t spf_tdd = _cfg->slot_per_frame();
      const size_t b_pos = sched.find('B');
      const size_t beacon_slot = (b_pos == std::string::npos) ? 0 : b_pos;
      if (htdd_symbol_ticks_ > 0xFFFF || spf_tdd > 0x1FFF) {
        throw std::runtime_error(
            "armHoudiniTdd: slot-granular ring out of framer range "
            "(symbol_ticks=" + std::to_string(htdd_symbol_ticks_) +
            ", spf=" + std::to_string(spf_tdd) + ")");
      }
      std::string tdd(spf_tdd, '2');    // every slot rx-gates
      // NB the ring's last rx entry abuts the tx-ish '6' across the frame
      // wrap, so the driver logs the HS-184 warm-return warning twice per
      // arm. ACCEPTED deliberately: a '0' guard would close the rx gate and
      // abandon the continuous capture (see the function comment); the
      // warning is about X-band T/R-switch timing, moot on this cabled
      // bench. (The guarded probe ring in DEMO_VERIFICATION.md 4.12 avoided
      // the warning; the shipped ring does not.)
      tdd.at(beacon_slot) = '6';        // + beacon strobe on the B slot
      htdd_frame_ticks_ = static_cast<long long>(spf_tdd) * htdd_symbol_ticks_;

      // PHYSICAL TX channel for the strobe (beacon_channel() is the logical index
      // within bs_channel; TDD_REPLAY_STROBE and the loaded RAM are on the real
      // DAC, e.g. bs_channel "B" -> ch1 = the cabled DAC_A). Using the logical 0
      // fired the strobe on ch0 (DAC_B, not cabled) so the beacon never reached
      // the UE.
      const auto bs_chans = Utils::strToChannels(_cfg->bs_channel());
      const size_t tx_ch =
          bs_chans.empty()
              ? 0
              : bs_chans.at(std::min(static_cast<size_t>(_cfg->beacon_channel()),
                                     bs_chans.size() - 1));
      long long t0 = 0;
      // Explicitly disarm any strobe left armed by a previous (e.g. killed) run --
      // TDD_CMD abort alone doesn't release it, and the replay RAM can't be filled
      // while strobe mode is enabled ("Disarm first"). Safe when nothing is armed.
      try {
        dev->writeSetting("TDD_REPLAY_STROBE",
                          "ch" + std::to_string(tx_ch) + ":off");
      } catch (...) {
      }
      r->xmit(buffs, static_cast<int>(n_load), 0, t0);  // load replay RAM
      dev->writeSetting("TDD_SCHED", tdd);
      // ONE burst per frame (loops=1) spanning the usable symbol: the RAM is
      // [beacon core 496][zeros], and len (2-sample units, driver contract)
      // covers (symbol - offs) samples, so the slot's DAC input is the beacon
      // followed by OUR zeros up to the window close, not engine-idle output.
      // (D5 measured post-burst idle as silent on silicon, but explicit zeros
      // remove the reliance.) Single-copy removes the k x 4096 anchor
      // ambiguity the old loops=forever multi-copy fill created; the UE's
      // acquisition just needs more detect windows to first see it
      // (~1 in 12.9 windows carries the beacon now). Samples == ticks at the
      // one supported rate (122.88 MSPS; the whole layer assumes it).
      const size_t span_units =
          static_cast<size_t>((htdd_symbol_ticks_ - kTddGridTicks) / 2);
      const size_t len_units = std::max<size_t>(
          (static_cast<size_t>(_cfg->beacon_size()) + 1) / 2,
          std::min(n_load / 2, span_units));
      dev->writeSetting("TDD_REPLAY_STROBE",
                        "ch" + std::to_string(tx_ch) +
                            ":len=" + std::to_string(len_units) +
                            ",loops=1,offs=" + std::to_string(kTddGridTicks));
      htdd_epoch_ = houdiniArmTdd(dev, htdd_symbol_ticks_,
                                  static_cast<long long>(spf_tdd));
      htdd_rx_cursor_ = 0;
      htdd_last_win_tick_ = 0;
      MLPD_INFO(
          "Houdini BS TDD armed: sched=%s epoch=%lld frame=%lld ticks, "
          "%zu pilot slot(s) %s beacon strobe %zu samp\n",
          tdd.c_str(), htdd_epoch_, htdd_frame_ticks_, htdd_rx_slots_.size(),
          htdd_rx_slots_.empty() ? "(none)" : "", n_load);
    }
  }
}

int BaseRadioSet::houdiniTddRx(size_t radio_id, void* const* buffs,
                               long long& frameTime) {
  if (htdd_rx_slots_.empty()) return 0;
  // Bound the BS capture run: loopRecv (unlike the client loop) doesn't stop at
  // max_frame, and an unbounded frame_id would grow the recorder's HDF5 dataset
  // without limit (-> extend crash at close). Returning <0 makes loopRecv set
  // running(false) and shut down cleanly.
  if (_cfg->max_frame() > 0 && htdd_frame_counter_ >= _cfg->max_frame())
    return -1;
  Radio* r = bsRadios.at(0).at(radio_id);
  const int n = static_cast<int>(_cfg->samps_per_slot());
  const size_t K = htdd_rx_slots_.size();  // rx slots/frame (pilot P + uplink U...)
  const size_t cur = htdd_rx_cursor_;

  // Non-first rx slot: serve it from the per-frame cache filled on cursor 0 (one
  // continuous read yields every rx slot of the frame). Shares the frame_id so the
  // recorder places P and U in the same frame.
  if (cur != 0) {
    std::memcpy(buffs[0], htdd_slot_cache_.data() + cur * static_cast<size_t>(n) * 2,
                static_cast<size_t>(n) * 4);
    const size_t slot = htdd_rx_slots_.at(cur);
    htdd_rx_cursor_ = (cur + 1) % K;
    frameTime = (htdd_cache_frame_ << 32) | (static_cast<long long>(slot) << 16);
    return n;
  }

  // cursor 0: CONTINUOUS framer receive (the Iris model -- framer armed once, RX
  // activated once in radioStart, no per-window arm/teardown). Read a bit more than
  // one frame + the pilot->last-slot span so every rx slot is fully contained
  // regardless of the (unaligned) read phase.
  const int span = (K > 1)
                       ? static_cast<int>(htdd_rx_slots_.back() -
                                          htdd_rx_slots_.front())
                       : 0;
  const int fn = static_cast<int>(htdd_frame_ticks_) + (span + 3) * n;
  htdd_cap_buf_.resize(static_cast<size_t>(fn) * 2);
  void* cb[1] = {htdd_cap_buf_.data()};
  long long ft = 0;
  const int cg = r->recv(cb, fn, ft);
  // This one read backs every rx slot of the frame, so its padding applies to all of
  // them; latch it before any later recv on this radio overwrites the radio's copy.
  htdd_frame_pad_ = r->lastPadSamples();
  if (cg < fn) {
    // Short read: the frame is not fully covered. align_slot() clamps a slot start
    // to cg, so any slot past the received data would be extracted from the tail
    // and look perfectly valid downstream. Fold the shortfall into the frame's
    // untrusted count so consumers refuse those slots instead of trusting them.
    // With rx_gap_break on (the driver default) a short return is how a dropped
    // packet surfaces, so this is the normal loss path, not an exotic one (AP-10).
    htdd_frame_pad_ += static_cast<size_t>(fn - cg);
  }
  if (cg < n) return (cg < 0) ? cg : 0;
  const int16_t* s = htdd_cap_buf_.data();
  std::vector<double> cse(static_cast<size_t>(cg) + 1, 0.0);
  for (int i = 0; i < cg; ++i) {
    const double re = s[2 * i], im = s[2 * i + 1];
    cse[i + 1] = cse[i] + re * re + im * im;
  }
  double best = 0.0;
  int at = 0;
  for (int t = 0; t + n <= cg; t += 128) {
    const double e = cse[t + n] - cse[t];
    if (e > best) { best = e; at = t; }
  }
  const double pilot_rms = std::sqrt(best / n);
  const double mean_rms = std::sqrt(cse[cg] / cg);
  // Presence gate: skip frames where no UE signal is on-air (don't advance the
  // frame counter -> the first real frame lands at recorder frame 0). A LOSS
  // of pilots mid-run is reported loudly [user 2026-08-30]: the UE pausing
  // its schedule (e.g. the AP-18 resync escalation hunting for a lost beacon)
  // shows up here as a quiet streak, and the BS should say so rather than
  // skip silently.
  if (pilot_rms < 120.0 || pilot_rms < 4.0 * mean_rms) {
    ++htdd_quiet_streak_;
    constexpr size_t kQuietWarnFrames = 200;  // ~0.2 s at 1 kHz frames
    if (htdd_frame_counter_ > 0 &&
        (htdd_quiet_streak_ == kQuietWarnFrames ||
         (htdd_quiet_streak_ > kQuietWarnFrames &&
          htdd_quiet_streak_ % 2000 == 0))) {
      htdd_quiet_warned_ = true;
      MLPD_WARN(
          "BS: UE PILOT LOST for %zu consecutive frames (last good frame "
          "%lld) -- UE schedule paused or link down\n",
          htdd_quiet_streak_, htdd_frame_counter_);
    }
    std::memcpy(buffs[0], s, static_cast<size_t>(n) * 4);
    frameTime = (htdd_frame_counter_ << 32) |
                (static_cast<long long>(htdd_rx_slots_.at(0)) << 16);
    return n;
  }
  if (htdd_quiet_warned_) {
    MLPD_WARN("BS: UE pilot RETURNED after %zu quiet frames (frame %lld)\n",
              htdd_quiet_streak_, htdd_frame_counter_);
    htdd_quiet_warned_ = false;
  }
  htdd_quiet_streak_ = 0;
  // The densest slot `at` is a UE slot -- pilot OR data. Identify it: the pilot is
  // identical repeated LTS symbols (high self-similarity at lag cp+fft); data is
  // distinct symbols (low). This keeps P/U tagged correctly so CSI comes from the
  // pilot and equalization from the data.
  auto selfsim = [&](int off) -> double {
    const int lag = static_cast<int>(_cfg->cp_size() + _cfg->fft_size());
    if (off < 0 || off + n > cg) return 0.0;
    double sr = 0, si = 0, sp = 0;
    for (int m = 0; m + lag < n; ++m) {
      const double a = s[2 * (off + m)], b = s[2 * (off + m) + 1];
      const double c = s[2 * (off + m + lag)], d = s[2 * (off + m + lag) + 1];
      sr += a * c + b * d;
      si += b * c - a * d;
      sp += a * a + b * b;
    }
    return sp > 0 ? std::sqrt(sr * sr + si * si) / sp : 0.0;
  };
  const int gap =
      static_cast<int>(htdd_rx_slots_.back() - htdd_pilot_slot_) * n;
  int p_at = at;
  if (selfsim(at) < 0.5) {  // `at` is a data slot -> the pilot is `gap` earlier
    if (selfsim(at - gap) >= 0.4) p_at = at - gap;
    else if (selfsim(at + gap) >= 0.4) p_at = at + gap;
  }
  // Centroid-align a slot's energy near `guess` -> transmitted [prefix][energy]
  // [postfix] layout (energy edge at ~prefix). Window ~1.25 slots so it can't
  // reach into an adjacent slot and drag the centroid.
  auto align_slot = [&](long long guess) -> long long {
    long long w0 = guess - n / 8;
    if (w0 < 0) w0 = 0;
    long long w1 = w0 + 5 * n / 4;
    if (w1 > cg) w1 = cg;
    double peak = 0.0;
    for (long long i = w0 + 64; i + 64 <= w1; ++i) {
      const double m = cse[i + 64] - cse[i - 64];
      if (m > peak) peak = m;
    }
    const double thr = 0.15 * peak;
    long long cnt = 0;
    double isum = 0.0;
    for (long long i = w0 + 64; i + 64 <= w1; ++i)
      if (cse[i + 64] - cse[i - 64] > thr) { ++cnt; isum += static_cast<double>(i); }
    long long st = (cnt > 0 ? std::llround(isum / cnt) : (guess + n / 2)) - n / 2;
    if (st < 0) st = 0;
    if (st + n > cg) st = cg - n;
    return st;
  };
  const long long p_start = align_slot(p_at);  // aligned pilot slot start
  // Fill the per-frame cache. Centroid-align EACH rx slot to its OWN energy near
  // its expected offset from the pilot -- the UE snaps each slot's tx time to the
  // 384-tick TDD grid independently, so the pilot->data spacing isn't exactly an
  // integer number of slots; extracting the data at pilot+gap would leave it
  // ~260 samples off. Aligning each slot lands every slot at [prefix..] so the
  // recorded data lines up with the pilot for offline equalization.
  htdd_slot_cache_.resize(K * static_cast<size_t>(n) * 2);
  long long u_start = -1;  // aligned start of the uplink-data slot, if present
  for (size_t k = 0; k < K; ++k) {
    const long long guess = p_start +
        (static_cast<long long>(htdd_rx_slots_.at(k)) -
         static_cast<long long>(htdd_pilot_slot_)) * n;
    const long long st = align_slot(guess);
    if (htdd_rx_slots_.at(k) != htdd_pilot_slot_) u_start = st;
    std::memcpy(htdd_slot_cache_.data() + k * static_cast<size_t>(n) * 2,
                s + st * 2, static_cast<size_t>(n) * 4);
  }
  if (getenv("HOUDINI_BS_RX_DEBUG") != nullptr) {
    static std::atomic<int> dc{0};
    if ((dc.fetch_add(1) % 20) == 0) {
      // Rederive the UE's realized schedule on the BS grid: the read stamp
      // (ns) + in-buffer position - epoch, folded into the frame, gives the
      // pilot slot's absolute offset from its scheduled slot boundary. The
      // number decomposes as prefix + round-trip latency - tx_advance +
      // grid/snap residuals; it must be CONSTANT within a run, and it is the
      // direct input for deriving tx_advance (DEMO_VERIFICATION.md 4.29).
      const long long stamp_ticks =
          llround(static_cast<double>(ft) * htdd_tick_rate_ / 1e9);
      const long long fr = htdd_frame_ticks_;
      long long grid_off =
          ((stamp_ticks + p_start - htdd_epoch_) % fr + fr) % fr;
      long long rel_pilot = grid_off -
          static_cast<long long>(htdd_pilot_slot_) * n;
      if (rel_pilot > fr / 2) rel_pilot -= fr;
      if (rel_pilot < -fr / 2) rel_pilot += fr;
      long long pu_err = -99999;
      if (u_start >= 0) {
        const long long slots_gap =
            static_cast<long long>(htdd_rx_slots_.back()) -
            static_cast<long long>(htdd_pilot_slot_);
        pu_err = (u_start - p_start) - slots_gap * n;
      }
      MLPD_INFO("HOUDINI_BS_RX: frame=%lld cg=%d pilot-rms=%.0f selfsim=%.2f "
                "p_start=%lld rx_slots=%zu pilot_grid_off=%lld pu_spacing_err="
                "%lld\n",
                htdd_frame_counter_, cg, pilot_rms, selfsim(at), p_start, K,
                rel_pilot, pu_err);
    }
  }
  std::memcpy(buffs[0], htdd_slot_cache_.data(), static_cast<size_t>(n) * 4);
  htdd_cache_frame_ = htdd_frame_counter_;
  ++htdd_frame_counter_;
  htdd_rx_cursor_ = (K > 1) ? 1 : 0;
  frameTime = (htdd_cache_frame_ << 32) |
              (static_cast<long long>(htdd_rx_slots_.at(0)) << 16);
  return n;
}

BaseRadioSet::~BaseRadioSet(void) {
  if (!_cfg->hub_ids().empty()) {
    for (unsigned int i = 0; i < hubs.size(); i++)
      SoapySDR::Device::unmake(hubs.at(i));
  }
  for (unsigned int c = 0; c < _cfg->num_cells(); c++)
    for (size_t i = 0; i < _cfg->n_bs_sdrs().at(c); i++)
      delete bsRadios.at(c).at(i);
}

void* BaseRadioSet::init_launch(void* in_context) {
  BaseRadioContext* context = reinterpret_cast<BaseRadioContext*>(in_context);
  context->brs->init(context);
  return 0;
}

void BaseRadioSet::init(BaseRadioContext* context) {
  int i = context->tid;
  int c = context->cell;
  std::atomic_ulong* thread_count = context->thread_count;
  delete context;

  MLPD_TRACE("Deleting context for tid: %d\n", i);

  auto channels = Utils::strToChannels(_cfg->bs_channel());
  SoapySDR::Kwargs args;
  SoapySDR::Kwargs rx_stream_args;
  SoapySDR::Kwargs tx_stream_args;
  if (_cfg->is_houdini()) {
    // SoapyHoudiniSDR BS node: bs_sdr_ids() holds the board IP. Address the
    // remote node directly (C++ auto-discovery is unreliable). RX host port
    // 10100+ keeps it clear of the UE stream (10002+); the beacon is device
    // BRAM replay (tx_mode=replay), so the TX rate is the DAC max (see below).
    args["driver"] = "houdinisdr";
    args["remote"] =
        "tcp://" + _cfg->bs_sdr_ids().at(c).at(i) + ":" + _cfg->remote_port();
    args["remote:driver"] = "houdinisdr-device";
    args["remote:type"] = "houdinisdr";
    // The FPGA egresses RX to the fixed fpga_port (10002 for ch1); the host must
    // bind that same port. The BS and UE are on different interface IPs, so both
    // can bind 10002 without colliding.
    rx_stream_args["local_port"] = "10002";
    // Break-at-gap (SH-253). The driver defaults this ON, but our whole gap
    // account depends on it: recvHoudini only compares timestamps BETWEEN reads,
    // so a splice INSIDE one returned buffer would be invisible to us. Ask for it
    // explicitly rather than inheriting a default another repo owns (AP-10).
    rx_stream_args["rx_gap_break"] = "1";
    tx_stream_args["tx_mode"] = "replay";
  } else if (kUseSoapyUHD == false) {
    args["driver"] = "iris";
    args["serial"] = _cfg->bs_sdr_ids().at(c).at(i);
  } else {
    args["driver"] = "uhd";
    args["addr"] = _cfg->bs_sdr_ids().at(c).at(i);
    std::cout << "Init bsRadios: " << args["addr"] << std::endl;
  }
  args["timeout"] = "1000000";
  try {
    bsRadios.at(c).at(i) = nullptr;
    // Houdini: RX + TX both at the app rate, tuned to the NCO -- all before
    // setupStream (Houdini forbids a live rate change). The beacon replay RAM
    // plays out at THIS configured rate and the RFDC does its own interpolation
    // up to the DAC (streaming.cpp: "the replay RAM plays out at this fabric
    // rate"); there is NO need to clock replay at the DAC max and pre-upsample
    // the beacon on the host (the old -1 sentinel did that and made the beacon
    // recur every 512 samples -- dense -- which buried find_beacon's 2-rep peak).
    bsRadios.at(c).at(i) =
        new Radio(args, SOAPY_SDR_CS16, channels, rx_stream_args, tx_stream_args,
                  _cfg->is_houdini() ? _cfg->rate() : 0.0,
                  _cfg->is_houdini() ? _cfg->rate() : 0.0,
                  _cfg->is_houdini() ? _cfg->nco() : 0.0);
  } catch (std::runtime_error& err) {
    // Name the radio by what it actually is, and SAY WHY it was dropped. This
    // used to print "Ignoring iris <addr>" (upstream RENEWLab hardware we do not
    // run) and throw err.what() away, so a base station that failed to open gave
    // only its address before surfacing as "serials were not discovered in the
    // network" -- which sends you to check discovery when the real cause was in
    // the exception all along (a missing data-plane route, a busy stream). The
    // client path already logs its reason; this matches it.
    const char* kind = kUseSoapyUHD ? "uhd device"
                                    : (_cfg->is_houdini() ? "houdini radio"
                                                          : "iris");
    std::cerr << "Ignoring " << kind << " " << _cfg->bs_sdr_ids().at(c).at(i)
              << ": " << err.what() << std::endl;
    if (bsRadios.at(c).at(i) != nullptr) {
      MLPD_TRACE("Deleting radio ptr due to exception\n");
      delete bsRadios.at(c).at(i);
      bsRadios.at(c).at(i) = nullptr;
    }
  }
  MLPD_TRACE("BaseRadioSet: Init complete\n");
  assert(thread_count->load() != 0);
  thread_count->store(thread_count->load() - 1);
}

void* BaseRadioSet::configure_launch(void* in_context) {
  BaseRadioContext* context = reinterpret_cast<BaseRadioContext*>(in_context);
  context->brs->configure(context);
  return 0;
}

void BaseRadioSet::configure(BaseRadioContext* context) {
  int i = context->tid;
  int c = context->cell;
  std::atomic_ulong* thread_count = context->thread_count;
  delete context;

  //load channels
  auto channels = Utils::strToChannels(_cfg->bs_channel());
  Radio* bsRadio = bsRadios.at(c).at(i);
  auto* dev = bsRadio->RawDev();
  SoapySDR::Kwargs info = dev->getHardwareInfo();
  for (auto ch : channels) {
    double rxgain = _cfg->rx_gain().at(ch);
    double txgain = _cfg->tx_gain().at(ch);
    bsRadios.at(c).at(i)->dev_init(_cfg, ch, rxgain, txgain);
  }

  assert(thread_count->load() != 0);
  thread_count->store(thread_count->load() - 1);
}

SoapySDR::Device* BaseRadioSet::baseRadio(size_t cellId) {
  if (cellId < hubs.size()) return (hubs.at(cellId));
  if (cellId < bsRadios.size() && bsRadios.at(cellId).size() > 0)
    return bsRadios.at(cellId).at(0)->RawDev();
  return NULL;
}

void BaseRadioSet::sync_delays(size_t cellIdx) {
  /*
     * Compute Sync Delays
     */
  SoapySDR::Device* base = baseRadio(cellIdx);
  if (base != NULL) base->writeSetting("SYNC_DELAYS", "");
}

void BaseRadioSet::radioTrigger(void) {
  for (size_t c = 0; c < _cfg->num_cells(); c++) {
    auto* base = baseRadio(c);
    if (base != NULL) {
      base->writeSetting("TRIGGER_GEN", "");
    }
  }
}

void BaseRadioSet::adjustDelays() {
  // adjust all trigger delay fwith respect to the max offset
  const auto min_max_offset =
      std::minmax_element(trigger_offsets_.begin(), trigger_offsets_.end());
  const int min_offset = *min_max_offset.first;
  const int ref_offset = *min_max_offset.second;
  const size_t diff_offset = ref_offset - min_offset;
  if (diff_offset >= _cfg->cp_size()) {
    for (size_t i = 0; i < trigger_offsets_.size(); i++) {
      auto* dev = bsRadios.at(0).at(i)->RawDev();
      const int delta = ref_offset - trigger_offsets_.at(i);
      std::printf("Sample adjusting delay of node %zu (offset %d) by %d\n", i,
                  trigger_offsets_.at(i), delta);
      const int iter = delta < 0 ? -delta : delta;
      for (int j = 0; j < iter; j++) {
        if (delta < 0) {
          dev->writeSetting("ADJUST_DELAYS", "-1");
        } else {
          dev->writeSetting("ADJUST_DELAYS", "1");
        }
      }
    }
  }
}

void BaseRadioSet::radioStart() {
  if (_cfg->is_houdini()) {
    // Both paths start the continuous BS RX stream here (they'd overflow if started
    // at ctor). Native TDD (bs_hw_framer): the armed framer GATES this continuous
    // stream every frame (the Iris model) -- houdiniTddRx reads it frame-by-frame,
    // no per-window arm/teardown. Software framer: plain continuous RX.
    activateHoudiniRx();
  } else if (!kUseSoapyUHD) {
    radioTrigger();
  }
}

void BaseRadioSet::readSensors() {
  for (size_t c = 0; c < _cfg->num_cells(); c++) {
    for (size_t i = 0; i < bsRadios.at(c).size(); i++) {
      auto* dev = bsRadios.at(c).at(i)->RawDev();
      std::cout << "TEMPs on Iris " << i << std::endl;
      std::cout << "ZYNQ_TEMP: " << dev->readSensor("ZYNQ_TEMP") << std::endl;
      std::cout << "LMS7_TEMP  : " << dev->readSensor("LMS7_TEMP") << std::endl;
      std::cout << "FE_TEMP  : " << dev->readSensor("FE_TEMP") << std::endl;
      std::cout << "TX0 TEMP  : " << dev->readSensor(SOAPY_SDR_TX, 0, "TEMP")
                << std::endl;
      std::cout << "TX1 TEMP  : " << dev->readSensor(SOAPY_SDR_TX, 1, "TEMP")
                << std::endl;
      std::cout << "RX0 TEMP  : " << dev->readSensor(SOAPY_SDR_RX, 0, "TEMP")
                << std::endl;
      std::cout << "RX1 TEMP  : " << dev->readSensor(SOAPY_SDR_RX, 1, "TEMP")
                << std::endl;
      std::cout << std::endl;
    }
  }
}

void BaseRadioSet::radioStop(void) {
  if (_cfg->is_houdini()) {
    // Native TDD teardown so the next run can re-arm. Full ladder, not abort
    // alone: abort latches gates_held on a running framer and skips TX_CLEAR
    // (3.2 + 4.24 in DEMO_VERIFICATION.md).
    for (size_t c = 0; c < bsRadios.size(); c++)
      for (size_t i = 0; i < bsRadios.at(c).size(); i++)
        if (bsRadios.at(c).at(i) != nullptr) {
          try {
            houdiniTddLadder(bsRadios.at(c).at(i)->RawDev());
          } catch (...) {
          }
        }
    return;
  }
  std::string tddConfStr = "{\"tdd_enabled\":false}";
  for (size_t c = 0; c < _cfg->num_cells(); c++) {
    for (size_t i = 0; i < bsRadios.at(c).size(); i++) {
      if (!kUseSoapyUHD) {
        auto* dev = bsRadios.at(c).at(i)->RawDev();
        dev->writeSetting("TDD_CONFIG", tddConfStr);
        dev->writeSetting("TDD_MODE", "false");
      }
      bsRadios.at(c)[i]->reset_DATA_clk_domain();
    }
  }
}

void BaseRadioSet::radioTx(const void* const* buffs) {
  long long frameTime(0);
  for (size_t c = 0; c < _cfg->num_cells(); c++) {
    for (size_t i = 0; i < bsRadios.at(c).size(); i++) {
      bsRadios.at(c).at(i)->xmit(buffs, _cfg->samps_per_slot(), 0, frameTime);
    }
  }
}

int BaseRadioSet::radioTx(size_t radio_id, size_t cell_id,
                          const void* const* buffs, int flags,
                          long long& frameTime) {
  if (_cfg->is_houdini()) {
    // The Houdini BS beacon is a free-running device replay armed at init, so
    // loopRecv's per-frame software beacon TX (baseTxBeacon) is a no-op here --
    // writing to the replay-mode TX stream would corrupt the loaded beacon.
    // Report the full slot as "sent" so baseTxBeacon doesn't log BAD Transmit.
    (void)radio_id; (void)cell_id; (void)buffs; (void)flags; (void)frameTime;
    return static_cast<int>(_cfg->samps_per_slot());
  }
  int w;
  // for UHD device xmit from host using frameTimeNs
  if (!kUseSoapyUHD) {
    w = bsRadios.at(cell_id).at(radio_id)->xmit(buffs, _cfg->samps_per_slot(),
                                                flags, frameTime);
  } else {
    long long frameTimeNs = SoapySDR::ticksToTimeNs(frameTime, _cfg->rate());
    w = bsRadios.at(cell_id).at(radio_id)->xmit(buffs, _cfg->samps_per_slot(),
                                                flags, frameTimeNs);
  }
  if (kDebugRadio) {
    std::cout << "cell " << cell_id << " radio " << radio_id << " tx returned "
              << w << std::endl;
  }
  return w;
}

void BaseRadioSet::radioRx(void* const* buffs) {
  long long frameTime(0);
  for (size_t c = 0; c < _cfg->num_cells(); c++) {
    for (size_t i = 0; i < bsRadios.at(c).size(); i++) {
      void* const* buff = buffs + (i * 2);
      bsRadios.at(c).at(i)->recv(buff, _cfg->samps_per_slot(), frameTime);
    }
  }
}

size_t BaseRadioSet::lastRxPadSamples(size_t radio_id, size_t cell_id) const {
  // Native-TDD path: every slot of a frame is served from one cached read, so the
  // frame-level count is the honest answer for each of them. Other paths read per
  // slot, so the radio's own count is.
  if (_cfg->is_houdini() && _cfg->bs_hw_framer()) return htdd_frame_pad_;
  if (cell_id < bsRadios.size() && radio_id < bsRadios.at(cell_id).size())
    return bsRadios.at(cell_id).at(radio_id)->lastPadSamples();
  return 0;
}

int BaseRadioSet::radioRx(size_t radio_id, size_t cell_id, void* const* buffs,
                          long long& frameTime) {
  if (_cfg->is_houdini() && _cfg->bs_hw_framer()) {
    // Native-TDD gated receive: arm a timed window at the next rx slot and tag
    // frameTime = (frame<<32)|(slot<<16), which loopRecv's HW-framer true-path
    // decodes to record the real pilot slot (fixing the software-path zeros).
    return this->houdiniTddRx(radio_id, buffs, frameTime);
  }
  return this->radioRx(radio_id, cell_id, buffs, _cfg->samps_per_slot(),
                       frameTime);
}

int BaseRadioSet::radioRx(size_t radio_id, size_t cell_id, void* const* buffs,
                          int numSamps, long long& frameTime) {
  int ret = 0;

  if (radio_id < bsRadios.at(cell_id).size()) {
    long long frameTimeNs = 0;
    ret = bsRadios.at(cell_id).at(radio_id)->recv(buffs, numSamps, frameTimeNs);
    // for UHD device recv using ticks
    if (kUseSoapyUHD == false)
      frameTime = frameTimeNs;
    else
      frameTime = SoapySDR::timeNsToTicks(frameTimeNs, _cfg->rate());
    if (kDebugRadio) {
      if (ret != numSamps) {
        std::cout << "recv returned " << ret << " from radio " << radio_id
                  << ", in cell " << cell_id << ". Expected: " << numSamps
                  << std::endl;
      } else {
        std::cout << "radio " << radio_id << " in cell " << cell_id
                  << ". Received " << ret << " at " << frameTime << std::endl;
      }
    }
  } else {
    MLPD_WARN("Invalid radio id: %zu in cell %zu\n", radio_id, cell_id);
    ret = 0;
  }
  return ret;
}
