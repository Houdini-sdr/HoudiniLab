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
    std::cout << "\033[1;31mERROR: the above base station serials were not "
                 "discovered in the network!\033[0m"
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
constexpr long long kTddGridTicks = 384;    // 3.125 us grid
constexpr long long kTddQuantumNs = 3125;
constexpr long long kTddSymTicks = 61440;   // 0.5 ms TDD symbol (comfortable
                                            // window >> samps_per_slot capture)
constexpr long long kTddArmMargin = 36864000;  // ~300 ms of ticks
inline long long tddNsOfTick(long long t) {
  return (t / kTddGridTicks) * kTddQuantumNs;  // t must be grid-aligned
}
}  // namespace

long long BaseRadioSet::houdiniArmTdd(SoapySDR::Device* dev,
                                      long long symbol_ticks,
                                      long long symbols_per_frame) {
  const std::string arm = "symbol_ticks=" + std::to_string(symbol_ticks) +
                          ",symbols_per_frame=" + std::to_string(symbols_per_frame) +
                          ",margin=" + std::to_string(kTddArmMargin);
  long long epoch = 0;
  bool accepted = false;
  for (int attempt = 0; attempt < 4 && !accepted; ++attempt) {
    dev->writeSetting("TDD_ARM", arm);
    std::stringstream ss(dev->readSetting("TDD_ARM"));
    std::string tok;
    while (ss >> tok) {
      const auto eq = tok.find('=');
      if (eq == std::string::npos) continue;
      const std::string k = tok.substr(0, eq), v = tok.substr(eq + 1);
      if (k == "epoch") epoch = std::stoll(v);
      if (k == "accepted") accepted = (v == "1");
    }
    if (!accepted) dev->writeSetting("TDD_CMD", "abort");
  }
  if (!accepted) throw std::runtime_error("Houdini TDD_ARM rejected");
  return epoch;
}

void BaseRadioSet::armHoudiniTdd(void) {
  // Comfortable 0.5 ms TDD symbol so the samps_per_slot (<= 4096) capture fits
  // well inside one rx_gate window (the pre-open guard would clip a capture that
  // filled the symbol). Beacon rides the framer's '6' replay-strobe symbol;
  // symbols 1..N are one rx_gate per sounder pilot slot. The capture is tagged
  // with the SOUNDER slot index so the recorder places it.
  //
  // NB (houdini_beacon_ab.py, .21->.22): the SAME beacon RAM scored by the
  // client's gold correlation gives 44.5 dB via the framer strobe vs only 10.2 dB
  // via a continuous activateXmit replay -- the strobe is SHARPER, not distorted
  // (an earlier "strobe distorted ~11 dB" note was the continuous mode mislabeled).
  // And a continuous replay can't coexist with the framer anyway: arming the framer
  // silences activateXmit (-33 dB) and any tx_gate schedule is arm-rejected without
  // a strobe. So the strobe is the only way to get beacon + rx_gate on one board.
  htdd_symbol_ticks_ = kTddSymTicks;
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
      dev->writeSetting("TDD_CMD", "abort");  // tear down any armed framer (E6)

      // The sounder pilot/uplink slots to receive (tag with these indices).
      const std::string& sched = _cfg->bs_array_frames().at(c).at(i);
      htdd_rx_slots_.clear();
      for (size_t s = 0; s < sched.size(); ++s) {
        const char ch = sched.at(s);
        if (ch == 'P' || ch == 'R' || ch == 'U' || ch == 'N')
          htdd_rx_slots_.push_back(s);
      }
      // TDD schedule: 1 beacon symbol + 1 rx_gate symbol per sounder pilot slot.
      const size_t spf_tdd = 1 + htdd_rx_slots_.size();
      std::string tdd(spf_tdd, '0');
      tdd[0] = '6';  // beacon strobe (+rx_gate, harmless)
      for (size_t k = 1; k < spf_tdd; ++k) tdd[k] = '2';  // rx_gate windows
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
      r->xmit(buffs, static_cast<int>(n_load), 0, t0);  // load replay RAM
      dev->writeSetting("TDD_SCHED", tdd);
      // loops=forever replays the full app-rate RAM back-to-back through the
      // whole beacon symbol (the TDD beacon path, HS-80 §11b). The RAM is one
      // ISOLATED beacon (beacon_size core + silence to 4096), so the symbol
      // carries that beacon every 4096 samples (~33 us) -- frequent enough that
      // the client's single-window find_beacon always catches one, sparse enough
      // that the trailing-energy threshold stays low (sharp 2-rep peak).
      dev->writeSetting("TDD_REPLAY_STROBE",
                        "ch" + std::to_string(tx_ch) +
                            ":len=" + std::to_string(n_load / 2) +
                            ",loops=forever,offs=" + std::to_string(kTddGridTicks));
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
  auto* dev = r->RawDev();

  // Round-robin over the pilot slots. TDD rx-gate symbol = 1 + cursor (symbol 0
  // is the beacon); the tag uses the SOUNDER slot index for the recorder.
  const size_t idx = htdd_rx_cursor_;
  const size_t tdd_sym = 1 + idx;
  const size_t sounder_slot = htdd_rx_slots_.at(idx);
  htdd_rx_cursor_ = (htdd_rx_cursor_ + 1) % htdd_rx_slots_.size();

  const long long now = static_cast<long long>(
      std::llround(static_cast<double>(dev->getHardwareTime("")) *
                   htdd_tick_rate_ / 1e9));
  (void)tdd_sym;  // rx_gate is armed at the LOCATED pilot offset, not a fixed symbol

  if (getenv("HOUDINI_RX_DEBUG") != nullptr) {
    static std::atomic<int> bc{0};
    if ((bc.fetch_add(1) % 40) == 0) {
      try {
        MLPD_INFO("Houdini BS TX bank: %s\n",
                  dev->readSetting("TX_BANK_STATUS").c_str());
      } catch (...) {
      }
    }
  }
  // BS pilot AUTO-LOCATE. The UE pilot is within-run STABLE (frequency-locked
  // clocks + anchored client timing), but its absolute frame position varies
  // run-to-run -- the client's first sync locks to one of the dense beacon's many
  // copies (the beacon fills slots 0..14), so a fixed rx_gate at symbol 1 can't
  // catch it. Instead LOCATE the pilot once (scan a whole frame -- both '6' and '2'
  // gate RX so the frame is fully receivable) and then arm the rx_gate at the
  // pilot's ACTUAL offset for the rest of the run. The BS tracks the UE, immune to
  // the run-to-run anchor; HOUDINI_TDD_SCAN just adds verbose logging.
  static std::atomic<long long> pilot_offset{-1};
  long long loc = pilot_offset.load();
  if (loc < 0) {
    const int scan_n = static_cast<int>(htdd_frame_ticks_);  // one full frame
    const long long slead =
        std::max((now - htdd_epoch_) / htdd_frame_ticks_ + 3, 3LL);
    const long long swt = htdd_epoch_ + slead * htdd_frame_ticks_;  // frame origin
    std::vector<int16_t> scan(static_cast<size_t>(scan_n) * 2, 0);
    void* sb[1] = {scan.data()};
    const int sg = r->recvTddWindow(sb, scan_n, tddNsOfTick(swt));
    double best_e = 0.0;
    int at = 0;
    if (sg > 4096) {
      std::vector<double> cs(static_cast<size_t>(sg) + 1, 0.0);
      for (int i = 0; i < sg; ++i) {
        const double re = scan[2 * i], im = scan[2 * i + 1];
        cs[i + 1] = cs[i] + re * re + im * im;
      }
      for (int s = 0; s + 4096 <= sg; s += 128) {
        const double e = (cs[s + 4096] - cs[s]) / 4096.0;
        if (e > best_e) { best_e = e; at = s; }
      }
      const double peak_rms = std::sqrt(best_e);
      const double mean_rms = std::sqrt(cs[sg] / sg);
      if (getenv("HOUDINI_TDD_SCAN") != nullptr) {
        MLPD_INFO("Houdini BS pilot-scan: got=%d peak-rms=%.0f@%d mean-rms=%.0f\n",
                  sg, peak_rms, at, mean_rms);
      }
      if (peak_rms > 120.0 && peak_rms > 4.0 * mean_rms) {
        loc = at;
        MLPD_INFO("Houdini BS pilot LOCATED at frame-offset %lld (peak-rms %.0f, "
                  "mean %.0f)\n",
                  loc, peak_rms, mean_rms);
        if (getenv("HOUDINI_TDD_SCAN") != nullptr) {
          FILE* f = std::fopen("/tmp/bs_scan.bin", "wb");
          if (f) {
            std::fwrite(scan.data(), sizeof(int16_t),
                        static_cast<size_t>(sg) * 2, f);
            std::fclose(f);
          }
        }
        // Self-calibrate the scan-vs-window pipeline offset. The whole-frame locate
        // scan and a short gated rx window deliver samples with DIFFERENT fixed
        // offsets (~2k samples), so a window armed at the scan index lands the pilot
        // late and truncates it. Iterate REAL-size (4096) trial captures, each time
        // centering the pilot energy centroid at the window center (== the
        // transmitted [prefix][3840 energy][postfix] structure, ~128-samp margins).
        // Centroid is unbiased (no edge threshold bias) and converges through
        // truncation; same-size captures each round keep it consistent with the run.
        const int n = static_cast<int>(_cfg->samps_per_slot());
        const long long center = n / 2;
        for (int it = 0; it < 5; ++it) {
          const long long nowc = static_cast<long long>(
              std::llround(static_cast<double>(dev->getHardwareTime("")) *
                           htdd_tick_rate_ / 1e9));
          const long long kc =
              std::max((nowc - htdd_epoch_) / htdd_frame_ticks_ + 2, 1LL);
          const long long wtc = htdd_epoch_ + kc * htdd_frame_ticks_ + loc;
          std::vector<int16_t> cal(static_cast<size_t>(n) * 2, 0);
          void* cb[1] = {cal.data()};
          const int cg = r->recvTddWindow(cb, n, tddNsOfTick(wtc));
          if (cg <= 256) break;
          // per-sample energy prefix sum -> CENTERED 256-sample power (unbiased,
          // unlike a right-aligned window). Peak, then the geometric centroid of
          // the hot region (>15% of peak) -- flat-topped pilot => sharp mask =>
          // centroid == the pilot's true center, robust to the noise floor.
          std::vector<double> cse(static_cast<size_t>(cg) + 1, 0.0);
          for (int i = 0; i < cg; ++i) {
            const double re = cal[2 * i], im = cal[2 * i + 1];
            cse[i + 1] = cse[i] + re * re + im * im;
          }
          double peak = 0.0;
          for (int i = 128; i + 128 <= cg; ++i) {
            const double m = (cse[i + 128] - cse[i - 128]) / 256.0;
            if (m > peak) peak = m;
          }
          const double thr = 0.15 * peak;
          long long cnt = 0;
          double isum = 0.0;
          for (int i = 128; i + 128 <= cg; ++i) {
            const double m = (cse[i + 128] - cse[i - 128]) / 256.0;
            if (m > thr) { ++cnt; isum += i; }
          }
          if (cnt == 0) break;
          const long long centroid = std::llround(isum / static_cast<double>(cnt));
          const long long adj = centroid - center;
          if (getenv("HOUDINI_TDD_SCAN") != nullptr)
            MLPD_INFO("Houdini BS pilot calib it=%d loc=%lld centroid=%lld adj=%lld\n",
                      it, loc, centroid, adj);
          loc += adj;
          if (loc < 0) loc += htdd_frame_ticks_;
          if (std::llabs(adj) <= 24) break;  // centered
        }
        MLPD_INFO("Houdini BS pilot CALIBRATED: loc=%lld (from at=%d)\n", loc, at);
        pilot_offset.store(loc);
      }
    }
    if (loc < 0) {
      // Not located yet: return the frame head as a dummy so loopRecv keeps
      // running and we scan again next call (fast -- no timing-out capture).
      const int n = static_cast<int>(_cfg->samps_per_slot());
      std::memcpy(buffs[0], scan.data(), static_cast<size_t>(n) * 4);
      const long long fid = htdd_frame_counter_;
      if (htdd_rx_cursor_ == 0) ++htdd_frame_counter_;
      frameTime = (fid << 32) | (static_cast<long long>(sounder_slot) << 16);
      return n;
    }
  }
  // Arm the rx_gate at the located pilot offset, a couple frames ahead + monotonic.
  // Re-read the clock: the whole-frame locate scan (when it runs) consumes >1 frame
  // of wall time, so the top-of-function `now` is stale by then -- using it here
  // arms the window in the past (activateStream TIME_ERROR).
  const long long now2 = static_cast<long long>(
      std::llround(static_cast<double>(dev->getHardwareTime("")) *
                   htdd_tick_rate_ / 1e9));
  long long k = std::max((now2 - htdd_epoch_) / htdd_frame_ticks_ + 2, 1LL);
  long long wt = htdd_epoch_ + k * htdd_frame_ticks_ + loc;
  while (wt <= htdd_last_win_tick_) wt += htdd_frame_ticks_;
  htdd_last_win_tick_ = wt;

  const int n = static_cast<int>(_cfg->samps_per_slot());
  const int got = r->recvTddWindow(buffs, n, tddNsOfTick(wt));
  if (getenv("HOUDINI_BS_RX_DEBUG") != nullptr && got > 0) {
    // Does the located rx_gate window actually LAND the pilot in the recorder
    // buffer? RMS staying high => locked + seated; RMS decaying over the run =>
    // pilot walks out of the 4096 window (clocks not frequency-locked).
    const int16_t* s = reinterpret_cast<const int16_t*>(buffs[0]);
    double e = 0.0;
    int16_t amx = 0;
    for (int i = 0; i < got; ++i) {
      const int16_t re = s[2 * i], im = s[2 * i + 1];
      e += static_cast<double>(re) * re + static_cast<double>(im) * im;
      const int16_t a = static_cast<int16_t>(std::abs(re));
      const int16_t b = static_cast<int16_t>(std::abs(im));
      if (a > amx) amx = a;
      if (b > amx) amx = b;
    }
    const long long now3 = static_cast<long long>(
        std::llround(static_cast<double>(dev->getHardwareTime("")) *
                     htdd_tick_rate_ / 1e9));
    MLPD_INFO("HOUDINI_BS_RX: frame=%lld loc=%lld wt=%lld got=%d rms=%.0f absmax=%d "
              "recvTicks=%lld (=%.1fms) armLead=%lld\n",
              htdd_frame_counter_, loc, wt, got, std::sqrt(e / got),
              static_cast<int>(amx), now3 - now2,
              static_cast<double>(now3 - now2) / htdd_tick_rate_ * 1e3, wt - now2);
  }
  // frame_id must be a small monotonic counter (0,1,2,...) like the Iris HW
  // framer -- the recorder EXTENDS its HDF5 dataset to frame_id, so an absolute
  // tick-derived frame number would blow it up. Advance once per frame (after
  // the last rx slot of the frame is served).
  const long long frame_id = htdd_frame_counter_;
  if (htdd_rx_cursor_ == 0) ++htdd_frame_counter_;
  // Tag like the Iris HW framer so the unmodified loopRecv true-path decodes it.
  frameTime = (frame_id << 32) | (static_cast<long long>(sounder_slot) << 16);
  return got;
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
    if (kUseSoapyUHD == false) {
      std::cerr << "Ignoring iris " << _cfg->bs_sdr_ids().at(c).at(i)
                << std::endl;
    } else {
      std::cerr << "Ignoring uhd device " << _cfg->bs_sdr_ids().at(c).at(i)
                << std::endl;
    }
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
    // Native TDD (bs_hw_framer): the framer is armed and RX windows are armed
    // per-recv (houdiniTddRx), so nothing to start here. Software-framer path:
    // start the continuous BS RX streams (they'd overflow if started at ctor).
    if (!_cfg->bs_hw_framer()) activateHoudiniRx();
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
    // Native TDD teardown so the next run can re-arm (an armed framer refuses
    // TDD_SCHED fills, E6). No Iris TDD_CONFIG / RESET_DATA_LOGIC on Houdini.
    for (size_t c = 0; c < bsRadios.size(); c++)
      for (size_t i = 0; i < bsRadios.at(c).size(); i++)
        if (bsRadios.at(c).at(i) != nullptr) {
          try {
            bsRadios.at(c).at(i)->RawDev()->writeSetting("TDD_CMD", "abort");
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
