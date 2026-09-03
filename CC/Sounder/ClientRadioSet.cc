/** @file ClientRadioSet.cc
  * @brief Defination file for the ClientRadioSet class.
  *
  * Copyright (c) 2018-2022, Rice University
  * RENEW OPEN SOURCE LICENSE: http://renew-wireless.org/license
  * ----------------------------------------------------------
  * Initializes and Configures Client Radios 
  * ----------------------------------------------------------
  */
#include <atomic>
#include <cstdlib>
#include "include/ClientRadioSet.h"

#include "SoapySDR/Errors.hpp"
#include "SoapySDR/Formats.hpp"
#include "SoapySDR/Time.hpp"
#include "SoapySDR/Device.hpp"
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

static void initAGC(SoapySDR::Device* dev, Config* cfg);


// Deliberate UE carrier detune for CFO-estimator validation (AP-33/AP-34).
// Both boards share a 10 MHz reference, so there is no natural CFO to measure
// against; HOUDINI_UE_RX_FREQ_OFFSET_HZ imposes a KNOWN one on the UE receive
// path only -- pure carrier offset, no sample-timing drift, so the beacon
// estimator can be checked for sign and scale against a truth it cannot infer.
// HOUDINI_UE_TX_FREQ_OFFSET_HZ detunes the UE transmit path instead, which is
// what the BS then sees. Both default to 0 = nominal.
static double envFreqOffsetHz(const char* name) {
  const char* v = std::getenv(name);
  return (v != nullptr) ? std::strtod(v, nullptr) : 0.0;
}
static double ueRxFreqOffsetHz(void) {
  static const double v = envFreqOffsetHz("HOUDINI_UE_RX_FREQ_OFFSET_HZ");
  return v;
}
static double ueTxFreqOffsetHz(void) {
  static const double v = envFreqOffsetHz("HOUDINI_UE_TX_FREQ_OFFSET_HZ");
  return v;
}

ClientRadioSet::ClientRadioSet(Config* cfg) : _cfg(cfg) {
  size_t num_radios = _cfg->num_cl_sdrs();

  //load channels
  auto channels = Utils::strToChannels(_cfg->cl_channel());
  radios.clear();
  radios.resize(num_radios);
  radioNotFound = false;
  std::vector<std::string> radioSerialNotFound;
  std::atomic_ulong thread_count = ATOMIC_VAR_INIT(num_radios);
  for (size_t i = 0; i < num_radios; i++) {
    ClientRadioContext* context = new ClientRadioContext;
    context->crs = this;
    context->thread_count = &thread_count;
    context->tid = i;
#ifdef THREADED_INIT
    pthread_t init_thread_;

    if (pthread_create(&init_thread_, NULL, ClientRadioSet::init_launch,
                       context) != 0) {
      delete context;
      throw std::runtime_error("ClientRadioSet - init thread create failed");
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
    if (radios.at(i) == nullptr) {
      radioNotFound = true;
      radioSerialNotFound.push_back(_cfg->cl_sdr_ids().at(i));
      while (num_radios != 0 && radios.at(num_radios - 1) == nullptr) {
        --num_radios;
        radios.pop_back();
      }
      if (i < num_radios) {
        radios.at(i) = std::move(radios.at(--num_radios));
        radios.pop_back();
      }
    }
  }
  radios.shrink_to_fit();

  for (size_t i = 0; i < radios.size(); i++) {
    radios.at(i)->printSettings();
  }
  if (radioNotFound == true) {
    for (auto st = radioSerialNotFound.begin(); st != radioSerialNotFound.end();
         st++)
      std::cout << "\033[1;31m" << *st << "\033[0m" << std::endl;
    std::cout << "\033[1;31mERROR: the above client radio(s) could not be "
                 "opened. The reason is logged above each address; note that "
                 "a radio can be discoverable (SoapySDRUtil --find) and still "
                 "fail here, e.g. no route to its data-plane address."
                 "\033[0m"
              << std::endl;
  } else {
    //beaconSize + 82 (BS FE delay) + 68 (path delay) + 17 (correlator delay) + 82 (Client FE Delay)
    for (size_t i = 0; i < radios.size(); i++) {
      const int clTrigOffset = _cfg->beacon_size() + _cfg->tx_advance(i);
      const int sf_start = clTrigOffset / _cfg->samps_per_slot();
      const int sp_start = clTrigOffset % _cfg->samps_per_slot();

      auto* dev = radios.at(i)->RawDev();

      // hw_frame is only for Iris
      if (_cfg->hw_framer() == true) {
        // hw corr block bitshifts the corr outout, hence the log2
        int corr_scale = std::max(0, int(std::log2(_cfg->corr_scale(i))));
        std::string corrConfString =
            "{\"corr_enabled\":true,\"corr_threshold\":" + std::to_string(1) +
            ",\"corr_scale\":" + std::to_string(corr_scale) + "}";
        dev->writeSetting("CORR_CONFIG", corrConfString);
        dev->writeRegisters("CORR_COE", 0, _cfg->coeffs());

        std::string tpcStr = _cfg->cl_power_ramp() ? "true" : "false";
        std::string tpcConfString =
            "{\"tpc_enabled\":" + tpcStr +
            ",\"min_gain\":" + std::to_string(_cfg->cl_power_ramp_lo()) +
            ",\"max_gain\":" + std::to_string(_cfg->cl_power_ramp_hi()) + "}";
        dev->writeSetting("TPC_CONFIG", tpcConfString);

        std::string tddSched = _cfg->cl_frames().at(i);
        for (size_t s = 0; s < _cfg->cl_frames().at(i).size(); s++) {
          char c = _cfg->cl_frames().at(i).at(s);
          if (c == 'U') {  // uplink data
            tddSched.replace(s, 1, "T");
          } else if (c == 'P') {  // user pilot data
            tddSched.replace(s, 1, "P");
          } else if (c == 'D') {  // downlink data
            tddSched.replace(s, 1, "R");
          } else if (c == 'N') {  // noise data
            tddSched.replace(s, 1, "G");
          } else {
            tddSched.replace(s, 1, "G");
          }
        }
        std::cout << "Client " << i << " schedule: " << tddSched << std::endl;
        nlohmann::json tddConf;
        tddConf["tdd_enabled"] = true;
        tddConf["frame_mode"] = _cfg->frame_mode();
        int max_frame_ =
            (int)(2.0 / ((_cfg->samps_per_slot() * _cfg->slot_per_frame()) /
                         _cfg->rate()));
        tddConf["max_frame"] =
            _cfg->frame_mode() == "free_running" ? 0 : max_frame_;
        //std::cout << "max_frames for client " << i << " is " << max_frame_ << std::endl;
        if (_cfg->cl_sdr_ch() == 2) tddConf["dual_pilot"] = true;
        tddConf["frames"] = json::array();
        tddConf["frames"].push_back(tddSched);
        tddConf["symbol_size"] = _cfg->samps_per_slot();
        std::string tddConfStr = tddConf.dump();

        dev->writeSetting("TDD_CONFIG", tddConfStr);

        dev->setHardwareTime(
            SoapySDR::ticksToTimeNs((sf_start << 16) | sp_start, _cfg->rate()),
            "TRIGGER");
        dev->writeSetting("TX_SW_DELAY",
                          "30");  // experimentally good value for dev front-end
        dev->writeSetting("TDD_MODE", "true");
        // write pilot to FPGA buffers
        for (char const& c : _cfg->cl_channel()) {
          std::string tx_ram = "TX_RAM_";
          dev->writeRegisters(tx_ram + c, 0, _cfg->pilot());
        }
        radios.at(i)->activateRecv();
        radios.at(i)->activateXmit();
        if (_cfg->frame_mode() == "free_running")
          dev->writeSetting("TRIGGER_GEN", "");
        else
          dev->writeSetting("CORR_START",
                            (_cfg->cl_channel() == "B") ? "B" : "A");
      } else {
        if (radios.at(i)->hasHardwareTrigger()) {
          dev->setHardwareTime(0, "TRIGGER");
          radios.at(i)->activateRecv();
          radios.at(i)->activateXmit();
          dev->writeSetting("TRIGGER_GEN", "");
        } else if (radios.at(i)->type() != Radio::Type::kSoapyUhd) {
          // No hardware trigger or correlator block (Houdini): software beacon
          // sync (the receiver's search) drives acquisition. Just start streams.
          radios.at(i)->activateRecv();
          radios.at(i)->activateXmit();
        } else {
          // For USRP clients always use the internal clock
          dev->setTimeSource("internal");
          dev->setClockSource("internal");
          dev->setHardwareTime(0, "UNKNOWN_PPS");
          radios.at(i)->activateRecv();
          radios.at(i)->activateXmit();
        }
      }
    }
    MLPD_INFO("%s done!\n", __func__);
  }
}

void* ClientRadioSet::init_launch(void* in_context) {
  ClientRadioContext* context =
      reinterpret_cast<ClientRadioContext*>(in_context);
  context->crs->init(context);
  return 0;
}

void ClientRadioSet::init(ClientRadioContext* context) {
  int i = context->tid;
  std::atomic_ulong* thread_count = context->thread_count;
  delete context;

  MLPD_TRACE("Deleting context for tid: %d\n", i);

  bool has_runtime_error(false);
  auto channels = Utils::strToChannels(_cfg->cl_channel());
  MLPD_TRACE("ClientRadioSet setting up radio: %d : %zu\n", (i + 1),
             _cfg->num_cl_sdrs());
  // What this radio needs to open, as a value (the radio never sees Config);
  // the factory is the only place that knows which backend a type is.
  RadioParams p;
  p.id = _cfg->cl_sdr_ids().at(i);
  p.label = "UE " + p.id;
  p.remote_port = _cfg->remote_port();
  p.channels = channels;
  p.rate_hz = _cfg->rate();
  p.nco_hz = _cfg->nco();
  p.rf_freq_hz = _cfg->radio_rf_freq();
  p.bw_filter_hz = _cfg->bw_filter();
  p.single_gain = _cfg->single_gain();
  p.rx_freq_offset_hz = ueRxFreqOffsetHz();
  p.tx_freq_offset_hz = ueTxFreqOffsetHz();
  // Houdini UE: one RX host port per radio; the UE feeds pilots live, so
  // host-fed streaming TX (SH-183); ue_tdd_pilot asks for the driver's TDD
  // tick anchor (SH-248/SH-301).
  p.rx_local_port = 10002 + i;
  p.tx_mode = "stream";
  p.tdd = _cfg->ue_tdd_pilot();
  const Radio::Type type = radioTypeFor(*_cfg);
  try {
    radios.at(i) = Radio::create(type, p);
  } catch (std::runtime_error& err) {
    has_runtime_error = true;
    MLPD_WARN("ClientRadioSet radio %d (%s, %s) setup failed: %s\n", i, p.id.c_str(),
              Radio::name(type), err.what());
    radios.at(i).reset();
  } catch (...) {
    MLPD_WARN("Unknown exception\n");
    radios.at(i).reset();
    throw;
  }
  if (has_runtime_error == false) {
    auto* dev = radios.at(i)->RawDev();
    SoapySDR::Kwargs info = dev->getHardwareInfo();

    for (auto ch : channels) {
      double rxgain = _cfg->cl_rxgain_vec().at(ch).at(
          i);  // w/CBRS 3.6GHz [0:105], 2.5GHZ [0:108]
      double txgain = _cfg->cl_txgain_vec().at(ch).at(
          i);  // w/CBRS 3.6GHz [0:105], 2.5GHZ [0:105]
      radios.at(i)->setup(ch, rxgain, txgain);
    }

    // The AGC block is an Iris feature (a capability, not a platform test).
    if (radios.at(i)->hasAgc()) {
      initAGC(dev, _cfg);
    }
  }
  MLPD_TRACE("ClientRadioSet: Init complete\n");
  assert(thread_count->load() != 0);
  thread_count->store(thread_count->load() - 1);
}

ClientRadioSet::~ClientRadioSet(void) { radios.clear(); }

void ClientRadioSet::radioStop(void) {
  if (_cfg->hw_framer()) {
    std::string corrConfStr = "{\"corr_enabled\":false}";
    std::string tddConfStr = "{\"tdd_enabled\":false}";
    for (auto& stop_radio : radios) {
      auto* dev = stop_radio->RawDev();
      dev->writeSetting("CORR_CONFIG", corrConfStr);
      const auto timeStamp =
          SoapySDR::timeNsToTicks(dev->getHardwareTime(""), _cfg->rate());
      std::cout << "device " << dev->getHardwareKey()
                << ": Frame=" << (timeStamp >> 32)
                << ", Symbol=" << ((timeStamp & 0xFFFFFFFF) >> 16) << std::endl;
      dev->writeSetting("TDD_CONFIG", tddConfStr);
      dev->writeSetting("TDD_MODE", "false");
      stop_radio->reset_DATA_clk_domain();
    }
  } else {
    for (auto& stop_radio : radios) {
      stop_radio->deactivateRecv();
      MLPD_TRACE("Stopping radio...\n");
    }
  }
}

int ClientRadioSet::triggers(int i) { return (radios.at(i)->getTriggers()); }

int ClientRadioSet::radioRx(size_t radio_id, void* const* buffs, int numSamps,
                            long long& frameTime) {
  if (radio_id < radios.size()) {
    int ret(0);
    if (_cfg->hw_framer()) {
      ret = radios.at(radio_id)->recv(buffs, numSamps, frameTime);
    } else {
      long long frameTimeNs(0);
      ret = radios.at(radio_id)->recv(buffs, numSamps, frameTimeNs);
      frameTime = SoapySDR::timeNsToTicks(frameTimeNs, _cfg->rate());
      if (kDebugRadio) {
        if (frameTimeNs < 2e9)
          std::cout << "client " << radio_id << " received " << ret << " at "
                    << frameTimeNs << std::endl;
      }
    }
    return ret;
  }
  std::cout << "invalid radio id " << radio_id << std::endl;
  return 0;
}

int ClientRadioSet::drainTxStatus(size_t radio_id) {
  if (radio_id >= radios.size() || radios.at(radio_id) == nullptr) return 0;
  return radios.at(radio_id)->drainTxStatus();
}

int ClientRadioSet::radioTx(size_t radio_id, const void* const* buffs,
                            int numSamps, int flags, long long& frameTime) {
  if (_cfg->hw_framer()) {
    return radios.at(radio_id)->xmit(buffs, numSamps, flags, frameTime);
  } else {
    // Houdini streaming pilot. With the `tdd=1` TX stream arg (set in init when
    // ue_tdd_pilot), the driver's TxTickAnchor accepts HAS_TIME starts on the
    // 3.125 us TDD window grid (SH-248/SH-301), so we snap the beacon-referenced
    // txTime to that grid -- fine enough to land in the BS rx_gate and, unlike
    // the whole-ms fallback, with NO 1 ms drift-cliff. ue_tx_advance_ticks is a
    // fine calibration (ticks) added before the snap. Non-TDD Houdini keeps the
    // whole-ms fallback.
    // The transmit time grid is the backend's (RadioHoudini snaps to its TDD
    // window grid after the tick advance; the others convert plainly).
    long long frameTimeNs = radios.at(radio_id)->txTimeNs(
        frameTime, _cfg->rate(), _cfg->ue_tdd_pilot(), _cfg->ue_tx_advance_ticks());
    const int r = radios.at(radio_id)->xmit(buffs, numSamps, flags, frameTimeNs);
    static const bool kTxDebug = std::getenv("HOUDINI_UE_TX_DEBUG") != nullptr;  // read once
    if (kTxDebug) {
      static std::atomic<int> c{0};
      if ((c.fetch_add(1) % 20) == 0) {
        try {
          const std::string b =
              radios.at(radio_id)->RawDev()->readSetting("TX_BANK_STATUS");
          const size_t p = b.find("ch1:");
          MLPD_INFO("UE TX dbg: xmit r=%d/%d txNs=%lld bank[%s]\n", r, numSamps,
                    frameTimeNs,
                    (p == std::string::npos ? b : b.substr(p, 70)).c_str());
        } catch (...) {
        }
      }
    }
    return r;
  }
}

static void initAGC(SoapySDR::Device* dev, Config* cfg) {
  /*
     * Initialize AGC parameters
     */
  json agcConf;
  agcConf["agc_enabled"] = cfg->cl_agc_en();
  agcConf["agc_gain_init"] = cfg->cl_agc_gain_init();
  std::string agcConfStr = agcConf.dump();

  dev->writeSetting("AGC_CONFIG", agcConfStr);
}
