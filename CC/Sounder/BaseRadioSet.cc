/** @file BaseRadioSet.cc
  * @brief Defination file for the BaseRadioSet class.
  *
  * Copyright (c) 2018-2022, Rice University
  * RENEW OPEN SOURCE LICENSE: http://renew-wireless.org/license
  * ----------------------------------------------------------
  *  Initializes and Configures Radios in the massive-MIMO base station 
  * ----------------------------------------------------------
  */
#include <cerrno>
#include "include/BaseRadioSet.h"
#include "include/rx_gap_sink.h"

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
#include "include/HoudiniFramer.h"
#include "include/IrisFramer.h"
#include "include/Radio.h"
#include "include/comms-lib.h"
#include "include/logger.h"
#include "include/macros.h"
#include "include/node_version.h"
#include "include/utils.h"
#include "nlohmann/json.hpp"

using json = nlohmann::json;

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
      if (bsRadios.at(c).at(i) == nullptr) {
        radioNotFound = true;
        radio_serial_not_found.push_back(_cfg->bs_sdr_ids().at(c).at(i));
        while (num_radios != 0 && bsRadios.at(c).at(num_radios - 1) == nullptr) {
          --num_radios;
          bsRadios.at(c).pop_back();
        }
        if (i < num_radios) {
          bsRadios.at(c).at(i) = std::move(bsRadios.at(c).at(--num_radios));
          bsRadios.at(c).pop_back();
        }
      }
    }
    bsRadios.at(c).shrink_to_fit();
    const size_t requested_radios = _cfg->n_bs_sdrs().at(c);
    _cfg->n_bs_sdrs().at(c) = num_radios;
    // A cell that LISTS radios and opens none is never success: fail loudly
    // rather than let the framer's start() iterate an empty vector and spin on
    // a dead stream (observed on the bench before this guard existed). A cell
    // configured with no radios proceeds, as it always did.
    if (bsRadios.at(c).empty() && requested_radios == 0) {
      // Not a failure (master proceeded), but on Houdini it means no beacon
      // is armed from this cell, which is worth one line.
      MLPD_WARN("cell %zu lists no base station radios: nothing armed from it\n", c);
    }
    if (bsRadios.at(c).empty() && requested_radios > 0) {
      radioNotFound = true;
      radio_serial_not_found.push_back(
          "(cell " + std::to_string(c) +
          ": no base station radios were constructed)");
    }
    if (radioNotFound == true) {
      // The count write-back above mutates the SHARED Config, which main()
      // builds ONCE and reuses across its retry loop. Restore the requested
      // topology so the in-process retry starts clean instead of constructing
      // nothing and failing with a misleading zero-radio message (observed
      // 2026-08-30: a transient server wedge on attempt 1 doomed attempt 2
      // before it touched a radio). The destructor iterates the vector, not
      // this count, so the restore cannot over-delete.
      _cfg->n_bs_sdrs().at(c) = requested_radios;
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

    for (size_t i = 0; i < bsRadios.at(c).size(); i++) {
      bsRadios.at(c).at(i)->printSettings();
    }
    // Measure Sync Delays now: a trigger-block operation, so only a backend
    // with the hardware trigger (Iris). (An Iris cell that lists no radios
    // used to reach an out-of-range index here; it is skipped.)
    if (!bsRadios.at(c).empty() && bsRadios.at(c).front()->hasHardwareTrigger()) {
      ensureFramer();
      static_cast<IrisFramer*>(framer_.get())->syncDelays(c);
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
  } else {
    if (calibrate_proc && _cfg->sample_cal_en() == true) {
      ensureFramer();
      this->syncTimeOffset();
      return;
    }
    ensureFramer();
    framer_->arm();
  }
}

void BaseRadioSet::ensureFramer() {
  if (framer_ != nullptr) return;
  // The platform is the radios' fact; with no radio constructed there is
  // nothing to frame, and the Iris framer is the historical default.
  houdini::sync::Platform platform = houdini::sync::Platform::kIrisUhd;
  for (const auto& cell : bsRadios)
    for (const auto& r : cell)
      if (r != nullptr) platform = r->platform();
  if (platform == houdini::sync::Platform::kHoudini) {
    framer_ = std::make_unique<HoudiniFramer>(_cfg, bsRadios);
  } else {
    framer_ = std::make_unique<IrisFramer>(_cfg, bsRadios, hubs, trigger_offsets_);
  }
}

BaseRadioSet::~BaseRadioSet(void) {
  if (!_cfg->hub_ids().empty()) {
    for (unsigned int i = 0; i < hubs.size(); i++)
      SoapySDR::Device::unmake(hubs.at(i));
  }
  // Iterate the vector, not the config count: on a failed attempt the config
  // count is restored to the REQUESTED topology (see the strip logic in the
  // ctor) while the vector holds only the radios actually constructed.
  for (unsigned int c = 0; c < _cfg->num_cells(); c++)
    for (size_t i = 0; i < bsRadios.at(c).size(); i++)
      bsRadios.at(c).at(i).reset();
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

  // What this radio needs to open, as a value (the radio never sees Config);
  // the factory is the only place that knows which backend a type is.
  RadioParams p;
  p.id = _cfg->bs_sdr_ids().at(c).at(i);
  p.label = "BS " + p.id;
  p.remote_port = _cfg->remote_port();
  p.channels = Utils::strToChannels(_cfg->bs_channel());
  p.rate_hz = _cfg->rate();
  p.nco_hz = _cfg->nco();
  p.rf_freq_hz = _cfg->radio_rf_freq();
  p.bw_filter_hz = _cfg->bw_filter();
  p.single_gain = _cfg->single_gain();
  // Houdini BS: the RX host port 10002 (the FPGA egresses ch1 there; the BS
  // and UE are on different interface IPs, so both can bind it), and the
  // beacon is device BRAM replay (tx_mode=replay).
  p.rx_local_port = 10002;
  p.tx_mode = "replay";
  const Radio::Type type = radioTypeFor(*_cfg);
  try {
    bsRadios.at(c).at(i) = Radio::create(type, p);
  } catch (std::runtime_error& err) {
    // Name the radio by what it actually is, and SAY WHY it was dropped. This
    // used to print "Ignoring iris <addr>" (upstream RENEWLab hardware we do not
    // run) and throw err.what() away, so a base station that failed to open gave
    // only its address before surfacing as "serials were not discovered in the
    // network" -- which sends you to check discovery when the real cause was in
    // the exception all along (a missing data-plane route, a busy stream). The
    // client path already logs its reason; this matches it.
    std::cerr << "Ignoring " << Radio::name(type) << " " << p.id << ": " << err.what()
              << std::endl;
    bsRadios.at(c).at(i).reset();
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
  for (auto ch : channels) {
    double rxgain = _cfg->rx_gain().at(ch);
    double txgain = _cfg->tx_gain().at(ch);
    bsRadios.at(c).at(i)->setup(ch, rxgain, txgain);
  }

  assert(thread_count->load() != 0);
  thread_count->store(thread_count->load() - 1);
}

void BaseRadioSet::radioTrigger(void) {
  // The calibration procedures trigger through the framer; a platform
  // without a trigger block has nothing to do.
  auto* iris = dynamic_cast<IrisFramer*>(framer_.get());
  if (iris != nullptr) iris->trigger();
}

void BaseRadioSet::radioStart() {
  if (framer_ != nullptr) framer_->start();
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
  if (framer_ != nullptr) framer_->stop();
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
  // The per-frame beacon transmission is the framer's: a software TX on
  // Iris, a no-op that reports the slot as sent on Houdini (device replay).
  return framer_->txBeacon(radio_id, cell_id, buffs, flags, frameTime);
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
  if (framer_ != nullptr && framer_->gatesRx()) return framer_->framePad();
  if (cell_id < bsRadios.size() && radio_id < bsRadios.at(cell_id).size())
    return bsRadios.at(cell_id).at(radio_id)->lastPadSamples();
  return 0;
}

int BaseRadioSet::radioRx(size_t radio_id, size_t cell_id, void* const* buffs,
                          long long& frameTime) {
  if (framer_ != nullptr && framer_->gatesRx()) {
    // A gating framer serves the slot itself, tagged (frame<<32)|(slot<<16),
    // which loopRecv's HW-framer true-path decodes to record the real slot.
    return framer_->rx(radio_id, buffs, frameTime);
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
