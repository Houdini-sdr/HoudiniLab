/** @file IrisFramer.cc
  * @brief The Iris (and Soapy-UHD) base-station framer, moved out of
  *        BaseRadioSet.cc (seam step S2) without change. Compile-only on this
  *        bench (build matrix); no Iris or UHD hardware here.
  *
  * Copyright (c) 2018-2022, Rice University
  * RENEW OPEN SOURCE LICENSE: http://renew-wireless.org/license
*/
#include "include/IrisFramer.h"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include "SoapySDR/Formats.hpp"
#include "SoapySDR/Time.hpp"
#include "include/comms-lib.h"
#include "include/logger.h"
#include "include/macros.h"
#include "include/utils.h"
#include "nlohmann/json.hpp"

using json = nlohmann::json;

void IrisFramer::arm() {
  // Local to the old constructor: the per-cell antenna count the beacon
  // weights are sized by.
  std::vector<size_t> num_bs_antenntas(cfg_->num_cells());
  for (size_t c = 0; c < cfg_->num_cells(); c++) {
    num_bs_antenntas[c] = radios_.at(c).size() * cfg_->bs_channel().length();
  }
    if (cfg_->sample_cal_en() == true) {
      const std::string filename = "files/iris_samp_offsets.dat";
      trigger_offsets_ = Utils::ReadVector(filename, false);
      size_t num_radios = cfg_->n_bs_sdrs()[0];
      if (trigger_offsets_.size() == num_radios) {
        adjustDelays();
      } else {
        std::printf(
            "The number of sample offsets in file does not match the number of "
            "radios.\n");
      }
    }

    nlohmann::json tddConf;
    tddConf["tdd_enabled"] = true;
    tddConf["frame_mode"] = "free_running";
    tddConf["max_frame"] = cfg_->max_frame();
    tddConf["symbol_size"] = cfg_->samps_per_slot();

    // write TDD schedule and beacons to FPFA buffers only for Iris
    for (size_t c = 0; c < cfg_->num_cells(); c++) {
      if (!kUseSoapyUHD) {
        size_t ndx = 0;
        for (size_t i = 0; i < radios_.at(c).size(); i++) {
          auto* dev = radios_.at(c).at(i)->RawDev();
          tddConf["frames"] = json::array();
          if (cfg_->internal_measurement() == true) {
            for (char const& bs_ch : cfg_->bs_channel()) {
              std::string tx_ram = "TX_RAM_";
              dev->writeRegisters(tx_ram + bs_ch, 0, cfg_->pilot());
            }
            tddConf["frames"].push_back(cfg_->bs_array_frames().at(c).at(i));
            std::cout << "Cell " << c << ", SDR " << i
                      << " calibration schedule : "
                      << cfg_->bs_array_frames().at(c).at(i) << std::endl;

          } else {
            tddConf["frames"] = json::array();

            const size_t frame_size =
                cfg_->bs_array_frames().at(c).at(i).size();
            std::string fw_frame = cfg_->bs_array_frames().at(c).at(i);

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
          if (cfg_->internal_measurement() == false ||
              cfg_->num_cl_antennas() > 0) {
            dev->writeRegisters("BEACON_RAM", 0, cfg_->beacon());
            std::string tx_ram_wgt = "BEACON_RAM_WGT_";
            for (char const& ch : cfg_->bs_channel()) {
              bool isBeaconAntenna =
                  !cfg_->beam_sweep() && ndx == cfg_->beacon_ant();
              std::vector<unsigned> beacon_weights(num_bs_antenntas[c],
                                                   isBeaconAntenna ? 1 : 0);
              if (cfg_->beam_sweep()) {
                for (size_t j = 0; j < num_bs_antenntas[c]; j++)
                  beacon_weights[j] = CommsLib::hadamard2(ndx, j);
              }
              dev->writeRegisters(tx_ram_wgt + ch, 0, beacon_weights);
              ++ndx;
            }

            dev->writeSetting("BEACON_START",
                              std::to_string(radios_.at(c).size()));
            tddConf["beacon_start"] = cfg_->prefix();
            tddConf["beacon_stop"] = cfg_->prefix() + cfg_->beacon_size();
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
        for (size_t i = 0; i < radios_.at(c).size(); i++) {
          auto* dev = radios_.at(c).at(i)->RawDev();
          radios_.at(c).at(i)->activateRecv();
          radios_.at(c).at(i)->activateXmit();
          dev->setHardwareTime(0, "TRIGGER");
        }
      } else {
        // Set freq and time source for multiple USRPs
        for (size_t i = 0; i < radios_.at(c).size(); i++) {
          auto* dev = radios_.at(c).at(i)->RawDev();
          dev->setClockSource("external");
          dev->setTimeSource("external");
          dev->setHardwareTime(0, "PPS");
        }
        // Wait for pps sync pulse
        std::this_thread::sleep_for(std::chrono::seconds(2));
        // Activate Rx and Tx streamers
        for (size_t i = 0; i < radios_.at(c).size(); i++) {
          radios_.at(c).at(i)->activateRecv();
          radios_.at(c).at(i)->activateXmit();
        }
      }
    }
  MLPD_INFO("BaseRadioSet done!\n");
}

SoapySDR::Device* IrisFramer::baseRadio(size_t cellId) {
  if (cellId < hubs_.size()) return (hubs_.at(cellId));
  if (cellId < radios_.size() && radios_.at(cellId).size() > 0)
    return radios_.at(cellId).at(0)->RawDev();
  return NULL;
}

void IrisFramer::syncDelays(size_t cellIdx) {
  /*
     * Compute Sync Delays
     */
  SoapySDR::Device* base = baseRadio(cellIdx);
  if (base != NULL) base->writeSetting("SYNC_DELAYS", "");
}

void IrisFramer::trigger() {
  for (size_t c = 0; c < cfg_->num_cells(); c++) {
    auto* base = baseRadio(c);
    if (base != NULL) {
      base->writeSetting("TRIGGER_GEN", "");
    }
  }
}

void IrisFramer::adjustDelays() {
  // adjust all trigger delay fwith respect to the max offset
  const auto min_max_offset =
      std::minmax_element(trigger_offsets_.begin(), trigger_offsets_.end());
  const int min_offset = *min_max_offset.first;
  const int ref_offset = *min_max_offset.second;
  const size_t diff_offset = ref_offset - min_offset;
  if (diff_offset >= cfg_->cp_size()) {
    for (size_t i = 0; i < trigger_offsets_.size(); i++) {
      auto* dev = radios_.at(0).at(i)->RawDev();
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


void IrisFramer::start() {
  // The trigger starts the Iris framer; Soapy UHD was started by its PPS
  // clock at arm().
  if (!kUseSoapyUHD) trigger();
}

void IrisFramer::stop() {
  std::string tddConfStr = "{\"tdd_enabled\":false}";
  for (size_t c = 0; c < cfg_->num_cells(); c++) {
    for (size_t i = 0; i < radios_.at(c).size(); i++) {
      if (!kUseSoapyUHD) {
        auto* dev = radios_.at(c).at(i)->RawDev();
        dev->writeSetting("TDD_CONFIG", tddConfStr);
        dev->writeSetting("TDD_MODE", "false");
      }
      radios_.at(c)[i]->reset_DATA_clk_domain();
    }
  }
}

int IrisFramer::txBeacon(size_t radio_id, size_t cell_id, const void* const* buffs, int flags,
                         long long& frameTime) {
  int w;
  // for UHD device xmit from host using frameTimeNs
  if (!kUseSoapyUHD) {
    w = radios_.at(cell_id).at(radio_id)->xmit(buffs, cfg_->samps_per_slot(), flags, frameTime);
  } else {
    long long frameTimeNs = SoapySDR::ticksToTimeNs(frameTime, cfg_->rate());
    w = radios_.at(cell_id).at(radio_id)->xmit(buffs, cfg_->samps_per_slot(), flags, frameTimeNs);
  }
  if (kDebugRadio) {
    std::cout << "cell " << cell_id << " radio " << radio_id << " tx returned " << w << std::endl;
  }
  return w;
}
