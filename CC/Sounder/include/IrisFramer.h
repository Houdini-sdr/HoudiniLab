/** @file IrisFramer.h
  * @brief The Iris (and Soapy-UHD) base-station framer: the TDD JSON, the
  *        beacon RAM and weights, the trigger, the sync delays. Moved out of
  *        BaseRadioSet (seam step S2) without change; this path has no
  *        hardware on this bench and is compile-only (build matrix).
  *
  * Copyright (c) 2018-2022, Rice University
  * RENEW OPEN SOURCE LICENSE: http://renew-wireless.org/license
*/
#ifndef IRIS_FRAMER_H_
#define IRIS_FRAMER_H_

#include <vector>

#include "BeaconFramer.h"
#include "SoapySDR/Device.hpp"

class IrisFramer : public BeaconFramer {
 public:
  IrisFramer(Config* cfg, Radios& radios, std::vector<SoapySDR::Device*>& hubs,
             std::vector<int>& trigger_offsets)
      : BeaconFramer(cfg, radios), hubs_(hubs), trigger_offsets_(trigger_offsets) {}

  /// The sample-offset calibration, the TDD schedule and beacon RAM on every
  /// radio, then the trigger time (Iris) or the PPS clock (Soapy UHD).
  void arm() override;
  /// The trigger (Iris); nothing for Soapy UHD, whose PPS started it.
  void start() override;
  /// TDD off and the data logic reset on every radio.
  void stop() override;
  bool gatesRx() const override { return false; }
  int rx(size_t, void* const*, long long&) override { return 0; }
  size_t framePad() const override { return 0; }
  /// Per-frame software beacon TX through the radio, in the time unit the
  /// backend wants.
  int txBeacon(size_t radio_id, size_t cell_id, const void* const* buffs, int flags,
               long long& frameTime) override;

  /// The hub or first radio of a cell, for the trigger and the sync delays.
  SoapySDR::Device* baseRadio(size_t cellId);
  void syncDelays(size_t cellIdx);
  void trigger();
  /// Adjust every radio's trigger delay against the largest measured offset.
  void adjustDelays();

 private:
  std::vector<SoapySDR::Device*>& hubs_;
  std::vector<int>& trigger_offsets_;
};

#endif  // IRIS_FRAMER_H_
