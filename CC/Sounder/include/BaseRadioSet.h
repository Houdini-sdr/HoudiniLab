/**
 * @file BaseRadioSet.h
 * @brief Declaration file for the BaseRadioSet class.
 */
#ifndef BASE_RADIO_SET_H_
#define BASE_RADIO_SET_H_

#include <atomic>
#include <cstddef>
#include <functional>
#include <memory>
#include <vector>

#include "BeaconFramer.h"
#include "Radio.h"
#include "SoapySDR/Device.hpp"
#include "config.h"

class BaseRadioSet {
 public:
  BaseRadioSet(Config* cfg, const bool calibrate_proc);
  ~BaseRadioSet(void);
  void radioTx(const void* const* buffs);
  void radioRx(void* const* buffs);
  int radioTx(size_t radio_id, size_t cell_id, const void* const* buffs,
              int flags, long long& frameTime);
  int radioRx(size_t radio_id, size_t cell_id, void* const* buffs,
              long long& frameTime);
  int radioRx(size_t radio_id, size_t cell_id, void* const* buffs, int numSamps,
              long long& frameTime);
  // Samples zero-padded into the window backing the LAST radioRx (0 = clean). The
  // caller stamps it onto the Packet so downstream (notably the CSI/view path) can
  // tell a slot carrying inserted zeros from an all-real one. See AP-10.
  size_t lastRxPadSamples(size_t radio_id, size_t cell_id) const;
  void radioStart(void);
  void radioStop(void);
  bool getRadioNotFound() { return radioNotFound; }
  void adjustDelays(void);

 private:
  // use for create pthread
  struct BaseRadioContext {
    BaseRadioSet* brs;
    std::atomic_ulong* thread_count;
    size_t tid;
    size_t cell;
  };
  void init(BaseRadioContext* context);
  void configure(BaseRadioContext* context);

  static void* init_launch(void* in_context);
  static void* configure_launch(void* in_context);

  /// The trigger for the calibration procedures (an Iris framer operation;
  /// nothing on a platform without one).
  void radioTrigger(void);
  int syncTimeOffset();
  /// Build the platform's framer once the radios exist (idempotent).
  void ensureFramer();
  void dciqCalibrationProc(size_t);
  void readSensors(void);

  Config* _cfg;
  std::vector<SoapySDR::Device*> hubs;
  std::vector<std::vector<std::unique_ptr<Radio>>> bsRadios;  // [cell, radio]
  std::vector<int> trigger_offsets_;
  bool radioNotFound;
  // What the base station does per frame, one object per platform
  // (BeaconFramer.h): the Iris TDD block or the Houdini ring and its gated
  // receive. Created once the radios are configured; null in a calibration
  // run that ends before arming.
  std::unique_ptr<BeaconFramer> framer_;
};

#endif  // BASE_RADIO_SET_H_
