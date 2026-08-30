/**
 * @file BaseRadioSet.h
 * @brief Declaration file for the BaseRadioSet class.
 */
#ifndef BASE_RADIO_SET_H_
#define BASE_RADIO_SET_H_

#include <atomic>
#include <cstddef>
#include <vector>

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

  void radioTrigger(void);
  // Houdini BS: load the Gold beacon into the beacon radio's TX replay RAM and
  // arm it free-running (the software-framer/Iris trigger path is bypassed).
  void armHoudiniBeacon(void);

  // Houdini native-TDD BS path (bs_hw_framer + radio_type=houdini): arm the FPGA
  // TDD framer -- beacon replay strobe @ slot0 + rx_gate on the pilot/uplink
  // slots -- so the BS transports ONLY those slots. houdiniTddRx arms a timed RX
  // window at the next rx slot and tags it (frame<<32|slot<<16) like the Iris HW
  // framer, so the unmodified loopRecv true-path records the real pilot slot.
  void buildHoudiniBeacon(std::vector<int16_t>& iq);  // -> replay RAM payload
  void armHoudiniTdd(void);
  static void houdiniTddLadder(SoapySDR::Device* dev);
  int houdiniTddRx(size_t radio_id, void* const* buffs, long long& frameTime);
  long long houdiniArmTdd(SoapySDR::Device* dev, long long symbol_ticks,
                          long long symbols_per_frame);  // returns epoch

 public:
  // Houdini BS: start the RX streams on demand (reverse link / UE pilots).
  void activateHoudiniRx(void);

 private:
  void sync_delays(size_t cellIdx);
  SoapySDR::Device* baseRadio(size_t cellId);
  int syncTimeOffset();
  void dciqCalibrationProc(size_t);
  void readSensors(void);

  Config* _cfg;
  std::vector<SoapySDR::Device*> hubs;
  std::vector<std::vector<Radio*>> bsRadios;  // [cell, iris]
  std::vector<int> trigger_offsets_;
  bool radioNotFound;

  // Houdini native-TDD framer state (single-cell single-radio HIL for now)
  double htdd_tick_rate_ = 122.88e6;
  long long htdd_epoch_ = 0;        // TDD_ARM epoch (ticks)
  long long htdd_frame_ticks_ = 0;  // symbols_per_frame * symbol_ticks
  // kTddSymTicks (61440 = 0.5 ms), NOT samps_per_slot. Measured on the running
  // demo: samps_per_slot is 4096, so one TDD symbol spans 15 sounder slots and
  // symbols_per_frame is 2 (TDD_SCHED "62"). The TX gate is therefore 15x coarser
  // than a slot, which is why the beacon strobe covers slots 0..14 rather than
  // slot 0 alone. An earlier comment here claimed this equalled samps_per_slot.
  long long htdd_symbol_ticks_ = 0;
  std::vector<size_t> htdd_rx_slots_;  // sounder slots with rx_gate (P/U/N)
  size_t htdd_pilot_slot_ = 0;         // the 'P' sounder slot (CSI reference)
  size_t htdd_rx_cursor_ = 0;
  std::vector<int16_t> htdd_slot_cache_;  // extracted rx slots for the current frame
  long long htdd_cache_frame_ = 0;        // frame_id tag shared by a frame's slots
  long long htdd_last_win_tick_ = 0;
  long long htdd_frame_counter_ = 0;  // 0,1,2,... like the Iris framer's frame_id
  std::vector<int16_t> htdd_cap_buf_;  // reused generous rx capture (extract 1 slot)
  // One continuous read at cursor 0 yields EVERY rx slot of the frame, so a gap in
  // that read taints the whole frame (pilot AND uplink data). Hold its padding count
  // for the frame and hand it to each slot served from the cache. See AP-10.
  size_t htdd_frame_pad_ = 0;
};

#endif  // BASE_RADIO_SET_H_
