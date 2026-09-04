/** @file HoudiniFramer.h
  * @brief The Houdini base-station framer: the beacon in the TX replay RAM,
  *        the native TDD ring with its strobe grid, the gated receive slots
  *        extracted from one continuous read per frame. Moved out of
  *        BaseRadioSet (seam step S2); the mechanics and their measured
  *        reasons are unchanged.
  *
  * RENEW OPEN SOURCE LICENSE: http://renew-wireless.org/license
*/
#ifndef HOUDINI_FRAMER_H_
#define HOUDINI_FRAMER_H_

#include <cstdint>
#include <functional>
#include <vector>

#include "BeaconFramer.h"
#include "SoapySDR/Device.hpp"

class HoudiniFramer : public BeaconFramer {
 public:
  HoudiniFramer(Config* cfg, Radios& radios) : BeaconFramer(cfg, radios) {}

  /// bs_hw_framer: the native TDD ring (beacon strobe on the B slot, rx on
  /// every slot); otherwise the free-running replay beacon.
  void arm() override;
  /// Start the continuous BS RX streams (they would overflow if started at
  /// construction); the armed framer gates them every frame.
  void start() override;
  /// The full teardown ladder on every radio (abort alone latches the gates
  /// and skips TX_CLEAR: DEMO_VERIFICATION 3.2 + 4.24).
  void stop() override;
  bool gatesRx() const override { return cfg_->bs_hw_framer(); }
  int rx(size_t radio_id, void* const* buffs, long long& frameTime) override;
  size_t framePad() const override { return htdd_frame_pad_; }
  /// The beacon is a device replay armed at arm(): the per-frame software
  /// beacon TX is a no-op that reports the full slot as sent (writing to the
  /// replay-mode stream would corrupt the loaded beacon).
  int txBeacon(size_t radio_id, size_t cell_id, const void* const* buffs, int flags,
               long long& frameTime) override;

  /// The replay RAM payload: the conjugated beacon core at the head of the
  /// 4096-deep RAM, peak-scaled to sync.beacon.tx_full_scale.
  void buildBeacon(std::vector<int16_t>& iq);
  /// The teardown ladder for one device.
  static void tddLadder(SoapySDR::Device* dev, bool skip_tx_clear);

 private:
  void armReplayBeacon();
  void armTdd();
  long long armTddOnce(SoapySDR::Device* dev, const std::function<void()>& resetup,
                       long long symbol_ticks, long long symbols_per_frame);

  // Native-TDD framer state (single-cell single-radio HIL for now).
  double htdd_tick_rate_ = 122.88e6;
  long long htdd_epoch_ = 0;         // TDD_ARM epoch (ticks)
  long long htdd_frame_ticks_ = 0;   // symbols_per_frame * symbol_ticks
  long long htdd_symbol_ticks_ = 0;  // one TDD symbol = one sounder slot
  std::vector<size_t> htdd_rx_slots_;  // sounder slots with rx_gate (P/U/N)
  size_t htdd_pilot_slot_ = 0;         // the 'P' sounder slot (CSI reference)
  size_t htdd_rx_cursor_ = 0;
  std::vector<int16_t> htdd_slot_cache_;  // extracted rx slots for the current frame
  long long htdd_cache_frame_ = 0;        // frame_id tag shared by a frame's slots
  long long htdd_last_win_tick_ = 0;
  long long htdd_frame_counter_ = 0;  // 0,1,2,... like the Iris framer's frame_id
  std::vector<int16_t> htdd_cap_buf_;  // reused generous rx capture
  // One continuous read yields EVERY rx slot of the frame, so a gap in that
  // read taints the whole frame. Held per frame and handed to each slot (AP-10).
  size_t htdd_frame_pad_ = 0;
  size_t htdd_quiet_streak_ = 0;  // consecutive presence-gated (no-pilot) frames
  bool htdd_quiet_warned_ = false;
};

#endif  // HOUDINI_FRAMER_H_
