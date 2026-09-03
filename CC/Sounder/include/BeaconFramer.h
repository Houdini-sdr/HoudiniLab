/** @file BeaconFramer.h
  * @brief What the base station does per frame, as one object per platform:
  *        arm the beacon and the frame schedule, start and stop it, and
  *        serve the receive slots the framer gates.
  *
  * docs/RADIO_PLATFORM_SEAM.md step S2. The Iris framer programs the TDD JSON,
  * the beacon RAM and the trigger; the Houdini framer loads the replay RAM,
  * arms the native TDD ring with its strobe grid, and extracts the gated
  * slots from a continuous read. BaseRadioSet owns one framer, picked from
  * the radios' platform, and asks it; it no longer asks which radio it holds.
  *
  * RENEW OPEN SOURCE LICENSE: http://renew-wireless.org/license
*/
#ifndef BEACON_FRAMER_H_
#define BEACON_FRAMER_H_

#include <cstddef>
#include <memory>
#include <vector>

#include "Radio.h"
#include "config.h"

class BeaconFramer {
 public:
  /// The radios a framer drives: [cell][radio], owned by the set.
  using Radios = std::vector<std::vector<std::unique_ptr<Radio>>>;

  /// The framer for a platform. Throws std::invalid_argument for a platform
  /// this build has no framer for.
  static std::unique_ptr<BeaconFramer> create(houdini::sync::Platform platform, Config* cfg,
                                              Radios& radios);
  virtual ~BeaconFramer() = default;

  /// Program the schedule and the beacon after the radios are configured
  /// (the base station constructor's last step). May throw.
  virtual void arm() = 0;
  /// Start the frame machinery (the trigger, or the continuous receive).
  virtual void start() = 0;
  /// Tear the framer down so the next run can re-arm.
  virtual void stop() = 0;

  /// True when this framer serves whole receive slots itself (rx()) rather
  /// than the caller reading the radio per slot.
  virtual bool gatesRx() const = 0;
  /// One gated receive slot into `buffs`, tagged (frame << 32 | slot << 16)
  /// in frameTime; the slot length in samples, 0 for a skipped frame, < 0
  /// to stop. Only when gatesRx().
  virtual int rx(size_t radio_id, void* const* buffs, long long& frameTime) = 0;
  /// Samples zero-padded into the frame the last rx() served (AP-10).
  virtual size_t framePad() const = 0;

  /// The per-frame beacon transmission the receive loop asks for. A framer
  /// whose beacon plays from the device (Houdini replay) reports the slot as
  /// sent and writes nothing.
  virtual int txBeacon(size_t radio_id, size_t cell_id, const void* const* buffs, int flags,
                       long long& frameTime) = 0;

 protected:
  BeaconFramer(Config* cfg, Radios& radios) : cfg_(cfg), radios_(radios) {}
  Config* cfg_;
  Radios& radios_;
};

#endif  // BEACON_FRAMER_H_
