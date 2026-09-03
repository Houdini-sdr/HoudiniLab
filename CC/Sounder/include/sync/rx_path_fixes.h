/**
 * @file sync/rx_path_fixes.h
 * @brief What a platform's receive path does to the samples that the
 *        consumers must undo, in one place: the Houdini RFSoC's matched-NCO
 *        R2C mixer delivers baseband CONJUGATED (AP-30 measured the sign),
 *        and its CSI needs the timing and phase fixes the dashboard ledger
 *        records (DEMO_VERIFICATION 4.x). Iris and UHD deliver baseband as
 *        is.
 *
 * Seam step S4 (docs/RADIO_PLATFORM_SEAM.md): the receiver's carrier
 * estimator and the recorder's CSI path both read this instead of asking
 * the configuration which radio it holds.
 */
#pragma once

#include "sync/sync_config.h"

namespace houdini {
namespace sync {

struct RxPathFixes {
  bool conjugate = false;   ///< undo the mixer's spectral inversion
  bool csi_timing = false;  ///< the CSI timing fix (recorder_worker)
  bool csi_phase = false;   ///< the CSI phase fix (recorder_worker)

  static RxPathFixes forPlatform(Platform p) {
    RxPathFixes f;
    if (p == Platform::kHoudini) {
      f.conjugate = true;
      f.csi_timing = true;
      f.csi_phase = true;
    }
    return f;
  }
};

}  // namespace sync
}  // namespace houdini
