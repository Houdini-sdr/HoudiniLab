/**
 * @file sync/detector.h
 * @brief The beacon detector as one object: replica, threshold form, pick
 *        rule and the replica-tail index convention, resolved once.
 *
 * Wraps CommsLib::find_beacon_avx with the decisions receiver.cc used to make
 * at its call sites (DEMO_VERIFICATION 8.138 to 8.154): which threshold form a
 * replica supports, which crossing to return, how far the first-path search
 * looks back and how much weaker a path may be, and how far the beacon END
 * sits past the index the correlator reports. All of it is fixed at
 * construction from the shape and the configuration, so a run cannot mix two
 * populations and a test can build the same detector the receiver runs.
 */
#pragma once

#include <complex>
#include <cstddef>
#include <cstdint>
#include <sys/types.h>
#include <vector>

#include "comms-lib.h"
#include "sync/sync_config.h"

namespace houdini {
namespace sync {

struct Detection {
  /// The detector's index in the searched window: the last sample of the
  /// beacon core, i.e. the correlator's last-sample-of-the-matched-field index
  /// with the replica tail applied. receiver.cc's frame arithmetic treats it
  /// as the beacon end (houdiniBeaconEnd), which is the convention every
  /// shape holds (8.154). -1 when nothing crossed.
  ssize_t index = -1;
  bool found() const { return index >= 0; }
};

class Detector {
 public:
  /// @param replica       the matched-filter reference (Config::gold_cf32)
  /// @param replica_reps  copies of it in the core; 1 forces the coherence form
  /// @param replica_tail  samples from the replica's end to the core's end
  /// @param cfg           threshold form (kAuto resolves here), pick, first-path
  ///                      (a negative first_path_window resolves to half the
  ///                      replica length, the pre-library derivation)
  /// @param houdini       false keeps the Iris/UHD path on the power-ratio form,
  ///                      the cluster-refined pick and no replica tail, untouched.
  Detector(const std::vector<std::complex<float>>& replica, size_t replica_reps,
           size_t replica_tail, const DetectorConfig& cfg, bool houdini);

  /// Search `n` samples. `corr_scale` keeps its historical meaning for the
  /// legacy forms (the bar is 1/corr_scale); for the coherence form it is
  /// also the bar's reciprocal, so the caller may pass the value the
  /// configuration derives from pfa. `pick` overrides the configured rule for
  /// callers that search a window which may hold several copies.
  Detection run(const std::complex<int16_t>* samples, size_t n,
                float corr_scale) const;
  Detection run(const std::complex<int16_t>* samples, size_t n,
                float corr_scale, CommsLib::BeaconPick pick) const;

  CommsLib::BeaconThresh form() const { return form_; }
  CommsLib::BeaconPick pick() const { return pick_; }
  /// Samples the beacon END sits past the correlator's index; 0 off Houdini.
  size_t replicaTail() const { return tail_; }
  bool singleCopy() const { return single_copy_; }
  int firstPathWindow() const { return first_path_window_; }
  double firstPathFloorDb() const { return first_path_floor_db_; }
  /// The bar for the coherence form at this replica length and the configured
  /// per-window false-alarm probability over `window_samples` (8.163):
  /// bar = 1 - pfa_index^(1/(L-1)), pfa_index = pfa_window / window_samples.
  double coherenceBar(double pfa_per_window, size_t window_samples) const;

  static CommsLib::BeaconThresh resolveForm(ThresholdForm f, bool single_copy,
                                            bool houdini);
  static CommsLib::BeaconPick resolvePick(PickRule p);

 private:
  std::vector<std::complex<float>> replica_;
  bool single_copy_;
  size_t tail_;
  CommsLib::BeaconThresh form_;
  CommsLib::BeaconPick pick_;
  int first_path_window_;
  double first_path_floor_db_;
};

}  // namespace sync
}  // namespace houdini
