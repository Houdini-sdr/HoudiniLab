/**
 * @file sync/detector.h
 * @brief The beacon detector as one object: shape, threshold form, pick rule
 *        and first-path knobs, resolved once.
 *
 * Wraps the correlator (CommsLib::find_beacon_ex, or find_beacon_cuda under
 * USE_CUDA) with the decisions receiver.cc used to make at its call sites
 * (DEMO_VERIFICATION 8.138 to 8.154): which threshold form the replica
 * supports, which crossing to return, how far the first-path search looks
 * back and how much weaker a path may be. The index convention belongs to the
 * BeaconShape and is applied here in one place for every backend. All of it is
 * fixed at construction, so a run cannot mix two populations and a test can
 * build the same detector the receiver runs.
 *
 * The API speaks the library's own vocabulary (ThresholdForm, PickRule); the
 * correlator's enums stay inside detector.cc, so a consumer of this header
 * links no FFT library for an enum.
 */
#pragma once

#include <complex>
#include <cstddef>
#include <cstdint>
#include <sys/types.h>

#include "sync/beacon_shape.h"
#include "sync/sync_config.h"

namespace houdini {
namespace sync {

struct Detection {
  /// The beacon END per BeaconShape's convention (the correlator's index plus
  /// the replica tail), or -1 when nothing crossed or the implied end fell
  /// outside the window.
  ssize_t end_index = -1;
  /// The decision statistic at the correlator's index, in the form's units
  /// (a coherence in [0, 1] for the normalised forms, a power ratio for
  /// kPowerRatio), and the bar it cleared. Zero when nothing crossed; NaN
  /// when the backend reports none (the CUDA correlator), so a fixture
  /// recorded from it cannot pass as a measured zero.
  double statistic = 0.0;
  double bar = 0.0;
  ThresholdForm form = ThresholdForm::kAuto;
  /// The correlator output at the index: the matched field's complex peak,
  /// which a phase tracker reads (architecture plan, P5).
  std::complex<float> peak{0.0f, 0.0f};
  /// The sub-sample position of the detected path relative to `end_index`, in
  /// samples: `end_index + frac_offset` is where the lobe's top actually sits
  /// (CommsLib::BeaconResult::frac_offset; measured in 8ah). Zero when no
  /// refinement is available, NaN when the backend does not report one. The
  /// integer `end_index` is unchanged by it, so the index convention is the
  /// same with and without this field.
  double frac_offset = 0.0;
  bool found() const { return end_index >= 0; }
};

enum class DetectorBackend { kPortable, kCuda };

class Detector {
 public:
  /// @param shape  the configured beacon (copied: the detector owns what it
  ///               needs and outlives nothing)
  /// @param cfg    the RESOLVED detector configuration (SyncConfig::resolve);
  ///               an unresolved first_path_window (-1) falls back to the
  ///               shape's default, an explicit value is bounded at twice the
  ///               replica, the correlator's own limit
  Detector(const BeaconShape& shape, const DetectorConfig& cfg);

  /// Search `n` samples with the bar 1/corr_scale (ThresholdPolicy), or, for
  /// the coherence form when the configuration set a false-alarm probability
  /// (DetectorConfig::pfa_applies), the bar that probability implies for
  /// this replica over `n` samples (ThresholdPolicy::coherenceBar). `pick`
  /// overrides the configured rule for a caller whose window may hold several
  /// beacon copies.
  Detection run(const std::complex<int16_t>* samples, size_t n, float corr_scale) const;
  Detection run(const std::complex<int16_t>* samples, size_t n, float corr_scale,
                PickRule pick) const;

  ThresholdForm form() const { return form_; }
  PickRule pick() const { return pick_; }
  size_t replicaTail() const { return shape_.replicaTail(); }
  bool singleCopy() const { return shape_.singleCopy(); }
  int firstPathWindow() const { return first_path_window_; }
  double firstPathFloorDb() const { return first_path_floor_db_; }
  /// True when the bar comes from the configured false-alarm probability
  /// rather than corr_scale (coherence form, pfa set).
  bool barFromPfa() const { return pfa_applies_; }
  double pfaPerWindow() const { return pfa_; }
  /// The corr_scale equivalent this detector applies for a window of `n`
  /// samples: the caller's value, or 1 / coherenceBar when barFromPfa().
  double effectiveScale(float corr_scale, size_t n) const;
  const BeaconShape& shape() const { return shape_; }

  DetectorBackend backend() const { return backend_; }
  const char* backendName() const;
  /// False for a backend that ignores the configured form, pick, first-path
  /// knobs and bar (the CUDA correlator: first crossing, power ratio).
  bool backendAppliesConfig() const { return backend_ == DetectorBackend::kPortable; }
  /// The same fact for the backend this library was built with, for a
  /// configuration resolving before any detector exists.
  static bool backendAppliesConfigByDefault();

  /// kAuto resolves to the normalised cross-correlation; a single-copy
  /// replica takes the coherence form whatever was asked (8.154). The
  /// platform default for Iris/UHD (the power-ratio form) is the
  /// configuration's business: SyncConfig::resolve.
  static ThresholdForm resolveForm(ThresholdForm requested, bool single_copy);

 private:
  const BeaconShape shape_;
  ThresholdForm form_;
  PickRule pick_;
  int first_path_window_;
  double first_path_floor_db_;
  bool pfa_applies_;
  double pfa_;
  DetectorBackend backend_;
};

}  // namespace sync
}  // namespace houdini
