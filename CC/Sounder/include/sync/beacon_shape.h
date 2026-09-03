/**
 * @file sync/beacon_shape.h
 * @brief The configured beacon as ONE object: the waveform, the correlator
 *        replica, the field geometry, and the index convention every consumer
 *        rests on.
 *
 * Until 2026-09-03 these facts were spread over eight scalars in Config, a
 * file-static in receiver.cc (the strobe offset and the Houdini/Iris branch of
 * the expected beacon end), the Detector (the replica tail) and the Python
 * probe (a third convention, the START of the matched field). Four homes for
 * one index is the class of defect AP-34(a) cost a bench session; this object
 * is the one home.
 *
 * THE INDEX CONVENTION. The correlator reports the LAST SAMPLE of the matched
 * field (measured, beacon_geometry_test kEndConvention: a core placed at pos
 * peaks at pos + core_len - 1). endFromCorrelatorIndex() adds the replica
 * tail, which is what every consumer calls the beacon END: the SNR guard's
 * span [end - core_len, end), the CFO estimator's field placement, and the
 * receiver's frame arithmetic against expectedEndOffset(). That "end" is the
 * last sample, one before the true one-past-end. The difference is a fixed
 * one-sample convention, calibrated into the UE's tx_advance and asserted by
 * the geometry test; it is documented here rather than corrected because
 * correcting it moves the anchor by one sample and re-derives the pilot
 * timing on silicon. Nothing else in the tree may define the convention.
 *
 * The platform decides two defaults the shapes predate: on Iris/UHD the
 * replica tail is zero (that framer never had one) and the beacon end in a
 * slot-aligned window is core + prefix, where on Houdini it is the strobe
 * offset + core with no prefix.
 */
#pragma once

#include <complex>
#include <cstddef>
#include <string>
#include <sys/types.h>
#include <vector>

#include "sync/cfo_estimator.h"
#include "sync/numerology.h"
#include "sync/sync_config.h"

namespace houdini {
namespace sync {
namespace shapes {
struct Desc;
}
}
}

namespace houdini {
namespace sync {

/// On Houdini the beacon core starts this many ticks after the slot's
/// window_open with no prefix: the TDD grid BaseRadioSet programs into the
/// framer, which the UE's pilot bursts ride as well. One definition.
constexpr long long kHoudiniStrobeOffsetTicks = 384;

class BeaconShape {
 public:
  /// Build from a shape name (legacy, legacy_guard, dot11, nr, nr_pss).
  /// Throws std::invalid_argument naming the valid names: a typo that quietly
  /// ships the old beacon is exactly the failure the parameter exists to
  /// make visible.
  static BeaconShape make(const std::string& name, Platform platform, const Numerology& num);
  /// Build from a shape the caller already has (tests, the dumper).
  static BeaconShape fromDesc(const shapes::Desc& d, Platform platform,
                              const Numerology& num);
  static std::vector<std::string> names();

  const std::string& name() const { return name_; }
  Platform platform() const { return platform_; }
  size_t prefixSamples() const { return num_.prefix_samples; }
  const Numerology& numerology() const { return num_; }

  /// The transmitted burst at the shape's own scale (see config.cc on why the
  /// scale is load-bearing) and the matched-filter reference.
  const std::vector<std::complex<float>>& core() const { return core_; }
  const std::vector<std::complex<float>>& replica() const { return replica_; }
  size_t coreLen() const { return core_.size(); }
  size_t replicaLen() const { return replica_.size(); }
  size_t replicaOff() const { return replica_off_; }
  size_t replicaReps() const { return replica_reps_; }
  /// A single-copy replica (the NR PSS) has no lag product to take: the
  /// repeat-check threshold forms score zero on it and the detector must use
  /// the coherence form.
  bool singleCopy() const { return replica_reps_ < 2; }
  size_t guardLen() const { return guard_len_; }
  double paprDb() const { return papr_db_; }

  /// Samples from the correlator's index to the beacon end: zero for every
  /// shape whose replica is its trailing field and on Iris/UHD; 144 for
  /// nr_pss on Houdini, whose replica is the LEADING PSS.
  size_t replicaTail() const { return tail_; }
  /// The beacon end implied by a correlator index in a window of `window_len`
  /// samples, or -1 when there was no detection or the implied end falls
  /// outside the window (the SNR guard could not judge it anyway).
  ssize_t endFromCorrelatorIndex(ssize_t idx, size_t window_len) const;
  /// Where a slot-aligned window is expected to hold the beacon end.
  ssize_t expectedEndOffset() const;
  /// The first-path back window's default: half the replica, what the
  /// pre-library correlator derived (64 at 128 taps, 32 at 64).
  int defaultFirstPathWindow() const { return static_cast<int>(replica_.size() / 2); }
  /// The estimator's field layout.
  const FieldGeometry& geometry() const { return geometry_; }

 private:
  BeaconShape() = default;
  std::string name_;
  Platform platform_ = Platform::kHoudini;
  Numerology num_;
  std::vector<std::complex<float>> core_, replica_;
  size_t replica_off_ = 0, replica_reps_ = 0, guard_len_ = 0, tail_ = 0;
  double papr_db_ = 0.0;
  FieldGeometry geometry_;
};

}  // namespace sync
}  // namespace houdini
