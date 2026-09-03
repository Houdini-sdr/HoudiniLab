/**
 * @file sync/detector.cc
 */
#include "sync/detector.h"

#include <algorithm>

#include "comms-lib.h"

namespace houdini {
namespace sync {

namespace {
// The correlator's vocabulary, kept private to this file.
CommsLib::BeaconThresh toCorrelator(ThresholdForm f) {
  switch (f) {
    case ThresholdForm::kPowerRatio: return CommsLib::BeaconThresh::kPowerRatio;
    case ThresholdForm::kCoherence: return CommsLib::BeaconThresh::kCoherence;
    case ThresholdForm::kNormalizedXCorr:
    case ThresholdForm::kAuto:
    default: return CommsLib::BeaconThresh::kNormalizedXCorr;
  }
}
CommsLib::BeaconPick toCorrelator(PickRule p) {
  switch (p) {
    case PickRule::kFirstCrossing: return CommsLib::BeaconPick::kFirstCrossing;
    case PickRule::kClusterRefined: return CommsLib::BeaconPick::kFirstClusterRefined;
    case PickRule::kArgmax: return CommsLib::BeaconPick::kTargetedArgmax;
    case PickRule::kFirstPath:
    default: return CommsLib::BeaconPick::kFirstPath;
  }
}
}  // namespace

ThresholdForm Detector::resolveForm(ThresholdForm requested, bool single_copy) {
  // A single-copy replica has no lag product to take, so the repeat-check
  // forms would multiply the true peak by the correlation one replica length
  // earlier -- silence before the beacon -- and score zero at the right index
  // (8.154). The replica decides, whatever was asked.
  if (single_copy) return ThresholdForm::kCoherence;
  if (requested == ThresholdForm::kAuto) return ThresholdForm::kNormalizedXCorr;
  return requested;
}

Detector::Detector(const BeaconShape& shape, const DetectorConfig& cfg)
    : shape_(shape),
      form_(resolveForm(cfg.threshold, shape.singleCopy())),
      pick_(cfg.pick),
      first_path_window_(cfg.first_path_window >= 0
                             ? std::min<int>(cfg.first_path_window,
                                             2 * static_cast<int>(shape.replicaLen()))
                             : shape.defaultFirstPathWindow()),
      first_path_floor_db_(cfg.first_path_floor_db),
#if defined(USE_CUDA)
      backend_(DetectorBackend::kCuda)
#else
      backend_(DetectorBackend::kPortable)
#endif
{
}

const char* Detector::backendName() const {
  return backend_ == DetectorBackend::kCuda ? "cuda" : "portable";
}

Detection Detector::run(const std::complex<int16_t>* samples, size_t n,
                        float corr_scale) const {
  return run(samples, n, corr_scale, pick_);
}

Detection Detector::run(const std::complex<int16_t>* samples, size_t n,
                        float corr_scale, PickRule pick) const {
  Detection d;
  d.bar = 1.0 / static_cast<double>(corr_scale);
#if defined(USE_CUDA)
  if (backend_ == DetectorBackend::kCuda) {
    // The GPU correlator returns the first crossing under the power-ratio
    // form and knows nothing of the pick rule or the first-path knobs
    // (backendAppliesConfig). The index convention is still applied here,
    // in the one place it is applied for every backend.
    const ssize_t idx = CommsLib::find_beacon_cuda(samples, shape_.replica(), n, corr_scale);
    d.end_index = shape_.endFromCorrelatorIndex(idx, n);
    d.form = ThresholdForm::kPowerRatio;
    return d;
  }
#endif
  const CommsLib::BeaconResult r = CommsLib::find_beacon_ex(
      samples, shape_.replica(), n, corr_scale, toCorrelator(pick), toCorrelator(form_),
      first_path_window_, first_path_floor_db_);
  d.end_index = shape_.endFromCorrelatorIndex(r.index, n);
  d.form = form_;
  // Evidence travels with a detection that IS one: an index whose implied
  // end falls outside the window is reported as none, with no statistic.
  if (d.end_index >= 0) {
    d.statistic = r.statistic;
    d.peak = r.peak;
  }
  return d;
}

}  // namespace sync
}  // namespace houdini
