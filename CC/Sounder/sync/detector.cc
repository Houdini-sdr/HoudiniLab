/**
 * @file sync/detector.cc
 */
#include "sync/detector.h"

#include <algorithm>
#include <cmath>

namespace houdini {
namespace sync {

CommsLib::BeaconThresh Detector::resolveForm(ThresholdForm f, bool single_copy,
                                             bool houdini) {
  // The Iris/UHD path keeps the power-ratio form: different hardware, no
  // measurements on this bench, and the shipped tuning notes describe that
  // population (receiver.cc, syncSearch).
  if (!houdini) return CommsLib::BeaconThresh::kPowerRatio;
  // A single-copy replica has no lag product to take, so the repeat-check
  // forms would multiply the true peak by the correlation one replica length
  // earlier -- silence before the beacon -- and score zero at the right index
  // (8.154). The replica decides, whatever was asked.
  if (single_copy) return CommsLib::BeaconThresh::kXCorrNoLag;
  switch (f) {
    case ThresholdForm::kPowerRatio: return CommsLib::BeaconThresh::kPowerRatio;
    case ThresholdForm::kCoherence: return CommsLib::BeaconThresh::kXCorrNoLag;
    case ThresholdForm::kNormalizedXCorr:
    case ThresholdForm::kAuto:
    default: return CommsLib::BeaconThresh::kNormalizedXCorr;
  }
}

CommsLib::BeaconPick Detector::resolvePick(PickRule p) {
  switch (p) {
    case PickRule::kFirstCrossing: return CommsLib::BeaconPick::kFirstCrossing;
    case PickRule::kClusterRefined: return CommsLib::BeaconPick::kFirstClusterRefined;
    case PickRule::kArgmax: return CommsLib::BeaconPick::kTargetedArgmax;
    case PickRule::kFirstPath:
    default: return CommsLib::BeaconPick::kFirstPath;
  }
}

Detector::Detector(const std::vector<std::complex<float>>& replica,
                   size_t replica_reps, size_t replica_tail,
                   const DetectorConfig& cfg, bool houdini)
    : replica_(replica),
      single_copy_(replica_reps < 2),
      tail_(replica_tail),
      form_(resolveForm(cfg.threshold, replica_reps < 2, houdini)),
      pick_(houdini ? resolvePick(cfg.pick)
                    : CommsLib::BeaconPick::kFirstClusterRefined),
      first_path_window_(cfg.first_path_window),
      first_path_floor_db_(cfg.first_path_floor_db),
      houdini_(houdini) {}

Detection Detector::run(const std::complex<int16_t>* samples, size_t n,
                        float corr_scale) const {
  return run(samples, n, corr_scale, pick_);
}

Detection Detector::run(const std::complex<int16_t>* samples, size_t n,
                        float corr_scale, CommsLib::BeaconPick pick) const {
  Detection d;
  ssize_t idx = CommsLib::find_beacon_avx(samples, replica_, n, corr_scale, pick,
                                          form_, first_path_window_,
                                          first_path_floor_db_);
  // THE CORRELATOR REPORTS THE LAST SAMPLE OF THE MATCHED FIELD, and every
  // consumer wants the beacon END. For the shipped shapes those coincide because
  // the replica is the trailing fine field; for nr_pss the replica is the
  // leading PSS and the end sits `tail_` later. A detection whose implied end
  // falls outside the window is reported as none: the SNR guard could not
  // measure it anyway.
  if (idx >= 0 && houdini_ && tail_ > 0) {
    if (idx + static_cast<ssize_t>(tail_) >= static_cast<ssize_t>(n)) return d;
    idx += static_cast<ssize_t>(tail_);
  }
  d.index = idx;
  return d;
}

double Detector::coherenceBar(double pfa_per_window, size_t window_samples) const {
  // A pure-noise window's coherence against an L-tap replica is Beta(1, L-1),
  // so P(coh > bar) per index is (1 - bar)^(L-1). For a per-window rate over
  // `window_samples` independent indices: bar = 1 - pfa_index^(1/(L-1)).
  // Measured against its prediction in beacon_geometry_test (8.163).
  const double L = static_cast<double>(std::max<size_t>(2, replica_.size()));
  const double pw = std::min(0.5, std::max(1e-12, pfa_per_window));
  const double pidx = pw / static_cast<double>(std::max<size_t>(1, window_samples));
  return 1.0 - std::pow(pidx, 1.0 / (L - 1.0));
}

}  // namespace sync
}  // namespace houdini
