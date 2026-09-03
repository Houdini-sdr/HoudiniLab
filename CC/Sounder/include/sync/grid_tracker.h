/**
 * @file sync/grid_tracker.h
 * @brief The UE's estimate of the BS frame grid: (reference, period), with two
 *        interchangeable estimators behind one interface.
 *
 * The shipped estimator is alpha-beta. AP-41 proposes a Kalman, NOT for
 * smoothing but for three things alpha-beta structurally cannot do:
 *
 *   (a) IRREGULAR dt. Observations arrive at a median 179 frames but range 10
 *       to 831, and fixed gains are optimal only for UNIFORM spacing. A Kalman
 *       carries dt in its transition and its process noise, so a long gap
 *       widens the covariance and the next observation is weighted accordingly;
 *       alpha-beta applies the same 0.5 / 0.1 to a 10-frame gap and an
 *       831-frame one.
 *   (b) COVARIANCE. An honest "how well do I know the time" number is what an
 *       adaptive cadence and a principled innovation gate are built from. The
 *       shipped +-kScatterTol gate is a fixed 1024 samples chosen for the
 *       alive/moved verdict, and using it as the outlier reject is what let a
 *       single edge-of-gate detection kick the rate 3.2 ppm (8.56).
 *   (c) a DOPPLER state for OTA, later.
 *
 * BOTH LIVE HERE, SELECTED AT RUNTIME, because "which is better" is a question
 * for the bench and not for this comment. Header-only and dependency-free so
 * grid_tracker_test can A/B them on synthetic traces with no radio.
 *
 * NOT IN SCOPE: fusing the beacon CARRIER channel. AP-41 is explicit that a
 * Kalman weights by inverse variance, so a BIASED sensor actively degrades the
 * state, and the carrier estimator is biased until AP-34(b) lands.
 */
#ifndef GRID_TRACKER_H_
#define GRID_TRACKER_H_

#include <algorithm>
#include <cmath>
#include <cstddef>

#include "sync/sync_config.h"

namespace houdini {
namespace sync {


/// The estimator's input. Its values have ONE owner, GridTrackerConfig in
/// sync_config.h (the JSON `tracker` block): construct from it, with the frame
/// length that turns step_ppm into a per-update limit in samples. The
/// defaults here are therefore the shipped ones, never a second copy.
struct TrackerConfig {
  explicit TrackerConfig(const GridTrackerConfig& g = GridTrackerConfig{},
                         double frame_samples = 0.0)
      : type(g.type),
        alpha(g.alpha),
        beta(g.beta),
        step_limit(g.step_ppm * 1e-6 * frame_samples),
        meas_var(g.kf_meas_var),
        rate_rw(g.kf_rate_rw),
        innov_gate(g.kf_innov_gate) {}
  TrackerType type;
  // alpha-beta
  double alpha;
  double beta;
  double step_limit;   ///< samples/frame per update; 0 = off
  // kalman
  double meas_var;     ///< R, samples^2. Detector scatter: sd 0.63-0.70
                             ///< measured post-fix, so ~0.5 is the variance.
  double rate_rw;     ///< q, samples^2/frame^3. Rate random walk: eps
                             ///< moved 0.23 ppm across a session, which is
                             ///< 0.028 samples/frame over ~6e5 frames, so
                             ///< q ~ 0.028^2/6e5 ~ 1.3e-9.
  double innov_gate;   ///< sigmas; 0 = off. A principled outlier reject.
  // NOTE: there is deliberately NO period band here. The plausibility clamp
  // lives with the caller that owns the period (receiver.cc), because this
  // class holds no state to clamp -- it returns GAINS. Fields named period_lo
  // and period_hi used to sit here, unread, which made them look enforced: a
  // caller could set them, drop its own clamp, and lose the band with no
  // compile or runtime signal. grid_tracker_test applies the clamp in its
  // harness for the same reason.
};

/**
 * The GAINS only. The caller keeps `ref` and `period` and applies the
 * corrections with its own arithmetic, so switching estimators cannot change
 * the shipped path's rounding: `receiver.cc` rounds `kf * period` to a whole
 * sample before adding the shift, and reproducing that inside a double-valued
 * tracker would have been a silent one-sample behaviour change on the arm that
 * is already gated. The estimator does not need the state anyway -- the caller
 * folds it into `resid` before calling.
 */
class GridTracker {
 public:
  void reset(const TrackerConfig& cfg) {
    cfg_ = cfg;
    // Start humble but not infinite. Whatever seeds this (the acquisition
    // confirm, or a re-anchor) is good to ~0.04 ppm on rate, and its offset
    // landed inside the confirm tolerance.
    p00_ = 100.0;   // samples^2
    p01_ = 0.0;
    p11_ = 1e-4;    // (samples/frame)^2
    shift_ = 0.0;
    dperiod_ = 0.0;
    innov_ = 0.0;
    updates_ = 0;
    rejected_ = 0;
  }

  /**
   * One observation the caller's outer gate already accepted.
   * @param kf    grid steps since the reference. Must be > 0 to inform rate.
   * @param resid observed minus predicted, samples.
   * @return true if the estimator took it. On false the caller must apply
   *         NOTHING: shift() and deltaPeriod() are zeroed.
   */
  bool update(long long kf, double resid) {
    shift_ = 0.0;
    dperiod_ = 0.0;
    innov_ = 0.0;
    if (kf <= 0) return false;
    const double dk = static_cast<double>(kf);
    if (cfg_.type == TrackerType::kKalman) {
      // Predict. F = [[1, dk], [0, 1]]. The caller re-anchors its reference to
      // frame kf on every accepted update, so dk IS the elapsed time and no
      // absolute frame index ever grows without bound.
      double p00 = p00_ + 2.0 * dk * p01_ + dk * dk * p11_;
      double p01 = p01_ + dk * p11_;
      double p11 = p11_;
      // Continuous rate-random-walk process noise. THIS is the part alpha-beta
      // has no equivalent of: a long gap widens the covariance, so the next
      // observation is weighted more against the prediction. Fixed gains apply
      // the same 0.5 / 0.1 to a 10-frame gap and an 831-frame one.
      const double q = cfg_.rate_rw;
      p00 += q * dk * dk * dk / 3.0;
      p01 += q * dk * dk / 2.0;
      p11 += q * dk;
      const double s = p00 + cfg_.meas_var;
      if (!(s > 0.0)) return false;
      innov_ = resid / std::sqrt(s);   // in sigmas
      if (cfg_.innov_gate > 0.0 && std::fabs(innov_) > cfg_.innov_gate) {
        ++rejected_;
        return false;
      }
      const double k0 = p00 / s;
      const double k1 = p01 / s;
      shift_ = k0 * resid;
      dperiod_ = k1 * resid;
      p00_ = std::max((1.0 - k0) * p00, 1e-12);
      p01_ = (1.0 - k0) * p01;
      p11_ = std::max(p11 - k1 * p01, 1e-18);
      // Keep the covariance a valid one. The simple (I - KH)P form is not
      // guaranteed positive-definite under rounding, and a covariance that
      // stops being a covariance fails SILENTLY: the gains stay finite and
      // plausible while the filter quietly stops being a filter. The diagonals
      // are floored above; this bounds the correlation so the off-diagonal can
      // never exceed what the diagonals permit.
      const double lim = 0.9999 * std::sqrt(p00_ * p11_);
      p01_ = std::min(lim, std::max(-lim, p01_));
    } else {
      shift_ = cfg_.alpha * resid;
      dperiod_ = cfg_.beta * resid / dk;
      if (cfg_.step_limit > 0.0)
        dperiod_ = std::min(cfg_.step_limit,
                            std::max(-cfg_.step_limit, dperiod_));
    }
    ++updates_;
    return true;
  }

  double shift() const { return shift_; }        ///< position correction
  double deltaPeriod() const { return dperiod_; }///< rate correction
  double innovSigmas() const { return innov_; }  ///< kalman only, 0 otherwise
  double timeSigma() const { return std::sqrt(p00_); }
  double rateSigma() const { return std::sqrt(p11_); }
  bool isKalman() const { return cfg_.type == TrackerType::kKalman; }
  size_t updates() const { return updates_; }
  size_t rejected() const { return rejected_; }

 private:
  TrackerConfig cfg_{};
  double p00_ = 100.0, p01_ = 0.0, p11_ = 1e-4;
  double shift_ = 0.0, dperiod_ = 0.0, innov_ = 0.0;
  size_t updates_ = 0, rejected_ = 0;
};

}  // namespace sync
}  // namespace houdini

#endif  // GRID_TRACKER_H_
