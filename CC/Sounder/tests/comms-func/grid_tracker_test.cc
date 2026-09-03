/**
 * @file grid_tracker_test.cc
 * @brief AP-41's A/B, offline: alpha-beta against the Kalman on traces built
 *        from THIS bench's measured arrival statistics. NO hardware.
 *
 * The rig can only ever run one of these at a time, so a bench A/B costs at
 * least two runs and answers only for the conditions those runs happened to
 * have. This runs both estimators over the SAME synthetic detections, which
 * makes the comparison paired and lets the conditions be dialled to the ones
 * that matter -- in particular the irregular spacing AP-41 says alpha-beta
 * cannot handle, which on this bench ranges 10 to 831 frames around a median
 * of 179.
 *
 * WHAT IT DOES NOT DECIDE. Synthetic detections carry the noise model we chose.
 * A win here is a PREDICTION for the rig, not a result; the rig leg is what
 * settles it. The numbers are printed so the prediction can be checked against
 * what the bench does, rather than quietly assumed.
 *
 * Build: CMake target grid_tracker_test. Run: ./grid_tracker_test (or ctest).
 */
#include <cinttypes>
#include <cmath>
#include <cstdio>
#include <random>
#include <string>
#include <vector>

#include "sync/grid_tracker.h"

namespace {

int g_fail = 0;

void check(bool ok, const std::string& what) {
  std::printf("%s  %s\n", ok ? "PASS" : "FAIL", what.c_str());
  if (!ok) ++g_fail;
}

constexpr double kFrame = 122880.0;
constexpr double kEpsPpm = 8.52;           // the measured pair
constexpr double kTruePeriod = kFrame * (1.0 + kEpsPpm * 1e-6);
constexpr double kScatterSd = 0.70;        // measured sync residual sd, samples
constexpr long long kScatterTol = 1024;    // the outer alive/moved gate
// The +-100 ppm plausibility band, applied by the harness because the caller
// owns it (see grid_tracker.h).
constexpr double kPeriodLo = kFrame * (1.0 - 100e-6);
constexpr double kPeriodHi = kFrame * (1.0 + 100e-6);

struct Trace {
  std::vector<long long> gaps;   ///< frames between consecutive detections
  std::vector<double> noise;     ///< detector scatter, samples
  std::vector<double> outlier;   ///< extra kick on selected detections
  std::vector<double> drift;     ///< true period at each detection
};

// DETERMINISTIC ACROSS STANDARD LIBRARIES, deliberately. std::mt19937 is
// specified bit-for-bit by the standard; std::normal_distribution and friends
// are NOT -- their algorithms are implementation-defined, so the same seed
// draws different numbers on libstdc++, libc++ and MSVC. This is registered as
// a ctest and its checks turn on statistical margins, so an implementation
// change would fail it with no code change and no bug. Build the draws from raw
// mt19937 output instead.
double u01(std::mt19937& g) {
  // mt19937 yields [0, 2^32); the +0.5 keeps the result strictly inside (0, 1),
  // which log() below requires.
  return (static_cast<double>(g()) + 0.5) / 4294967296.0;
}

double gauss(std::mt19937& g) {   // Box-Muller
  const double u1 = u01(g), u2 = u01(g);
  return std::sqrt(-2.0 * std::log(u1)) * std::cos(2.0 * M_PI * u2);
}

/// Detections spaced as the bench spaces them, with a slow rate random walk.
Trace makeTrace(unsigned seed, int n, bool irregular, double outlier_rate,
                double drift_ppm_per_1e5) {
  std::mt19937 rng(seed);
  // Measured: median 179 frames, range 10-831. A lognormal reproduces that
  // shape far better than a uniform, and the shape is the point.
  auto lognormal_gap = [&rng]() {
    return std::exp(std::log(179.0) + 0.75 * gauss(rng));
  };
  Trace t;
  double period = kTruePeriod;
  long long elapsed = 0;
  for (int i = 0; i < n; ++i) {
    long long g =
        irregular ? std::max<long long>(
                        10, std::min<long long>(
                                831, static_cast<long long>(lognormal_gap())))
                  : 260;
    elapsed += g;
    // A slow, real rate walk: the session-scale 0.23 ppm drift.
    period = kTruePeriod * (1.0 + drift_ppm_per_1e5 * 1e-6 *
                                      static_cast<double>(elapsed) / 1e5);
    t.gaps.push_back(g);
    t.noise.push_back(kScatterSd * gauss(rng));
    // An occasional edge-of-gate detection: inside the +-1024 alive/moved gate,
    // so the shipped code ACCEPTS it, which is exactly the case 8.56 is about.
    t.outlier.push_back(u01(rng) < outlier_rate ? 900.0 : 0.0);
    t.drift.push_back(period);
  }
  return t;
}

struct Result {
  double rate_rms_ppm;    ///< rms period error over the run
  double time_rms;        ///< rms |resid| the tracker would have seen, samples
  double final_ppm_err;
  size_t updates, rejected, gated;
  double last_time_sigma;
  bool is_kalman;
};

Result run(const Trace& t, houdini::sync::TrackerConfig cfg) {
  houdini::sync::GridTracker tr;
  tr.reset(cfg);
  // The caller owns ref and period, and applies the tracker's gains with the
  // SAME arithmetic receiver.cc uses (round kf*period to a whole sample, then
  // add the rounded shift). That is the point of the gains-only interface:
  // swapping estimators cannot change the shipped path's rounding.
  // Seeded as the acquisition confirm seeds it, good to ~0.04 ppm.
  long long ref = 0;
  double period = kFrame * (1.0 + (kEpsPpm + 0.04) * 1e-6);
  auto gridStart = [&](long long n) {
    return ref + std::llround(static_cast<double>(n) * period);
  };
  // EVERYTHING stays in ABSOLUTE sample coordinates. (An earlier cut of this
  // harness reset the truth origin after each update, which walked truth and
  // estimate apart immediately and left the outer gate rejecting all but the
  // first of 400 detections. The estimators were fine; the instrument was not.)
  double beacon_abs = 0.0;
  double rate_se = 0.0, time_se = 0.0;
  size_t n = 0, gated = 0;
  // Grid steps since the CURRENT reference. A detection the outer gate throws
  // away does not move that reference, so the gap accumulates.
  long long since_ref = 0;
  for (size_t i = 0; i < t.gaps.size(); ++i) {
    beacon_abs += static_cast<double>(t.gaps[i]) * t.drift[i];
    since_ref += t.gaps[i];
    const double observed = beacon_abs + t.noise[i] + t.outlier[i];
    const double resid = observed - static_cast<double>(gridStart(since_ref));
    if (std::llabs(std::llround(resid)) > kScatterTol) {   // outer alive/moved
      ++gated;
      continue;
    }
    time_se += resid * resid;
    ++n;
    if (tr.update(since_ref, resid)) {
      ref = gridStart(since_ref) + std::llround(tr.shift());
      period += tr.deltaPeriod();
      // The plausibility band belongs to the CALLER, exactly as in
      // receiver.cc -- GridTracker returns gains and holds no period to clamp.
      period = std::min(kPeriodHi, std::max(kPeriodLo, period));
      since_ref = 0;
    }
    const double err_ppm = (period - t.drift[i]) / kFrame * 1e6;
    rate_se += err_ppm * err_ppm;
  }
  Result r{};
  r.rate_rms_ppm = n ? std::sqrt(rate_se / static_cast<double>(n)) : 0.0;
  r.time_rms = n ? std::sqrt(time_se / static_cast<double>(n)) : 0.0;
  r.final_ppm_err = (period - t.drift.back()) / kFrame * 1e6;
  r.updates = tr.updates();
  r.rejected = tr.rejected();
  r.gated = gated;
  r.last_time_sigma = tr.timeSigma();
  r.is_kalman = tr.isKalman();
  return r;
}

houdini::sync::TrackerConfig ab() {
  houdini::sync::TrackerConfig c;
  c.type = houdini::sync::TrackerType::kAlphaBeta;
  c.alpha = 0.5;
  c.beta = 0.1;
  c.step_limit = 0.5e-6 * kFrame;   // HOUDINI_GRID_STEP_PPM default
  return c;
}

houdini::sync::TrackerConfig abBare() {
  houdini::sync::TrackerConfig c = ab();
  c.step_limit = 0.0;      // no slew limit: the estimator with nothing added
  return c;
}

houdini::sync::TrackerConfig kf(double gate = 0.0) {
  houdini::sync::TrackerConfig c = ab();
  c.type = houdini::sync::TrackerType::kKalman;
  c.step_limit = 0.0;      // the kalman's robustness is the innovation gate
  c.meas_var = kScatterSd * kScatterSd;
  c.rate_rw = 1e-9;
  c.innov_gate = gate;
  return c;
}

void row(const char* label, const Result& r) {
  char sig[32];
  // alpha-beta carries no covariance, so printing its untouched PRIOR next to
  // the kalman's converged value would invite a comparison that means nothing.
  if (r.is_kalman)
    std::snprintf(sig, sizeof(sig), "%6.2f", r.last_time_sigma);
  else
    std::snprintf(sig, sizeof(sig), "%6s", "n/a");
  std::printf("  %-34s rate rms %8.4f ppm  final %+8.4f  time rms %6.2f samp  "
              "upd %3zu rej %2zu gated %2zu  sigma_t %s\n",
              label, r.rate_rms_ppm, r.final_ppm_err, r.time_rms, r.updates,
              r.rejected, r.gated, sig);
}

}  // namespace

int main() {
  std::printf("frame %.0f, eps %.2f ppm, detector scatter sd %.2f samples\n\n",
              kFrame, kEpsPpm, kScatterSd);

  // ---- 1. regular spacing: alpha-beta's home ground ---------------------
  std::printf("=== REGULAR spacing (260 frames), no outliers ===\n");
  {
    Trace t = makeTrace(1, 400, false, 0.0, 0.23);
    Result a = run(t, ab()), k = run(t, kf());
    row("alpha-beta", a);
    row("kalman", k);
    check(k.rate_rms_ppm < a.rate_rms_ppm * 1.5,
          "on regular spacing the kalman is no worse than 1.5x alpha-beta");
  }
  std::printf("\n");

  // ---- 2. the spacing the bench ACTUALLY has ---------------------------
  std::printf("=== IRREGULAR spacing (median 179, range 10-831), no outliers ===\n");
  double a_irr = 0.0, k_irr = 0.0;
  {
    Trace t = makeTrace(2, 400, true, 0.0, 0.23);
    Result a = run(t, ab()), k = run(t, kf());
    row("alpha-beta", a);
    row("kalman", k);
    a_irr = a.rate_rms_ppm;
    k_irr = k.rate_rms_ppm;
    check(k.rate_rms_ppm < a.rate_rms_ppm,
          "on the bench's OWN irregular spacing the kalman is better");
  }
  std::printf("\n");

  // ---- 3. outliers inside the alive/moved gate -------------------------
  // 8.56's case: a detection at 900 samples is INSIDE the +-1024 gate, so the
  // shipped code accepts it and it reaches the estimator.
  //
  // COMPARE LIKE WITH LIKE. An earlier cut of this leg put alpha-beta WITH its
  // slew limit against a BARE kalman and reported the kalman 4x worse, which
  // says nothing about the estimators: it compares one that was robustified
  // against one that was not. Alpha-beta's slew limit and the kalman's
  // innovation gate are the same idea reached two ways, so the honest table is
  // bare against bare and robust against robust.
  //
  // Bare against bare, the two are close for a reason worth stating: at the
  // median 179-frame gap the kalman's rate gain works out near beta/dk, so
  // both move the period by a similar amount on a 900-sample kick. The
  // difference is what each can do ABOUT it. Alpha-beta can only clamp the
  // step, which also clamps genuine convergence. The kalman knows its own
  // uncertainty, so it can reject the observation as implausible and keep its
  // gain -- and the gate self-opens if it ever rejects too long, because a
  // stale state widens the covariance and shrinks the normalized innovation.
  std::printf("=== IRREGULAR + 5%% edge-of-gate outliers (accepted by +-1024) ===\n");
  {
    Trace t = makeTrace(3, 400, true, 0.05, 0.23);
    Result ab_bare = run(t, abBare()), kf_bare = run(t, kf());
    Result ab_rob = run(t, ab()), kf_rob = run(t, kf(4.0));
    std::printf("  -- bare: neither estimator robustified --\n");
    row("alpha-beta, no slew limit", ab_bare);
    row("kalman, no innovation gate", kf_bare);
    std::printf("  -- robust: each with its own outlier defence --\n");
    row("alpha-beta + slew limit", ab_rob);
    row("kalman + 4-sigma innovation gate", kf_rob);
    // The rms hides the thing that matters. Bare alpha-beta ends the run FAR
    // off and, because its grid walked out of the +-1024 alive/moved window,
    // loses detections outright -- on the rig that is an escalation. The bare
    // kalman ends converged and loses none. Assert on both, or the summary
    // number reports a near-tie between an estimator that held lock and one
    // that did not.
    check(kf_bare.rate_rms_ppm < ab_bare.rate_rms_ppm * 2.0,
          "bare vs bare, the kalman's transient rms is not materially worse");
    check(std::fabs(kf_bare.final_ppm_err) < std::fabs(ab_bare.final_ppm_err),
          "bare vs bare, the kalman CONVERGES where alpha-beta does not");
    check(kf_bare.gated <= ab_bare.gated,
          "bare vs bare, the kalman loses no detections to a drifted grid");
    check(kf_rob.rate_rms_ppm < ab_rob.rate_rms_ppm,
          "robust vs robust, the kalman is better");
    check(kf_rob.rate_rms_ppm < kf_bare.rate_rms_ppm,
          "the innovation gate is what makes the kalman arm worth running");
    check(kf_rob.rejected > 0, "the innovation gate actually rejects something");
    std::printf("  robust/robust ratio: kalman is %.2fx alpha-beta's error\n",
                ab_rob.rate_rms_ppm > 0
                    ? kf_rob.rate_rms_ppm / ab_rob.rate_rms_ppm : 0.0);
  }
  std::printf("\n");

  // ---- 4. the covariance has to MEAN something -------------------------
  std::printf("=== does sigma_t track reality? ===\n");
  {
    Trace t = makeTrace(4, 400, true, 0.0, 0.23);
    Result k = run(t, kf());
    // A useful uncertainty is neither collapsed to zero nor stuck at its prior.
    check(k.last_time_sigma > 0.0 && k.last_time_sigma < 100.0,
          "sigma_t converges off its prior without collapsing to zero");
    std::printf("  sigma_t settles at %.3f samples against a %.2f-sample "
                "detector scatter\n", k.last_time_sigma, kScatterSd);
    check(k.last_time_sigma < 5.0 * kScatterSd,
          "sigma_t is the right ORDER as the measurement noise");
  }
  std::printf("\n");

  std::printf("=== prediction for the rig ===\n");
  std::printf("  On this bench's spacing the kalman's rate error is %.2fx\n",
              a_irr > 0 ? k_irr / a_irr : 0.0);
  std::printf("  alpha-beta's. That is a PREDICTION from a chosen noise model,\n");
  std::printf("  not a result. HOUDINI_TRACKER=kf runs the same comparison on\n");
  std::printf("  silicon; if the bench disagrees with this, the bench is right.\n");

  std::printf("\nRESULT: %s (%d failure(s))\n", g_fail ? "FAIL" : "PASS", g_fail);
  return g_fail ? 1 : 0;
}
