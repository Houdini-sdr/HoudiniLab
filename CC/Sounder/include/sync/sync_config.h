/**
 * @file sync/sync_config.h
 * @brief The UE synchronisation configuration: values in one struct, their
 *        schema in one static table, loaded from the JSON `sync` block,
 *        validated, resolved, and printed with the provenance of every value.
 *
 * WHY ONE TABLE. Until 2026-09-03 these values were about thirty HOUDINI_*
 * environment variables read at their point of use in receiver.cc,
 * comms-lib-portable.cc and BaseRadioSet.cc. Three costs were paid for that
 * (docs/SYNC_LIBRARY_ARCHITECTURE.md section 1): a knob accepted a nonsense
 * value silently (AP-56), a threshold inherited across beacons became a cliff
 * (DEMO_VERIFICATION 8.157), and the effective configuration of a run was only
 * knowable from its log. Every knob is one row of `schema()`: its JSON path,
 * the environment name it used to have, its range, its policy, and typed
 * accessors into the value struct. Loading, validation, the startup print and
 * the walkthrough's knob table are all generated from that one table.
 *
 * SCHEMA AND VALUES ARE SEPARATE. `schema()` is a static table of specs whose
 * accessors are plain functions of a SyncConfig; it holds no pointer into any
 * object, can be read from a const object, and needs no instance to render
 * the documentation. (Its first version kept raw pointers into `*this`, which
 * let a const reference hand out writable storage; the architecture review of
 * 2026-09-03 called it, and this is the fix.)
 *
 * ENVIRONMENT OVERRIDES ARE A MIGRATION AID. `allow_env_overrides` defaults to
 * true for this release because the bench scripts in tests/demo-verify still
 * drive sweeps through the environment; every override is logged, and a JSON
 * config can set the flag false. The plan is to flip the default next release
 * and remove the path after (architecture plan, sections 5 and 8).
 *
 * WHAT AN OUT-OF-RANGE ENVIRONMENT VALUE DOES, knob by knob, because the old
 * readers were not uniform: four integer knobs (`resync.retry_max`,
 * `resync.escalate_episodes`, `resync.hold_offgrid`, `cfo.log_every`) were
 * clamped to at least 1, three knobs (`beacon.tx_full_scale`,
 * `detector.first_path_floor_db`, `detector.first_path_window`) IGNORED a value
 * outside their range and kept what was in place, and every other numeric
 * knob passed anything finite through with no range at all -- which is what
 * AP-56 complained about. Now: EnvPolicy::kClamp pulls the value to the
 * nearest bound with a note (the four ints as before, the formerly unbounded
 * knobs as the AP-56 fix); EnvPolicy::kIgnoreOutOfRange keeps the value
 * already in place with a note (the three that always did). Garbage is
 * always refused.
 *
 * The value struct has no JSON dependency; the loader in sync_config.cc does.
 */
#pragma once

#include <cmath>
#include <cstddef>
#include <limits>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <variant>
#include <vector>


namespace houdini {
namespace sync {

/// Which decision statistic the detector thresholds on. kAuto resolves from
/// the replica: a single-copy replica (the NR PSS) has no lag product to take
/// and gets the coherence form; a repeated replica gets the normalised
/// cross-correlation. See CommsLib::BeaconThresh for the arithmetic and
/// DEMO_VERIFICATION 8.138 / 8.144 / 8.154 for the measurements.
enum class ThresholdForm : int { kAuto = 0, kPowerRatio, kNormalizedXCorr, kCoherence };

/// Which threshold crossing the detector returns. See CommsLib::BeaconPick.
enum class PickRule : int { kFirstCrossing = 0, kClusterRefined, kArgmax, kFirstPath };

enum class TrackerType : int { kAlphaBeta = 0, kKalman };

/// Where a value came from.
enum class Source { kDefault, kJson, kEnv, kDerived };
const char* name(Source s);
const char* name(ThresholdForm f);
const char* name(PickRule p);
const char* name(TrackerType t);

struct BeaconConfig {
  std::string type = "legacy";  ///< beacon_shapes name
  double tx_full_scale = 0.6;   ///< transmit peak as a fraction of full scale
};

/// The detection threshold as the correlator understands it: the bar is
/// 1 / corr_scale for every form, and each resync retry relaxes it by one
/// (corr_scale + attempt), the ladder the receiver has always run. Owned here,
/// so the bar is configuration with a range and a record, not a number handed
/// to the detector per call (architecture review 2026-09-03, item 1).
struct ThresholdPolicy {
  double corr_scale = 10.0;       ///< resync bar = 1 / corr_scale
  double corr_scale_init = 10.0;  ///< acquisition bar = 1 / corr_scale_init
  /// The scale to apply at resync attempt `attempt` (0 = first look).
  double relaxed(int attempt) const { return corr_scale + static_cast<double>(attempt); }
};

struct DetectorConfig {
  ThresholdForm threshold = ThresholdForm::kAuto;
  /// RESERVED for phase P3 (architecture plan): the coherence form's bar will
  /// be derived from this and the replica length (8.163). NOT APPLIED YET --
  /// the detector still uses `bar` for every form, and validate() says so
  /// when a value is given.
  double pfa_per_window = 1e-3;
  PickRule pick = PickRule::kFirstPath;
  /// Samples of back-search from the peak. -1 (the default) means "half the
  /// replica length", which is what the pre-library code derived: 64 for a
  /// 128-tap replica, 32 for the 64-tap dot11 and nr replicas. resolve()
  /// fills it in from the shape.
  int first_path_window = -1;
  double first_path_floor_db = -9.0;  ///< how much weaker an earlier path may be
  ThresholdPolicy bar;
};

struct ConfirmConfig {
  double snr_floor_db = 30.0;  ///< in-window SNR a detection must clear
};

struct CfoConfig {
  int index_guard = 8;   ///< samples the estimator's windows slide LATER (AP-39)
  int window_margin = 0; ///< samples shrunk from BOTH ends of each window (8.164)
  int log_every = 10;    ///< print one beacon-CFO line in N
};

struct GridTrackerConfig {
  TrackerType type = TrackerType::kAlphaBeta;
  double alpha = 0.5;
  double beta = 0.1;
  double step_ppm = 0.5;    ///< per-update slew limit on the rate
  double max_ppm = 100.0;   ///< plausibility band for the period
  double trust_ppm = 1.0;   ///< tracked vs fresh-confirm disagreement bound
  double kf_meas_var = 0.5;
  double kf_rate_rw = 1e-9;
  double kf_innov_gate = 4.0;
};

struct ResyncConfig {
  double residual_ppm = 0.1;      ///< assumed clock error after tracking
  double scatter_tol_us = 2.0;    ///< tracking gate, time
  double confirm_tol_us = 5.2083; ///< acquisition gate, time (never looser)
  /// Timing slack budgeted to drift between looks, samples. NaN means "a
  /// quarter of the OFDM zero prefix"; resolve() fills it in.
  double sync_tol_samples = std::numeric_limits<double>::quiet_NaN();
  int retry_max = 100;
  int escalate_episodes = 2;
  int hold_offgrid = 2;
  int acq_refine_span = 200;      ///< frames of baseline before trusting a rate
  double acq_max_ppm = 100.0;
};

/// What resolve() needs from outside the configuration: the shape and the
/// slot layout the sentinels are derived from.
struct ResolveContext {
  size_t replica_len = 0;      ///< taps in the correlator replica
  double prefix_samples = 0.0; ///< the OFDM zero prefix, samples
};

struct SyncConfig {
  BeaconConfig beacon;
  DetectorConfig detector;
  ConfirmConfig confirm;
  CfoConfig cfo;
  GridTrackerConfig tracker;
  ResyncConfig resync;
  bool allow_env_overrides = true;

  /// What an out-of-range ENVIRONMENT value does (JSON is always strict).
  enum class EnvPolicy { kClamp, kIgnoreOutOfRange };

  /// Typed accessors: a pair of plain functions of a SyncConfig, one per
  /// constness. No object is captured.
  template <class T>
  struct Access {
    T& (*ref)(SyncConfig&);
    const T& (*cref)(const SyncConfig&);
  };
  using Accessor = std::variant<Access<double>, Access<int>, Access<bool>, Access<std::string>,
                                Access<ThresholdForm>, Access<PickRule>, Access<TrackerType>>;

  /// One row of the schema.
  struct Spec {
    const char* path;   ///< JSON path under "sync", dotted
    const char* env;    ///< the environment name it replaces, or nullptr
    double lo, hi;      ///< inclusive range for numeric kinds
    const char* doc;    ///< one sentence, for the generated table
    Accessor access;
    const char* const* enum_names;  ///< enum kinds: nullptr-terminated names
    EnvPolicy env_policy;
    bool isNumeric() const {
      return std::holds_alternative<Access<double>>(access) ||
             std::holds_alternative<Access<int>>(access);
    }
    bool isEnum() const {
      return std::holds_alternative<Access<ThresholdForm>>(access) ||
             std::holds_alternative<Access<PickRule>>(access) ||
             std::holds_alternative<Access<TrackerType>>(access);
    }
  };
  /// The schema: one static table, built once, no instance needed.
  static const std::vector<Spec>& schema();
  /// The spec for a path, or nullptr.
  static const Spec* spec(std::string_view path);

  /// Where each value came from, by schema index. Unknown paths read kDefault.
  Source provenanceOf(std::string_view path) const;
  bool wasSet(std::string_view path) const { return provenanceOf(path) != Source::kDefault; }
  /// Validation and override notes that did not stop the load. Printed at
  /// startup.
  const std::vector<std::string>& warnings() const { return warnings_; }

  /// The shipped defaults (every provenance kDefault).
  static SyncConfig defaults();
  /// Load from the `sync` object of a config, given as its JSON text (nullopt
  /// = absent), plus the legacy top-level "beacon_type" and the first client's
  /// "corr_scale" / "corr_scale_init" when the caller has them. The library
  /// does not see the config FILE: the caller hands it the block and the
  /// legacy keys. Unknown keys under `sync` throw std::invalid_argument; keys
  /// whose name starts with "_" are comments; a key containing a dot is
  /// refused with a reason. Then the environment, when allowed; then
  /// validation.
  static SyncConfig load(const std::optional<std::string>& sync_block_json,
                         const std::optional<std::string>& legacy_beacon_type = std::nullopt,
                         const std::optional<double>& legacy_corr_scale = std::nullopt,
                         const std::optional<double>& legacy_corr_scale_init = std::nullopt);
  /// Convenience for tests and tools: the WHOLE config file's text, from
  /// which the block and the legacy keys are taken.
  static SyncConfig loadFromText(const std::string& root_json_text);
  /// Fill the sentinels (first_path_window, sync_tol_samples) from the shape
  /// and slot layout, marking them kDerived, so describe() prints the values
  /// actually used. Idempotent; an explicit value is left alone.
  void resolve(const ResolveContext& ctx);

  /// Effective values with provenance, one line per knob.
  std::string describe() const;
  /// The walkthrough's knob table, generated so it cannot drift.
  static std::string schemaMarkdown();
  /// One knob's current value as text ("derived" for an unresolved sentinel).
  std::string valueText(const Spec& s) const;

 private:
  std::vector<Source> provenance_;
  std::vector<std::string> warnings_;
  void validate();
  void setProvenance(size_t index, Source s);
};

}  // namespace sync
}  // namespace houdini
