/**
 * @file sync/sync_config.h
 * @brief The UE synchronisation configuration: one struct, one table of knobs,
 *        loaded from the JSON config's `sync` block, validated, and printed
 *        with the provenance of every value.
 *
 * WHY ONE TABLE. Until 2026-09-03 these values were about thirty HOUDINI_*
 * environment variables read at their point of use in receiver.cc,
 * comms-lib-portable.cc and BaseRadioSet.cc. Three costs were paid for that
 * (docs/SYNC_LIBRARY_ARCHITECTURE.md section 1): a knob accepted a nonsense
 * value silently (AP-56), a threshold inherited across beacons became a cliff
 * (DEMO_VERIFICATION 8.157), and the effective configuration of a run was only
 * knowable from its log. Every knob is now one row of `knobs()`: its JSON path,
 * the environment name it used to have, its range, and where it lives in this
 * struct. Loading, validation, the startup print and the walkthrough's knob
 * table are all generated from that one table, so they cannot disagree.
 *
 * ENVIRONMENT OVERRIDES ARE A MIGRATION AID. `allow_env_overrides` defaults to
 * true for this release because the bench scripts in tests/demo-verify still
 * drive sweeps through the environment; every override is logged, an
 * out-of-range numeric override is CLAMPED into the knob's range with a note
 * (the old readers clamped too, so a script's `=0` keeps meaning "the minimum"),
 * and a JSON config can set the flag false. The plan is to flip the default
 * next release and remove the path after (architecture plan, sections 5 and 8).
 *
 * The struct itself has no JSON dependency; `load()` in sync_config.cc does.
 */
#pragma once

#include <cmath>
#include <cstddef>
#include <limits>
#include <map>
#include <string>
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

struct BeaconConfig {
  std::string type = "legacy";  ///< beacon_shapes name
  double tx_full_scale = 0.6;   ///< transmit peak as a fraction of full scale
};

struct DetectorConfig {
  ThresholdForm threshold = ThresholdForm::kAuto;
  /// RESERVED for phase P3 (architecture plan): the coherence form's bar will
  /// be derived from this and the replica length (8.163). NOT APPLIED YET --
  /// the receiver still passes the config's corr_scale for every form, and
  /// validate() says so when a value is given.
  double pfa_per_window = 1e-3;
  PickRule pick = PickRule::kFirstPath;
  /// Samples of back-search from the peak. -1 (the default) means "half the
  /// replica length", which is what the pre-library code derived: 64 for a
  /// 128-tap replica, 32 for the 64-tap dot11 and nr replicas. Resolved in
  /// Detector's constructor; read it back from Detector::firstPathWindow().
  int first_path_window = -1;
  double first_path_floor_db = -9.0;  ///< how much weaker an earlier path may be
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
  /// quarter of the OFDM zero prefix"; the receiver resolves it, because the
  /// prefix is not known here.
  double sync_tol_samples = std::numeric_limits<double>::quiet_NaN();
  int retry_max = 100;
  int escalate_episodes = 2;
  int hold_offgrid = 2;
  int acq_refine_span = 200;      ///< frames of baseline before trusting a rate
  double acq_max_ppm = 100.0;
};

struct SyncConfig {
  BeaconConfig beacon;
  DetectorConfig detector;
  ConfirmConfig confirm;
  CfoConfig cfo;
  GridTrackerConfig tracker;
  ResyncConfig resync;
  bool allow_env_overrides = true;

  /// Where each value came from, by JSON path: "default", "json" or "env".
  std::map<std::string, std::string> provenance;
  /// Validation notes that did not stop the load (a floor above the expected
  /// SNR, an override that was clamped or ignored). Printed at startup.
  std::vector<std::string> warnings;

  /// A typed pointer to one knob's storage inside a SyncConfig.
  using Target = std::variant<double*, int*, bool*, std::string*, ThresholdForm*,
                              PickRule*, TrackerType*>;

  /// One knob: how it is named, bounded and stored. `target` points INTO the
  /// SyncConfig that produced the table, so a table is only valid for that
  /// object's lifetime and must be taken from the object it will modify.
  struct Knob {
    const char* path;   ///< JSON path under "sync", dotted
    const char* env;    ///< the environment name it replaces, or nullptr
    double lo, hi;      ///< inclusive range for numeric kinds
    const char* doc;    ///< one sentence, for the generated table
    Target target;
    const char* const* enum_names;  ///< enum kinds: nullptr-terminated names
    bool isNumeric() const {
      return std::holds_alternative<double*>(target) || std::holds_alternative<int*>(target);
    }
    bool isEnum() const {
      return !std::holds_alternative<double*>(target) && !std::holds_alternative<int*>(target) &&
             !std::holds_alternative<bool*>(target) && !std::holds_alternative<std::string*>(target);
    }
  };
  std::vector<Knob> knobs();
  /// The same table over a const object, for reading only.
  std::vector<Knob> knobs() const;

  /// The shipped defaults (every provenance "default").
  static SyncConfig defaults();
  /// Load: defaults, then the `sync` object of `root_json_text` (the WHOLE
  /// config file's text, parsed here), then the environment when allowed.
  /// Unknown keys under `sync` throw std::invalid_argument; keys whose name
  /// (any segment) starts with "_" are comments. A legacy top-level
  /// "beacon_type" is accepted into beacon.type, and is an error when both are
  /// present and disagree.
  static SyncConfig load(const std::string& root_json_text);
  /// Range checks and cross-constraints. Throws std::invalid_argument on a
  /// value that cannot be run; appends to `warnings` for the rest.
  void validate();
  /// Effective values with provenance, one line per knob.
  std::string describe() const;
  /// The walkthrough's knob table, generated so it cannot drift.
  static std::string schemaMarkdown();
  /// One knob's current value as text ("derived" for a NaN or -1 sentinel).
  static std::string valueText(const Knob& k);

  static const char* name(ThresholdForm f);
  static const char* name(PickRule p);
  static const char* name(TrackerType t);
};

}  // namespace sync
}  // namespace houdini
