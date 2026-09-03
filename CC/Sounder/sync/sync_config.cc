/**
 * @file sync/sync_config.cc
 * @brief SyncConfig: the knob table, the loader, validation, and the two
 *        generated views (startup description, walkthrough table).
 */
#include "sync/sync_config.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <sstream>
#include <stdexcept>

#include "nlohmann/json.hpp"

namespace houdini {
namespace sync {

namespace {

const char* const kThresholdNames[] = {"auto", "power", "xcorr", "coherence",
                                       nullptr};
const char* const kPickNames[] = {"first_crossing", "cluster_refined", "argmax",
                                  "first_path", nullptr};
const char* const kTrackerNames[] = {"alpha_beta", "kalman", nullptr};

// The environment spellings that predate the table, for the override path
// and for the error message when one is refused.
struct EnvAlias {
  const char* env;
  const char* value;   // env spelling
  const char* canon;   // table spelling
};
const EnvAlias kEnumAliases[] = {
    {"HOUDINI_BEACON_THRESH", "nolag", "coherence"},
    {"HOUDINI_BEACON_THRESH", "norm", "xcorr"},
    {"HOUDINI_BEACON_PICK", "first", "first_crossing"},
    {"HOUDINI_BEACON_PICK", "firstpath", "first_path"},
    {"HOUDINI_BEACON_PICK", "first-path", "first_path"},
    {"HOUDINI_TRACKER", "ab", "alpha_beta"},
    {"HOUDINI_TRACKER", "kf", "kalman"},
};

int enumIndex(const char* const* names, const std::string& v) {
  for (int i = 0; names[i] != nullptr; ++i)
    if (v == names[i]) return i;
  return -1;
}

std::string lower(std::string s) {
  for (auto& c : s) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
  return s;
}

// Follow a dotted path into a JSON object; nullptr when absent.
const nlohmann::json* at(const nlohmann::json& root, const std::string& path) {
  const nlohmann::json* cur = &root;
  size_t start = 0;
  while (true) {
    const size_t dot = path.find('.', start);
    const std::string key = path.substr(start, dot == std::string::npos
                                                   ? std::string::npos
                                                   : dot - start);
    if (!cur->is_object() || !cur->contains(key)) return nullptr;
    cur = &(*cur)[key];
    if (dot == std::string::npos) return cur;
    start = dot + 1;
  }
}

// Every leaf path under an object, dotted, so an unknown key can be named.
void leaves(const nlohmann::json& j, const std::string& prefix,
            std::vector<std::string>* out) {
  if (!j.is_object()) {
    out->push_back(prefix);
    return;
  }
  for (auto it = j.begin(); it != j.end(); ++it) {
    const std::string p = prefix.empty() ? it.key() : prefix + "." + it.key();
    if (it.value().is_object()) leaves(it.value(), p, out);
    else out->push_back(p);
  }
}

}  // namespace

const char* SyncConfig::name(ThresholdForm f) { return kThresholdNames[static_cast<int>(f)]; }
const char* SyncConfig::name(PickRule p) { return kPickNames[static_cast<int>(p)]; }
const char* SyncConfig::name(TrackerType t) { return kTrackerNames[static_cast<int>(t)]; }

std::vector<SyncConfig::Knob> SyncConfig::knobs() {
  using K = Knob;
  return {
      // beacon
      {"beacon.type", nullptr, K::kString, 0, 0,
       "Which beacon waveform the base station transmits (legacy, legacy_guard, dot11, nr, nr_pss).",
       &beacon.type, nullptr},
      {"beacon.tx_full_scale", "HOUDINI_BEACON_FS", K::kDouble, 1e-3, 1.0,
       "Transmit peak of the beacon as a fraction of DAC full scale. 0.6 shipped; lower it to stand in for path loss on a cable.",
       &beacon.tx_full_scale, nullptr},
      // detector
      {"detector.threshold", "HOUDINI_BEACON_THRESH", K::kEnum, 0, 0,
       "Decision statistic: auto picks coherence for a single-copy replica and the normalised cross-correlation otherwise; power is the pre-2026-09 form.",
       &detector.threshold, kThresholdNames},
      {"detector.pfa_per_window", nullptr, K::kDouble, 1e-9, 0.5,
       "False-alarm probability per search window for the coherence form; the bar is derived from it and the replica length.",
       &detector.pfa_per_window, nullptr},
      {"detector.pick", "HOUDINI_BEACON_PICK", K::kEnum, 0, 0,
       "Which crossing is returned: first_path (shipped), argmax, cluster_refined, or first_crossing (unsafe on a strong link).",
       &detector.pick, kPickNames},
      {"detector.first_path_window", "HOUDINI_FIRST_PATH_WIN", K::kInt, 0, 4095,
       "Samples the first-path search looks back from the peak. Must stay inside the preamble's self-coherent plateau.",
       &detector.first_path_window, nullptr},
      {"detector.first_path_floor_db", "HOUDINI_FIRST_PATH_DB", K::kDouble, -30.0, 0.0,
       "How much weaker, in dB of path power, an earlier arrival may be and still be taken as the first path.",
       &detector.first_path_floor_db, nullptr},
      // confirm
      {"confirm.snr_floor_db", "HOUDINI_SYNC_SNR_DB", K::kDouble, -10.0, 80.0,
       "In-window SNR a detection must clear. A property of the link and the waveform: re-derive it when either changes.",
       &confirm.snr_floor_db, nullptr},
      // cfo
      {"cfo.index_guard", "HOUDINI_CFO_INDEX_GUARD", K::kInt, 0, 64,
       "Samples the carrier estimator's windows slide later than the detected end (AP-39).",
       &cfo.index_guard, nullptr},
      {"cfo.window_margin", nullptr, K::kInt, 0, 32,
       "Samples shrunk from both ends of each estimator window so neither touches the burst's edge (8.164).",
       &cfo.window_margin, nullptr},
      {"cfo.log_every", "HOUDINI_CFO_LOG_EVERY", K::kInt, 1, 1000000,
       "Print one beacon-CFO log line in this many.",
       &cfo.log_every, nullptr},
      // tracker
      {"tracker.type", "HOUDINI_TRACKER", K::kEnum, 0, 0,
       "Which estimator tracks the base station frame grid: alpha_beta (shipped) or kalman.",
       &tracker.type, kTrackerNames},
      {"tracker.alpha", "HOUDINI_GRID_ALPHA", K::kDouble, 0.0, 1.0,
       "Fraction of each accepted residual applied to the schedule.", &tracker.alpha, nullptr},
      {"tracker.beta", "HOUDINI_GRID_BETA", K::kDouble, 0.0, 1.0,
       "Fraction of the residual applied to the frame period estimate.", &tracker.beta, nullptr},
      {"tracker.step_ppm", "HOUDINI_GRID_STEP_PPM", K::kDouble, 0.0, 1000.0,
       "Most one detection may move the period estimate, ppm. 0 disables the limit.", &tracker.step_ppm, nullptr},
      {"tracker.max_ppm", "HOUDINI_GRID_MAX_PPM", K::kDouble, 0.1, 10000.0,
       "Absolute band the period estimate may occupy either side of nominal, ppm.", &tracker.max_ppm, nullptr},
      {"tracker.trust_ppm", "HOUDINI_GRID_TRUST_PPM", K::kDouble, 0.0, 1000.0,
       "How far the tracked period and a fresh acquisition confirm may disagree before the confirm is preferred, ppm.",
       &tracker.trust_ppm, nullptr},
      {"tracker.kalman.meas_var", "HOUDINI_KF_MEAS_VAR", K::kDouble, 1e-6, 1e6,
       "Kalman only: assumed detector scatter variance, samples squared.", &tracker.kf_meas_var, nullptr},
      {"tracker.kalman.rate_rw", "HOUDINI_KF_RATE_RW", K::kDouble, 0.0, 1.0,
       "Kalman only: how fast the frame period wanders, samples squared per frame cubed.", &tracker.kf_rate_rw, nullptr},
      {"tracker.kalman.innov_gate", "HOUDINI_KF_INNOV_GATE", K::kDouble, 0.0, 100.0,
       "Kalman only: sigmas an observation may sit from the prediction before it is ignored. 0 disables.",
       &tracker.kf_innov_gate, nullptr},
      // resync
      {"resync.residual_ppm", "HOUDINI_SYNC_RESIDUAL_PPM", K::kDouble, 1e-4, 1000.0,
       "Assumed worst-case clock error after tracking; with sync_tol_samples it sets how often the beacon is looked at.",
       &resync.residual_ppm, nullptr},
      {"resync.scatter_tol_us", "HOUDINI_SCATTER_TOL_US", K::kDouble, 0.01, 1000.0,
       "How far a detection may land from the tracked grid and still count as the same beacon, microseconds.",
       &resync.scatter_tol_us, nullptr},
      {"resync.confirm_tol_us", "HOUDINI_CONFIRM_TOL_US", K::kDouble, 0.01, 1000.0,
       "The same tolerance during acquisition. Never applied looser than the tracking gate.",
       &resync.confirm_tol_us, nullptr},
      {"resync.sync_tol_samples", "HOUDINI_SYNC_TOL_SAMPLES", K::kDouble, 1.0, 1e6,
       "Timing slack budgeted to drift between looks, samples. Default: a quarter of the OFDM zero prefix.",
       &resync.sync_tol_samples, nullptr},
      {"resync.retry_max", "HOUDINI_RESYNC_RETRY_MAX", K::kInt, 1, 100000,
       "Misses in one resync period before the client logs an exhausted episode.", &resync.retry_max, nullptr},
      {"resync.escalate_episodes", "HOUDINI_ESCALATE_EPISODES", K::kInt, 1, 1000,
       "Consecutive exhausted episodes before the client abandons tracking and re-acquires.",
       &resync.escalate_episodes, nullptr},
      {"resync.hold_offgrid", "HOUDINI_HOLD_OFFGRID", K::kInt, 1, 1000,
       "Consecutive off-grid detections before the beacon counts as moved.", &resync.hold_offgrid, nullptr},
      {"resync.acq_refine_span", "HOUDINI_ACQ_REFINE_SPAN", K::kInt, 2, 100000,
       "Frames of baseline acquisition wants before it trusts its rate estimate.", &resync.acq_refine_span, nullptr},
      {"resync.acq_max_ppm", "HOUDINI_ACQ_MAX_PPM", K::kDouble, 0.1, 10000.0,
       "Plausibility band applied to a rate that acquisition hands back, ppm.", &resync.acq_max_ppm, nullptr},
      {"allow_env_overrides", nullptr, K::kBool, 0, 0,
       "Whether HOUDINI_* environment variables may override these values (each override is logged). Default true this release.",
       &allow_env_overrides, nullptr},
  };
}

SyncConfig SyncConfig::defaults() {
  SyncConfig c;
  for (const auto& k : c.knobs()) c.provenance[k.path] = "default";
  return c;
}

namespace {

// Assign a parsed value into a knob. Returns an error string, empty on success.
std::string assign(SyncConfig::Knob& k, const nlohmann::json& v,
                   const std::string& src) {
  using K = SyncConfig::Knob;
  switch (k.kind) {
    case K::kDouble: {
      if (!v.is_number()) return "expects a number";
      const double d = v.get<double>();
      if (!std::isfinite(d)) return "is not finite";
      if (d < k.lo || d > k.hi) {
        std::ostringstream o;
        o << "value " << d << " outside [" << k.lo << ", " << k.hi << "]";
        return o.str();
      }
      *static_cast<double*>(k.target) = d;
      return "";
    }
    case K::kInt: {
      if (!v.is_number()) return "expects an integer";
      const double d = v.get<double>();
      if (!std::isfinite(d) || d != std::floor(d)) return "expects a whole number";
      if (d < k.lo || d > k.hi) {
        std::ostringstream o;
        o << "value " << d << " outside [" << k.lo << ", " << k.hi << "]";
        return o.str();
      }
      *static_cast<int*>(k.target) = static_cast<int>(d);
      return "";
    }
    case K::kBool: {
      if (v.is_boolean()) { *static_cast<bool*>(k.target) = v.get<bool>(); return ""; }
      if (v.is_number()) { *static_cast<bool*>(k.target) = v.get<double>() != 0.0; return ""; }
      if (v.is_string()) {
        const std::string s = lower(v.get<std::string>());
        if (s == "true" || s == "1" || s == "yes") { *static_cast<bool*>(k.target) = true; return ""; }
        if (s == "false" || s == "0" || s == "no") { *static_cast<bool*>(k.target) = false; return ""; }
      }
      return "expects true or false";
    }
    case K::kString: {
      if (!v.is_string()) return "expects a string";
      *static_cast<std::string*>(k.target) = v.get<std::string>();
      return "";
    }
    case K::kEnum: {
      if (!v.is_string()) return "expects a name";
      std::string s = lower(v.get<std::string>());
      if (src == "env") {
        for (const auto& a : kEnumAliases)
          if (std::strcmp(a.env, k.env ? k.env : "") == 0 && s == a.value) s = a.canon;
      }
      const int i = enumIndex(k.enum_names, s);
      if (i < 0) {
        std::string names;
        for (int j = 0; k.enum_names[j]; ++j) names += std::string(j ? ", " : "") + k.enum_names[j];
        return "unknown name \"" + s + "\" (one of: " + names + ")";
      }
      // Every enum in this struct is an `enum class X : int`-compatible type
      // with contiguous values from 0 in table order.
      *static_cast<int*>(k.target) = i;
      return "";
    }
  }
  return "unhandled kind";
}

}  // namespace

SyncConfig SyncConfig::load(const std::string& root_json_text) {
  SyncConfig c = defaults();
  const auto root = nlohmann::json::parse(root_json_text, nullptr, true, true);
  const nlohmann::json empty = nlohmann::json::object();
  const nlohmann::json& blk = root.contains("sync") ? root["sync"] : empty;
  if (!blk.is_object()) throw std::invalid_argument("sync: must be an object");

  // Unknown keys are errors: a typo that quietly leaves a knob at its default
  // is exactly the failure this loader exists to make visible.
  std::vector<std::string> present;
  leaves(blk, "", &present);
  auto ks = c.knobs();
  for (const auto& p : present) {
    if (p.compare(0, 1, "_") == 0) continue;  // "_note" style comments allowed
    bool known = false;
    for (const auto& k : ks) if (p == k.path) { known = true; break; }
    if (!known) throw std::invalid_argument("sync." + p + ": unknown key");
  }
  for (auto& k : ks) {
    const nlohmann::json* v = at(blk, k.path);
    if (v == nullptr) continue;
    const std::string err = assign(k, *v, "json");
    if (!err.empty()) throw std::invalid_argument("sync." + std::string(k.path) + ": " + err);
    c.provenance[k.path] = "json";
  }
  // The legacy top-level beacon_type (config.cc reads it for genPilots).
  if (root.contains("beacon_type") && root["beacon_type"].is_string()) {
    const std::string top = root["beacon_type"].get<std::string>();
    if (c.provenance["beacon.type"] == "json" && top != c.beacon.type) {
      throw std::invalid_argument("beacon_type \"" + top + "\" and sync.beacon.type \"" +
                                  c.beacon.type + "\" disagree");
    }
    if (c.provenance["beacon.type"] != "json") {
      c.beacon.type = top;
      c.provenance["beacon.type"] = "json";
    }
  }
  // Environment overrides, when allowed. Unlike the old readers these are
  // range-checked the same way as JSON, and every one is recorded.
  for (auto& k : ks) {
    if (k.env == nullptr) continue;
    const char* e = std::getenv(k.env);
    if (e == nullptr) continue;
    if (!c.allow_env_overrides) {
      c.warnings.push_back(std::string(k.env) + "=\"" + e +
                           "\" IGNORED: sync.allow_env_overrides is false");
      continue;
    }
    nlohmann::json v;
    switch (k.kind) {
      case Knob::kDouble: case Knob::kInt: {
        char* end = nullptr;
        const double d = std::strtod(e, &end);
        if (end == e || *end != '\0') {
          c.warnings.push_back(std::string(k.env) + "=\"" + e + "\" is not a number -- ignored");
          continue;
        }
        v = d;
        break;
      }
      case Knob::kBool: case Knob::kString: case Knob::kEnum:
        v = std::string(e);
        break;
    }
    const std::string err = assign(k, v, "env");
    if (!err.empty()) {
      c.warnings.push_back(std::string(k.env) + "=\"" + e + "\": " + err + " -- ignored");
      continue;
    }
    c.provenance[k.path] = "env";
    c.warnings.push_back(std::string(k.env) + " overrides sync." + k.path + " (" + e + ")");
  }
  c.validate();
  return c;
}

void SyncConfig::validate() {
  // Cross-constraints the ledger paid for.
  if (resync.confirm_tol_us < resync.scatter_tol_us) {
    // Not an error: sync_geometry clamps confirm to scatter. Say so.
    warnings.push_back("resync.confirm_tol_us is tighter than scatter_tol_us; "
                       "acquisition uses the tighter value (8.65)");
  }
  if (detector.pick == PickRule::kFirstCrossing) {
    warnings.push_back("detector.pick first_crossing false-locks on the beacon's own "
                       "preamble once the link is strong (AP-34); diagnostic only");
  }
  if (detector.threshold == ThresholdForm::kPowerRatio) {
    warnings.push_back("detector.threshold power is a different test at every "
                       "received level (8.138); diagnostic only");
  }
  if (tracker.type == TrackerType::kAlphaBeta && tracker.alpha == 0.0 && tracker.beta == 0.0) {
    warnings.push_back("tracker alpha and beta are both 0: the grid is fixed-period");
  }
  if (beacon.type != "legacy" && provenance["confirm.snr_floor_db"] == "default") {
    warnings.push_back("confirm.snr_floor_db is the legacy default with beacon.type \"" +
                       beacon.type + "\": a quieter waveform is rejected wholesale at a "
                       "level legacy clears (8.157); derive the floor from this beacon's "
                       "in-window SNR");
  }
}

std::string SyncConfig::describe() const {
  std::ostringstream o;
  SyncConfig& self = const_cast<SyncConfig&>(*this);  // knobs() needs pointers into *this
  o << "sync configuration (value  provenance):\n";
  for (const auto& k : self.knobs()) {
    o << "  " << k.path << " = ";
    switch (k.kind) {
      case Knob::kDouble: o << *static_cast<double*>(k.target); break;
      case Knob::kInt: o << *static_cast<int*>(k.target); break;
      case Knob::kBool: o << (*static_cast<bool*>(k.target) ? "true" : "false"); break;
      case Knob::kString: o << *static_cast<std::string*>(k.target); break;
      case Knob::kEnum: o << k.enum_names[*static_cast<int*>(k.target)]; break;
    }
    auto it = provenance.find(k.path);
    o << "  [" << (it == provenance.end() ? "default" : it->second) << "]\n";
  }
  for (const auto& w : warnings) o << "  note: " << w << "\n";
  return o.str();
}

std::string SyncConfig::schemaMarkdown() {
  SyncConfig c = defaults();
  std::ostringstream o;
  o << "| key | default | was | range | what it does |\n| --- | --- | --- | --- | --- |\n";
  for (const auto& k : c.knobs()) {
    o << "| `sync." << k.path << "` | ";
    switch (k.kind) {
      case Knob::kDouble: {
        const double d = *static_cast<double*>(k.target);
        if (std::isnan(d)) o << "derived"; else o << d;
        break;
      }
      case Knob::kInt: o << *static_cast<int*>(k.target); break;
      case Knob::kBool: o << (*static_cast<bool*>(k.target) ? "true" : "false"); break;
      case Knob::kString: o << "`" << *static_cast<std::string*>(k.target) << "`"; break;
      case Knob::kEnum: o << "`" << k.enum_names[*static_cast<int*>(k.target)] << "`"; break;
    }
    o << " | " << (k.env ? std::string("`") + k.env + "`" : "") << " | ";
    if (k.kind == Knob::kDouble || k.kind == Knob::kInt) o << k.lo << " to " << k.hi;
    else if (k.kind == Knob::kEnum) {
      for (int j = 0; k.enum_names[j]; ++j) o << (j ? ", " : "") << k.enum_names[j];
    }
    o << " | " << k.doc << " |\n";
  }
  return o.str();
}

}  // namespace sync
}  // namespace houdini
