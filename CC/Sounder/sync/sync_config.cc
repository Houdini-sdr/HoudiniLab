/**
 * @file sync/sync_config.cc
 * @brief SyncConfig: the static schema, the loader, validation, resolution,
 *        and the two generated views (startup description, walkthrough table).
 */
#include "sync/sync_config.h"

#include <algorithm>
#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <sstream>
#include <stdexcept>

#include "nlohmann/json.hpp"

namespace houdini {
namespace sync {

namespace {

const char* const kThresholdNames[] = {"auto", "power", "xcorr", "coherence", nullptr};
const char* const kPickNames[] = {"first_crossing", "cluster_refined", "argmax", "first_path",
                                  nullptr};
const char* const kTrackerNames[] = {"alpha_beta", "kalman", nullptr};
const char* const kSourceNames[] = {"default", "json", "env", "derived"};
const char* const kPlatformNames[] = {"houdini", "iris_uhd"};

// The environment spellings that predate the table.
struct EnvAlias {
  const char* env;
  const char* value;  // env spelling
  const char* canon;  // table spelling
};
const EnvAlias kEnumAliases[] = {
    {"HOUDINI_BEACON_THRESH", "nolag", "coherence"},
    {"HOUDINI_BEACON_PICK", "first", "first_crossing"},
    {"HOUDINI_BEACON_PICK", "firstpath", "first_path"},
    {"HOUDINI_BEACON_PICK", "first-path", "first_path"},
    {"HOUDINI_TRACKER", "ab", "alpha_beta"},
    {"HOUDINI_TRACKER", "kf", "kalman"},
    {"HOUDINI_TRACKER", "1", "kalman"},
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
    const std::string key =
        path.substr(start, dot == std::string::npos ? std::string::npos : dot - start);
    if (!cur->is_object() || !cur->contains(key)) return nullptr;
    cur = &(*cur)[key];
    if (dot == std::string::npos) return cur;
    start = dot + 1;
  }
}

// Every leaf path under an object, dotted, so an unknown key can be named.
// A comment key ("_" first) is skipped on the RAW key, dot or no dot. A
// non-comment key that itself contains a dot ("detector.threshold" as ONE key)
// can never be reached by the dotted walker, so it is refused with a reason
// rather than passing as known.
void leaves(const nlohmann::json& j, const std::string& prefix, std::vector<std::string>* out) {
  if (!j.is_object()) {
    out->push_back(prefix);
    return;
  }
  for (auto it = j.begin(); it != j.end(); ++it) {
    if (!it.key().empty() && it.key()[0] == '_') continue;
    if (it.key().find('.') != std::string::npos) {
      throw std::invalid_argument("sync." + (prefix.empty() ? "" : prefix + ".") + it.key() +
                                  ": the key contains a dot; nest it as objects instead");
    }
    const std::string p = prefix.empty() ? it.key() : prefix + "." + it.key();
    if (it.value().is_object()) leaves(it.value(), p, out);
    else out->push_back(p);
  }
}

// Typed reads and writes through the accessor variant.
int enumValue(const SyncConfig& c, const SyncConfig::Spec& s) {
  if (auto a = std::get_if<SyncConfig::Access<ThresholdForm>>(&s.access)) return static_cast<int>(a->cref(c));
  if (auto a = std::get_if<SyncConfig::Access<PickRule>>(&s.access)) return static_cast<int>(a->cref(c));
  if (auto a = std::get_if<SyncConfig::Access<TrackerType>>(&s.access)) return static_cast<int>(a->cref(c));
  return -1;
}

void setEnum(SyncConfig& c, const SyncConfig::Spec& s, int i) {
  if (auto a = std::get_if<SyncConfig::Access<ThresholdForm>>(&s.access)) {
    a->ref(c) = static_cast<ThresholdForm>(i);
  } else if (auto a2 = std::get_if<SyncConfig::Access<PickRule>>(&s.access)) {
    a2->ref(c) = static_cast<PickRule>(i);
  } else if (auto a3 = std::get_if<SyncConfig::Access<TrackerType>>(&s.access)) {
    a3->ref(c) = static_cast<TrackerType>(i);
  }
}

// Assign a parsed value into a knob. Returns an error string, empty on success.
// `clamp` (an environment override with policy kClamp) pulls an out-of-range
// number into the range and reports it through `note` instead of failing.
std::string assign(SyncConfig& c, const SyncConfig::Spec& s, const nlohmann::json& v,
                   bool from_env, bool clamp, std::string* note) {
  if (auto ad = std::get_if<SyncConfig::Access<double>>(&s.access)) {
    if (!v.is_number()) return "expects a number";
    double d = v.get<double>();
    if (!std::isfinite(d)) return "is not finite";
    if (d < s.lo || d > s.hi) {
      if (!clamp) {
        std::ostringstream o;
        o << "value " << d << " outside [" << s.lo << ", " << s.hi << "]";
        return o.str();
      }
      const double cl = d < s.lo ? s.lo : s.hi;
      std::ostringstream o;
      o << "clamped " << d << " to " << cl;
      *note = o.str();
      d = cl;
    }
    ad->ref(c) = d;
    return "";
  }
  if (auto ai = std::get_if<SyncConfig::Access<int>>(&s.access)) {
    if (!v.is_number()) return "expects an integer";
    double d = v.get<double>();
    if (!std::isfinite(d)) return "is not finite";
    if (d != std::floor(d)) {
      if (!clamp) return "expects a whole number";
      std::ostringstream o;
      o << "floored " << d << " to " << std::floor(d);
      *note = o.str();
      d = std::floor(d);
    }
    if (d < s.lo || d > s.hi) {
      if (!clamp) {
        std::ostringstream o;
        o << "value " << d << " outside [" << s.lo << ", " << s.hi << "]";
        return o.str();
      }
      const double cl = d < s.lo ? s.lo : s.hi;
      std::ostringstream o;
      o << (note->empty() ? "" : *note + "; ") << "clamped " << d << " to " << cl;
      *note = o.str();
      d = cl;
    }
    ai->ref(c) = static_cast<int>(d);
    return "";
  }
  if (auto ab = std::get_if<SyncConfig::Access<bool>>(&s.access)) {
    if (v.is_boolean()) { ab->ref(c) = v.get<bool>(); return ""; }
    if (v.is_number()) { ab->ref(c) = v.get<double>() != 0.0; return ""; }
    if (v.is_string()) {
      const std::string t = lower(v.get<std::string>());
      if (t == "true" || t == "1" || t == "yes") { ab->ref(c) = true; return ""; }
      if (t == "false" || t == "0" || t == "no") { ab->ref(c) = false; return ""; }
    }
    return "expects true or false";
  }
  if (auto as = std::get_if<SyncConfig::Access<std::string>>(&s.access)) {
    if (!v.is_string()) return "expects a string";
    as->ref(c) = v.get<std::string>();
    return "";
  }
  // enum
  if (!v.is_string()) return "expects a name";
  std::string t = lower(v.get<std::string>());
  if (from_env && s.env != nullptr) {
    for (const auto& a : kEnumAliases)
      if (std::strcmp(a.env, s.env) == 0 && t == a.value) t = a.canon;
  }
  const int i = enumIndex(s.enum_names, t);
  if (i < 0) {
    std::string names;
    for (int j = 0; s.enum_names[j]; ++j) names += std::string(j ? ", " : "") + s.enum_names[j];
    return "unknown name \"" + t + "\" (one of: " + names + ")";
  }
  setEnum(c, s, i);
  return "";
}

// Accessor pairs as plain functions of a SyncConfig, one macro per member so
// the schema below reads as a table.
#define KNOB_ACCESS(T, member)                                                          \
  SyncConfig::Access<T> {                                                               \
    [](SyncConfig& c) -> T& { return c.member; },                                       \
        [](const SyncConfig& c) -> const T& { return c.member; }                        \
  }

using EP = SyncConfig::EnvPolicy;

}  // namespace

const char* name(Source s) { return kSourceNames[static_cast<int>(s)]; }
const char* name(Platform p) { return kPlatformNames[static_cast<int>(p)]; }
const char* name(ThresholdForm f) { return kThresholdNames[static_cast<int>(f)]; }
const char* name(PickRule p) { return kPickNames[static_cast<int>(p)]; }
const char* name(TrackerType t) { return kTrackerNames[static_cast<int>(t)]; }

const std::vector<SyncConfig::Spec>& SyncConfig::schema() {
  static const std::vector<Spec> kSchema = {
      // beacon
      {"beacon.type", nullptr, 0, 0,
       "Which beacon waveform the base station transmits (legacy, legacy_guard, dot11, nr, nr_pss).",
       KNOB_ACCESS(std::string, beacon.type), nullptr, EP::kClamp},
      {"beacon.tx_full_scale", "HOUDINI_BEACON_FS", 1e-3, 1.0,
       "Transmit peak of the beacon as a fraction of DAC full scale. 0.6 shipped; lower it to stand in for path loss on a cable.",
       KNOB_ACCESS(double, beacon.tx_full_scale), nullptr, EP::kIgnoreOutOfRange},
      // detector
      {"detector.threshold", "HOUDINI_BEACON_THRESH", 0, 0,
       "Decision statistic: auto picks coherence for a single-copy replica and the normalised cross-correlation otherwise; power is the pre-2026-09 form and the Iris/UHD default.",
       KNOB_ACCESS(ThresholdForm, detector.threshold), kThresholdNames, EP::kClamp},
      {"detector.pfa_per_window", nullptr, 1e-9, 0.5,
       "RESERVED (phase P3), not applied yet: the false-alarm probability per search window the coherence form's bar will be derived from.",
       KNOB_ACCESS(double, detector.pfa_per_window), nullptr, EP::kClamp},
      {"detector.pick", "HOUDINI_BEACON_PICK", 0, 0,
       "Which crossing is returned: first_path (the Houdini default), argmax, cluster_refined, or first_crossing (the Iris/UHD default; unsafe on a strong link).",
       KNOB_ACCESS(PickRule, detector.pick), kPickNames, EP::kClamp},
      {"detector.first_path_window", "HOUDINI_FIRST_PATH_WIN", -1, 4095,
       "Samples the first-path search looks back from the peak; -1 (default) means half the replica length. Must stay inside the preamble's self-coherent plateau. A correlator quantity: samples, not scaled with the rate.",
       KNOB_ACCESS(int, detector.first_path_window), nullptr, EP::kIgnoreOutOfRange},
      {"detector.first_path_floor_db", "HOUDINI_FIRST_PATH_DB", -30.0, 0.0,
       "How much weaker, in dB of path power, an earlier arrival may be and still be taken as the first path.",
       KNOB_ACCESS(double, detector.first_path_floor_db), nullptr, EP::kIgnoreOutOfRange},
      {"detector.corr_scale", nullptr, 1e-4, 1e7,
       "Resync detection threshold: the bar is 1 / corr_scale, relaxed by one per retry. Read from the legacy per-client top-level array when absent.",
       KNOB_ACCESS(double, detector.bar.corr_scale), nullptr, EP::kClamp},
      {"detector.corr_scale_init", nullptr, 1e-4, 1e7,
       "Acquisition detection threshold (bar 1 / corr_scale_init); defaults to corr_scale.",
       KNOB_ACCESS(double, detector.bar.corr_scale_init), nullptr, EP::kClamp},
      {"detector.corr_threads", "SOUNDER_CORR_THREADS", 1, 256,
       "Threads for the correlator's matched filter. 1 shipped; measured a net loss below ~4 on the rig host.",
       KNOB_ACCESS(int, detector.corr_threads), nullptr, EP::kClamp},
      // confirm
      {"confirm.snr_floor_db", "HOUDINI_SYNC_SNR_DB", -10.0, 80.0,
       "In-window SNR a detection must clear. A property of the link and the waveform: re-derive it when either changes.",
       KNOB_ACCESS(double, confirm.snr_floor_db), nullptr, EP::kClamp},
      // cfo
      {"cfo.index_guard", "HOUDINI_CFO_INDEX_GUARD", 0, 64,
       "Samples the carrier estimator's windows slide later than the detected end (AP-39). A correlator quantity: samples, not scaled with the rate.",
       KNOB_ACCESS(int, cfo.index_guard), nullptr, EP::kClamp},
      {"cfo.window_margin", nullptr, 0, 32,
       "Samples shrunk from both ends of each estimator window so neither touches the burst's edge (8.164). A correlator quantity: samples, not scaled with the rate.",
       KNOB_ACCESS(int, cfo.window_margin), nullptr, EP::kClamp},
      {"cfo.log_every", "HOUDINI_CFO_LOG_EVERY", 1, 1000000,
       "Print one beacon-CFO log line in this many.", KNOB_ACCESS(int, cfo.log_every), nullptr,
       EP::kClamp},
      // tracker
      {"tracker.type", "HOUDINI_TRACKER", 0, 0,
       "Which estimator tracks the base station frame grid: alpha_beta (shipped) or kalman.",
       KNOB_ACCESS(TrackerType, tracker.type), kTrackerNames, EP::kClamp},
      {"tracker.alpha", "HOUDINI_GRID_ALPHA", 0.0, 1.0,
       "Fraction of each accepted residual applied to the schedule.", KNOB_ACCESS(double, tracker.alpha),
       nullptr, EP::kClamp},
      {"tracker.beta", "HOUDINI_GRID_BETA", 0.0, 1.0,
       "Fraction of the residual applied to the frame period estimate.", KNOB_ACCESS(double, tracker.beta),
       nullptr, EP::kClamp},
      {"tracker.step_ppm", "HOUDINI_GRID_STEP_PPM", 0.0, 1000.0,
       "Most one detection may move the period estimate, ppm. 0 disables the limit.",
       KNOB_ACCESS(double, tracker.step_ppm), nullptr, EP::kClamp},
      {"tracker.max_ppm", "HOUDINI_GRID_MAX_PPM", 0.1, 10000.0,
       "Absolute band the period estimate may occupy either side of nominal, ppm.",
       KNOB_ACCESS(double, tracker.max_ppm), nullptr, EP::kClamp},
      {"tracker.trust_ppm", "HOUDINI_GRID_TRUST_PPM", 0.0, 1000.0,
       "How far the tracked period and a fresh acquisition confirm may disagree before the confirm is preferred, ppm.",
       KNOB_ACCESS(double, tracker.trust_ppm), nullptr, EP::kClamp},
      {"tracker.kalman.meas_var", "HOUDINI_KF_MEAS_VAR", 1e-6, 1e6,
       "Kalman only: assumed detector scatter variance, samples squared.",
       KNOB_ACCESS(double, tracker.kf_meas_var), nullptr, EP::kClamp},
      {"tracker.kalman.rate_rw", "HOUDINI_KF_RATE_RW", 0.0, 1.0,
       "Kalman only: how fast the frame period wanders, samples squared per frame cubed.",
       KNOB_ACCESS(double, tracker.kf_rate_rw), nullptr, EP::kClamp},
      {"tracker.kalman.innov_gate", "HOUDINI_KF_INNOV_GATE", 0.0, 100.0,
       "Kalman only: sigmas an observation may sit from the prediction before it is ignored. 0 disables.",
       KNOB_ACCESS(double, tracker.kf_innov_gate), nullptr, EP::kClamp},
      // resync
      {"resync.residual_ppm", "HOUDINI_SYNC_RESIDUAL_PPM", 1e-4, 1000.0,
       "Assumed worst-case clock error after tracking; with sync_tol_samples it sets how often the beacon is looked at.",
       KNOB_ACCESS(double, resync.residual_ppm), nullptr, EP::kClamp},
      {"resync.scatter_tol_us", "HOUDINI_SCATTER_TOL_US", 0.01, 1000.0,
       "How far a detection may land from the tracked grid and still count as the same beacon, microseconds.",
       KNOB_ACCESS(double, resync.scatter_tol_us), nullptr, EP::kClamp},
      {"resync.confirm_tol_us", "HOUDINI_CONFIRM_TOL_US", 0.01, 1000.0,
       "The same tolerance during acquisition. Never applied looser than the tracking gate.",
       KNOB_ACCESS(double, resync.confirm_tol_us), nullptr, EP::kClamp},
      {"resync.sync_tol_samples", "HOUDINI_SYNC_TOL_SAMPLES", 0.5, 1e6,
       "Timing slack budgeted to drift between looks, samples. Default: a quarter of the OFDM zero prefix.",
       KNOB_ACCESS(double, resync.sync_tol_samples), nullptr, EP::kClamp},
      {"resync.retry_max", "HOUDINI_RESYNC_RETRY_MAX", 1, 100000,
       "Misses in one resync period before the client logs an exhausted episode.",
       KNOB_ACCESS(int, resync.retry_max), nullptr, EP::kClamp},
      {"resync.escalate_episodes", "HOUDINI_ESCALATE_EPISODES", 1, 1000,
       "Consecutive exhausted episodes before the client abandons tracking and re-acquires.",
       KNOB_ACCESS(int, resync.escalate_episodes), nullptr, EP::kClamp},
      {"resync.hold_offgrid", "HOUDINI_HOLD_OFFGRID", 1, 1000,
       "Consecutive off-grid detections before the beacon counts as moved.",
       KNOB_ACCESS(int, resync.hold_offgrid), nullptr, EP::kClamp},
      {"resync.acq_refine_span", "HOUDINI_ACQ_REFINE_SPAN", 2, 100000,
       "Frames of baseline acquisition wants before it trusts its rate estimate.",
       KNOB_ACCESS(int, resync.acq_refine_span), nullptr, EP::kClamp},
      {"resync.acq_max_ppm", "HOUDINI_ACQ_MAX_PPM", 0.1, 10000.0,
       "Plausibility band applied to a rate that acquisition hands back, ppm.",
       KNOB_ACCESS(double, resync.acq_max_ppm), nullptr, EP::kClamp},
      {"allow_env_overrides", nullptr, 0, 0,
       "Whether HOUDINI_* environment variables may override these values (each override is logged; see the policy column). Default true this release.",
       KNOB_ACCESS(bool, allow_env_overrides), nullptr, EP::kClamp},
  };
  return kSchema;
}

#undef KNOB_ACCESS

const SyncConfig::Spec* SyncConfig::spec(std::string_view path) {
  for (const auto& s : schema())
    if (path == s.path) return &s;
  return nullptr;
}

size_t SyncConfig::indexOf(std::string_view path) {
  const auto& sc = schema();
  for (size_t i = 0; i < sc.size(); ++i)
    if (path == sc[i].path) return i;
  throw std::logic_error("sync schema: no such knob \"" + std::string(path) + "\"");
}

Source SyncConfig::provenanceOf(std::string_view path) const {
  const auto& sc = schema();
  for (size_t i = 0; i < sc.size(); ++i)
    if (path == sc[i].path) return i < provenance_.size() ? provenance_[i] : Source::kDefault;
  return Source::kDefault;
}

void SyncConfig::setProvenance(size_t index, Source s) {
  if (provenance_.size() < schema().size()) provenance_.resize(schema().size(), Source::kDefault);
  if (index >= provenance_.size()) throw std::logic_error("sync schema: provenance index out of range");
  provenance_[index] = s;
}

SyncConfig SyncConfig::defaults() {
  SyncConfig c;
  c.provenance_.assign(schema().size(), Source::kDefault);
  return c;
}

SyncConfig SyncConfig::loadFromText(const std::string& root_json_text) {
  const auto root = nlohmann::json::parse(root_json_text, nullptr, true, true);
  std::optional<std::string> block;
  if (root.contains("sync") && !root["sync"].is_null()) block = root["sync"].dump();
  std::optional<std::string> bt;
  if (root.contains("beacon_type")) {
    if (!root["beacon_type"].is_string())
      throw std::invalid_argument("beacon_type: expects a string");
    bt = root["beacon_type"].get<std::string>();
  }
  SyncConfig c = load(block, bt);
  // The sounder's own fallbacks for the legacy arrays: 1 for an absent
  // corr_scale, corr_scale for an absent corr_scale_init.
  const bool has_cs = root.contains("corr_scale") && root["corr_scale"].is_array() &&
                      !root["corr_scale"].empty();
  const bool has_csi = root.contains("corr_scale_init") && root["corr_scale_init"].is_array() &&
                       !root["corr_scale_init"].empty();
  const double cs = has_cs ? root["corr_scale"][0].get<double>() : 1.0;
  const double csi = has_csi ? root["corr_scale_init"][0].get<double>() : cs;
  c.adoptLegacyThreshold(cs, csi, has_cs, has_csi);
  return c;
}

void SyncConfig::adoptLegacyThreshold(double corr_scale, double corr_scale_init,
                                      bool corr_scale_in_file, bool corr_scale_init_in_file) {
  auto adopt = [this](const char* path, double v, bool in_file, const char* what) {
    if (provenanceOf(path) == Source::kJson) return;  // the sync block wins
    std::string note;
    const std::string err = assign(*this, *spec(path), nlohmann::json(v), false, false, &note);
    if (!err.empty()) throw std::invalid_argument(std::string(what) + ": " + err);
    setProvenance(path, in_file ? Source::kJson : Source::kDerived);
  };
  adopt("detector.corr_scale", corr_scale, corr_scale_in_file, "corr_scale");
  if (corr_scale_init_in_file) {
    adopt("detector.corr_scale_init", corr_scale_init, true, "corr_scale_init");
  } else if (provenanceOf("detector.corr_scale_init") != Source::kJson) {
    // An absent corr_scale_init follows the EFFECTIVE corr_scale, whichever
    // source set that (the block wins over the array).
    detector.bar.corr_scale_init = detector.bar.corr_scale;
    setProvenance("detector.corr_scale_init", Source::kDerived);
  }
}

SyncConfig SyncConfig::load(const std::optional<std::string>& sync_block_json,
                            const std::optional<std::string>& legacy_beacon_type) {
  SyncConfig c = defaults();
  const auto& sc = schema();
  nlohmann::json blk = nlohmann::json::object();
  if (sync_block_json.has_value()) {
    blk = nlohmann::json::parse(*sync_block_json, nullptr, true, true);
    if (blk.is_null()) blk = nlohmann::json::object();
    if (!blk.is_object()) throw std::invalid_argument("sync: must be an object");
  }
  // Unknown keys are errors: a typo that quietly leaves a knob at its default
  // is exactly the failure this loader exists to make visible.
  std::vector<std::string> present;
  leaves(blk, "", &present);
  for (const auto& p : present) {
    if (spec(p) == nullptr) throw std::invalid_argument("sync." + p + ": unknown key");
  }
  for (size_t i = 0; i < sc.size(); ++i) {
    const nlohmann::json* v = at(blk, sc[i].path);
    if (v == nullptr) continue;
    std::string note;
    const std::string err = assign(c, sc[i], *v, false, false, &note);
    if (!err.empty()) throw std::invalid_argument("sync." + std::string(sc[i].path) + ": " + err);
    c.setProvenance(i, Source::kJson);
  }
  // The legacy key the caller found at the top level of its file.
  if (legacy_beacon_type.has_value()) {
    if (c.provenanceOf("beacon.type") == Source::kJson && *legacy_beacon_type != c.beacon.type) {
      throw std::invalid_argument("beacon_type \"" + *legacy_beacon_type +
                                  "\" and sync.beacon.type \"" + c.beacon.type + "\" disagree");
    }
    if (c.provenanceOf("beacon.type") != Source::kJson) {
      std::string note;
      const std::string err = assign(c, *spec("beacon.type"), nlohmann::json(*legacy_beacon_type),
                                     false, false, &note);
      if (!err.empty()) throw std::invalid_argument("beacon_type: " + err);
      c.setProvenance("beacon.type", Source::kJson);
    }
  }
  // corr_scale_init follows corr_scale unless set (the sounder's own rule).
  if (c.provenanceOf("detector.corr_scale_init") == Source::kDefault &&
      c.provenanceOf("detector.corr_scale") != Source::kDefault) {
    c.detector.bar.corr_scale_init = c.detector.bar.corr_scale;
    c.setProvenance("detector.corr_scale_init", Source::kDerived);
  }
  // Environment overrides, when allowed: range-checked like JSON, then the
  // knob's policy decides what an out-of-range number does (clamp, or keep
  // the value already in place, each with a note); garbage is refused and
  // reported; every override is recorded.
  for (size_t i = 0; i < sc.size(); ++i) {
    const auto& s = sc[i];
    if (s.env == nullptr) continue;
    const char* e = std::getenv(s.env);
    if (e == nullptr) continue;
    if (!c.allow_env_overrides) {
      c.warnings_.push_back(std::string(s.env) + "=\"" + e +
                            "\" IGNORED: sync.allow_env_overrides is false");
      continue;
    }
    nlohmann::json v;
    if (s.isNumeric()) {
      char* end = nullptr;
      const double d = std::strtod(e, &end);
      if (end == e || *end != '\0') {
        c.warnings_.push_back(std::string(s.env) + "=\"" + e + "\" is not a number -- ignored");
        continue;
      }
      v = d;
    } else {
      v = std::string(e);
    }
    std::string note;
    const bool clamp = s.env_policy == EnvPolicy::kClamp;
    const std::string err = assign(c, s, v, true, clamp, &note);
    if (!err.empty()) {
      c.warnings_.push_back(std::string(s.env) + "=\"" + e + "\": " + err +
                            (clamp ? " -- ignored"
                                   : " -- ignored, the value already in place (" + c.valueText(s) +
                                         ") kept, as the old reader did"));
      continue;
    }
    c.setProvenance(i, Source::kEnv);
    c.warnings_.push_back(std::string(s.env) + " overrides sync." + s.path + " (" + e + ")" +
                          (note.empty() ? "" : " [" + note + "]"));
  }
  c.validate();
  return c;
}

void SyncConfig::resolve(const ResolveContext& ctx) {
  // -1 means "half the replica length": what the pre-library correlator
  // derived by default (64 at 128 taps, 32 at 64).
  if (detector.first_path_window < 0 && ctx.replica_len > 0) {
    detector.first_path_window = static_cast<int>(ctx.replica_len / 2);
    setProvenance("detector.first_path_window", Source::kDerived);
  }
  // NaN means "a quarter of the OFDM zero prefix".
  if (std::isnan(resync.sync_tol_samples) && ctx.prefix_samples > 0.0) {
    resync.sync_tol_samples = ctx.prefix_samples / 4.0;
    setProvenance("resync.sync_tol_samples", Source::kDerived);
  }
  // The Iris/UHD defaults are the rules that framer has always run (master
  // returned the first threshold crossing under the power-ratio form); the
  // Houdini defaults are the measured ones. A value the JSON sets is honoured
  // on either platform, which is what makes the change a choice.
  if (ctx.platform == Platform::kIrisUhd) {
    if (!wasSet("detector.pick")) {
      detector.pick = PickRule::kFirstCrossing;
      setProvenance("detector.pick", Source::kDerived);
    }
    if (!wasSet("detector.threshold")) {
      detector.threshold = ThresholdForm::kPowerRatio;
      setProvenance("detector.threshold", Source::kDerived);
    }
  }
  // A single-copy replica has no lag product to take: the detector runs the
  // coherence form whatever was asked (8.154), and the record must say so.
  if (ctx.single_copy_replica && detector.threshold != ThresholdForm::kCoherence) {
    detector.threshold = ThresholdForm::kCoherence;
    setProvenance("detector.threshold", Source::kDerived);
  }
  // Several clients: the receiver applies the legacy per-client arrays, not
  // the block's one value.
  clients_ = ctx.clients;
  validate();
}

double ThresholdPolicy::coherenceBar(size_t replica_len, double pfa_per_window,
                                     size_t window_samples) {
  const double L = static_cast<double>(std::max<size_t>(2, replica_len));
  const double pw = std::min(0.5, std::max(1e-12, pfa_per_window));
  const double pidx = pw / static_cast<double>(std::max<size_t>(1, window_samples));
  return 1.0 - std::pow(pidx, 1.0 / (L - 1.0));
}

void SyncConfig::validate() {
  warnings_.erase(std::remove_if(warnings_.begin(), warnings_.end(),
                                 [](const std::string& w) { return w.rfind("note:", 0) == 0; }),
                  warnings_.end());
  auto note = [this](const std::string& w) { warnings_.push_back("note: " + w); };
  // Cross-constraints the ledger paid for.
  if (resync.confirm_tol_us < resync.scatter_tol_us) {
    note("resync.confirm_tol_us is tighter than scatter_tol_us; acquisition uses the tighter "
         "value (8.65)");
  }
  if (detector.pick == PickRule::kFirstCrossing) {
    note(std::string("detector.pick first_crossing false-locks on the beacon's own preamble once "
                     "the link is strong (AP-34)") +
         (provenanceOf("detector.pick") == Source::kDerived
              ? "; it is the Iris/UHD default, the rule that framer has always run"
              : "; diagnostic only on Houdini"));
  }
  if (clients_ > 1 && (wasSet("detector.corr_scale") || wasSet("detector.corr_scale_init"))) {
    note(std::to_string(clients_) + " clients are configured: the receiver applies the legacy "
         "per-client corr_scale / corr_scale_init arrays, not sync.detector.corr_scale");
  }
  if (detector.pick == PickRule::kArgmax) {
    note("detector.pick argmax returns the STRONGEST path, which over the air is often a "
         "reflection that hops as the channel fades (8.143); diagnostic only");
  }
  if (detector.threshold == ThresholdForm::kPowerRatio) {
    note(std::string("detector.threshold power is a different test at every received level "
                     "(8.138)") +
         (provenanceOf("detector.threshold") == Source::kDerived
              ? "; it is the Iris/UHD default, the form that framer has always run"
              : "; diagnostic only on Houdini"));
  }
  if (detector.first_path_window > 512) {
    note("detector.first_path_window above 512 reaches past any preamble plateau and widens "
         "the SNR guard to most of a slot; the correlator caps it at twice the replica length");
  }
  if (wasSet("detector.pfa_per_window")) {
    note("detector.pfa_per_window is RESERVED (phase P3) and not applied: the detector still "
         "uses corr_scale for every form");
  }
  if (tracker.type == TrackerType::kAlphaBeta && tracker.alpha == 0.0 && tracker.beta == 0.0) {
    note("tracker alpha and beta are both 0: the grid is fixed-period");
  }
  if (beacon.type != "legacy" && !wasSet("confirm.snr_floor_db")) {
    note("confirm.snr_floor_db is the legacy default with beacon.type \"" + beacon.type +
         "\": a quieter waveform is rejected wholesale at a level legacy clears (8.157); derive "
         "the floor from this beacon's in-window SNR");
  }
}

std::string SyncConfig::valueText(const Spec& s) const {
  std::ostringstream o;
  if (auto ad = std::get_if<Access<double>>(&s.access)) {
    if (std::isnan(ad->cref(*this))) return "derived";
    o << ad->cref(*this);
  } else if (auto ai = std::get_if<Access<int>>(&s.access)) {
    if (ai->cref(*this) < 0 && s.lo < 0) return "derived";
    o << ai->cref(*this);
  } else if (auto ab = std::get_if<Access<bool>>(&s.access)) {
    o << (ab->cref(*this) ? "true" : "false");
  } else if (auto as = std::get_if<Access<std::string>>(&s.access)) {
    o << as->cref(*this);
  } else {
    o << s.enum_names[enumValue(*this, s)];
  }
  return o.str();
}

std::string SyncConfig::describe() const {
  std::ostringstream o;
  o << "sync configuration (value  provenance):\n";
  for (const auto& s : schema())
    o << "  " << s.path << " = " << valueText(s) << "  [" << name(provenanceOf(s.path)) << "]\n";
  for (const auto& w : warnings_) o << "  " << (w.rfind("note:", 0) == 0 ? w : "note: " + w) << "\n";
  return o.str();
}

std::string SyncConfig::schemaMarkdown() {
  const SyncConfig c = defaults();
  std::ostringstream o;
  o << "| key | default | was | range | env out of range | what it does |\n"
       "| --- | --- | --- | --- | --- | --- |\n";
  for (const auto& s : schema()) {
    o << "| `sync." << s.path << "` | ";
    const std::string v = c.valueText(s);
    if (std::holds_alternative<Access<std::string>>(s.access) || s.isEnum()) o << "`" << v << "`";
    else o << v;
    o << " | " << (s.env ? std::string("`") + s.env + "`" : "") << " | ";
    if (s.isNumeric()) o << s.lo << " to " << s.hi;
    else if (s.isEnum()) {
      for (int j = 0; s.enum_names[j]; ++j) o << (j ? ", " : "") << s.enum_names[j];
    }
    o << " | ";
    if (s.env != nullptr) {
      if (!s.isNumeric()) o << "refused";
      else o << (s.env_policy == EnvPolicy::kClamp ? "clamped" : "ignored, value kept");
    }
    o << " | " << s.doc << " |\n";
  }
  return o.str();
}

}  // namespace sync
}  // namespace houdini
