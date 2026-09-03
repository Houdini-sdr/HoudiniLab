/**
 * @file sync_config_test.cc
 * @brief SyncConfig: defaults, JSON, ranges, unknown keys, environment
 *        overrides, provenance, cross-constraint notes.
 *
 * AP-56 asked for this: a knob that lands a finite nonsense value used to be
 * accepted silently. Every documented knob is loaded here at both bounds and
 * one step outside; the outside value must throw, the bounds must load.
 */
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <variant>

#include "sync/sync_config.h"

using houdini::sync::SyncConfig;
using houdini::sync::ThresholdForm;
using houdini::sync::PickRule;
using houdini::sync::TrackerType;

namespace {
int g_fail = 0;
void check(bool ok, const std::string& what) {
  std::printf("%s  %s\n", ok ? "PASS" : "FAIL", what.c_str());
  if (!ok) ++g_fail;
}
bool throws(const std::string& json) {
  try {
    SyncConfig::load(json);
    return false;
  } catch (const std::exception&) {
    return true;
  }
}
}  // namespace

// The operator's shell may export HOUDINI_* (the bench scripts do). Every
// section below except the environment one must see none of them.
void clearEnv() {
  SyncConfig probe = SyncConfig::defaults();
  for (const auto& k : probe.knobs())
    if (k.env != nullptr) unsetenv(k.env);
}

int main() {
  clearEnv();
  // 1. Defaults are the shipped values, every provenance "default".
  {
    const auto c = SyncConfig::load("{}");
    check(c.beacon.type == "legacy" && c.beacon.tx_full_scale == 0.6,
          "defaults: beacon legacy at 0.6 FS");
    check(c.detector.threshold == ThresholdForm::kAuto &&
              c.detector.pick == PickRule::kFirstPath &&
              c.detector.first_path_window == -1 &&
              c.detector.first_path_floor_db == -9.0,
          "defaults: detector auto / first_path / window derived (-1) / -9 dB");
    check(c.confirm.snr_floor_db == 30.0, "defaults: 30 dB floor");
    check(c.cfo.index_guard == 8 && c.cfo.window_margin == 0 && c.cfo.log_every == 10,
          "defaults: cfo guard 8, margin 0, log 1 in 10");
    check(c.tracker.type == TrackerType::kAlphaBeta && c.tracker.alpha == 0.5 &&
              c.tracker.beta == 0.1 && c.tracker.step_ppm == 0.5 &&
              c.tracker.max_ppm == 100.0 && c.tracker.trust_ppm == 1.0,
          "defaults: alpha-beta 0.5 / 0.1, step 0.5 ppm, band 100, trust 1");
    check(c.resync.residual_ppm == 0.1 && c.resync.scatter_tol_us == 2.0 &&
              c.resync.confirm_tol_us == 5.2083 && c.resync.retry_max == 100 &&
              c.resync.escalate_episodes == 2 && c.resync.hold_offgrid == 2 &&
              c.resync.acq_refine_span == 200 && c.resync.acq_max_ppm == 100.0,
          "defaults: resync 0.1 ppm / 2.0 us / 5.2083 us / 100 / 2 / 2 / 200 / 100");
    check(std::isnan(c.resync.sync_tol_samples), "defaults: sync_tol_samples derived (NaN)");
    bool all_default = true;
    for (const auto& kv : c.provenance) all_default &= (kv.second == "default");
    check(all_default && c.provenance.size() == c.knobs().size(),
          "defaults: every knob marked default (" + std::to_string(c.knobs().size()) + " knobs)");
  }
  // 2. JSON values land, with provenance.
  {
    const auto c = SyncConfig::load(R"({"sync": {"detector": {"threshold": "coherence",
        "pick": "argmax", "first_path_window": 32}, "confirm": {"snr_floor_db": 12.5},
        "tracker": {"type": "kalman", "kalman": {"innov_gate": 3}},
        "resync": {"scatter_tol_us": 1.5}, "beacon": {"type": "nr_pss"}}})");
    check(c.detector.threshold == ThresholdForm::kCoherence &&
              c.detector.pick == PickRule::kArgmax && c.detector.first_path_window == 32,
          "json: detector enums and int");
    check(c.confirm.snr_floor_db == 12.5 && c.tracker.type == TrackerType::kKalman &&
              c.tracker.kf_innov_gate == 3.0 && c.resync.scatter_tol_us == 1.5,
          "json: nested doubles");
    check(c.provenance.at("confirm.snr_floor_db") == "json" &&
              c.provenance.at("resync.residual_ppm") == "default",
          "json: provenance marks only what was given");
    const auto n = SyncConfig::load(R"({"sync": {"beacon": {"type": "nr_pss"}}})");
    bool noted = false;
    for (const auto& w : n.warnings) noted |= (w.find("8.157") != std::string::npos);
    check(noted, "json: a non-legacy beacon with the DEFAULT floor is noted (8.157)");
    bool noted_when_set = false;
    for (const auto& w : c.warnings) noted_when_set |= (w.find("8.157") != std::string::npos);
    check(!noted_when_set, "json: no note when the floor was set explicitly");
  }
  // 3. Ranges: both bounds load, one step outside throws, for every numeric knob.
  {
    SyncConfig probe = SyncConfig::defaults();
    int tested = 0, bad = 0;
    for (const auto& k : probe.knobs()) {
      if (!k.isNumeric()) continue;
      auto mk = [&](double v) {
        // build {"sync": {a: {b: {c: v}}}} from the dotted path
        std::string path = k.path;
        std::string open, close;
        size_t start = 0;
        while (true) {
          const size_t dot = path.find('.', start);
          const std::string key = path.substr(start, dot == std::string::npos ? std::string::npos : dot - start);
          open += "{\"" + key + "\": ";
          close += "}";
          if (dot == std::string::npos) break;
          start = dot + 1;
        }
        char num[64];
        std::snprintf(num, sizeof num, "%.17g", v);
        return "{\"sync\": " + open + num + close + "}";
      };
      const bool is_int = std::holds_alternative<int*>(k.target);
      const double step = is_int ? 1.0 : (k.hi - k.lo) * 1e-3;
      const bool lo_ok = !throws(mk(k.lo));
      const bool hi_ok = !throws(mk(k.hi));
      const bool below_throws = throws(mk(k.lo - step));
      const bool above_throws = throws(mk(k.hi + step));
      ++tested;
      if (!(lo_ok && hi_ok && below_throws && above_throws)) {
        ++bad;
        std::printf("      %s: lo %d hi %d below-throws %d above-throws %d\n", k.path,
                    lo_ok, hi_ok, below_throws, above_throws);
      }
    }
    int numeric = 0;
    for (const auto& k : probe.knobs()) numeric += k.isNumeric() ? 1 : 0;
    check(bad == 0 && tested == numeric,
          "ranges: every numeric knob loads at its bounds and throws one step outside (" +
              std::to_string(tested) + " of " + std::to_string(numeric) + ")");
  }
  // 4. Typos are errors, not defaults.
  check(throws(R"({"sync": {"detector": {"thresold": "auto"}}})"), "unknown key under sync throws");
  check(throws(R"({"sync": {"detector": {"pick": "strongest"}}})"), "unknown enum name throws");
  check(throws(R"({"sync": {"resync": {"retry_max": 2.5}}})"), "non-integer for an int knob throws");
  check(throws(R"({"sync": {"confirm": {"snr_floor_db": "30"}}})"), "string for a number throws");
  check(!throws(R"({"sync": {"_note": "comments allowed", "confirm": {"snr_floor_db": 25}}})"),
        "underscore keys are comments");
  check(!throws(R"({"sync": {"tracker": {"kalman": {"_why": "nested comment", "innov_gate": 3}}}})"),
        "nested underscore keys are comments too");
  check(!throws(R"({"sync": null})"), "sync: null is treated as absent");
  check(throws(R"({"beacon_type": 7})"), "a non-string top-level beacon_type throws");
  check(!throws(R"({"beacon_type": "nr", "sync": {"beacon": {"type": "nr"}}})"),
        "top-level and sync beacon types that agree do not throw");
  // 5. The legacy top-level beacon_type.
  {
    const auto c = SyncConfig::load(R"({"beacon_type": "dot11"})");
    check(c.beacon.type == "dot11" && c.provenance.at("beacon.type") == "json",
          "top-level beacon_type is accepted into sync.beacon.type");
    check(throws(R"({"beacon_type": "dot11", "sync": {"beacon": {"type": "nr"}}})"),
          "top-level and sync beacon types that disagree throw");
  }
  // 6. Environment overrides: applied and logged when allowed, refused when not.
  {
    setenv("HOUDINI_SCATTER_TOL_US", "3.5", 1);
    setenv("HOUDINI_BEACON_THRESH", "nolag", 1);
    setenv("HOUDINI_TRACKER", "kf", 1);
    const auto c = SyncConfig::load("{}");
    check(c.resync.scatter_tol_us == 3.5 && c.provenance.at("resync.scatter_tol_us") == "env",
          "env: numeric override lands with provenance env");
    check(c.detector.threshold == ThresholdForm::kCoherence && c.tracker.type == TrackerType::kKalman,
          "env: legacy spellings nolag and kf map to coherence and kalman");
    const auto d = SyncConfig::load(R"({"sync": {"allow_env_overrides": false}})");
    check(d.resync.scatter_tol_us == 2.0 && d.provenance.at("resync.scatter_tol_us") == "default",
          "env: refused when allow_env_overrides is false");
    bool said = false;
    for (const auto& w : d.warnings) said |= (w.find("IGNORED") != std::string::npos);
    check(said, "env: the refusal is reported");
    setenv("HOUDINI_SCATTER_TOL_US", "abc", 1);
    const auto e = SyncConfig::load("{}");
    check(e.resync.scatter_tol_us == 2.0, "env: a non-number is ignored, not zero");
    setenv("HOUDINI_SCATTER_TOL_US", "5000", 1);
    const auto f = SyncConfig::load("{}");
    check(f.resync.scatter_tol_us == 1000.0, "env: an out-of-range value is CLAMPED to the range");
    bool clamped_note = false;
    for (const auto& w : f.warnings) clamped_note |= (w.find("clamped") != std::string::npos);
    check(clamped_note, "env: the clamp is reported");
    setenv("HOUDINI_ESCALATE_EPISODES", "0", 1);
    setenv("HOUDINI_RESYNC_RETRY_MAX", "2.5", 1);
    const auto g = SyncConfig::load("{}");
    check(g.resync.escalate_episodes == 1 && g.resync.retry_max == 2,
          "env: an int knob at 0 clamps to its minimum and 2.5 floors to 2 (the old readers' meaning)");
    setenv("HOUDINI_BEACON_PICK", "first", 1);
    const auto h = SyncConfig::load("{}");
    check(h.detector.pick == PickRule::kFirstCrossing, "env: the legacy spelling first maps to first_crossing");
    bool noted_first = false;
    for (const auto& w : h.warnings) noted_first |= (w.find("AP-34") != std::string::npos);
    check(noted_first, "validate: first_crossing is noted as diagnostic only");
    setenv("HOUDINI_BEACON_PICK", "strongest", 1);
    const auto i = SyncConfig::load("{}");
    check(i.detector.pick == PickRule::kFirstPath, "env: an unknown enum name is ignored, not fatal");
    clearEnv();
  }
  // 7. The generated views exist and name every knob.
  {
    const auto c = SyncConfig::load("{}");
    const std::string d = c.describe();
    const std::string m = SyncConfig::schemaMarkdown();
    check(d.find("resync.scatter_tol_us = 2") != std::string::npos, "describe names values");
    check(d.find("tracker.type = alpha_beta") != std::string::npos, "describe names an enum by its name");
    check(d.find("resync.sync_tol_samples = derived") != std::string::npos &&
              d.find("detector.first_path_window = derived") != std::string::npos,
          "describe prints derived for the two sentinels");
    const auto pw = SyncConfig::load(R"({"sync": {"detector": {"pfa_per_window": 0.01}}})");
    bool reserved = false;
    for (const auto& w : pw.warnings) reserved |= (w.find("RESERVED") != std::string::npos);
    check(reserved, "validate: a pfa_per_window value is noted as reserved and not applied");
    const auto fixed = SyncConfig::load(R"({"sync": {"tracker": {"alpha": 0, "beta": 0}}})");
    bool fixed_note = false;
    for (const auto& w : fixed.warnings) fixed_note |= (w.find("fixed-period") != std::string::npos);
    check(fixed_note, "validate: alpha = beta = 0 is noted as a fixed-period grid");
    check(m.find("`sync.confirm.snr_floor_db`") != std::string::npos &&
              m.find("`HOUDINI_SYNC_SNR_DB`") != std::string::npos,
          "schema table carries the key and the environment name it replaces");
  }
  std::printf("\nRESULT: %s (%d failure(s))\n", g_fail ? "FAIL" : "PASS", g_fail);
  return g_fail ? 1 : 0;
}
