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
#include <fstream>
#include <string>
#include <variant>

#include "sync/sync_config.h"

using houdini::sync::SyncConfig;
using houdini::sync::ThresholdForm;
using houdini::sync::PickRule;
using houdini::sync::TrackerType;
using houdini::sync::Source;

namespace {
int g_fail = 0;
void check(bool ok, const std::string& what) {
  std::printf("%s  %s\n", ok ? "PASS" : "FAIL", what.c_str());
  if (!ok) ++g_fail;
}
bool throws(const std::string& json) {
  try {
    SyncConfig::loadFromText(json);
    return false;
  } catch (const std::exception&) {
    return true;
  }
}
}  // namespace

// The operator's shell may export HOUDINI_* (the bench scripts do). Every
// section below except the environment one must see none of them.
void clearEnv() {
  for (const auto& k : SyncConfig::schema())
    if (k.env != nullptr) unsetenv(k.env);
}

// The walkthrough's knob table is generated from the schema and committed;
// this diff is what keeps the two from drifting (commit 8253025 was the
// drift it prevents). The path comes from CMake; without it the check is
// skipped, loudly.
void checkWalkthroughTable(const char* path) {
  std::ifstream in(path);
  if (!in) {
    check(false, std::string("walkthrough table: cannot open ") + path);
    return;
  }
  std::string line, block;
  bool inside = false, seen = false;
  while (std::getline(in, line)) {
    if (line.rfind("<!-- sync-knob-table:begin", 0) == 0) { inside = true; seen = true; continue; }
    if (line.rfind("<!-- sync-knob-table:end", 0) == 0) { inside = false; continue; }
    if (inside) block += line + "\n";
  }
  const std::string generated = SyncConfig::schemaMarkdown();
  check(seen, "walkthrough table: begin/end markers present");
  check(block == generated, "walkthrough table matches the generated schema (regenerate with "
                            "./build/sync_config_schema if this fails)");
}

int main(int argc, char** argv) {
  clearEnv();
  if (argc > 1) checkWalkthroughTable(argv[1]);
  else std::printf("SKIP  walkthrough table diff (no path given)\n");
  // 1. Defaults are the shipped values, every provenance "default".
  {
    const auto c = SyncConfig::loadFromText("{}");
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
    // A config FILE with nothing in it still adopts the sounder's threshold
    // fallbacks (derived); the library's own defaults() marks every knob
    // default.
    const auto d = SyncConfig::defaults();
    for (const auto& s : SyncConfig::schema()) all_default &= (d.provenanceOf(s.path) == Source::kDefault);
    int derived = 0;
    for (const auto& s : SyncConfig::schema()) derived += (c.provenanceOf(s.path) == Source::kDerived) ? 1 : 0;
    check(all_default && c.provenanceOf("no.such.key") == Source::kDefault && derived == 2 &&
              c.provenanceOf("detector.corr_scale") == Source::kDerived,
          "defaults: every knob marked default (" + std::to_string(SyncConfig::schema().size()) +
              " knobs); an empty file derives only the two legacy thresholds; an unknown path reads default, never throws");
  }
  // 2. JSON values land, with provenance.
  {
    const auto c = SyncConfig::loadFromText(R"({"sync": {"detector": {"threshold": "coherence",
        "pick": "argmax", "first_path_window": 32}, "confirm": {"snr_floor_db": 12.5},
        "tracker": {"type": "kalman", "kalman": {"innov_gate": 3}},
        "resync": {"scatter_tol_us": 1.5}, "beacon": {"type": "nr_pss"}}})");
    check(c.detector.threshold == ThresholdForm::kCoherence &&
              c.detector.pick == PickRule::kArgmax && c.detector.first_path_window == 32,
          "json: detector enums and int");
    check(c.confirm.snr_floor_db == 12.5 && c.tracker.type == TrackerType::kKalman &&
              c.tracker.kf_innov_gate == 3.0 && c.resync.scatter_tol_us == 1.5,
          "json: nested doubles");
    check(c.provenanceOf("confirm.snr_floor_db") == Source::kJson &&
              c.provenanceOf("resync.residual_ppm") == Source::kDefault,
          "json: provenance marks only what was given");
    const auto n = SyncConfig::loadFromText(R"({"sync": {"beacon": {"type": "nr_pss"}}})");
    bool noted = false;
    for (const auto& w : n.warnings()) noted |= (w.find("8.157") != std::string::npos);
    check(noted, "json: a non-legacy beacon with the DEFAULT floor is noted (8.157)");
    bool noted_when_set = false;
    for (const auto& w : c.warnings()) noted_when_set |= (w.find("8.157") != std::string::npos);
    check(!noted_when_set, "json: no note when the floor was set explicitly");
  }
  // 3. Ranges: both bounds load, one step outside throws, for every numeric knob.
  {
    int tested = 0, bad = 0;
    for (const auto& k : SyncConfig::schema()) {
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
      const bool is_int = std::holds_alternative<SyncConfig::Access<int>>(k.access);
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
    for (const auto& k : SyncConfig::schema()) numeric += k.isNumeric() ? 1 : 0;
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
    const auto c = SyncConfig::loadFromText(R"({"beacon_type": "dot11"})");
    check(c.beacon.type == "dot11" && c.provenanceOf("beacon.type") == Source::kJson,
          "top-level beacon_type is accepted into sync.beacon.type");
    check(throws(R"({"beacon_type": "dot11", "sync": {"beacon": {"type": "nr"}}})"),
          "top-level and sync beacon types that disagree throw");
  }
  // 6. Environment overrides: applied and logged when a config allows them,
  //    refused (and reported) by default.
  const std::string kEnvOn = R"({"sync": {"allow_env_overrides": true}})";
  {
    setenv("HOUDINI_SCATTER_TOL_US", "3.5", 1);
    setenv("HOUDINI_BEACON_THRESH", "nolag", 1);
    setenv("HOUDINI_TRACKER", "kf", 1);
    const auto c = SyncConfig::loadFromText(kEnvOn);
    check(c.resync.scatter_tol_us == 3.5 && c.provenanceOf("resync.scatter_tol_us") == Source::kEnv,
          "env: numeric override lands with provenance env");
    check(c.detector.threshold == ThresholdForm::kCoherence && c.tracker.type == TrackerType::kKalman,
          "env: legacy spellings nolag and kf map to coherence and kalman");
    const auto d0 = SyncConfig::loadFromText("{}");
    bool said0 = false;
    for (const auto& w : d0.warnings()) said0 |= (w.find("IGNORED") != std::string::npos);
    check(d0.resync.scatter_tol_us == 2.0 && !d0.allow_env_overrides && said0,
          "env: refused and reported by DEFAULT (decided 2026-09-03)");
    const auto d = SyncConfig::loadFromText(R"({"sync": {"allow_env_overrides": false}})");
    check(d.resync.scatter_tol_us == 2.0 && d.provenanceOf("resync.scatter_tol_us") == Source::kDefault,
          "env: refused when allow_env_overrides is false");
    bool said = false;
    for (const auto& w : d.warnings()) said |= (w.find("IGNORED") != std::string::npos);
    check(said, "env: the refusal is reported");
    setenv("HOUDINI_SCATTER_TOL_US", "abc", 1);
    const auto e = SyncConfig::loadFromText(kEnvOn);
    check(e.resync.scatter_tol_us == 2.0, "env: a non-number is ignored, not zero");
    setenv("HOUDINI_SCATTER_TOL_US", "5000", 1);
    const auto f = SyncConfig::loadFromText(kEnvOn);
    check(f.resync.scatter_tol_us == 1000.0, "env: an out-of-range value is CLAMPED to the range");
    bool clamped_note = false;
    for (const auto& w : f.warnings()) clamped_note |= (w.find("clamped") != std::string::npos);
    check(clamped_note, "env: the clamp is reported");
    setenv("HOUDINI_ESCALATE_EPISODES", "0", 1);
    setenv("HOUDINI_RESYNC_RETRY_MAX", "2.5", 1);
    const auto g = SyncConfig::loadFromText(kEnvOn);
    check(g.resync.escalate_episodes == 1 && g.resync.retry_max == 2,
          "env: an int knob at 0 clamps to its minimum and 2.5 floors to 2 (the old readers' meaning)");
    setenv("HOUDINI_BEACON_PICK", "first", 1);
    const auto h = SyncConfig::loadFromText(kEnvOn);
    check(h.detector.pick == PickRule::kFirstCrossing, "env: the legacy spelling first maps to first_crossing");
    bool noted_first = false;
    for (const auto& w : h.warnings()) noted_first |= (w.find("AP-34") != std::string::npos);
    check(noted_first, "validate: first_crossing is noted as diagnostic only");
    setenv("HOUDINI_BEACON_PICK", "strongest", 1);
    const auto i = SyncConfig::loadFromText(kEnvOn);
    check(i.detector.pick == PickRule::kFirstPath, "env: an unknown enum name is ignored, not fatal");
    clearEnv();
    // The three knobs whose old readers IGNORED an out-of-range value keep
    // doing so; the formerly unbounded knobs clamp.
    setenv("HOUDINI_BEACON_FS", "0", 1);
    setenv("HOUDINI_FIRST_PATH_DB", "3", 1);
    setenv("HOUDINI_FIRST_PATH_WIN", "5000", 1);
    setenv("HOUDINI_GRID_ALPHA", "50", 1);
    const auto j = SyncConfig::loadFromText(kEnvOn);
    check(j.beacon.tx_full_scale == 0.6 && j.detector.first_path_floor_db == -9.0 &&
              j.detector.first_path_window == -1,
          "env: BEACON_FS=0, FIRST_PATH_DB=3, FIRST_PATH_WIN=5000 keep their defaults (as before)");
    int ignored_notes = 0;
    for (const auto& w : j.warnings()) ignored_notes += (w.find("kept, as the old reader did") != std::string::npos);
    check(ignored_notes == 3, "env: each ignored override is reported (" + std::to_string(ignored_notes) + " of 3)");
    check(j.provenanceOf("beacon.tx_full_scale") == Source::kDefault &&
              j.provenanceOf("detector.first_path_floor_db") == Source::kDefault &&
              j.provenanceOf("detector.first_path_window") == Source::kDefault,
          "env: an ignored override leaves provenance at default");
    check(!throws(R"({"sync": {"_note.v2": "x"}})"), "a comment key with a dot is still a comment");
    check(j.tracker.alpha == 1.0, "env: GRID_ALPHA=50 clamps to 1 (it used to pass through, AP-56)");
    const std::string dj = j.describe();
    check(dj.find("tracker.alpha = 1  [env]") != std::string::npos, "describe shows [env] for an override");
    clearEnv();
    check(throws(R"({"sync": {"detector.threshold": "power"}})"),
          "a flat dotted key is refused, not silently ignored");
    const auto mc = SyncConfig::loadFromText(R"({"sync": {"detector": {"threshold": "Coherence"}}})");
    check(mc.detector.threshold == ThresholdForm::kCoherence, "json: enum names are case-insensitive");
    const auto sb = SyncConfig::loadFromText(R"({"sync": {"allow_env_overrides": "false"}})");
    check(!sb.allow_env_overrides, "json: a string false is accepted for a bool knob");
  }
  // 7. The generated views exist and name every knob.
  {
    const auto c = SyncConfig::loadFromText("{}");
    const std::string d = c.describe();
    const std::string m = SyncConfig::schemaMarkdown();
    check(d.find("resync.scatter_tol_us = 2") != std::string::npos, "describe names values");
    check(d.find("tracker.type = alpha_beta") != std::string::npos, "describe names an enum by its name");
    check(d.find("resync.sync_tol_samples = derived") != std::string::npos &&
              d.find("detector.first_path_window = derived") != std::string::npos,
          "describe prints derived for the two sentinels");
    const auto pw = SyncConfig::loadFromText(R"({"sync": {"detector": {"pfa_per_window": 0.01}}})");
    bool reserved = false;
    for (const auto& w : pw.warnings()) reserved |= (w.find("RESERVED") != std::string::npos);
    check(!reserved, "validate: pfa_per_window is no longer reserved (P3 applies it to the coherence form)");
    const auto fixed = SyncConfig::loadFromText(R"({"sync": {"tracker": {"alpha": 0, "beta": 0}}})");
    bool fixed_note = false;
    for (const auto& w : fixed.warnings()) fixed_note |= (w.find("fixed-period") != std::string::npos);
    check(fixed_note, "validate: alpha = beta = 0 is noted as a fixed-period grid");
    check(m.find("`sync.confirm.snr_floor_db`") != std::string::npos &&
              m.find("`HOUDINI_SYNC_SNR_DB`") != std::string::npos,
          "schema table carries the key and the environment name it replaces");
  }
  // 12. resolve(): the sentinels fill from the shape with provenance "derived";
  //     an explicit value is left alone; the operation is idempotent.
  {
    auto c = SyncConfig::loadFromText("{}");
    check(c.valueText(*SyncConfig::spec("detector.first_path_window")) == "derived" &&
              c.valueText(*SyncConfig::spec("resync.sync_tol_samples")) == "derived",
          "resolve: before, both sentinels print as derived");
    c.resolve({128, 160.0});
    check(c.detector.first_path_window == 64 && c.resync.sync_tol_samples == 40.0 &&
              c.provenanceOf("detector.first_path_window") == Source::kDerived &&
              c.provenanceOf("resync.sync_tol_samples") == Source::kDerived,
          "resolve: 128-tap replica gives a 64-sample window, 160 prefix gives 40 samples, both derived");
    c.resolve({64, 80.0});
    check(c.detector.first_path_window == 64 && c.resync.sync_tol_samples == 40.0,
          "resolve: idempotent (a second call with another shape changes nothing)");
    auto e = SyncConfig::loadFromText(R"({"sync": {"detector": {"first_path_window": 16}}})");
    e.resolve({128, 160.0});
    check(e.detector.first_path_window == 16 && e.provenanceOf("detector.first_path_window") == Source::kJson,
          "resolve: an explicit value is left alone with its provenance");
    check(c.describe().find("detector.first_path_window = 64  [derived]") != std::string::npos,
          "resolve: describe() prints the resolved value and its provenance");
  }
  // 13. The legacy per-client threshold arrays feed the policy; corr_scale_init
  //     defaults to corr_scale; the sync block wins over the legacy key.
  {
    const auto a = SyncConfig::loadFromText(R"({"corr_scale": [25, 30]})");
    check(a.detector.bar.corr_scale == 25.0 && a.detector.bar.corr_scale_init == 25.0 &&
              a.provenanceOf("detector.corr_scale") == Source::kJson &&
              a.provenanceOf("detector.corr_scale_init") == Source::kDerived &&
              a.detector.bar.relaxed(3) == 28.0,
          "legacy corr_scale: first client's value lands (json), init follows it (derived), relaxed() adds the retry");
    // The sounder's fallback for an ABSENT corr_scale is 1, and it is
    // recorded as derived, not left at the library's 10 (round 4, HIGH 4).
    const auto none = SyncConfig::loadFromText("{}");
    check(none.detector.bar.corr_scale == 1.0 && none.detector.bar.corr_scale_init == 1.0 &&
              none.provenanceOf("detector.corr_scale") == Source::kDerived,
          "no corr_scale anywhere: the sounder's fallback of 1 is adopted and marked derived");
    const auto b = SyncConfig::loadFromText(R"({"corr_scale": [25], "corr_scale_init": [40]})");
    check(b.detector.bar.corr_scale == 25.0 && b.detector.bar.corr_scale_init == 40.0,
          "legacy corr_scale_init: its own value when given");
    const auto d = SyncConfig::loadFromText(R"({"corr_scale": [25], "sync": {"detector": {"corr_scale": 12}}})");
    check(d.detector.bar.corr_scale == 12.0 && d.detector.bar.corr_scale_init == 12.0,
          "sync.detector.corr_scale wins over the legacy array; init still follows");
    check(SyncConfig::defaults().detector.bar.corr_scale == 10.0, "the library's own default corr_scale is 10");
  }
  // 15. Platform defaults derive on Iris/UHD only, and the coherence bar
  //     formula is what 8.163 measured.
  {
    auto i = SyncConfig::loadFromText("{}");
    i.resolve({128, 160.0, houdini::sync::Platform::kIrisUhd});
    check(i.detector.pick == PickRule::kFirstCrossing && i.detector.threshold == ThresholdForm::kPowerRatio &&
              i.provenanceOf("detector.pick") == Source::kDerived &&
              i.provenanceOf("detector.threshold") == Source::kDerived,
          "resolve: Iris/UHD derives first_crossing and power (the framer's old rules)");
    auto h = SyncConfig::loadFromText("{}");
    h.resolve({128, 160.0, houdini::sync::Platform::kHoudini});
    check(h.detector.pick == PickRule::kFirstPath && h.detector.threshold == ThresholdForm::kAuto &&
              h.provenanceOf("detector.pick") == Source::kDefault,
          "resolve: Houdini keeps the shipped defaults");
    // P3: the probability applies to the coherence form only, and only when set.
    auto pf = SyncConfig::loadFromText(R"({"sync": {"detector": {"pfa_per_window": 1e-3}}})");
    houdini::sync::ResolveContext rpf;
    rpf.replica_len = 128; rpf.prefix_samples = 160; rpf.single_copy_replica = true;
    pf.resolve(rpf);
    check(pf.detector.pfa_applies && pf.detector.threshold == ThresholdForm::kCoherence,
          "P3: pfa set + a single-copy replica -> the pfa bar applies");
    auto px = SyncConfig::loadFromText(R"({"sync": {"detector": {"pfa_per_window": 1e-3}}})");
    houdini::sync::ResolveContext rpx;
    rpx.replica_len = 128; rpx.prefix_samples = 160;
    px.resolve(rpx);
    bool ignored = false;
    for (const auto& w : px.warnings()) ignored |= (w.find("applies to the coherence form only") != std::string::npos);
    check(!px.detector.pfa_applies && ignored,
          "P3: pfa set on a repeated-field replica -> not applied, and noted");
    auto pu = SyncConfig::loadFromText("{}");
    pu.resolve(rpf);
    check(!pu.detector.pfa_applies, "P3: pfa unset -> corr_scale applies even on the coherence form");
    auto pc = SyncConfig::loadFromText(R"({"sync": {"detector": {"pfa_per_window": 1e-3}}})");
    houdini::sync::ResolveContext rpc = rpf;
    rpc.backend_applies_config = false;
    pc.resolve(rpc);
    bool not_in_force = false;
    for (const auto& w : pc.warnings()) not_in_force |= (w.find("not in force") != std::string::npos);
    check(!pc.detector.pfa_applies && not_in_force,
          "P3: a backend that applies no configuration -> the probability is noted as not in force");
    const double bar = houdini::sync::ThresholdPolicy::coherenceBar(128, 1e-3, 4096);
    const double want = 1.0 - std::pow(1e-3 / 4096.0, 1.0 / 127.0);
    check(std::fabs(bar - want) < 1e-12 && bar > 0.09 && bar < 0.13,
          "coherenceBar: 1 - (pfa/window)^(1/(L-1)), about 0.11 at 128 taps, 1e-3 over 4096");
    check(std::string(houdini::sync::name(houdini::sync::Platform::kIrisUhd)) == "iris_uhd", "Platform names");
    // resolve() validates what it derived: the Iris defaults carry their
    // notes; a single-copy replica is recorded as coherence; several clients
    // are noted when the block sets a bar.
    bool iris_noted = false;
    for (const auto& w : i.warnings()) iris_noted |= (w.find("Iris/UHD default") != std::string::npos);
    check(iris_noted, "resolve on Iris/UHD: the derived pick and form carry their notes");
    auto sc = SyncConfig::loadFromText("{}");
    houdini::sync::ResolveContext rc;
    rc.replica_len = 128; rc.prefix_samples = 160; rc.platform = houdini::sync::Platform::kIrisUhd;
    rc.single_copy_replica = true;
    sc.resolve(rc);
    check(sc.detector.threshold == ThresholdForm::kCoherence &&
              sc.provenanceOf("detector.threshold") == Source::kDerived,
          "resolve: a single-copy replica records coherence even where power is the platform default");
    auto mc = SyncConfig::loadFromText(R"({"sync": {"detector": {"corr_scale": 12}}})");
    houdini::sync::ResolveContext rc2;
    rc2.replica_len = 128; rc2.prefix_samples = 160; rc2.clients = 2;
    mc.resolve(rc2);
    bool multi_noted = false;
    for (const auto& w : mc.warnings()) multi_noted |= (w.find("2 clients") != std::string::npos);
    check(multi_noted, "resolve: with 2 clients the per-client arrays are noted as the ones applied");
    auto mc2 = SyncConfig::loadFromText(R"({"corr_scale": [25, 30]})");
    mc2.resolve(rc2);
    bool multi_noted2 = false;
    for (const auto& w : mc2.warnings()) multi_noted2 |= (w.find("2 clients") != std::string::npos);
    check(!multi_noted2, "resolve: the multi-client note fires only when the sync BLOCK set a bar, not for the arrays");
    // The fact is recorded by load(): a resolve() with no adoption at all
    // still notes it, and a repeated adoption cannot fake it.
    auto lr = SyncConfig::load(std::string(R"({"detector": {"corr_scale": 25}})"));
    lr.resolve(rc2);
    bool lr_noted = false;
    for (const auto& w : lr.warnings()) lr_noted |= (w.find("2 clients") != std::string::npos);
    check(lr_noted, "load + resolve without adoption still notes a block-set bar with 2 clients");
    mc2.adoptLegacyThreshold(25.0, 25.0, true, false);
    mc2.resolve(rc2);
    bool re_noted = false;
    for (const auto& w : mc2.warnings()) re_noted |= (w.find("2 clients") != std::string::npos);
    check(!re_noted, "a repeated adoption of the array does not turn into a block-set note");
    auto ih = SyncConfig::loadFromText(R"({"sync": {"detector": {"pick": "first_crossing"}}})");
    houdini::sync::ResolveContext rch;
    rch.replica_len = 128; rch.prefix_samples = 160; rch.platform = houdini::sync::Platform::kHoudini;
    ih.resolve(rch);
    bool diag = false;
    for (const auto& w : ih.warnings()) diag |= (w.find("diagnostic only on Houdini") != std::string::npos);
    check(diag, "validate: an explicit first_crossing on Houdini is noted as diagnostic; the wording follows the platform");
    sc.resolve(rc);
    const size_t n1 = sc.warnings().size();
    sc.resolve(rc);
    check(sc.warnings().size() == n1, "resolve: idempotent notes (a second call adds none)");
  }
  // 14. The schema is static and const-correct: a spec is found by path, and a
  //     const object can be read through it.
  {
    const SyncConfig c = SyncConfig::defaults();
    const auto* s = SyncConfig::spec("tracker.alpha");
    const auto* none = SyncConfig::spec("tracker.nope");
    check(s != nullptr && none == nullptr && c.valueText(*s) == "0.5",
          "schema: spec() finds a path, returns nullptr otherwise, and reads a const object");
    check(&SyncConfig::schema() == &SyncConfig::schema(), "schema: one static table");
  }
  std::printf("\nRESULT: %s (%d failure(s))\n", g_fail ? "FAIL" : "PASS", g_fail);
  return g_fail ? 1 : 0;
}
