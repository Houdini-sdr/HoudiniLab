/**
 * @file golden_window_test.cc
 * @brief The library against resync windows recorded on silicon.
 *
 * The migration's byte-identity guard. Each fixture is one targeted resync
 * window the sounder dumped (HOUDINI_DUMP_RESYNC_WIN) together with the index
 * and in-window SNR the shipped code computed for it, on 2026-09-03 on the
 * rig, stack fpga c88e0b5f / device+host 3a0aa361, at 0.6 FS. The library's
 * Detector and SnrWindowGuard must return the same index and the same SNR
 * (to 0.01 dB) for every window; the CFO estimator must return a finite value
 * within the link's plausible band. A refactor that moves any of these
 * numbers is a behaviour change and has to say so.
 *
 * Fixture layout: <dir>/<shape>/resyncwin_NN.bin (complex<int16>, n samples)
 * and resyncwin_NN.txt (key value lines: n, sync_index, snr, ...). Windows
 * dumped after 2026-09-03 also carry the detector settings they were taken
 * under (corr_scale, thresh, pick, first_path_window, first_path_floor_db,
 * snr_floor_db, snr_guard, replica_tail, beacon_type), asserted below when
 * present. `cfo_hz` is NOT written by the sounder: it is a baseline added by
 * hand from the library at commit 10d0fe0, so the estimator has an identity
 * check rather than a plausibility band.
 */
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "sync/beacon_shapes.h"
#include "sync/beacon_shape.h"
#include "sync/cfo_estimator.h"
#include "sync/confirm.h"
#include "sync/detector.h"
#include "sync/sync_config.h"

namespace {
int g_fail = 0;
void check(bool ok, const std::string& what) {
  std::printf("%s  %s\n", ok ? "PASS" : "FAIL", what.c_str());
  if (!ok) ++g_fail;
}
// Line-wise: "key value". Numeric values land in the map; a non-numeric value
// (beacon_type legacy) is kept as text and does not stop the parse.
struct Meta {
  std::map<std::string, double> num;
  std::map<std::string, std::string> text;
  bool has(const std::string& k) const { return num.count(k) != 0; }
  double at(const std::string& k) const { return num.at(k); }
};
Meta readMeta(const std::string& path) {
  Meta m;
  std::ifstream f(path);
  std::string line;
  while (std::getline(f, line)) {
    std::istringstream ls(line);
    std::string k, v;
    if (!(ls >> k >> v)) continue;
    char* end = nullptr;
    const double d = std::strtod(v.c_str(), &end);
    if (end != v.c_str() && *end == '\0') m.num[k] = d;
    else m.text[k] = v;
  }
  return m;
}
bool readWindow(const std::string& path, std::vector<std::complex<int16_t>>* out) {
  std::ifstream f(path, std::ios::binary);
  if (!f) return false;
  f.seekg(0, std::ios::end);
  const auto bytes = static_cast<size_t>(f.tellg());
  f.seekg(0);
  // An empty or truncated window is not a fixture: it must not count as one
  // (under a backend that skips the replay the count is the only check).
  if (bytes < sizeof(std::complex<int16_t>)) return false;
  out->resize(bytes / sizeof(std::complex<int16_t>));
  f.read(reinterpret_cast<char*>(out->data()), static_cast<std::streamsize>(bytes));
  return static_cast<bool>(f);
}
}  // namespace

int main(int argc, char** argv) {
  // Nothing from the operator's shell may change what this test builds.
  {
    for (const auto& k : houdini::sync::SyncConfig::schema())
      if (k.env != nullptr) unsetenv(k.env);
  }
  const std::string dir = argc > 1 ? argv[1] : "tests/comms-func/fixtures/golden";
  using houdini::sync::BeaconShape;
  using houdini::sync::Numerology;
  using houdini::sync::Detector;
  using houdini::sync::DetectorConfig;
  using houdini::sync::PickRule;
  using houdini::sync::Platform;
  using houdini::sync::SnrWindowGuard;
  using houdini::sync::Source;
  using houdini::sync::SyncConfig;
  using houdini::sync::ThresholdForm;
  // The first-path window derives from the replica: 64 at 128 taps, 32 at 64.
  // The pre-library correlator did exactly this (comms-lib-portable.cc
  // firstPathWindow), and a fixed 64 would have doubled dot11's and nr's. The
  // shape owns the default; resolve() and the detector both read it.
  for (const char* name : {"dot11", "nr", "legacy", "nr_pss"}) {
    const auto shape = BeaconShape::make(name, Platform::kHoudini, Numerology::houdiniDefault());
    auto cfg = SyncConfig::defaults();
    cfg.resolve({shape.replicaLen(), 160.0, Platform::kHoudini});
    Detector det(shape, cfg.detector);
    check(det.firstPathWindow() == static_cast<int>(shape.replicaLen() / 2) &&
              cfg.detector.first_path_window == det.firstPathWindow() &&
              shape.defaultFirstPathWindow() == det.firstPathWindow(),
          std::string(name) + ": first-path window derives to half the replica (" +
              std::to_string(det.firstPathWindow()) + ") in the shape, the config and the detector");
  }
  // The shape object: names, the index convention, the expected end.
  {
    bool threw = false;
    try { BeaconShape::make("legacyy", Platform::kHoudini, Numerology::houdiniDefault()); } catch (const std::invalid_argument&) { threw = true; }
    check(threw && BeaconShape::names().size() == 5, "BeaconShape: an unknown name throws; five names");
    const auto h = BeaconShape::make("nr_pss", Platform::kHoudini, Numerology::houdiniDefault());
    const auto i = BeaconShape::make("nr_pss", Platform::kIrisUhd, Numerology::houdiniDefault());
    check(h.replicaTail() == 144 && i.replicaTail() == 0,
          "BeaconShape: the replica tail is a Houdini convention (144 for nr_pss), zero on Iris/UHD");
    check(h.expectedEndOffset() == 384 + static_cast<ssize_t>(h.coreLen()) &&
              i.expectedEndOffset() == static_cast<ssize_t>(i.coreLen() + 160),
          "BeaconShape: expected end is strobe + core on Houdini, core + prefix on Iris/UHD");
    check(h.endFromCorrelatorIndex(-1, 1000) == -1 && h.endFromCorrelatorIndex(100, 1000) == 244 &&
              h.endFromCorrelatorIndex(900, 1000) == -1 && i.endFromCorrelatorIndex(900, 1000) == 900,
          "BeaconShape: end = index + tail, -1 when none or past the window, index itself with no tail");
    // An NR shape at a rate that cannot hold the spacing builds the shipped
    // symbols and says so; the shipped rate holds it; a rate that gives a
    // non-power-of-two size is a fallback too (127 points would segfault the
    // FFT plan).
    Numerology low = Numerology::houdiniDefault();
    low.rate_hz = 61.44e6;
    Numerology odd = Numerology::houdiniDefault();
    odd.rate_hz = 121.92e6;
    const auto nr_low = BeaconShape::make("nr_pss", Platform::kHoudini, low);
    const auto nr_odd = BeaconShape::make("nr_pss", Platform::kHoudini, odd);
    check(!nr_low.numerologyHeld() && nr_low.replicaLen() == 128 && nr_low.coreLen() == h.coreLen() &&
              !nr_odd.numerologyHeld() && nr_odd.replicaLen() == 128 && h.numerologyHeld(),
          "NR at 61.44 or 121.92 MSPS falls back to the shipped 128-point symbols and reports it; 122.88 holds");
    check(!Numerology::houdiniDefault().ifftSizeIfExact(128).has_value() &&
              Numerology::houdiniDefault().ifftSizeIfExact(127).value_or(0) == 128,
          "ifftSizeIfExact: 128 points carry 127 tones (DC is nulled), not 128");
    const auto l = BeaconShape::make("legacy", Platform::kHoudini, Numerology::houdiniDefault());
    check(l.geometry().usable() && l.geometry().core_len == 496 && l.geometry().fine_len == 128 &&
              l.singleCopy() == false && h.singleCopy(),
          "BeaconShape: geometry from the shape; legacy repeats, nr_pss is a single copy");
  }
  // Detector behaviour that the fixtures exercise only indirectly.
  {
    check(Detector::resolveForm(ThresholdForm::kAuto, false) == ThresholdForm::kNormalizedXCorr &&
              Detector::resolveForm(ThresholdForm::kPowerRatio, false) == ThresholdForm::kPowerRatio &&
              Detector::resolveForm(ThresholdForm::kNormalizedXCorr, true) == ThresholdForm::kCoherence,
          "resolveForm: auto is xcorr, an explicit form is kept, a single-copy replica forces coherence");
    // The Iris/UHD defaults are the framer's old rules, derived by resolve();
    // a JSON value is honoured on either platform.
    auto iris = SyncConfig::defaults();
    iris.resolve({128, 160.0, Platform::kIrisUhd});
    check(iris.detector.pick == PickRule::kFirstCrossing &&
              iris.detector.threshold == ThresholdForm::kPowerRatio &&
              iris.provenanceOf("detector.pick") == Source::kDerived,
          "resolve on Iris/UHD: first_crossing and power, provenance derived (the pre-library rules)");
    auto iris_set = SyncConfig::loadFromText(R"({"sync": {"detector": {"pick": "first_path"}}})");
    iris_set.resolve({128, 160.0, Platform::kIrisUhd});
    check(iris_set.detector.pick == PickRule::kFirstPath, "resolve on Iris/UHD: a configured pick is honoured");
    auto hou = SyncConfig::defaults();
    hou.resolve({128, 160.0, Platform::kHoudini});
    check(hou.detector.pick == PickRule::kFirstPath && hou.detector.threshold == ThresholdForm::kAuto,
          "resolve on Houdini: the shipped defaults stand");
    const auto d = BeaconShape::make("nr_pss", Platform::kHoudini, Numerology::houdiniDefault());
    DetectorConfig dc;
    dc.first_path_window = 4095;
    Detector capped(d, dc);
    check(capped.firstPathWindow() == 2 * static_cast<int>(d.replicaLen()),
          "an explicit first-path window is capped at twice the replica (" + std::to_string(capped.firstPathWindow()) + ")");
    Detector idet(BeaconShape::make("nr_pss", Platform::kIrisUhd, Numerology::houdiniDefault()), iris.detector);
    check(idet.replicaTail() == 0 && idet.pick() == PickRule::kFirstCrossing &&
              idet.form() == ThresholdForm::kCoherence,
          "off Houdini: no replica tail, the derived first-crossing pick, and the replica still forces coherence");
#if defined(USE_CUDA)
    check(idet.backend() == houdini::sync::DetectorBackend::kCuda && !idet.backendAppliesConfig(),
          "this build's backend is the CUDA correlator, which does not apply the configuration");
#else
    check(idet.backend() == houdini::sync::DetectorBackend::kPortable && idet.backendAppliesConfig(),
          "this build's backend is the portable correlator, which applies the configuration");
#endif
    // The tail pushes an end past the window: reported as no detection.
    Detector det(d, DetectorConfig{});
    std::vector<std::complex<int16_t>> tiny(d.coreLen() + 8, std::complex<int16_t>(0, 0));
    for (size_t i = 0; i < d.coreLen(); ++i)
      tiny[i + 8] = std::complex<int16_t>(static_cast<int16_t>(d.core()[i].real() * 8000),
                                          static_cast<int16_t>(d.core()[i].imag() * 8000));
    // Positive control first: on the full buffer the detector finds the core
    // end where it was placed (8 + core_len - 1), with its evidence.
    const auto full = det.run(tiny.data(), tiny.size(), 10.0f);
    check(full.end_index == static_cast<ssize_t>(8 + d.coreLen() - 1),
          "positive control: the placed core is found at its end (" + std::to_string(full.end_index) + ")");
    check(full.form == ThresholdForm::kCoherence && full.statistic > full.bar && full.statistic <= 1.0 &&
              std::abs(full.peak) > 0.0f && full.bar == 0.1,
          "the detection carries its statistic (a coherence above the bar), the bar and the complex peak");
    // Then the PSS ends 144 samples before the core end; a window that stops at
    // the PSS end + 100 cannot hold the implied core end.
    const auto r = det.run(tiny.data(), 8 + 128 + 100, 10.0f);
    check(!r.found() && r.statistic == 0.0, "a detection whose implied core end falls outside the window is reported as none");
    // guardFor: one rule naming both reasons, the resolved first-path window
    // or the 64-sample echo allowance, whichever is larger.
    check(SnrWindowGuard::guardFor(64) == 64 && SnrWindowGuard::guardFor(32) == 64 &&
              SnrWindowGuard::guardFor(100) == 100 && SnrWindowGuard::guardFor(4) == 64,
          "guardFor: the resolved window or the 64-sample echo allowance, whichever is larger");
  }
  // P3 on the recorded nr_pss windows: with a false-alarm probability set,
  // the coherence bar comes from it (0.113 for 128 taps, 1e-3 over 4096)
  // instead of 1/corr_scale, and every recorded index is unchanged: the true
  // peak is far above either bar.
  {
    auto pcfg = SyncConfig::loadFromText(R"({"sync": {"detector": {"pfa_per_window": 1e-3}}})");
    const auto d = BeaconShape::make("nr_pss", Platform::kHoudini, Numerology::houdiniDefault());
    pcfg.resolve({d.replicaLen(), 160.0, Platform::kHoudini, true, 1});
    Detector det(d, pcfg.detector);
    check(det.barFromPfa() && std::fabs(det.effectiveScale(100.0f, 4096) - 1.0 / 0.112978) < 1e-3,
          "P3: the nr_pss detector takes its bar from the probability (scale 8.85 for 1e-3 over 4096)");
    int same = 0, total = 0;
    for (int i = 0; i < 12; ++i) {
      char nb[64];
      std::snprintf(nb, sizeof nb, "/nr_pss/resyncwin_%02d", i);
      std::vector<std::complex<int16_t>> w;
      if (!readWindow(dir + nb + ".bin", &w)) continue;
      const auto meta = readMeta(dir + nb + ".txt");
      if (!meta.has("sync_index")) continue;
      ++total;
      const auto r = det.run(w.data(), w.size(), 100.0f);
      if (r.end_index == static_cast<ssize_t>(meta.at("sync_index")) && std::fabs(r.bar - 0.112978) < 1e-4) ++same;
    }
    check(total == 12 && same == 12, "P3: all 12 nr_pss windows return the recorded index under the pfa bar, bar 0.113 recorded");
    const auto l = BeaconShape::make("legacy", Platform::kHoudini, Numerology::houdiniDefault());
    auto lcfg = SyncConfig::loadFromText(R"({"sync": {"detector": {"pfa_per_window": 1e-3}}})");
    lcfg.resolve({l.replicaLen(), 160.0, Platform::kHoudini, false, 1});
    Detector ldet(l, lcfg.detector);
    check(!ldet.barFromPfa() && ldet.effectiveScale(100.0f, 4096) == 100.0,
          "P3: the legacy (xcorr) detector ignores the probability and keeps corr_scale");
  }
  const auto cfg = houdini::sync::SyncConfig::loadFromText("{}");
  const float kCorrScale = 10.0f;
  int windows = 0;
  int with_stat = 0;  // fixtures too old to carry a statistic are not silently counted
  std::map<std::string, int> per_shape;
  // Windows per shape: legacy and nr_pss 00-05 (morning) + 06-11 (with the
  // statistic); dot11 00-05 recorded after round 4 for the guard rule.
  const std::map<std::string, int> kExpected = {{"legacy", 12}, {"nr_pss", 12}, {"dot11", 6}};
  for (const char* shape : {"legacy", "nr_pss", "dot11"}) {
    houdini::sync::shapes::Shape sh;
    if (!houdini::sync::shapes::parse(shape, &sh)) { check(false, std::string("parse ") + shape); continue; }
    // Built the way the receiver builds them: the shape, a config resolved
    // against it, the detector, the guard from the resolved window.
    const auto d = BeaconShape::fromDesc(houdini::sync::shapes::make(sh), Platform::kHoudini);
    auto rcfg = cfg;
    rcfg.resolve({d.replicaLen(), 160.0, Platform::kHoudini});
    Detector det(d, rcfg.detector);
    SnrWindowGuard guard(d.coreLen(), rcfg.confirm.snr_floor_db,
                         SnrWindowGuard::guardFor(det.firstPathWindow()));
    houdini::sync::RepetitionPhaseEstimator cfo(d.geometry(), rcfg.cfo.window_margin, true);
    // A backend that ignores the configured form and pick (CUDA) cannot
    // reproduce windows recorded by the one that applies them: under such a
    // build the replay is skipped, said once per shape, but the fixtures are
    // still OPENED and COUNTED, so a missing or half-populated set fails the
    // count checks below rather than passing vacuously (round 7).
    const bool replay = det.backendAppliesConfig();
    if (!replay) {
      std::printf("SKIP  %s: backend %s does not apply the configuration; the windows are counted, not replayed\n",
                  shape, det.backendName());
    }
    // 00-05 recorded 2026-09-03 morning (index and SNR); 06-11 that afternoon
    // by the review-fix build, which also records the statistic and the bar.
    for (int i = 0; i < 12; ++i) {
      char nb[64];
      std::snprintf(nb, sizeof nb, "/resyncwin_%02d", i);
      const std::string base = dir + "/" + shape + nb;
      std::vector<std::complex<int16_t>> w;
      if (!readWindow(base + ".bin", &w)) continue;
      const auto meta = readMeta(base + ".txt");
      ++windows;
      ++per_shape[shape];
      if (!replay) continue;
      if (!meta.has("sync_index") || !meta.has("snr")) {
        check(false, std::string(shape) + " window " + std::to_string(i) + ": fixture txt lacks sync_index/snr");
        continue;
      }
      // Fixtures recorded after 2026-09-03 carry the settings they were taken
      // under; when present they must match what this test runs.
      float corr_scale = kCorrScale;
      if (meta.has("corr_scale")) corr_scale = static_cast<float>(meta.at("corr_scale"));
      // Every recorded setting that moves the index or the SNR must match
      // what this test runs, or the comparison is between two configurations.
      const std::string tag = std::string(shape) + " window " + std::to_string(i);
      // Enumerations are recorded by NAME, so a reordering cannot silently
      // re-key a fixture.
      if (meta.text.count("thresh"))
        check(meta.text.at("thresh") == houdini::sync::name(det.form()), tag + ": recorded threshold form matches (" + meta.text.at("thresh") + ")");
      else check(false, tag + ": fixture records the threshold form by name");
      if (meta.text.count("pick"))
        check(meta.text.at("pick") == houdini::sync::name(det.pick()), tag + ": recorded pick rule matches (" + meta.text.at("pick") + ")");
      else check(false, tag + ": fixture records the pick rule by name");
      if (meta.has("first_path_window"))
        check(det.firstPathWindow() == static_cast<int>(meta.at("first_path_window")), tag + ": recorded first-path window matches");
      if (meta.has("first_path_floor_db"))
        check(std::fabs(det.firstPathFloorDb() - meta.at("first_path_floor_db")) < 1e-6, tag + ": recorded first-path floor matches");
      if (meta.has("snr_floor_db"))
        check(std::fabs(guard.floorDb() - meta.at("snr_floor_db")) < 1e-6, tag + ": recorded SNR floor matches");
      if (meta.has("snr_guard"))
        check(guard.guard() == static_cast<size_t>(meta.at("snr_guard")), tag + ": recorded SNR guard matches");
      if (meta.has("replica_tail"))
        check(det.replicaTail() == static_cast<size_t>(meta.at("replica_tail")), tag + ": recorded replica tail matches");
      const auto det_res = det.run(w.data(), w.size(), corr_scale);
      const long long want = static_cast<long long>(meta.at("sync_index"));
      char what[160];
      std::snprintf(what, sizeof what, "%s window %d: index %lld (recorded %lld)", shape, i,
                    static_cast<long long>(det_res.end_index), want);
      check(det_res.end_index == want, what);
      // The statistic, when the fixture recorded one (dumps after the
      // Detection widening of 2026-09-03): a change that leaves the argmax
      // alone but moves the margin is visible here.
      if (meta.has("statistic") && std::isfinite(meta.at("statistic"))) {  // NaN = recorded by a backend without evidence
        ++with_stat;
        std::snprintf(what, sizeof what, "%s window %d: statistic %.5g (recorded %.5g)", shape, i,
                      det_res.statistic, meta.at("statistic"));
        check(std::fabs(det_res.statistic - meta.at("statistic")) <= 1e-4 * std::max(1.0, std::fabs(meta.at("statistic"))), what);
      }
      // Informational, not a check: where the strongest crossing sits and how
      // much stronger it is than the picked first path. The fixtures' recorded
      // statistic (8.176) showed picks as low as 2 dB above the bar on legacy.
      const auto am = det.run(w.data(), w.size(), corr_scale, PickRule::kArgmax);
      {
        std::printf("INFO  %s window %d: picked %lld stat %.4g | argmax %lld stat %.4g | pick - argmax = %lld samples, %.1f dB below \n",
                    shape, i, static_cast<long long>(det_res.end_index), det_res.statistic,
                    static_cast<long long>(am.end_index), am.statistic,
                    static_cast<long long>(det_res.end_index - am.end_index),
                    am.statistic > 0 ? 10.0 * std::log10(am.statistic / std::max(1e-12, det_res.statistic)) : 0.0);
      }
      const double snr = guard.snrDb(w.data(), w.size(), det_res.end_index);
      std::snprintf(what, sizeof what, "%s window %d: snr %.2f dB (recorded %.2f)", shape, i,
                    snr, meta.at("snr"));
      check(std::fabs(snr - meta.at("snr")) < 0.01, what);
      // The estimator at the shipped placement (detector end + guard, as the
      // receiver does): finite and inside +-50 kHz at 122.88 MSPS.
      const float f = cfo.estimate(w.data(), w.size(),
                                   static_cast<int>(det_res.end_index + rcfg.cfo.index_guard));
      const double hz = static_cast<double>(f) * Numerology::houdiniDefault().rate_hz;
      if (meta.has("cfo_hz")) {
        std::snprintf(what, sizeof what, "%s window %d: beacon cfo %+.0f Hz (recorded %+.0f)",
                      shape, i, hz, meta.at("cfo_hz"));
        check(std::fabs(hz - meta.at("cfo_hz")) < 1.0, what);
      } else {
        std::snprintf(what, sizeof what, "%s window %d: beacon cfo %+.3f Hz finite and plausible",
                      shape, i, hz);
        check(std::isfinite(f) && std::fabs(hz) < 50e3, what);
      }
    }
  }
  check(windows == 30, "found " + std::to_string(windows) + " fixture windows (expected 30)");
  // Reported, not asserted: NaN is a legal answer, but these windows are all
  // mid-slice detections with room to bracket, so a nonzero count on the
  // portable backend means the estimator stopped refining and is worth reading
  // before it is trusted (8ai).
  // Said out loud because "all 30 windows return their recorded index AND
  // statistic" would be false: the morning's fixtures predate the statistic
  // and only the later ones carry it (review, 8aj).
  std::printf("INFO  %d of %d windows carry a recorded statistic; the rest check the index only\n",
              with_stat, windows);
  for (const auto& kv : kExpected)
    check(per_shape[kv.first] == kv.second, kv.first + ": " + std::to_string(per_shape[kv.first]) + " of " +
                                                std::to_string(kv.second) + " fixture windows present (a half-populated set fails here, not quietly)");
  std::printf("\nRESULT: %s (%d failure(s))\n", g_fail ? "FAIL" : "PASS", g_fail);
  return g_fail ? 1 : 0;
}
