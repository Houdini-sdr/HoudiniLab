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
#include <string>
#include <vector>

#include "beacon_shapes.h"
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
  out->resize(bytes / sizeof(std::complex<int16_t>));
  f.read(reinterpret_cast<char*>(out->data()), static_cast<std::streamsize>(bytes));
  return true;
}
}  // namespace

int main(int argc, char** argv) {
  // Nothing from the operator's shell may change what this test builds.
  {
    houdini::sync::SyncConfig probe = houdini::sync::SyncConfig::defaults();
    for (const auto& k : probe.knobs())
      if (k.env != nullptr) unsetenv(k.env);
  }
  const std::string dir = argc > 1 ? argv[1] : "tests/comms-func/fixtures/golden";
  // The first-path window derives from the replica: 64 at 128 taps, 32 at 64.
  // The pre-library correlator did exactly this (comms-lib-portable.cc
  // firstPathWindow), and a fixed 64 would have doubled dot11's and nr's.
  for (const char* shape : {"dot11", "nr", "legacy", "nr_pss"}) {
    beacon_shapes::Shape sh = beacon_shapes::Shape::kLegacy;
    if (!beacon_shapes::parse(shape, &sh)) {
      check(false, std::string("parse ") + shape);
      continue;
    }
    const auto d = beacon_shapes::make(sh);
    houdini::sync::Detector det(d.replica, d.replica_reps, d.replica_tail(),
                                houdini::sync::SyncConfig::defaults().detector, true);
    check(det.firstPathWindow() == static_cast<int>(d.replica.size() / 2),
          std::string(shape) + ": first-path window derives to half the replica (" +
              std::to_string(det.firstPathWindow()) + ")");
  }
  // The configuration the windows were recorded under: shipped defaults, so
  // xcorr + first-path for legacy and coherence + first-path for nr_pss, the
  // 30 dB floor, corr_scale 10 (files/houdini-ul.json).
  // Detector behaviour that the fixtures exercise only indirectly.
  {
    using houdini::sync::Detector;
    using houdini::sync::ThresholdForm;
    check(Detector::resolveForm(ThresholdForm::kAuto, false, false) == CommsLib::BeaconThresh::kPowerRatio &&
              Detector::resolveForm(ThresholdForm::kCoherence, false, false) == CommsLib::BeaconThresh::kPowerRatio,
          "resolveForm: the Iris/UHD path keeps the power-ratio form whatever is asked");
    check(Detector::resolveForm(ThresholdForm::kNormalizedXCorr, true, true) == CommsLib::BeaconThresh::kXCorrNoLag,
          "resolveForm: a single-copy replica forces the coherence form");
    beacon_shapes::Shape sh = beacon_shapes::Shape::kNrPss;
    const auto d = beacon_shapes::make(sh);
    houdini::sync::DetectorConfig dc;
    dc.first_path_window = 4095;
    Detector capped(d.replica, d.replica_reps, d.replica_tail(), dc, true);
    check(capped.firstPathWindow() == 2 * static_cast<int>(d.replica.size()),
          "an explicit first-path window is capped at twice the replica (" + std::to_string(capped.firstPathWindow()) + ")");
    Detector iris(d.replica, d.replica_reps, d.replica_tail(), houdini::sync::DetectorConfig{}, false);
    check(iris.replicaTail() == 0 && iris.pick() == CommsLib::BeaconPick::kFirstClusterRefined,
          "off Houdini: no replica tail, cluster-refined pick");
    // The tail pushes an end past the window: reported as no detection.
    Detector det(d.replica, d.replica_reps, d.replica_tail(), houdini::sync::DetectorConfig{}, true);
    std::vector<std::complex<int16_t>> tiny(d.core.size() + 8, std::complex<int16_t>(0, 0));
    for (size_t i = 0; i < d.core.size(); ++i)
      tiny[i + 8] = std::complex<int16_t>(static_cast<int16_t>(d.core[i].real() * 8000),
                                          static_cast<int16_t>(d.core[i].imag() * 8000));
    // Positive control first: on the full buffer the detector finds the core
    // end where it was placed (8 + core_len - 1).
    const auto full = det.run(tiny.data(), tiny.size(), 10.0f);
    check(full.index == static_cast<ssize_t>(8 + d.core.size() - 1),
          "positive control: the placed core is found at its end (" + std::to_string(full.index) + ")");
    // Then the PSS ends 144 samples before the core end; a window that stops at
    // the PSS end + 100 cannot hold the implied core end.
    const auto r = det.run(tiny.data(), 8 + 128 + 100, 10.0f);
    check(!r.found(), "a detection whose implied core end falls outside the window is reported as none");
    // guardFor: the pre-library guard (64 unless configured), never narrower
    // than the resolved window.
    using houdini::sync::SnrWindowGuard;
    check(SnrWindowGuard::guardFor(-1, 32) == 64 && SnrWindowGuard::guardFor(-1, 64) == 64 &&
              SnrWindowGuard::guardFor(16, 32) == 32 && SnrWindowGuard::guardFor(100, 32) == 100 &&
              SnrWindowGuard::guardFor(0, 4) == 8,
          "guardFor: 64 by default, the configured value when given, never below the window or 8");
  }
  const auto cfg = houdini::sync::SyncConfig::load("{}");
  const float kCorrScale = 10.0f;
  int windows = 0;
  for (const char* shape : {"legacy", "nr_pss"}) {
    beacon_shapes::Shape sh;
    if (!beacon_shapes::parse(shape, &sh)) { check(false, std::string("parse ") + shape); continue; }
    const auto d = beacon_shapes::make(sh);
    houdini::sync::Detector det(d.replica, d.replica_reps, d.replica_tail(), cfg.detector, true);
    houdini::sync::SnrWindowGuard guard(
        cfg.confirm.snr_floor_db,
        houdini::sync::SnrWindowGuard::guardFor(cfg.detector.first_path_window,
                                                det.firstPathWindow()));
    houdini::sync::FieldGeometry g;
    g.core_len = static_cast<int>(d.core.size());
    g.fine_off = static_cast<int>(d.fine_off); g.fine_len = static_cast<int>(d.fine_len);
    g.fine_reps = static_cast<int>(d.fine_reps);
    g.coarse_off = static_cast<int>(d.coarse_off); g.coarse_len = static_cast<int>(d.coarse_len);
    g.coarse_reps = static_cast<int>(d.coarse_reps);
    houdini::sync::RepetitionPhaseEstimator cfo(g, cfg.cfo.window_margin, true);
    for (int i = 0; i < 6; ++i) {
      char nb[64];
      std::snprintf(nb, sizeof nb, "/resyncwin_%02d", i);
      const std::string base = dir + "/" + shape + nb;
      std::vector<std::complex<int16_t>> w;
      if (!readWindow(base + ".bin", &w)) continue;
      const auto meta = readMeta(base + ".txt");
      ++windows;
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
      if (meta.has("thresh"))
        check(static_cast<int>(det.form()) == static_cast<int>(meta.at("thresh")), tag + ": recorded threshold form matches");
      if (meta.has("pick"))
        check(static_cast<int>(det.pick()) == static_cast<int>(meta.at("pick")), tag + ": recorded pick rule matches");
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
                    static_cast<long long>(det_res.index), want);
      check(det_res.index == want, what);
      const double snr = guard.snrDb(w.data(), w.size(), det_res.index, d.core.size());
      std::snprintf(what, sizeof what, "%s window %d: snr %.2f dB (recorded %.2f)", shape, i,
                    snr, meta.at("snr"));
      check(std::fabs(snr - meta.at("snr")) < 0.01, what);
      // The estimator at the shipped placement (detector end + guard, as the
      // receiver does): finite and inside +-50 kHz at 122.88 MSPS.
      const float f = cfo.estimate(w.data(), w.size(),
                                   static_cast<int>(det_res.index + cfg.cfo.index_guard));
      const double hz = static_cast<double>(f) * 122.88e6;
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
  check(windows >= 12, "found " + std::to_string(windows) + " fixture windows (expected 12)");
  std::printf("\nRESULT: %s (%d failure(s))\n", g_fail ? "FAIL" : "PASS", g_fail);
  return g_fail ? 1 : 0;
}
