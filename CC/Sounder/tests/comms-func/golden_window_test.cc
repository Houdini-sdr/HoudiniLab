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
 * and resyncwin_NN.txt (key value lines: n, sync_index, snr, ...).
 */
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include "include/beacon_shapes.h"
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
std::map<std::string, double> readMeta(const std::string& path) {
  std::map<std::string, double> m;
  std::ifstream f(path);
  std::string k;
  double v;
  while (f >> k >> v) m[k] = v;
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
  const std::string dir = argc > 1 ? argv[1] : "tests/comms-func/fixtures/golden";
  // The configuration the windows were recorded under: shipped defaults, so
  // xcorr + first-path for legacy and coherence + first-path for nr_pss, the
  // 30 dB floor, corr_scale 10 (files/houdini-ul.json).
  const auto cfg = houdini::sync::SyncConfig::load("{}");
  const float kCorrScale = 10.0f;
  int windows = 0;
  for (const char* shape : {"legacy", "nr_pss"}) {
    beacon_shapes::Shape sh;
    if (!beacon_shapes::parse(shape, &sh)) { check(false, std::string("parse ") + shape); continue; }
    const auto d = beacon_shapes::make(sh);
    houdini::sync::Detector det(d.replica, d.replica_reps, d.replica_tail(), cfg.detector, true);
    houdini::sync::SnrWindowGuard guard(cfg.confirm.snr_floor_db,
                                        std::max<size_t>(8, cfg.detector.first_path_window));
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
      const auto det_res = det.run(w.data(), w.size(), kCorrScale);
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
      std::snprintf(what, sizeof what, "%s window %d: beacon cfo %+.0f Hz finite and plausible",
                    shape, i, hz);
      check(std::isfinite(f) && std::fabs(hz) < 50e3, what);
    }
  }
  check(windows >= 12, "found " + std::to_string(windows) + " fixture windows (expected 12)");
  std::printf("\nRESULT: %s (%d failure(s))\n", g_fail ? "FAIL" : "PASS", g_fail);
  return g_fail ? 1 : 0;
}
