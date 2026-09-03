/**
 * @file beacon_shape_dump.cc
 * @brief Write each candidate beacon's TX core and correlator replica to disk,
 *        from the ONE definition in include/sync/beacon_shapes.h.
 *
 * The bench probes are Python and must not re-implement the sequences: a
 * re-implementation that agrees with itself proves nothing, and one that
 * disagrees means the bench measures a beacon the build does not transmit.
 * AP-34(a) is what that costs. So the C++ that the sounder will build the
 * beacon from also writes the files the probes read.
 *
 * Formats match what Config::genPilots already dumps under HOUDINI_DUMP_GOLD,
 * so the existing probes read these without changes:
 *   <name>_core.bin     complex<int16_t>, the transmit core at `--peak` counts
 *   <name>_replica.bin  float pairs (re, im), the matched-filter reference
 *   shapes.json         geometry every probe needs (offsets, lengths, PAPR)
 *
 * Run: ./beacon_shape_dump --out DIR [--peak 19660]
 */
#include <cmath>
#include <complex>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include "sync/beacon_shapes.h"
#include "sync/beacon_shape.h"
#include "sync/confirm.h"
#include "sync/detector.h"

int main(int argc, char** argv) {
  std::string out = ".";
  // 0.6 of full scale, which is what HoudiniFramer::buildBeacon puts the core at.
  double peak = 0.6 * 32767.0;
  for (int i = 1; i < argc; ++i) {
    if (!std::strcmp(argv[i], "--out") && i + 1 < argc) out = argv[++i];
    else if (!std::strcmp(argv[i], "--peak") && i + 1 < argc) peak = std::atof(argv[++i]);
    else { std::fprintf(stderr, "unknown arg %s\n", argv[i]); return 2; }
  }
  std::string js = "{\n  \"peak_counts\": " + std::to_string(peak) + ",\n  \"shapes\": {\n";
  bool first = true;
  for (const char* name : houdini::sync::shapes::kAllNames) {
    houdini::sync::shapes::Shape sh;
    if (!houdini::sync::shapes::parse(name, &sh)) { std::fprintf(stderr, "bad %s\n", name); return 2; }
    const auto d = houdini::sync::shapes::make(sh);
    double pk = 0.0;
    for (const auto& v : d.core) pk = std::max(pk, static_cast<double>(std::norm(v)));
    const double g = peak / std::sqrt(pk);

    const std::string cp = out + "/" + d.name + "_core.bin";
    FILE* f = std::fopen(cp.c_str(), "wb");
    if (!f) { std::perror(cp.c_str()); return 1; }
    for (const auto& v : d.core) {
      // Round, do not truncate: truncation biases every sample toward zero and
      // costs a fraction of a dB that is indistinguishable from a real result.
      const std::complex<int16_t> s(
          static_cast<int16_t>(std::lround(std::max(-32767.0, std::min(32767.0, v.real() * g)))),
          static_cast<int16_t>(std::lround(std::max(-32767.0, std::min(32767.0, v.imag() * g)))));
      std::fwrite(&s, sizeof s, 1, f);
    }
    std::fclose(f);

    const std::string rp = out + "/" + d.name + "_replica.bin";
    f = std::fopen(rp.c_str(), "wb");
    if (!f) { std::perror(rp.c_str()); return 1; }
    for (const auto& v : d.replica) {
      const float xy[2] = {v.real(), v.imag()};
      std::fwrite(xy, sizeof(float), 2, f);
    }
    std::fclose(f);

    // The RESOLVED decisions the sounder would run this shape with, at the
    // shipped defaults on Houdini, so the Python probes read a number instead
    // of re-deriving the rule (a mismatch is then a JSON key, not a silent
    // divergence): the threshold form, the pick, the replica tail, the
    // first-path window, the SNR guard, the expected end, and the coherence
    // bar at the reserved pfa over a 4096-sample window.
    const auto shape = houdini::sync::BeaconShape::fromDesc(d, houdini::sync::Platform::kHoudini);
    auto cfg = houdini::sync::SyncConfig::defaults();
    cfg.resolve({shape.replicaLen(), 160.0, houdini::sync::Platform::kHoudini});
    const houdini::sync::Detector det(shape, cfg.detector);
    char buf[1024];
    std::snprintf(buf, sizeof buf,
                  "%s    \"%s\": {\"core_len\": %zu, \"replica_len\": %zu, "
                  "\"replica_off\": %zu, \"replica_reps\": %zu, "
                  "\"fine_off\": %zu, \"fine_len\": %zu, \"fine_reps\": %zu, "
                  "\"guard_len\": %zu, \"coarse_off\": %zu, \"coarse_len\": %zu, "
                  "\"coarse_reps\": %zu, \"papr_db\": %.3f, "
                  "\"form\": \"%s\", \"pick\": \"%s\", \"replica_tail\": %zu, "
                  "\"first_path_window\": %d, \"first_path_floor_db\": %.1f, "
                  "\"snr_guard\": %zu, \"end_offset_houdini\": %zd, "
                  "\"coherence_bar_pfa%.0e_4096\": %.6f}",
                  first ? "" : ",\n", d.name.c_str(), d.core.size(),
                  d.replica.size(), d.replica_off, d.replica_reps, d.fine_off, d.fine_len, d.fine_reps,
                  d.guard_len, d.coarse_off, d.coarse_len, d.coarse_reps,
                  d.papr_db(), houdini::sync::name(det.form()), houdini::sync::name(det.pick()),
                  shape.replicaTail(), det.firstPathWindow(), det.firstPathFloorDb(),
                  houdini::sync::SnrWindowGuard::guardFor(det.firstPathWindow()),
                  shape.expectedEndOffset(), cfg.detector.pfa_per_window,
                  houdini::sync::ThresholdPolicy::coherenceBar(shape.replicaLen(), cfg.detector.pfa_per_window, 4096));
    js += buf;
    first = false;
    std::printf("%-13s core %4zu  replica %3zu  fine@%zu x%zu  guard %zu  PAPR %.2f dB\n",
                d.name.c_str(), d.core.size(), d.replica.size(), d.fine_off,
                d.fine_len, d.guard_len, d.papr_db());
  }
  js += "\n  }\n}\n";
  const std::string jp = out + "/shapes.json";
  FILE* f = std::fopen(jp.c_str(), "wb");
  if (!f) { std::perror(jp.c_str()); return 1; }
  std::fwrite(js.data(), 1, js.size(), f);
  std::fclose(f);
  std::printf("wrote %zu shapes to %s\n",
              sizeof(houdini::sync::shapes::kAllNames) / sizeof(*houdini::sync::shapes::kAllNames),
              out.c_str());
  return 0;
}
