/**
 * @file houdini_bs_ue_sync.cc
 * @brief Drive BOTH ends of the Houdini HIL link with the sounder's real radio
 *        classes: construct the actual BaseRadioSet (which now arms the Gold
 *        beacon replay on the BS board) and the actual ClientRadioSet (the UE),
 *        then run receiver.cc::syncSearch's exact correlator (find_beacon on
 *        config_->gold_cf32()) on the UE RX. This proves the BaseRadioSet
 *        Houdini backend transmits the beacon the UE acquires -- replacing the
 *        external beacon_tx_gold helper with the sounder's own BS class.
 *
 * The boards are wired TX(.21 ch1) -> RX(.22 ch1) only, so the BS->UE beacon is
 * exercised here; the reverse link (BS receiving UE pilots) needs bidirectional
 * wiring + a Houdini-compatible BS receive framer and is out of scope.
 *
 * Build: CMake target houdini_bs_ue_sync (links the full sounder sources).
 * Run (venv SoapySDR runtime):
 *   ./houdini_bs_ue_sync --conf files/houdini-1u.json --iters 20
 */
#include <sys/types.h>

#include <chrono>
#include <complex>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include "include/BaseRadioSet.h"
#include "include/ClientRadioSet.h"
#include "include/comms-lib.h"
#include "include/config.h"

int main(int argc, char** argv) {
  std::string conf = "files/houdini-1u.json";
  int iters = 20;
  for (int i = 1; i < argc - 1; ++i) {
    if (!std::strcmp(argv[i], "--conf")) conf = argv[i + 1];
    if (!std::strcmp(argv[i], "--iters")) iters = std::atoi(argv[i + 1]);
  }

  Config cfg(conf, "logs", /*bs_only=*/false, /*client_only=*/false,
             /*calibrate=*/false);
  std::printf("radio_type=%s  bs_present=%d  client_present=%d\n",
              cfg.radio_type().c_str(), cfg.bs_present(), cfg.client_present());

  // Real BS class -- constructing it arms the Gold beacon replay on the BS board.
  BaseRadioSet bs(&cfg, false);
  if (bs.getRadioNotFound()) {
    std::fprintf(stderr, "BS radio not found\n");
    return 1;
  }
  // Real UE class.
  ClientRadioSet ue(&cfg);
  if (ue.getRadioNotFound()) {
    std::fprintf(stderr, "UE radio not found\n");
    return 1;
  }

  const size_t window =
      static_cast<size_t>(static_cast<float>(cfg.samps_per_slot()) * 2.33f);
  std::vector<std::complex<int16_t>> buf(window);
  std::vector<void*> b{buf.data()};
  const float scale = cfg.corr_scale(0);
  std::printf("UE syncing on the BaseRadioSet beacon: window %zu, %d frames\n\n",
              window, iters);

  int hits = 0;
  for (int it = 0; it < iters; ++it) {
    long long t = 0;
    const int r = ue.radioRx(0, b.data(), static_cast<int>(window), t);
    if (r != static_cast<int>(window)) {
      std::printf("frame %3d: short rx %d/%zu\n", it, r, window);
      continue;
    }
    const auto t0 = std::chrono::steady_clock::now();
#if defined(USE_CUDA)
    const ssize_t idx =
        CommsLib::find_beacon_cuda(buf.data(), cfg.gold_cf32(), window, scale);
#else
    const ssize_t idx =
        CommsLib::find_beacon_avx(buf.data(), cfg.gold_cf32(), window, scale);
#endif
    const double us = std::chrono::duration<double, std::micro>(
                          std::chrono::steady_clock::now() - t0)
                          .count();
    if (idx >= 0) ++hits;
    std::printf("frame %3d: find_beacon -> %6zd  (%6.0f us)  %s\n", it, idx, us,
                idx >= 0 ? "*** SYNC ***" : "searching");
    std::fflush(stdout);
  }
  std::printf("\n%d/%d frames synced on the BaseRadioSet-driven Gold beacon\n",
              hits, iters);
  return hits > 0 ? 0 : 1;
}
