/**
 * @file houdini_loopback.cc
 * @brief Closed-loop HIL test over the now-bidirectional Houdini link, driven by
 *        the sounder's real radio classes:
 *          1. BaseRadioSet (.21) replays the Gold beacon on TX ch1  [forward]
 *          2. ClientRadioSet (.22) syncs on that beacon on RX ch1
 *          3. ClientRadioSet (.22) transmits a Gold pilot on TX ch1  [reverse]
 *          4. BaseRadioSet (.21) receives on RX ch1 and find_beacon detects it
 *        i.e. the UE responds to the beacon and the BS hears it -- the loop is
 *        closed. Wiring: .21 DAC_A->.22 ADC_C (forward) and .22 DAC_A->.21 ADC_C
 *        (reverse), both ch1, both matched-NCO Zone 1.
 *
 * Build: CMake target houdini_loopback. Run (venv SoapySDR runtime):
 *   ./houdini_loopback --conf files/houdini-1u.json --iters 20
 */
#include <sys/types.h>

#include <atomic>
#include <chrono>
#include <complex>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <thread>
#include <vector>

#include "include/BaseRadioSet.h"
#include "include/ClientRadioSet.h"
#include "include/comms-lib.h"
#include "include/config.h"
#include "include/macros.h"

namespace {
using ci16 = std::complex<int16_t>;

ssize_t detect(const ci16* buf, const std::vector<std::complex<float>>& match,
               size_t win, float scale) {
#if defined(USE_CUDA)
  return CommsLib::find_beacon_cuda(buf, match, win, scale);
#else
  return CommsLib::find_beacon_avx(buf, match, win, scale);
#endif
}
}  // namespace

int main(int argc, char** argv) {
  std::string conf = "files/houdini-1u.json";
  int iters = 20;
  for (int i = 1; i < argc - 1; ++i) {
    if (!std::strcmp(argv[i], "--conf")) conf = argv[i + 1];
    if (!std::strcmp(argv[i], "--iters")) iters = std::atoi(argv[i + 1]);
  }

  Config cfg(conf, "logs", false, false, false);
  BaseRadioSet bs(&cfg, false);  // arms beacon replay + activates BS RX
  if (bs.getRadioNotFound()) { std::fprintf(stderr, "BS not found\n"); return 1; }
  ClientRadioSet ue(&cfg);
  if (ue.getRadioNotFound()) { std::fprintf(stderr, "UE not found\n"); return 1; }

  const int slot = static_cast<int>(cfg.samps_per_slot());
  const long long frame = static_cast<long long>(cfg.samps_per_frame());
  const float scale = cfg.corr_scale(0);
  const auto& gold = cfg.gold_cf32();

  // UE reverse "pilot": the STS+gold core of the beacon (conjugated for the R2C
  // mixer), zero-padded to one slot -> a timed burst the BS find_beacon detects.
  const auto& bc = cfg.beacon_ci16();
  const int p = cfg.prefix();
  const int n = cfg.beacon_size();
  std::vector<ci16> pilot(slot, ci16(0, 0));
  for (int k = 0; k < n && k < slot; ++k)
    pilot[k] = ci16(bc.at(p + k).real(),
                    static_cast<int16_t>(-bc.at(p + k).imag()));

  // --- Phase 1: UE acquires the BS beacon (forward link) ---
  const size_t uwin = static_cast<size_t>(static_cast<float>(slot) * 2.33f);
  std::vector<ci16> ubuf(uwin);
  long long ue_time = 0;
  ssize_t sidx = -1;
  for (int i = 0; i < 30 && sidx < 0; ++i) {
    void* b[1] = {ubuf.data()};
    if (ue.radioRx(0, b, static_cast<int>(uwin), ue_time) != (int)uwin) continue;
    sidx = detect(ubuf.data(), gold, uwin, scale);
  }
  std::printf("[fwd] UE synced on BS beacon: idx=%zd, ue_time=%lld\n", sidx,
              ue_time);
  if (sidx < 0) { std::fprintf(stderr, "UE never synced\n"); return 1; }

  // --- Phase 2: UE transmits the pilot continuously (reverse), BS receives ---
  // The two boards are not clock-synced, so absolute-time (scheduled-burst)
  // coordination between the UE TX and the BS RX is impossible. Instead the UE
  // streams the pilot continuously (flags=0 = immediate) from a background
  // thread, so a pilot is always on the wire whenever the BS captures.
  // Start the BS RX only now (fresh), so it hasn't overflowed during the sync.
  bs.activateHoudiniRx();
  const size_t bwin = static_cast<size_t>(2 * frame);
  std::vector<ci16> bbuf(bwin);
  std::printf("[rev] UE streaming pilot continuously; BS RX window %zu\n\n",
              bwin);

  std::atomic<bool> tx_run{true};
  std::atomic<long long> tx_calls{0};
  std::thread txth([&] {
    const void* tb[1] = {pilot.data()};
    while (tx_run.load()) {
      long long tt = 0;
      ue.radioTx(0, tb, slot, 0, tt);  // flags=0 -> immediate streaming
      tx_calls.fetch_add(1);
    }
  });

  int hits = 0;
  for (int it = 0; it < iters; ++it) {
    long long bt = 0;
    void* bb[1] = {bbuf.data()};
    const int rr = bs.radioRx(0, 0, bb, static_cast<int>(bwin), bt);
    ssize_t idx = -1;
    if (rr == static_cast<int>(bwin)) {
      idx = detect(bbuf.data(), gold, bwin, scale);
      if (idx < 0) {  // try the other conj sense (R2C)
        for (auto& c : bbuf) c = ci16(c.real(), static_cast<int16_t>(-c.imag()));
        idx = detect(bbuf.data(), gold, bwin, scale);
      }
    }
    if (idx >= 0) ++hits;
    std::printf("it %2d: BS rx=%d  find_beacon=%zd  %s\n", it, rr, idx,
                idx >= 0 ? "*** UE PILOT @ BS ***" : "");
    std::fflush(stdout);
  }
  tx_run.store(false);
  txth.join();
  std::printf(
      "\n%d/%d BS captures detected the UE pilot (%lld UE tx calls) -- reverse "
      "link %s\n",
      hits, iters, tx_calls.load(), hits > 0 ? "CLOSED" : "not detected");
  return hits > 0 ? 0 : 1;
}
