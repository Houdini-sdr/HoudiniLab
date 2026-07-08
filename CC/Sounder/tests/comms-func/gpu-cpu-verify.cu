/**
 * @file gpu-cpu-verify.cu
 * @brief Assert the GPU beacon detector (CommsLib::find_beacon_cuda) returns the
 *        SAME peak index as the CPU golden reference (CommsLib::find_beacon_avx)
 *        on identical beacon-in-noise inputs, across window sizes and beacon
 *        positions. The receiver uses that peak index for sync, so matching
 *        indices is the correctness contract. Needs no hardware.
 *
 * Same synthetic 2x128 beacon as correlator-rate-bench / gpu-correlator-bench.
 * A clear (high-SNR) beacon means both paths should land on the exact same
 * index despite float summation-order differences.
 *
 * Build on the DGX Spark:
 *   nvcc -O3 -std=c++17 -arch=native -I include -I . \
 *     tests/comms-func/gpu-cpu-verify.cu find_beacon_cuda.cu \
 *     comms-lib-avx.cc comms-lib-portable.cc utils.cc -lpthread -o gpu-cpu-verify
 *   ./gpu-cpu-verify
 */
#include <sys/types.h>

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <random>
#include <vector>

#include "include/comms-lib.h"

namespace {
constexpr int kSeqLen = 128;
constexpr int kGoldReps = 2;

std::vector<std::complex<float>> MakeMatch(int seq_len) {
  std::mt19937 rng(0x5eedU);
  std::uniform_int_distribution<int> bit(0, 1);
  std::vector<std::complex<float>> m(seq_len);
  for (int i = 0; i < seq_len; ++i)
    m[i] = std::complex<float>(bit(rng) ? 0.5f : -0.5f, bit(rng) ? 0.5f : -0.5f);
  return m;
}

std::vector<std::complex<int16_t>> MakeBuffer(
    const std::vector<std::complex<float>>& match, size_t window,
    size_t beacon_pos, unsigned seed) {
  std::mt19937 rng(seed);
  std::normal_distribution<float> noise(0.0f, 0.002f);
  std::vector<std::complex<float>> buf(window);
  for (size_t i = 0; i < window; ++i)
    buf[i] = std::complex<float>(noise(rng), noise(rng));
  for (int r = 0; r < kGoldReps; ++r)
    for (int i = 0; i < static_cast<int>(match.size()); ++i) {
      const size_t idx = beacon_pos + static_cast<size_t>(r) * match.size() + i;
      if (idx < window) buf[idx] += match[i];
    }
  auto clamp16 = [](float v) {
    return static_cast<int16_t>(
        std::lround(std::max(-1.0f, std::min(1.0f, v)) * 32767.0f));
  };
  std::vector<std::complex<int16_t>> ci(window);
  for (size_t i = 0; i < window; ++i)
    ci[i] =
        std::complex<int16_t>(clamp16(buf[i].real()), clamp16(buf[i].imag()));
  return ci;
}
}  // namespace

int main() {
  const auto match = MakeMatch(kSeqLen);
  const std::vector<size_t> windows = {1024,  2048,  4096,  8192,
                                       16384, 32768, 65536};
  const unsigned seeds[] = {0xC0FFEEU, 0x1234U, 0xBEEFU};

  int total = 0, fails = 0, no_detect = 0;
  printf("%9s %9s %7s  %8s %8s  %s\n", "window", "beaconpos", "seed", "cpu",
         "gpu", "result");
  for (size_t n : windows) {
    for (unsigned seed : seeds) {
      const size_t beacon_pos = 501 + (seed & 0xFFu);  // vary the position
      if (beacon_pos + static_cast<size_t>(kGoldReps) * kSeqLen + 8 >= n)
        continue;
      const auto buf = MakeBuffer(match, n, beacon_pos, seed);
      const ssize_t cpu = CommsLib::find_beacon_avx(buf.data(), match, n, 1.0f);
      const ssize_t gpu = CommsLib::find_beacon_cuda(buf.data(), match, n, 1.0f);
      const bool ok = (cpu == gpu);
      ++total;
      if (!ok) ++fails;
      if (cpu < 0) ++no_detect;  // beacon should always be found here
      printf("%9zu %9zu 0x%05x  %8zd %8zd  %s\n", n, beacon_pos, seed, cpu, gpu,
             ok ? "MATCH" : "MISMATCH");
    }
  }
  printf("\n%d/%d peaks matched", total - fails, total);
  if (no_detect) printf("  (WARNING: %d cases detected no beacon)", no_detect);
  printf(" -> %s\n", (fails == 0 && no_detect == 0) ? "PASS" : "FAIL");
  return (fails == 0 && no_detect == 0) ? 0 : 1;
}
