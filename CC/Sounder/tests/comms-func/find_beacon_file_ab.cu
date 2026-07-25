/**
 * @file find_beacon_file_ab.cu
 * @brief Run BOTH CommsLib::find_beacon_cuda and ::find_beacon_avx on a captured
 *        cs16 window + a gold_cf32 file, and print both indices. Isolates a
 *        GPU/CPU divergence on REAL beacon data (dense gold + silent rx_gate half)
 *        that the synthetic single-beacon gpu-cpu-verify never exercises.
 *
 * Build on the DGX Spark:
 *   nvcc -O3 -std=c++17 -arch=native -I include -I . \
 *     tests/comms-func/find_beacon_file_ab.cu find_beacon_cuda.cu \
 *     comms-lib-avx.cc comms-lib-portable.cc utils.cc -lpthread -o /tmp/fb_ab
 *   /tmp/fb_ab /tmp/cl_win.bin /tmp/gold.bin [corr_scale]
 */
#include <complex>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <vector>

#include "include/comms-lib.h"

static std::vector<std::complex<int16_t>> LoadCs16(const char* p) {
  FILE* f = std::fopen(p, "rb");
  if (!f) { std::perror(p); std::exit(1); }
  std::fseek(f, 0, SEEK_END);
  long bytes = std::ftell(f);
  std::fseek(f, 0, SEEK_SET);
  std::vector<std::complex<int16_t>> v(bytes / 4);
  if (std::fread(v.data(), 4, v.size(), f) != v.size()) { std::perror("read"); }
  std::fclose(f);
  return v;
}

static std::vector<std::complex<float>> LoadCf32(const char* p) {
  FILE* f = std::fopen(p, "rb");
  if (!f) { std::perror(p); std::exit(1); }
  std::fseek(f, 0, SEEK_END);
  long bytes = std::ftell(f);
  std::fseek(f, 0, SEEK_SET);
  std::vector<std::complex<float>> v(bytes / 8);
  if (std::fread(v.data(), 8, v.size(), f) != v.size()) { std::perror("read"); }
  std::fclose(f);
  return v;
}

int main(int argc, char** argv) {
  if (argc < 3) {
    std::fprintf(stderr, "usage: %s <cs16.bin> <gold_cf32.bin> [corr_scale]\n",
                 argv[0]);
    return 2;
  }
  const float cs = (argc > 3) ? std::atof(argv[3]) : 1.0f;
  auto raw = LoadCs16(argv[1]);
  auto gold = LoadCf32(argv[2]);
  const size_t win = raw.size();
  std::printf("window=%zu gold=%zu corr_scale=%.3f\n", win, gold.size(), cs);

  const ssize_t g = CommsLib::find_beacon_cuda(raw.data(), gold, win, cs);
  const ssize_t c = CommsLib::find_beacon_avx(raw.data(), gold, win, cs);
  std::printf("find_beacon_cuda -> %ld\n", g);
  std::printf("find_beacon_avx  -> %ld\n", c);
  std::printf("%s\n", (g == c) ? "MATCH" : "*** DIVERGENCE ***");
  return 0;
}
