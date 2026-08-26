/**
 * @file beacon_hil.cu
 * @brief Run the Sounder's production beacon correlator (CommsLib::find_beacon_avx
 *        and find_beacon_cuda) on a LIVE over-the-air capture, closing the loop
 *        from real RF to the real detector (not the standalone Python matched
 *        filter). Two modes:
 *
 *   --dump-tx <file>   write the 128-sample beacon match (interleaved float I,Q)
 *                      so the Python TX side plays exactly this sequence.
 *   --detect <cap.bin> read a capture (interleaved int16 I,Q from the RX board),
 *                      run find_beacon_avx (+ find_beacon_cuda when built with
 *                      -DUSE_CUDA) and print the detected beacon index. Both
 *                      conjugation senses are tried (the matched-NCO R2C path may
 *                      deliver the beacon conjugated).
 *
 * The match is the same deterministic 2x128 sequence gpu-cpu-verify uses (a
 * stand-in for CommsLib::getSequence(GOLD_IFFT); swap in the real Gold once the
 * getSequence/muFFT path is linked).
 *
 * Build on the DGX Spark:
 *   nvcc -O3 -std=c++17 -arch=native -DUSE_CUDA -DHOUDINI_USE_CUDA -I include -I . \
 *     tests/comms-func/beacon_hil.cu find_beacon_cuda.cu \
 *     comms-lib-avx.cc comms-lib-portable.cc utils.cc -lpthread -o beacon_hil
 */
#include <sys/types.h>

#include <algorithm>
#include <chrono>
#include <complex>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <random>
#include <vector>

#include "include/comms-lib.h"

namespace {
constexpr int kSeqLen = 128;

std::vector<std::complex<float>> MakeMatch(int seq_len) {
  std::mt19937 rng(0x5eedU);
  std::uniform_int_distribution<int> bit(0, 1);
  std::vector<std::complex<float>> m(seq_len);
  for (int i = 0; i < seq_len; ++i)
    m[i] = std::complex<float>(bit(rng) ? 0.5f : -0.5f, bit(rng) ? 0.5f : -0.5f);
  return m;
}

std::vector<std::complex<int16_t>> ReadCaptureI16(const char* path) {
  std::ifstream f(path, std::ios::binary | std::ios::ate);
  if (!f) { std::fprintf(stderr, "cannot open %s\n", path); std::exit(1); }
  const size_t bytes = static_cast<size_t>(f.tellg());
  f.seekg(0);
  std::vector<int16_t> raw(bytes / sizeof(int16_t));
  f.read(reinterpret_cast<char*>(raw.data()), static_cast<std::streamsize>(bytes));
  std::vector<std::complex<int16_t>> cap(raw.size() / 2);
  for (size_t i = 0; i < cap.size(); ++i)
    cap[i] = std::complex<int16_t>(raw[2 * i], raw[2 * i + 1]);
  return cap;
}
}  // namespace

int main(int argc, char** argv) {
  const auto match = MakeMatch(kSeqLen);
  const char* mode = argc > 1 ? argv[1] : "";

  if (std::strcmp(mode, "--dump-tx") == 0 && argc > 2) {
    std::ofstream o(argv[2], std::ios::binary);
    for (const auto& c : match) {
      float v[2] = {c.real(), c.imag()};
      o.write(reinterpret_cast<const char*>(v), sizeof(v));
    }
    std::printf("wrote %d-sample beacon match to %s\n", kSeqLen, argv[2]);
    return 0;
  }

  if (std::strcmp(mode, "--detect") == 0 && argc > 2) {
    auto cap = ReadCaptureI16(argv[2]);
    size_t window = cap.size();
    float scale = 1.0f;
    for (int i = 3; i < argc - 1; ++i) {
      if (!std::strcmp(argv[i], "--window"))
        window = std::min(cap.size(), static_cast<size_t>(std::atol(argv[i + 1])));
      if (!std::strcmp(argv[i], "--scale")) scale = std::atof(argv[i + 1]);
    }
    std::printf("capture %zu samples; window %zu; scale %.2f\n",
                cap.size(), window, scale);
    bool any = false;
    for (int cj = 0; cj < 2; ++cj) {
      std::vector<std::complex<int16_t>> raw = cap;
      if (cj)
        for (auto& c : raw)
          c = std::complex<int16_t>(c.real(), static_cast<int16_t>(-c.imag()));
      const ssize_t a =
          CommsLib::find_beacon_avx(raw.data(), match, window, scale);
      std::printf("  conj=%d  find_beacon_avx  -> %zd%s\n", cj, a,
                  a >= 0 ? "   *** DETECTED ***" : "");
      any = any || (a >= 0);
#ifdef USE_CUDA
      const ssize_t g =
          CommsLib::find_beacon_cuda(raw.data(), match, window, scale);
      std::printf("  conj=%d  find_beacon_cuda -> %zd%s%s\n", cj, g,
                  g >= 0 ? "   *** DETECTED ***" : "",
                  (a >= 0 && g == a) ? "  (CPU==GPU)" : "");
#endif
    }
    std::printf("\nRESULT: %s\n",
                any ? "beacon DETECTED on live RF by the Sounder correlator"
                    : "no beacon found");
    return any ? 0 : 1;
  }

  if (std::strcmp(mode, "--stream") == 0 && argc > 2) {
    // The receiver.cc clientSyncBeacon loop, driven by the Houdini radio: read one
    // search window of raw cint16 from stdin, run syncSearch (find_beacon), repeat.
    const size_t window = static_cast<size_t>(std::atol(argv[2]));
    float scale = 1.0f;
    for (int i = 3; i < argc - 1; ++i)
      if (!std::strcmp(argv[i], "--scale")) scale = std::atof(argv[i + 1]);
    std::vector<int16_t> raw(window * 2);
    std::vector<std::complex<int16_t>> cap(window);
    size_t it = 0, hits = 0;
    while (std::fread(raw.data(), sizeof(int16_t), window * 2, stdin) ==
           window * 2) {
      for (size_t i = 0; i < window; ++i)
        cap[i] = std::complex<int16_t>(raw[2 * i], raw[2 * i + 1]);
      const auto t0 = std::chrono::steady_clock::now();
      const ssize_t a =
          CommsLib::find_beacon_avx(cap.data(), match, window, scale);
      const double us = std::chrono::duration<double, std::micro>(
                            std::chrono::steady_clock::now() - t0).count();
      ++it;
      if (a >= 0) ++hits;
#ifdef USE_CUDA
      const ssize_t g =
          CommsLib::find_beacon_cuda(cap.data(), match, window, scale);
      std::printf("iter %3zu  avx=%6zd  cuda=%6zd  avx %6.0f us  %s\n", it, a, g,
                  us, a >= 0 ? "*** SYNC ***" : "searching");
#else
      std::printf("iter %3zu  avx=%6zd  avx %6.0f us  %s\n", it, a, us,
                  a >= 0 ? "*** SYNC ***" : "searching");
#endif
      std::fflush(stdout);
    }
    std::fprintf(stderr, "stream: %zu windows, %zu with SYNC\n", it, hits);
    return 0;
  }

  std::fprintf(stderr,
               "usage: %s --dump-tx <file> | --detect <cap.bin> "
               "[--window N] [--scale S] | --stream <window> [--scale S]\n",
               argv[0]);
  return 2;
}
