/**
 * @file bench-correlator-rate.cc
 * @brief Throughput benchmark for the UE-side beacon correlator
 *        (CommsLib::find_beacon_avx), to find the max sample rate it can
 *        sustain in real time.
 *
 * Real-time bound: the correlator must process a check_window of samples faster
 * than that window's air-time, i.e. corr_time(window) < window / sample_rate.
 * So the max sustainable sample rate is  window / time_per_call.
 *
 * Self-contained by design: it synthesizes a representative beacon-in-noise
 * buffer and links only comms-lib-avx.cc + utils.cc (the correlator path uses
 * no FFT), so it builds without muFFT. Correlator throughput is a function of
 * the window and the sequence length, not the beacon's sample values, so a
 * stand-in +/-1 sequence with the real Gold beacon's dimensions (kGoldReps x
 * kSeqLen) is representative; a high-SNR embed also lets us assert detection
 * correctness at each window size.
 *
 * It calls the same CommsLib::find_beacon_avx the receiver uses, so it is
 * architecture-agnostic: rerun it after the ARM/NEON port to read the max
 * sample rate on the DGX Spark.
 */
#include <sys/types.h>  // ssize_t

#include <algorithm>
#include <chrono>
#include <cmath>
#include <complex>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <random>
#include <string>
#include <vector>

#include "comms-lib.h"

namespace {
constexpr int kSeqLen = 128;   // Gold-code rep length == matched-filter length
constexpr int kGoldReps = 2;   // beacon is the sequence repeated twice
constexpr float kShortMax = 32768.0f;
// The +/-1 stand-in sequence has non-ideal autocorrelation (unlike the real
// Gold code), so the detected peak can sit a few samples off the true beacon
// start. A modest tolerance keeps this a meaningful "locked onto the beacon"
// check without demanding ideal-code exactness (throughput is unaffected).
constexpr ssize_t kDetectTol = 16;

// Deterministic +/-1 complex stand-in for one Gold-IFFT beacon rep.
std::vector<std::complex<float>> MakeMatch(int seq_len) {
  std::mt19937 rng(0x5eedU);
  std::uniform_int_distribution<int> bit(0, 1);
  std::vector<std::complex<float>> m(seq_len);
  for (int i = 0; i < seq_len; ++i) {
    m[i] = std::complex<float>(bit(rng) ? 0.5f : -0.5f,
                               bit(rng) ? 0.5f : -0.5f);
  }
  return m;
}

// A window of noise with kGoldReps reps of the beacon embedded at beacon_pos,
// returned as cint16 (the format the radio delivers to the correlator).
std::vector<std::complex<int16_t>> MakeBuffer(
    const std::vector<std::complex<float>>& match, size_t window,
    size_t beacon_pos) {
  std::mt19937 rng(0xC0FFEEU);
  std::normal_distribution<float> noise(0.0f, 0.002f);
  std::vector<std::complex<float>> buf(window);
  for (size_t i = 0; i < window; ++i) {
    buf[i] = std::complex<float>(noise(rng), noise(rng));
  }
  for (int r = 0; r < kGoldReps; ++r) {
    for (int i = 0; i < static_cast<int>(match.size()); ++i) {
      const size_t idx = beacon_pos + static_cast<size_t>(r) * match.size() + i;
      if (idx < window) buf[idx] += match[i];
    }
  }
  auto clamp16 = [](float v) {
    return static_cast<int16_t>(
        std::lround(std::max(-1.0f, std::min(1.0f, v)) * kShortMax));
  };
  std::vector<std::complex<int16_t>> ci(window);
  for (size_t i = 0; i < window; ++i) {
    ci[i] = std::complex<int16_t>(clamp16(buf[i].real()), clamp16(buf[i].imag()));
  }
  return ci;
}
}  // namespace

int main(int argc, char** argv) {
  const size_t beacon_pos = 501;
  const int iters = (argc > 1) ? std::atoi(argv[1]) : 200;
  const int warmup = 20;
  const std::vector<size_t> windows = {1024,  2048,  4096,  8192,
                                       16384, 32768, 65536};
  const std::vector<double> rates = {30.72e6, 61.44e6, 122.88e6};  // min..max

  const auto match = MakeMatch(kSeqLen);

  printf("UE beacon correlator (CommsLib::find_beacon_avx) throughput\n");
  printf(
      "seqLen=%d reps=%d iters=%d   Houdini rates: 30.72 / 61.44 / 122.88 "
      "MSPS\n\n",
      kSeqLen, kGoldReps, iters);
  printf("%9s %12s %10s   %-14s %s\n", "window", "usec/call", "MSPS",
         "sustains", "detect");

  for (size_t w : windows) {
    const auto buf = MakeBuffer(match, w, beacon_pos);

    // Correctness: the detected sync index should land on the embedded beacon.
    const ssize_t idx = CommsLib::find_beacon_avx(buf.data(), match, w, 1.0f);
    const ssize_t sync = idx - 2 * kSeqLen + 1;
    const bool detected =
        std::llabs(static_cast<long long>(sync) -
                   static_cast<long long>(beacon_pos)) <= kDetectTol;

    for (int i = 0; i < warmup; ++i) {
      (void)CommsLib::find_beacon_avx(buf.data(), match, w, 1.0f);
    }
    const auto t0 = std::chrono::steady_clock::now();
    for (int i = 0; i < iters; ++i) {
      (void)CommsLib::find_beacon_avx(buf.data(), match, w, 1.0f);
    }
    const auto t1 = std::chrono::steady_clock::now();

    const double per_call =
        std::chrono::duration<double>(t1 - t0).count() / iters;
    const double throughput = w / per_call;  // samples/sec

    std::string sus;
    for (double r : rates) sus += (throughput >= r) ? " Y " : " . ";

    printf("%9zu %12.2f %10.1f   %-14s %s\n", w, per_call * 1e6,
           throughput / 1e6, sus.c_str(), detected ? "OK" : "MISS");
    if (!detected) {
      printf("            (detect MISS: idx=%zd sync=%zd expected~%zu)\n", idx,
             sync, beacon_pos);
    }
  }
  return 0;
}
