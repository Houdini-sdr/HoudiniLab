/**
 * @file gpu-correlator-bench.cu
 * @brief GPU prototype of the UE beacon correlator for the DGX Spark (GB10).
 *
 * Phase-1 spike: single channel, high sample rate. Runs the same detection
 * pipeline as the CPU find_beacon (matched filter -> auto-corr -> |.|^2 ->
 * trailing-window threshold -> first peak) on the Blackwell GPU, using UNIFIED
 * (managed) memory so there are no CPU<->GPU copies -- the point of GB10's
 * coherent Grace+Blackwell memory. It reports sustained throughput AND per-call
 * latency vs Houdini's 30.72 / 61.44 / 122.88 MSPS, and checks the detected peak
 * lands on the embedded beacon. Same synthetic 2x128 beacon-in-noise as
 * correlator-rate-bench, so the two are directly comparable.
 *
 * Build on the DGX Spark:
 *   nvcc -O3 -std=c++17 -arch=native gpu-correlator-bench.cu -o gpu-corr-bench
 *   (if -arch=native isn't supported by your nvcc, use the GB10 arch, e.g.
 *    -arch=sm_121)
 * Run:
 *   ./gpu-corr-bench 2000
 *
 * NOTE: written on a box without CUDA and NOT compiled here -- expect a
 * possible small fixup on first build; send me any nvcc error.
 */
#include <cuda_runtime.h>

#include <algorithm>
#include <chrono>
#include <climits>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <random>
#include <string>
#include <vector>

#define CUDA_CHECK(call)                                              \
  do {                                                                \
    cudaError_t err__ = (call);                                       \
    if (err__ != cudaSuccess) {                                       \
      fprintf(stderr, "CUDA error %s:%d: %s\n", __FILE__, __LINE__,   \
              cudaGetErrorString(err__));                             \
      std::exit(1);                                                   \
    }                                                                 \
  } while (0)

static constexpr int kSeqLen = 128;   // Gold-code rep length == filter length
static constexpr int kGoldReps = 2;   // beacon = sequence repeated twice
static constexpr int kDetectTol = 16;

__constant__ float2 c_g[kSeqLen];      // beacon taps (conjugated in the kernel)

// short2 (cint16) -> float2, /32767 -- the radio delivers cint16.
__global__ void ConvertKernel(const short2* __restrict__ ci, int n,
                              float2* __restrict__ fc) {
  const int k = blockIdx.x * blockDim.x + threadIdx.x;
  if (k >= n) return;
  fc[k] = make_float2(ci[k].x * (1.0f / 32767.0f), ci[k].y * (1.0f / 32767.0f));
}

// Matched filter: corr[k] = sum_{j=0}^{M-1} in[k+j-(M-1)] * conj(g[j]), with
// out-of-range samples treated as 0 (== the CPU correlate's M-1 leading zeros).
__global__ void CorrelateKernel(const float2* __restrict__ fc, int n, int m,
                               float2* __restrict__ corr) {
  const int k = blockIdx.x * blockDim.x + threadIdx.x;
  if (k >= n) return;
  float accr = 0.0f, acci = 0.0f;
  for (int j = 0; j < m; ++j) {
    const int fi = k + j - (m - 1);
    if (fi >= 0 && fi < n) {
      const float2 x = fc[fi];
      const float2 g = c_g[j];
      accr += x.x * g.x + x.y * g.y;   // Re{x * conj(g)}
      acci += x.y * g.x - x.x * g.y;   // Im{x * conj(g)}
    }
  }
  corr[k] = make_float2(accr, acci);
}

// peak_metric[k] = |corr[k]*conj(corr[k-M])|^2 ; thresh[k] = sum |corr|^2 over
// [k-M, k-1] ; flag[k] = (scale*peak_metric > thresh).
__global__ void DetectKernel(const float2* __restrict__ corr, int n, int m,
                            float scale, int* __restrict__ flag) {
  const int k = blockIdx.x * blockDim.x + threadIdx.x;
  if (k >= n) return;
  float pm = 0.0f;
  if (k >= m) {
    const float2 a = corr[k];
    const float2 b = corr[k - m];
    const float re = a.x * b.x + a.y * b.y;   // Re{a*conj(b)}
    const float im = a.y * b.x - a.x * b.y;   // Im{a*conj(b)}
    pm = re * re + im * im;                    // |a*conj(b)|^2
  }
  float th = 0.0f;
  for (int i = k - m; i < k; ++i) {
    if (i >= 0) {
      const float2 c = corr[i];
      th += c.x * c.x + c.y * c.y;
    }
  }
  flag[k] = (scale * pm > th) ? 1 : 0;
}

__global__ void FirstPeakKernel(const int* __restrict__ flag, int n,
                               int* __restrict__ out_min) {
  const int k = blockIdx.x * blockDim.x + threadIdx.x;
  if (k < n && flag[k]) atomicMin(out_min, k);
}

int main(int argc, char** argv) {
  const int iters = (argc > 1) ? std::atoi(argv[1]) : 2000;
  const int m = kSeqLen;
  const size_t beacon_pos = 501;
  const std::vector<size_t> windows = {1024,  2048,  4096,  8192,
                                       16384, 32768, 65536};
  const double rates[3] = {30.72e6, 61.44e6, 122.88e6};

  // Beacon taps (deterministic +/-1, same as the CPU bench) -> constant memory.
  float2 h_g[kSeqLen];
  {
    std::mt19937 rng(0x5eedU);
    std::uniform_int_distribution<int> bit(0, 1);
    for (int i = 0; i < m; ++i)
      h_g[i] = make_float2(bit(rng) ? 0.5f : -0.5f, bit(rng) ? 0.5f : -0.5f);
  }
  CUDA_CHECK(cudaMemcpyToSymbol(c_g, h_g, sizeof(h_g)));

  cudaDeviceProp prop;
  CUDA_CHECK(cudaGetDeviceProperties(&prop, 0));
  printf("GPU beacon correlator prototype on: %s\n", prop.name);
  printf(
      "seqLen=%d reps=%d iters=%d   unified memory; rates 30.72/61.44/122.88 "
      "MSPS\n\n",
      m, kGoldReps, iters);
  printf("%9s %11s %10s %9s  %-12s %s\n", "window", "us/call", "MSPS", "lat_us",
         "sustains", "detect");

  const int kTPB = 256;
  for (size_t n : windows) {
    short2* d_ci = nullptr;
    float2* d_fc = nullptr;
    float2* d_corr = nullptr;
    int* d_flag = nullptr;
    int* d_peak = nullptr;
    CUDA_CHECK(cudaMallocManaged(&d_ci, n * sizeof(short2)));
    CUDA_CHECK(cudaMallocManaged(&d_fc, n * sizeof(float2)));
    CUDA_CHECK(cudaMallocManaged(&d_corr, n * sizeof(float2)));
    CUDA_CHECK(cudaMallocManaged(&d_flag, n * sizeof(int)));
    CUDA_CHECK(cudaMallocManaged(&d_peak, sizeof(int)));

    // beacon-in-noise (cint16) written straight into managed memory
    {
      std::mt19937 rng(0xC0FFEEU);
      std::normal_distribution<float> noise(0.0f, 0.002f);
      std::vector<float> re(n), im(n);
      for (size_t i = 0; i < n; ++i) {
        re[i] = noise(rng);
        im[i] = noise(rng);
      }
      for (int r = 0; r < kGoldReps; ++r)
        for (int i = 0; i < m; ++i) {
          const size_t idx = beacon_pos + static_cast<size_t>(r) * m + i;
          if (idx < n) {
            re[idx] += h_g[i].x;
            im[idx] += h_g[i].y;
          }
        }
      for (size_t i = 0; i < n; ++i) {
        const float r0 = std::min(std::max(re[i], -1.0f), 1.0f);
        const float i0 = std::min(std::max(im[i], -1.0f), 1.0f);
        d_ci[i] = make_short2(static_cast<short>(std::lround(r0 * 32767.0f)),
                              static_cast<short>(std::lround(i0 * 32767.0f)));
      }
    }

    const int blocks = static_cast<int>((n + kTPB - 1) / kTPB);
    auto pipeline = [&]() {
      ConvertKernel<<<blocks, kTPB>>>(d_ci, (int)n, d_fc);
      CorrelateKernel<<<blocks, kTPB>>>(d_fc, (int)n, m, d_corr);
      DetectKernel<<<blocks, kTPB>>>(d_corr, (int)n, m, 1.0f, d_flag);
      FirstPeakKernel<<<blocks, kTPB>>>(d_flag, (int)n, d_peak);
    };

    for (int i = 0; i < 30; ++i) pipeline();  // warmup
    CUDA_CHECK(cudaDeviceSynchronize());

    // correctness: does the detected peak land on the embedded beacon?
    *d_peak = INT_MAX;
    pipeline();
    CUDA_CHECK(cudaDeviceSynchronize());
    const int peak = (*d_peak == INT_MAX) ? -1 : *d_peak;
    const long sync = static_cast<long>(peak) - 2L * m + 1;
    const bool detected =
        std::labs(sync - static_cast<long>(beacon_pos)) <= kDetectTol;

    // per-call latency: one synced call (4 launches + execute + sync)
    CUDA_CHECK(cudaDeviceSynchronize());
    auto l0 = std::chrono::steady_clock::now();
    pipeline();
    CUDA_CHECK(cudaDeviceSynchronize());
    auto l1 = std::chrono::steady_clock::now();
    const double lat = std::chrono::duration<double, std::micro>(l1 - l0).count();

    // sustained throughput: streamed launches, one sync at the end
    auto s0 = std::chrono::steady_clock::now();
    for (int i = 0; i < iters; ++i) pipeline();
    CUDA_CHECK(cudaDeviceSynchronize());
    auto s1 = std::chrono::steady_clock::now();
    const double per = std::chrono::duration<double>(s1 - s0).count() / iters;
    const double msps = (n / per) / 1e6;

    std::string sus;
    for (double rt : rates) sus += (n / per >= rt) ? " Y " : " . ";
    printf("%9zu %11.2f %10.1f %9.1f  %-12s %s\n", n, per * 1e6, msps, lat,
           sus.c_str(), detected ? "OK" : "MISS");

    CUDA_CHECK(cudaFree(d_ci));
    CUDA_CHECK(cudaFree(d_fc));
    CUDA_CHECK(cudaFree(d_corr));
    CUDA_CHECK(cudaFree(d_flag));
    CUDA_CHECK(cudaFree(d_peak));
  }
  return 0;
}
