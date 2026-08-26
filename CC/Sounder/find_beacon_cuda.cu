/**
 * @file find_beacon_cuda.cu
 * @brief GPU beacon detector for the DGX Spark (GB10) -- production path for
 *        Receiver::syncSearch when built with -DUSE_CUDA (CMake HOUDINI_USE_CUDA).
 *
 * Mirrors CommsLib::find_beacon_avx (the portable CPU version, which stays the
 * default and the numeric reference): matched filter -> auto-corr -> |.|^2 ->
 * trailing-window threshold -> first peak, returning the same peak index. Runs
 * on the GPU over UNIFIED (managed) memory, so the copies below are coherent and
 * cheap on GB10 (no PCIe transfer). A persistent per-thread context holds the
 * stream + buffers so there is no per-frame allocation.
 *
 * Prototype-grade (validated against the CPU path via the same synthetic beacon
 * in gpu-correlator-bench.cu). Follow-ups when needed: fuse the 4 kernels + a
 * CUDA graph (cut launch latency), have the radio DMA straight into managed
 * memory (drop the input memcpy), batch channels (phase 2).
 */
#include <cuda_runtime.h>

#include <climits>
#include <complex>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <sys/types.h>
#include <vector>

#include "include/comms-lib.h"

namespace {

#define CUDA_ABORT(call)                                                     \
  do {                                                                       \
    cudaError_t err__ = (call);                                              \
    if (err__ != cudaSuccess) {                                             \
      fprintf(stderr, "find_beacon_cuda: CUDA error %s:%d: %s\n", __FILE__,  \
              __LINE__, cudaGetErrorString(err__));                          \
      std::abort();                                                          \
    }                                                                        \
  } while (0)

__global__ void ConvertKernel(const short2* __restrict__ ci, int n,
                              float2* __restrict__ fc) {
  const int k = blockIdx.x * blockDim.x + threadIdx.x;
  if (k >= n) return;
  fc[k] = make_float2(ci[k].x * (1.0f / 32767.0f), ci[k].y * (1.0f / 32767.0f));
}

// corr[k] = sum_{j} in[k+j-(M-1)] * conj(g[j]); out-of-range samples = 0.
__global__ void CorrelateKernel(const float2* __restrict__ fc, int n,
                               const float2* __restrict__ g, int m,
                               float2* __restrict__ corr) {
  const int k = blockIdx.x * blockDim.x + threadIdx.x;
  if (k >= n) return;
  float accr = 0.0f, acci = 0.0f;
  for (int j = 0; j < m; ++j) {
    const int fi = k + j - (m - 1);
    if (fi >= 0 && fi < n) {
      const float2 x = fc[fi];
      const float2 gg = g[j];
      accr += x.x * gg.x + x.y * gg.y;   // Re{x * conj(g)}
      acci += x.y * gg.x - x.x * gg.y;   // Im{x * conj(g)}
    }
  }
  corr[k] = make_float2(accr, acci);
}

__global__ void DetectKernel(const float2* __restrict__ corr, int n, int m,
                            float scale, int* __restrict__ flag) {
  const int k = blockIdx.x * blockDim.x + threadIdx.x;
  if (k >= n) return;
  float pm = 0.0f;
  if (k >= m) {
    const float2 a = corr[k];
    const float2 b = corr[k - m];
    const float re = a.x * b.x + a.y * b.y;
    const float im = a.y * b.x - a.x * b.y;
    pm = re * re + im * im;   // |corr[k] * conj(corr[k-M])|^2
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

// Persistent per-thread GPU context: stream + managed buffers, reused across
// frames (no per-call alloc). Grows on demand. One per RX thread => per-thread
// stream, no cross-thread contention.
struct CudaCtx {
  cudaStream_t stream = nullptr;
  short2* d_ci = nullptr;
  float2* d_fc = nullptr;
  float2* d_corr = nullptr;
  float2* d_g = nullptr;
  int* d_flag = nullptr;
  int* d_peak = nullptr;
  size_t cap = 0;   // sample-window capacity
  int mcap = 0;     // beacon-length capacity

  void ensure(size_t n, int m) {
    if (stream == nullptr) CUDA_ABORT(cudaStreamCreate(&stream));
    if (d_peak == nullptr) CUDA_ABORT(cudaMallocManaged(&d_peak, sizeof(int)));
    if (n > cap) {
      if (d_ci) {
        cudaFree(d_ci);
        cudaFree(d_fc);
        cudaFree(d_corr);
        cudaFree(d_flag);
      }
      CUDA_ABORT(cudaMallocManaged(&d_ci, n * sizeof(short2)));
      CUDA_ABORT(cudaMallocManaged(&d_fc, n * sizeof(float2)));
      CUDA_ABORT(cudaMallocManaged(&d_corr, n * sizeof(float2)));
      CUDA_ABORT(cudaMallocManaged(&d_flag, n * sizeof(int)));
      cap = n;
    }
    if (m > mcap) {
      if (d_g) cudaFree(d_g);
      CUDA_ABORT(cudaMallocManaged(&d_g, m * sizeof(float2)));
      mcap = m;
    }
  }

  ~CudaCtx() {  // best-effort; ignore errors during teardown
    if (d_ci) cudaFree(d_ci);
    if (d_fc) cudaFree(d_fc);
    if (d_corr) cudaFree(d_corr);
    if (d_g) cudaFree(d_g);
    if (d_flag) cudaFree(d_flag);
    if (d_peak) cudaFree(d_peak);
    if (stream) cudaStreamDestroy(stream);
  }
};

thread_local CudaCtx g_ctx;

}  // namespace

ssize_t CommsLib::find_beacon_cuda(
    const std::complex<int16_t>* raw_samples,
    const std::vector<std::complex<float>>& match_samples, size_t check_window,
    float corr_scale) {
  const int n = static_cast<int>(check_window);
  const int m = static_cast<int>(match_samples.size());
  if (n <= 0 || m <= 0) return -1;

  g_ctx.ensure(check_window, m);

  // Copy beacon (tiny) + input into unified memory -- coherent/cheap on GB10.
  // std::complex<float>==float2 and std::complex<int16_t>==short2 by layout.
  std::memcpy(g_ctx.d_g, match_samples.data(),
              static_cast<size_t>(m) * sizeof(float2));
  std::memcpy(g_ctx.d_ci, raw_samples,
              static_cast<size_t>(n) * sizeof(short2));
  *g_ctx.d_peak = INT_MAX;

  const int tpb = 256;
  const int blocks = (n + tpb - 1) / tpb;
  ConvertKernel<<<blocks, tpb, 0, g_ctx.stream>>>(g_ctx.d_ci, n, g_ctx.d_fc);
  CorrelateKernel<<<blocks, tpb, 0, g_ctx.stream>>>(g_ctx.d_fc, n, g_ctx.d_g, m,
                                                    g_ctx.d_corr);
  DetectKernel<<<blocks, tpb, 0, g_ctx.stream>>>(g_ctx.d_corr, n, m, corr_scale,
                                                 g_ctx.d_flag);
  FirstPeakKernel<<<blocks, tpb, 0, g_ctx.stream>>>(g_ctx.d_flag, n,
                                                    g_ctx.d_peak);
  CUDA_ABORT(cudaStreamSynchronize(g_ctx.stream));

  return (*g_ctx.d_peak == INT_MAX) ? ssize_t(-1) : ssize_t(*g_ctx.d_peak);
}
