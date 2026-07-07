/**
 * @file comms-lib-portable.cc
 * @brief Portable (Intel + ARM) optimized comms kernels.
 *
 * These replace the hand-written AVX kernels in comms-lib-avx.cc with clean,
 * auto-vectorizable C++ that the compiler lowers to AVX2/AVX-512 on x86 and
 * NEON/SVE on aarch64 (build with -O3 -march=native / -mcpu=native). There are
 * no architecture #ifdefs and no SIMD intrinsics here, so one source is optimal
 * on both -- and, unlike comms-lib-avx.cc (guarded by #if defined(__x86_64__)),
 * this compiles on the DGX Spark.
 */
#include <algorithm>
#include <complex>
#include <cstdlib>
#include <thread>
#include <vector>

#include "include/comms-lib.h"

namespace {
// Thread count for correlate_mt: explicit request wins; else SOUNDER_CORR_THREADS
// env; else 1 (single-threaded until a persistent pool replaces per-call spawn).
unsigned ResolveThreads(unsigned requested) {
  if (requested > 0) return requested;
  if (const char* e = std::getenv("SOUNDER_CORR_THREADS")) {
    const int v = std::atoi(e);
    if (v > 0) return static_cast<unsigned>(v);
  }
  return 1u;
}
}  // namespace

// Matched-filter cross-correlation, portable + multi-threaded, equivalent to
// CommsLib::correlate_avx:
//   out[k] = sum_{j=0}^{M-1} in[k+j] * conj(g[j]),   in = [M-1 zeros, f]
// with out sized f.size()+g.size()-1 (tail beyond position N left zero).
//
// Vectorization strategy: deinterleave to struct-of-arrays (separate re/im),
// then loop taps-outer / positions-inner so the inner loop is an independent
// FMA stream over contiguous memory. That vectorizes WITHOUT -ffast-math (no
// float-reduction reassociation needed). Output positions split across threads
// write disjoint ranges, so no synchronization is required.
std::vector<std::complex<float>> CommsLib::correlate_mt(
    const std::vector<std::complex<float>>& f,
    const std::vector<std::complex<float>>& g, unsigned num_threads) {
  const size_t N = f.size();
  const size_t M = g.size();
  std::vector<std::complex<float>> out(N + M - 1, std::complex<float>(0, 0));
  if (N == 0 || M == 0) return out;

  // Beacon taps in SoA.
  std::vector<float> gr(M), gi(M);
  for (size_t j = 0; j < M; ++j) {
    gr[j] = g[j].real();
    gi[j] = g[j].imag();
  }
  // in = [M-1 zeros, f], deinterleaved to SoA (length N+M-1).
  const size_t in_len = N + M - 1;
  std::vector<float> inr(in_len, 0.0f);
  std::vector<float> ini(in_len, 0.0f);
  for (size_t t = 0; t < N; ++t) {
    inr[t + (M - 1)] = f[t].real();
    ini[t + (M - 1)] = f[t].imag();
  }

  std::vector<float> outr(N, 0.0f);
  std::vector<float> outi(N, 0.0f);

  // Compute out[k0..k1). Cache tiling: sweep all M taps over a block of output
  // positions whose working set (outr/outi + the inr slice) stays L1-resident,
  // before advancing. Without it the taps-outer loop rereads the whole input M
  // times from memory and large windows go memory-bandwidth bound. The inner
  // k-loop stays the vectorized FMA stream.
  constexpr size_t kBlock = 1024;
  auto work = [&](size_t k0, size_t k1) {
    for (size_t kb = k0; kb < k1; kb += kBlock) {
      const size_t ke = std::min(kb + kBlock, k1);
      for (size_t j = 0; j < M; ++j) {
        const float grj = gr[j];
        const float gij = gi[j];
        for (size_t k = kb; k < ke; ++k) {
          const float xr = inr[k + j];
          const float xi = ini[k + j];
          outr[k] += xr * grj + xi * gij;  // Re{in * conj(g)}
          outi[k] += xi * grj - xr * gij;  // Im{in * conj(g)}
        }
      }
    }
  };

  const unsigned nt = ResolveThreads(num_threads);
  if (nt > 1 && N >= 8192) {  // amortize per-call spawn only on real work
    std::vector<std::thread> pool;
    pool.reserve(nt);
    const size_t chunk = (N + nt - 1) / nt;
    for (unsigned t = 0; t < nt; ++t) {
      const size_t k0 = static_cast<size_t>(t) * chunk;
      const size_t k1 = std::min(N, k0 + chunk);
      if (k0 >= k1) break;
      pool.emplace_back(work, k0, k1);
    }
    for (auto& th : pool) th.join();
  } else {
    work(0, N);
  }

  for (size_t k = 0; k < N; ++k) {
    out[k] = std::complex<float>(outr[k], outi[k]);
  }
  return out;
}
