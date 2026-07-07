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
#include <condition_variable>
#include <cstdlib>
#include <functional>
#include <mutex>
#include <thread>
#include <vector>

#include "include/comms-lib.h"

namespace {
// Thread count for correlate_mt: explicit request wins; else SOUNDER_CORR_THREADS
// env; else 1. Capped at the pool size (hardware concurrency) at dispatch.
unsigned ResolveThreads(unsigned requested) {
  if (requested > 0) return requested;
  if (const char* e = std::getenv("SOUNDER_CORR_THREADS")) {
    const int v = std::atoi(e);
    if (v > 0) return static_cast<unsigned>(v);
  }
  return 1u;
}

// Persistent fork-join pool: worker threads are created once and reused, so
// per-frame correlation pays no thread-creation cost (the per-call std::thread
// spawn it replaces made threading a net loss on the rig host). Not reentrant
// -- find_beacon runs one correlation at a time.
class ForkJoinPool {
 public:
  static ForkJoinPool& Instance() {
    static ForkJoinPool inst;
    return inst;
  }
  unsigned size() const { return n_; }

  // Split [0,total) into `parts` contiguous chunks; run body(k0,k1) on each.
  // The calling thread runs chunk 0; workers run the rest. Blocks until done.
  void Run(size_t total, unsigned parts,
           const std::function<void(size_t, size_t)>& body) {
    parts = std::min<unsigned>(parts, n_);
    if (parts <= 1) {
      body(0, total);
      return;
    }
    const size_t chunk = (total + parts - 1) / parts;
    {
      std::lock_guard<std::mutex> lk(m_);
      body_ = &body;
      remaining_ = parts - 1;
      for (unsigned p = 1; p < parts; ++p) {
        const size_t k0 = std::min(total, static_cast<size_t>(p) * chunk);
        const size_t k1 = std::min(total, k0 + chunk);
        queue_.push_back({k0, k1});
      }
    }
    cv_work_.notify_all();
    body(0, std::min(total, chunk));  // caller executes chunk 0
    std::unique_lock<std::mutex> lk(m_);
    cv_done_.wait(lk, [this] { return remaining_ == 0; });
    body_ = nullptr;
  }

 private:
  struct Range {
    size_t k0, k1;
  };

  ForkJoinPool() {
    n_ = std::thread::hardware_concurrency();
    if (n_ < 1) n_ = 1;
    for (unsigned i = 0; i + 1 < n_; ++i) {  // n_-1 workers + the caller
      workers_.emplace_back([this] { Worker(); });
    }
  }
  ~ForkJoinPool() {
    {
      std::lock_guard<std::mutex> lk(m_);
      stop_ = true;
    }
    cv_work_.notify_all();
    for (auto& t : workers_) t.join();
  }
  void Worker() {
    for (;;) {
      Range r;
      const std::function<void(size_t, size_t)>* body;
      {
        std::unique_lock<std::mutex> lk(m_);
        cv_work_.wait(lk, [this] { return stop_ || !queue_.empty(); });
        if (stop_ && queue_.empty()) return;
        r = queue_.back();
        queue_.pop_back();
        body = body_;  // read under lock (stable for the whole Run)
      }
      (*body)(r.k0, r.k1);
      std::lock_guard<std::mutex> lk(m_);
      if (--remaining_ == 0) cv_done_.notify_one();
    }
  }

  unsigned n_ = 1;
  std::vector<std::thread> workers_;
  std::vector<Range> queue_;
  const std::function<void(size_t, size_t)>* body_ = nullptr;
  size_t remaining_ = 0;
  bool stop_ = false;
  std::mutex m_;
  std::condition_variable cv_work_, cv_done_;
};
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
  if (nt > 1 && N >= 4096) {
    ForkJoinPool::Instance().Run(N, nt, work);
  } else {
    work(0, N);
  }

  for (size_t k = 0; k < N; ++k) {
    out[k] = std::complex<float>(outr[k], outi[k]);
  }
  return out;
}
