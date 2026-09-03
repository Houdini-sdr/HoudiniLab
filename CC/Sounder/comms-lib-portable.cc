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
#include <cassert>
#include <chrono>
#include <complex>
#include <condition_variable>
#include <cstdio>
#include <cstdlib>
#include <functional>
#include <iostream>
#include <mutex>
#include <queue>
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

// ---------------------------------------------------------------------------
// Beacon detection (find_beacon), moved here from comms-lib-avx.cc so the whole
// UE detection path is portable and builds on aarch64 (the DGX Spark). It uses
// the portable matched filter (correlate_mt) plus the trivial portable kernels
// below, replacing the x86-only abs2_avx / auto_corr_mult_avx.
// ---------------------------------------------------------------------------
namespace {
// |z|^2 per element (auto-vectorizes). == abs2_avx(vector<complex<float>>).
std::vector<float> Abs2(const std::vector<std::complex<float>>& f) {
  std::vector<float> out(f.size());
  for (size_t i = 0; i < f.size(); ++i) {
    const float re = f[i].real();
    const float im = f[i].imag();
    out[i] = re * re + im * im;
  }
  return out;
}

// out[i] = f[i] * conj(f[i-dly]), 0 for i < dly. Reinforces the double peak of a
// 2-rep beacon. == auto_corr_mult_avx(f, dly, conj=true).
std::vector<std::complex<float>> AutoCorrMult(
    const std::vector<std::complex<float>>& f, int dly) {
  std::vector<std::complex<float>> out(f.size(), std::complex<float>(0, 0));
  for (size_t i = static_cast<size_t>(dly); i < f.size(); ++i) {
    out[i] = f[i] * std::conj(f[i - dly]);
  }
  return out;
}

// Trailing box-window sum: out[i] = sum(f[i-window .. i-1]), f[<0]=0. O(n)
// running sum, == correlate_avx_s(f, ones(window)) but ~window x cheaper.
std::vector<float> TrailingWindowSum(const std::vector<float>& f,
                                     size_t window) {
  std::vector<float> out(f.size());
  double run = 0.0;  // double accumulator: no drift over long windows
  for (size_t i = 0; i < f.size(); ++i) {
    if (i >= 1) run += f[i - 1];
    if (i >= window + 1) run -= f[i - 1 - window];
    out[i] = static_cast<float>(run);
  }
  return out;
}
}  // namespace

// Correlate against the 2-rep Gold beacon, reinforce the double peak, threshold
// against trailing local energy, return a peak index (or -1).
//
// `pick` selects WHICH crossing is returned, and the choice matters
// because the frame anchor is derived from it.
//
// The threshold admits every index whose peak-to-local-energy ratio clears
// 1/corr_scale, and on a real link several do: measured on the bench, a detecting
// call had a median of 4 crossings and up to 12. Two DIFFERENT things generate those
// crossings and they need opposite treatment:
//
//   - Sidelobes of one beacon, within a sequence length of its true peak. Returning
//     the earliest of these (the historical behaviour) returned a sidelobe rather
//     than the peak on 58 of 62 detections. Here the STRONGEST is right.
//   - Separate beacon COPIES. The beacon strobe covers slots 0..14 (the TDD symbol
//     is 15 slots), so a 9.7k-sample acquisition window holds two or three copies
//     4096 samples apart. Between copies the strongest is arbitrary -- whichever is
//     momentarily louder wins -- while the earliest is at least repeatable across
//     restarts, which matters because receiver.cc anchors the pilot reference ONCE
//     and never re-anchors. NOT VERIFIED to affect the link: measured across 6
//     restarts, the pilot slot was captured every time (ADC peak 557..955 counts,
//     repeat quality 0.997..1.000) and the copy index did not track whether the
//     run's constellation was good or bad. Repeatability is the argument here, not
//     a demonstrated fault.
//
// So: take the earliest crossing, then refine within one sequence length of it. That
// is deterministic across runs and still lands on the peak rather than its skirt.
// How far back kFirstPath looks, and how strong an earlier path must be.
//
// THE WINDOW MUST STAY INSIDE THE PREAMBLE PLATEAU. The shipped beacon's STS
// field manufactures crossings ~365 samples before the true peak; a back window
// anywhere near that reintroduces the fault kTargetedArgmax removes. Half a
// sequence length (64 samples at L=128) is 0.52 us, which covers indoor excess
// delay (RMS spread 20-100 ns) with margin and is 5x inside the plateau. An
// outdoor channel with 1-2 us of spread needs a longer window AND a preamble
// whose plateau is further away than that; do not raise one without the other.
static int firstPathWindow(int seq_len) {
  // Read ONCE. Re-reading getenv per call would let the window change mid-run,
  // making the residual distribution a mixture of two populations -- the same
  // argument that makes the pick rule a read-once constant in receiver.cc, not
  // honoured here until now.
  static const int cached = [] {
    const char* e = std::getenv("HOUDINI_FIRST_PATH_WIN");
    if (e == nullptr) return -1;
    char* end = nullptr;
    const long v = std::strtol(e, &end, 10);
    if (end != e && *end == '\0' && v >= 0 && v < 4096) {
      return static_cast<int>(v);
    }
    // atoi("64x") returns 0, which silently DISABLED the back-search.
    std::fprintf(stderr,
                 "[find_beacon] HOUDINI_FIRST_PATH_WIN=\"%s\" is not an integer "
                 "in [0, 4096) -- using the default\n", e);
    return -1;
  }();
  if (cached >= 0) return std::min(cached, seq_len * 2);
  return seq_len / 2;
}
// How far below the peak an earlier path may be and still be taken as the first
// path, in dB of PATH POWER.
//
// THE CONVERSION IS THE WHOLE POINT AND I GOT IT WRONG FIRST TIME. The ranking
// statistic is |gc[i]|^2 * |gc[i-L]|^2, which is 4th order in path AMPLITUDE,
// so a path whose POWER is x times the peak's scores x^2 -- not x. Setting the
// floor to 0.25 "for 6 dB" therefore demanded a path only 1.5 dB down, and the
// back-search never fired on any multipath channel: measured +7, +23 and +39
// samples of bias, identical to plain argmax, which is what a rule that never
// fires looks like.
static double firstPathDb() {
  double db = -9.0;
  if (const char* e = std::getenv("HOUDINI_FIRST_PATH_DB")) {
    char* end = nullptr;
    const double v = std::strtod(e, &end);
    if (end != e && *end == '\0' && std::isfinite(v) && v <= 0.0 && v >= -30.0) {
      db = v;
    } else {
      // FAIL LOUD. strtod("abc") returns 0.0, which passed the old range test
      // and set the floor to 1.0 -- silently disabling the back-search in the
      // rule this knob exists to tune.
      std::fprintf(stderr,
                   "[find_beacon] HOUDINI_FIRST_PATH_DB=\"%s\" is not a number "
                   "in [-30, 0] -- using %g dB\n", e, db);
    }
  }
  return db;
}
static const double kFirstPathDb = firstPathDb();

// THE FLOOR DEPENDS ON THE STATISTIC'S ORDER, AND THE FIRST VERSION SQUARED IT
// UNCONDITIONALLY. kPowerRatio, kNormalized and kNormalizedXCorr are all 4th
// order in path AMPLITUDE, so a path at x times the peak's POWER scores x^2 and
// the floor is 10^(db/10) squared. kXCorrNoLag is |gc|^2/(E*E_rep) -- 2nd order
// in amplitude, FIRST order in power -- so squaring makes a nominal -9 dB floor
// an actual -18 dB one, and a sidelobe 18 dB down gets promoted to "first path".
// Measured: nolag+first-path read -190 samples on a SINGLE-PATH channel with the
// squared floor and -129 with the corrected one.
static double firstPathFloorFrac(CommsLib::BeaconThresh form, double db) {
  const double p = std::pow(10.0, db / 10.0);
  return form == CommsLib::BeaconThresh::kXCorrNoLag ? p : p * p;
}
// -9 dB is close to the margin: bisected offline, the hardest gated multipath
// case (weak direct, echo +40, stronger) flips from correct to +39 between
// -8.8 dB and -8.0 dB. So the shipped default clears it by under 1 dB, which is
// thin -- an earlier comment here claimed it "admits a direct path well under
// half the echo's power" and that was wishful. Widen with HOUDINI_FIRST_PATH_DB
// if a channel needs it, and re-run beacon_geometry_test when you do.

int CommsLib::find_beacon_avx(
    const std::vector<std::complex<float>>& raw_samples,
    const std::vector<std::complex<float>>& match_samples, float corr_scale,
    BeaconPick pick, BeaconThresh thresh_form) {
  // The pre-SyncConfig entry: first-path knobs from the environment (read
  // once) with the historical defaults.
  return find_beacon_avx(raw_samples, match_samples, corr_scale, pick,
                         thresh_form,
                         firstPathWindow(static_cast<int>(match_samples.size())),
                         kFirstPathDb);
}

int CommsLib::find_beacon_avx(
    const std::vector<std::complex<float>>& raw_samples,
    const std::vector<std::complex<float>>& match_samples, float corr_scale,
    BeaconPick pick, BeaconThresh thresh_form, int first_path_window,
    double first_path_db) {
  const int seqLen = static_cast<int>(match_samples.size());
#ifdef TEST_BENCH
  const auto t0 = std::chrono::steady_clock::now();
#endif
  const std::vector<std::complex<float>> gold_corr =
      CommsLib::correlate_mt(raw_samples, match_samples);
#ifdef TEST_BENCH
  const auto t1 = std::chrono::steady_clock::now();
#endif
  const std::vector<std::complex<float>> gold_auto_corr =
      AutoCorrMult(gold_corr, seqLen);
  const std::vector<float> peak_metric = Abs2(gold_auto_corr);
#ifdef TEST_BENCH
  const auto t2 = std::chrono::steady_clock::now();
#endif
  const std::vector<float> corr_abs = Abs2(gold_corr);
  const std::vector<float> thresh = TrailingWindowSum(corr_abs, seqLen);
#ifdef TEST_BENCH
  const auto t3 = std::chrono::steady_clock::now();
#endif
  assert(peak_metric.size() == thresh.size());
  // The decision statistic. kNormalized divides by the energy term SQUARED,
  // which makes it dimensionless -- see BeaconThresh. Computed in double
  // because thresh^2 underflows a float on a quiet window (thresh runs down to
  // ~1e-20 on this bench, and 1e-40 is a denormal), and an underflowed
  // denominator would admit every index instead of none.
  const bool normalized = thresh_form == BeaconThresh::kNormalized;
  const bool nolag = thresh_form == BeaconThresh::kXCorrNoLag;
  const bool xcorr = thresh_form == BeaconThresh::kNormalizedXCorr || nolag;
  // For kNormalizedXCorr: trailing energy of the RAW samples, and the replica's
  // energy. |gc[i]|^2 / (E_raw[i] * E_rep) is the normalised cross-correlation,
  // 2nd order over 2nd order, so it is a coherence in [0,1] and does not move
  // with received level.
  //
  // ALIGNMENT, WHICH THE FIRST VERSION GOT WRONG TWICE. gc[i] correlates the
  // window [i-L+1, i] against the replica -- measured, not assumed: for a core
  // placed at `pos`, the peak lands at pos + core_len - 1, the LAST sample of
  // the matched field. So the energy term has to cover exactly that window.
  //   (1) TrailingWindowSum is EXCLUSIVE of i: it returns sum over [i-L, i-1].
  //       That convention is deliberate for `thresh` (compare a peak against
  //       the energy BEFORE it) and wrong here, so this computes its own.
  //   (2) correlate_mt returns N + M - 1 samples, so the metric arrays are
  //       longer than the raw input by L-1; sizing the energy array from the
  //       raw length silently zeroed the tail of the search window.
  std::vector<double> raw_energy;
  double rep_energy = 0.0;
  if (xcorr) {
    const std::vector<float> raw_abs = Abs2(raw_samples);
    raw_energy.assign(peak_metric.size(), 0.0);
    double run = 0.0;
    for (size_t i = 0; i < raw_abs.size() && i < raw_energy.size(); ++i) {
      run += static_cast<double>(raw_abs[i]);
      if (i >= static_cast<size_t>(seqLen))
        run -= static_cast<double>(raw_abs[i - seqLen]);
      raw_energy[i] = run;
    }
    for (const auto& v : match_samples) rep_energy += std::norm(v);
  }
  auto ranking = [&](size_t i) {
    const double t = static_cast<double>(thresh[i]);
    if (nolag) {
      // |gc[i]|^2 / (E_raw[i] * E_rep): a coherence in [0,1]. No second window,
      // so no repeat check -- the peak stands on its own.
      if (i >= raw_energy.size() || i >= corr_abs.size()) return 0.0;
      const double e = raw_energy[i] * rep_energy;
      if (e <= 0.0) return 0.0;
      return static_cast<double>(corr_abs[i]) / e;
    }
    if (xcorr) {
      // peak_metric[i] = |gc[i]|^2 * |gc[i-L]|^2, so normalising it needs BOTH
      // windows' energies. Below the lag there is no second window and the
      // index cannot be a repeat, so it scores zero rather than dividing by a
      // window that does not exist.
      if (i < static_cast<size_t>(seqLen) || i >= raw_energy.size()) return 0.0;
      const double e1 = raw_energy[i] * rep_energy;
      const double e2 = raw_energy[i - seqLen] * rep_energy;
      if (e1 <= 0.0 || e2 <= 0.0) return 0.0;
      return static_cast<double>(peak_metric[i]) / (e1 * e2);
    }
    const double den = normalized ? t * t : t;
    return static_cast<double>(peak_metric[i]) / (den + 1e-30);
  };
  const double bar = 1.0 / static_cast<double>(corr_scale);
  std::queue<int> valid_peaks;
  for (size_t i = 0; i < peak_metric.size(); ++i) {
    if (ranking(i) > bar) {
      valid_peaks.push(static_cast<int>(i));
    }
  }
  if (std::getenv("FIND_BEACON_DEBUG") != nullptr) {
    double best_ratio = 0.0;
    size_t best_i = 0, best_pm_i = 0;
    float best_pm = 0.0f;
    for (size_t i = 0; i < peak_metric.size(); ++i) {
      const double r = peak_metric[i] / (thresh[i] + 1e-30);
      if (r > best_ratio) { best_ratio = r; best_i = i; }
      if (peak_metric[i] > best_pm) { best_pm = peak_metric[i]; best_pm_i = i; }
    }
    std::fprintf(stderr,
                 "[find_beacon] n=%zu corr_scale=%.1f  max(pm/thr)=%.4g at %zu  "
                 "max_pm=%.4g at %zu (thr there=%.4g)  #valid=%zu -> %d\n",
                 peak_metric.size(), corr_scale, best_ratio, best_i, best_pm,
                 best_pm_i, thresh[best_pm_i], valid_peaks.size(),
                 valid_peaks.empty() ? -1 : valid_peaks.front());
  }
#ifdef TEST_BENCH
  const auto t4 = std::chrono::steady_clock::now();
  const auto us = [](auto a, auto b) {
    return std::chrono::duration<double, std::micro>(b - a).count();
  };
  std::cout << "Correlate took " << us(t0, t1) << " usec\n"
            << "AutoCorr+Abs took " << us(t1, t2) << " usec\n"
            << "Threshold took " << us(t2, t3) << " usec\n"
            << "PeakDetect took " << us(t3, t4) << " usec" << std::endl;
#endif
  if (valid_peaks.empty()) return -1;
  if (pick == BeaconPick::kFirstCrossing) return valid_peaks.front();
  if (pick == BeaconPick::kTargetedArgmax) {
    // The caller has asserted the window cannot hold two beacon copies, so there
    // is no copy ambiguity to be repeatable ABOUT and the strongest crossing is
    // simply the beacon. Ranking by ratio rather than by peak_metric alone keeps
    // this consistent with the refine branch below and with the threshold test
    // itself, which is a ratio.
    int best = valid_peaks.front();
    double best_ratio = -1.0;
    while (!valid_peaks.empty()) {
      const int i = valid_peaks.front();
      valid_peaks.pop();
      const double r = ranking(i);
      if (r > best_ratio) { best_ratio = r; best = i; }
    }
    return best;
  }
  if (pick == BeaconPick::kFirstPath) {
    // Peak first, exactly as kTargetedArgmax.
    int best = valid_peaks.front();
    double best_ratio = -1.0;
    while (!valid_peaks.empty()) {
      const int i = valid_peaks.front();
      valid_peaks.pop();
      const double r = ranking(i);
      if (r > best_ratio) { best_ratio = r; best = i; }
    }
    // Then the earliest index within the delay-spread window that still clears
    // `kFirstPathFrac` of the peak. Searched over ALL indices, not only the
    // threshold crossings: a weak first path can sit under the absolute bar
    // while being unambiguous relative to the peak beside it, which is the
    // whole reason the test is a FRACTION of the peak.
    // Bounded the way the env reader bounds it: at most two replica lengths.
    const int win = std::max(0, std::min(first_path_window, seqLen * 2));
    const double floor_ratio =
        firstPathFloorFrac(thresh_form, first_path_db) * best_ratio;
    int first = best;
    for (int i = std::max(0, best - win); i < best; ++i) {
      if (ranking(static_cast<size_t>(i)) >= floor_ratio) { first = i; break; }
    }
    return first;
  }
  // valid_peaks is built in increasing index order, so front() is the earliest
  // crossing and therefore selects the earliest beacon copy in the window. Refine
  // only inside that copy: anything more than a sequence length later is a different
  // copy and must not be allowed to win.
  const int first = valid_peaks.front();
  int best = first;
  double best_ratio = -1.0;
  while (!valid_peaks.empty()) {
    const int i = valid_peaks.front();
    valid_peaks.pop();
    if (i - first > seqLen) break;
    const double r = ranking(i);
    if (r > best_ratio) { best_ratio = r; best = i; }
  }
  return best;
}

// Real-time entry: cint16 samples straight from the radio -> cfloat -> detect.
ssize_t CommsLib::find_beacon_avx(
    const std::complex<int16_t>* raw_samples,
    const std::vector<std::complex<float>>& match_samples, size_t check_window,
    float corr_scale, BeaconPick pick, BeaconThresh thresh_form) {
  return find_beacon_avx(raw_samples, match_samples, check_window, corr_scale,
                         pick, thresh_form,
                         firstPathWindow(static_cast<int>(match_samples.size())),
                         kFirstPathDb);
}

ssize_t CommsLib::find_beacon_avx(
    const std::complex<int16_t>* raw_samples,
    const std::vector<std::complex<float>>& match_samples, size_t check_window,
    float corr_scale, BeaconPick pick, BeaconThresh thresh_form,
    int first_path_window, double first_path_db) {
  static constexpr float kShortMaxFloat = 32767.0f;
  std::vector<std::complex<float>> sync_compare(check_window);
  for (size_t i = 0; i < check_window; ++i) {
    sync_compare[i] = std::complex<float>(
        static_cast<float>(raw_samples[i].real()) / kShortMaxFloat,
        static_cast<float>(raw_samples[i].imag()) / kShortMaxFloat);
  }
  return CommsLib::find_beacon_avx(sync_compare, match_samples, corr_scale,
                                   pick, thresh_form, first_path_window,
                                   first_path_db);
}

// Element-wise complex multiply, portable equivalent of the float
// complex_mult_avx: out[i] = f[i] * (conj ? conj(g[i]) : g[i]), size min(f,g).
// Used by receiver.cc CFO estimation; small (half a beacon) so plain portable
// C++ is fine, and it builds on aarch64 where complex_mult_avx does not.
std::vector<std::complex<float>> CommsLib::complex_mult(
    const std::vector<std::complex<float>>& f,
    const std::vector<std::complex<float>>& g, bool conj) {
  const size_t n = std::min(f.size(), g.size());
  std::vector<std::complex<float>> out(n);
  for (size_t i = 0; i < n; ++i) {
    out[i] = conj ? f[i] * std::conj(g[i]) : f[i] * g[i];
  }
  return out;
}
