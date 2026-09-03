/*

 Communications Library:
   a) Generate pilot/preamble sequences
   b) OFDM modulation

---------------------------------------------------------------------
 Copyright (c) 2018-2019, Rice University 
 RENEW OPEN SOURCE LICENSE: http://renew-wireless.org/license
 Author(s): Rahman Doost-Mohamamdy: doost@rice.edu
	    Oscar Bejarano: obejarano@rice.edu
---------------------------------------------------------------------
*/

// NO INCLUDE GUARD until 2026-09-02. Every translation unit included this once
// and directly, so it never bit; the first HEADER to include it (receiver.h,
// which now names CommsLib::BeaconPick in a signature) would have made the class
// definition arrive twice in most of the build.
#pragma once

#include <complex.h>
#include <math.h>
#include <stdio.h> /* for fprintf */
#include <stdlib.h>
#include <string.h> /* for memcpy */
#include <unistd.h>

#include <algorithm>
#include <cstring>
#include <fstream>  // std::ifstream
#include <iostream>
#include <thread>
#include <vector>

#include "fft.h"

static constexpr size_t kPilotSubcarrierSpacing = 12;
static constexpr size_t kDefaultPilotScOffset = 6;

static inline double computeAbs(std::complex<double> x) { return std::abs(x); }

//template <typename T>
//static inline T computeAbs(std::complex<T> x) { return std::abs(x); }
static inline double computePower(std::complex<double> x) {
  return std::pow(std::abs(x), 2);
}
static inline double computeSquare(double x) { return x * x; }

class CommsLib {
 public:
  enum SequenceType {
    STS_SEQ,
    LTS_SEQ,
    LTS_SEQ_F,
    LTE_ZADOFF_CHU,
    LTE_ZADOFF_CHU_F,
    GOLD_IFFT,
    HADAMARD
  };

  enum ModulationOrder { QPSK = 2, QAM16 = 4, QAM64 = 6 };

  CommsLib(std::string);
  ~CommsLib();

  static std::vector<std::vector<float>> getSequence(size_t type,
                                                     size_t seq_len = 0);
  static std::vector<std::complex<float>> modulate(std::vector<uint8_t>, int);
  static std::vector<size_t> getDataSc(
      size_t fftSize, size_t DataScNum,
      size_t PilotScOffset = kDefaultPilotScOffset);
  static std::vector<size_t> getNullSc(size_t fftSize, size_t DataScNum);
  static std::vector<std::complex<float>> getPilotScValue(
      size_t fftSize, size_t DataScNum,
      size_t PilotScOffset = kDefaultPilotScOffset);
  static std::vector<size_t> getPilotScIndex(
      size_t fftSize, size_t DataScNum,
      size_t PilotScOffset = kDefaultPilotScOffset);
  static std::vector<std::complex<float>> FFT(
      const std::vector<std::complex<float>>&, int, bool fft_shift = false);
  static std::vector<std::complex<float>> IFFT(
      const std::vector<std::complex<float>>&, int, float scale = 0.5,
      bool normalize = true, bool fft_shift = false);

  static int findLTS(const std::vector<std::complex<float>>& iq, int seqLen);
  static size_t find_pilot_seq(const std::vector<std::complex<float>>& iq,
                               const std::vector<std::complex<float>>& pilot,
                               size_t seqLen);
  template <typename T>
  //static std::vector<T> convolve(std::vector<T> const& f, std::vector<T> const& g);
  static std::vector<T> convolve(std::vector<T> const& f,
                                 std::vector<T> const& g) {
    /* Convolution of two vectors
         * Source:
         * https://stackoverflow.com/questions/24518989/how-to-perform-1-dimensional-valid-convolution
         */
    int const nf = f.size();
    int const ng = g.size();
    int const n = nf + ng - 1;
    std::vector<T> out(n, 0);
    for (auto i(0); i < n; ++i) {
      int const jmn = (i >= ng - 1) ? i - (ng - 1) : 0;
      int const jmx = (i < nf - 1) ? i : nf - 1;
      for (auto j(jmn); j <= jmx; ++j) {
        out[i] += f[j] * g[i - j];
      }
    }
    return out;
  }
  static float find_max_abs(const std::vector<std::complex<float>>& in);
  static std::vector<std::complex<float>> csign(
      const std::vector<std::complex<float>>& iq);
  static inline int hadamard2(int i, int j) {
    return (__builtin_parity(i & j) != 0 ? -1 : 1);
  }
  static std::vector<float> magnitudeFFT(
      std::vector<std::complex<float>> const&, std::vector<float> const&,
      size_t);
  static std::vector<float> hannWindowFunction(size_t);
  static double windowFunctionPower(std::vector<float> const&);
  template <typename T>
  static T findTone(std::vector<T> const&, double, double, size_t,
                    const size_t delta = 10);
  static float measureTone(std::vector<std::complex<float>> const&,
                           std::vector<float> const&, double, double, size_t,
                           const size_t delta = 10);

  // WHICH threshold crossing find_beacon_avx returns. The choice is not a
  // refinement detail: it decides whether a strong link anchors on the beacon or
  // on its own preamble, and the right answer differs between a wide window that
  // may hold several beacon copies and a targeted slice that provably holds one.
  enum class BeaconPick {
    // Earliest crossing in the window, unrefined. HISTORICAL, and unsafe on a
    // strong link -- see kTargetedArgmax.
    kFirstCrossing,
    // Earliest crossing, then the best ratio within one sequence length of it.
    // Selects the earliest beacon COPY (repeatable across restarts, which the
    // once-only pilot anchor needs) without returning that copy's leading skirt.
    // For windows wide enough to hold more than one copy: acquisition.
    kFirstClusterRefined,
    // Strongest crossing anywhere in the window. ONLY valid when the window
    // cannot hold two beacon copies -- true of the targeted resync slice, which
    // is lead+tail (~812 samples at shipped defaults) against a copy spacing of
    // one FULL FRAME, 122880 samples: the strobe plays loops=1 once per TDD
    // frame, so there is exactly one beacon per millisecond. (The old
    // loops=forever era filled a symbol with ~15 copies 4096 apart and that
    // number got repeated here by habit -- it is 30x too conservative, which
    // matters because this precondition is what makes the rule legal and
    // over-the-air drift will want a wider slice.) There the earliest-crossing rule is actively wrong: the beacon's
    // own STS preamble is 16-periodic, 16 divides the 128-sample correlator lag,
    // so the STS field is lag-128 self-coherent and manufactures crossings a few
    // hundred samples before the true peak. Whether they cross is a function of
    // RECEIVED LEVEL, because the test compares a 4th-order quantity against a
    // 2nd-order one, so a link that is fine today false-locks after a gain
    // change. Measured: the shipped beacon flips from -1 to -274..-256 samples
    // between 1600 and 3200 counts PEAK, reaching -363 by 12800
    // (beacon_geometry_test; kLevels are peak counts, not rms).
    kTargetedArgmax,
    // FIRST PATH: argmax, then walk BACK to the earliest crossing still within
    // `frac` of the peak, bounded by a delay-spread window.
    //
    // WHY kTargetedArgmax IS NOT THE RIGHT RULE OVER THE AIR. On a cabled bench
    // there is one path, so the strongest crossing IS the beacon. In a
    // multipath channel the matched filter has one peak per path, and the
    // strongest is frequently a reflection arriving after the direct one. Two
    // things follow, and the second is worse than the first: the timing
    // reference is biased late by the excess delay, and -- because which path
    // is strongest CHANGES as the channel fades -- the reference JUMPS between
    // paths. A frame grid tracked off a reference that hops is worse than one
    // tracked off a slightly late but stable reference.
    //
    // First-path detection is the standard answer and it degenerates to argmax
    // when there is only one path, so it is safe on this bench too. The back
    // window must stay well inside the preamble's self-coherent plateau (~365
    // samples early for the shipped beacon) or it reintroduces exactly the
    // failure kTargetedArgmax exists to remove.
    kFirstPath,
  };

  // WHICH FORM THE DETECTION THRESHOLD TAKES. This is the difference between a
  // knob that names a fixed thing and one that does not.
  enum class BeaconThresh {
    // SHIPPED: corr_scale * |gc[i]|^2|gc[i-L]|^2 > sum_L |gc|^2. The numerator
    // is 4th order in received amplitude and the denominator 2nd, so the
    // decision ratio carries units of power and moves with LEVEL. Measured over
    // a 64x level sweep on the shipped beacon, the statistic at the true peak
    // runs 0.0777 to 321.4 -- a spread of 4136, which is 64^2 to within noise.
    // So a fixed corr_scale is a different test at every received level, and
    // the beacon's own STS preamble crosses it above ~3200 counts peak.
    kPowerRatio,
    // Schmidl & Cox 1997, section III: divide by the energy term SQUARED so the
    // statistic is dimensionless and bounded in [0,1]. Same corr_scale meaning
    // (the bar is 1/corr_scale), now a FRACTION rather than a power. Measured
    // over the same sweep: 0.9845 to 0.9843, a spread of 1.00. The preamble
    // plateau lands at 1/L^2 -- also level-independent -- so one threshold
    // separates them everywhere.
    kNormalized,
    // The one above is WRONG AS FORMULATED, kept because the measurement that
    // killed it is worth keeping. Schmidl & Cox's R(d) is the LOCAL SIGNAL
    // ENERGY of the repeated half-symbol; `thresh` here is the trailing energy
    // of the MATCHED FILTER OUTPUT, which is not the same quantity, and the
    // algebra does not carry over. Measured: it makes the peak statistic
    // level-invariant (spread 1.00 against 4136) and then selects the WRONG
    // peak on the shorter guarded beacons -- dot11 lands at -65, exactly one
    // fine-field length early, on every seed at every level, because dividing
    // by a squared trailing sum favours the index with less preceding
    // correlation energy, which is the FIRST repetition.
    //
    // This is the normalised CROSS-CORRELATION, which is the right shape for a
    // matched filter: |gc[i]|^2 / (E_raw[i] * E_rep) is a coherence in [0,1],
    // level-invariant because both numerator and denominator are 2nd order in
    // received amplitude. The repeat check is then the product of the two
    // coherences, which is still in [0,1].
    kNormalizedXCorr,
    // NR-SHAPED: normalised matched filter with NO lag product at all.
    //
    // Our detector is 802.11-shaped -- a repeated-pair autocorrelation stacked
    // on a matched filter -- and that lag product is exactly what lets a
    // 16-periodic preamble alias into a 128-sample lag. NR does not have the
    // failure class: PSS is a NON-REPEATING m-sequence found by a plain matched
    // filter, and the peak is the peak.
    //
    // THIS IS THAT DETECTOR ONLY HALF-WAY, AND THE LIMIT MATTERS. It drops the
    // lag product, but the correlator reference stays the FINE field -- the
    // repeated gold/LTS/TRS symbol -- because that is what Config hands the
    // detector. So it runs a plain matched filter against a symbol that appears
    // TWICE, which is precisely the ambiguity NR's non-repeating PSS avoids.
    // Measured accordingly: with the floor correctly converted for a 2nd-order
    // statistic the residual is -129 on legacy_guard, exactly one fine_len, so
    // the failure is rep1/rep2 AMBIGUITY and not the loss of a noise-spike
    // check. The real NR architecture -- the non-repeating PSS as the replica --
    // is the `nr_pss` shape (beacon_shapes.h), which syncSearch runs through
    // THIS form automatically: exact at every level offline, and on silicon it
    // acquires and tracks like legacy (DEMO_VERIFICATION 8.154-8.162). Its one
    // cost is the noise separation below: about one rejected noise-window
    // crossing per acquisition hunt.
    //
    // The other trade is discrimination against the preamble: the lag product
    // puts the preamble plateau ~1/L^2 below the peak, and without it the
    // separation is only ~1/L, 21 dB instead of 42.
    kCoherence,  // (was kXCorrNoLag; "nolag" survives only as the env alias)
  };

  // Functions using AVX
  static int find_beacon(const std::vector<std::complex<float>>& raw_samples);
  static int find_beacon_avx(
      const std::vector<std::complex<float>>& raw_samples,
      const std::vector<std::complex<float>>& match_samples, float corr_scale,
      BeaconPick pick = BeaconPick::kFirstCrossing,
      BeaconThresh thresh_form = BeaconThresh::kPowerRatio);

  ///Find Beacon with raw samples from the radio
  static int find_beacon(const std::complex<int16_t>* raw_samples,
                         size_t check_window);

  static ssize_t find_beacon_avx(
      const std::complex<int16_t>* raw_samples,
      const std::vector<std::complex<float>>& match_samples,
      size_t check_window, float corr_scale,
      BeaconPick pick = BeaconPick::kFirstCrossing,
      BeaconThresh thresh_form = BeaconThresh::kPowerRatio);
  // The first-path knobs made EXPLICIT. The two-argument-shorter overloads
  // above use the defaults (half the replica, kDefaultFirstPathFloorDb); the
  // configured values come through sync::SyncConfig and the library passes
  // them here, so a run has ONE source of truth and nothing in the correlator
  // reads the environment.
  static constexpr double kDefaultFirstPathFloorDb = -9.0;
  /// Threads for correlate_mt (sync.detector.corr_threads); 0 leaves the
  /// current setting. Read at dispatch, so set it before the first search.
  static void setCorrelatorThreads(unsigned n);
  //   first_path_window  samples of back-search from the peak (0..2*seqLen)
  //   first_path_db      how much weaker an earlier path may be, dB <= 0
  static int find_beacon_avx(
      const std::vector<std::complex<float>>& raw_samples,
      const std::vector<std::complex<float>>& match_samples, float corr_scale,
      BeaconPick pick, BeaconThresh thresh_form, int first_path_window,
      double first_path_db);
  static ssize_t find_beacon_avx(
      const std::complex<int16_t>* raw_samples,
      const std::vector<std::complex<float>>& match_samples,
      size_t check_window, float corr_scale, BeaconPick pick,
      BeaconThresh thresh_form, int first_path_window, double first_path_db);
  // The detection with its evidence: the index, the decision statistic at it
  // (in the form's units) and the correlator output there (the matched
  // field's complex peak). The find_beacon_avx overloads return .index.
  struct BeaconResult {
    ssize_t index = -1;
    double statistic = 0.0;
    std::complex<float> peak{0.0f, 0.0f};
  };
  /// The radio's samples as the correlator takes them: full scale to +-1.
  static std::vector<std::complex<float>> toCorrelatorScale(
      const std::complex<int16_t>* raw, size_t n);
  static BeaconResult find_beacon_ex(
      const std::vector<std::complex<float>>& raw_samples,
      const std::vector<std::complex<float>>& match_samples, float corr_scale,
      BeaconPick pick, BeaconThresh thresh_form, int first_path_window,
      double first_path_db);
  static BeaconResult find_beacon_ex(
      const std::complex<int16_t>* raw_samples,
      const std::vector<std::complex<float>>& match_samples,
      size_t check_window, float corr_scale, BeaconPick pick,
      BeaconThresh thresh_form, int first_path_window, double first_path_db);
  // GPU beacon detector (find_beacon_cuda.cu), defined only when built with
  // -DUSE_CUDA (CMake HOUDINI_USE_CUDA), which is OFF by default and OFF on the
  // rig. NOTE it still returns the FIRST crossing (atomicMin over the index), so
  // it does NOT match find_beacon_avx with any other BeaconPick: it needs the same
  // argmax-by-ratio change before the GPU path can be used for acquisition.
  static ssize_t find_beacon_cuda(
      const std::complex<int16_t>* raw_samples,
      const std::vector<std::complex<float>>& match_samples,
      size_t check_window, float corr_scale);

  static std::vector<float> correlate_avx_s(std::vector<float> const& f,
                                            std::vector<float> const& g);
  static std::vector<int16_t> correlate_avx_si(std::vector<int16_t> const& f,
                                               std::vector<int16_t> const& g);
  static std::vector<float> abs2_avx(std::vector<std::complex<float>> const& f);
  static std::vector<int32_t> abs2_avx(
      std::vector<std::complex<int16_t>> const& f);
  static std::vector<std::complex<float>> auto_corr_mult_avx(
      std::vector<std::complex<float>> const& f, const int dly,
      const bool conj = true);
  static std::vector<std::complex<int16_t>> auto_corr_mult_avx(
      std::vector<std::complex<int16_t>> const& f, const int dly,
      const bool conj = true);
  static std::vector<std::complex<float>> correlate_avx(
      std::vector<std::complex<float>> const& f,
      std::vector<std::complex<float>> const& g);
  static std::vector<std::complex<int16_t>> correlate_avx(
      std::vector<std::complex<int16_t>> const& f,
      std::vector<std::complex<int16_t>> const& g);
  // Portable + multi-threaded matched filter (see comms-lib-portable.cc),
  // equivalent to the float correlate_avx. Compiles and auto-vectorizes on both
  // x86 and aarch64. num_threads=0 => the value set by setCorrelatorThreads
  // (sync.detector.corr_threads); if none was set, SOUNDER_CORR_THREADS read
  // once (the bench tools' path); else 1.
  static std::vector<std::complex<float>> correlate_mt(
      const std::vector<std::complex<float>>& f,
      const std::vector<std::complex<float>>& g, unsigned num_threads = 0);
  static std::vector<std::complex<float>> complex_mult_avx(
      std::vector<std::complex<float>> const& f,
      std::vector<std::complex<float>> const& g, const bool conj);
  static std::vector<std::complex<int16_t>> complex_mult_avx(
      std::vector<std::complex<int16_t>> const& f,
      std::vector<std::complex<int16_t>> const& g, const bool conj);
  // Portable element-wise complex multiply (see comms-lib-portable.cc),
  // equivalent to the float complex_mult_avx. Builds on x86 and aarch64.
  static std::vector<std::complex<float>> complex_mult(
      const std::vector<std::complex<float>>& f,
      const std::vector<std::complex<float>>& g, bool conj);
  //private:
  //    static inline float** init_qpsk();
  //    static inline float** init_qam16();
  //    static inline float** init_qam64();
};
