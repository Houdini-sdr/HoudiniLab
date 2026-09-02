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
    // is lead+tail (~812 samples at shipped defaults) against a 4096-sample copy
    // spacing. There the earliest-crossing rule is actively wrong: the beacon's
    // own STS preamble is 16-periodic, 16 divides the 128-sample correlator lag,
    // so the STS field is lag-128 self-coherent and manufactures crossings a few
    // hundred samples before the true peak. Whether they cross is a function of
    // RECEIVED LEVEL, because the test compares a 4th-order quantity against a
    // 2nd-order one, so a link that is fine today false-locks after a gain
    // change. Measured: the shipped beacon flips from -1 to -363 samples between
    // 1600 and 3200 counts RMS (beacon_geometry_test).
    kTargetedArgmax,
  };

  // Functions using AVX
  static int find_beacon(const std::vector<std::complex<float>>& raw_samples);
  static int find_beacon_avx(
      const std::vector<std::complex<float>>& raw_samples,
      const std::vector<std::complex<float>>& match_samples, float corr_scale,
      BeaconPick pick = BeaconPick::kFirstCrossing);

  ///Find Beacon with raw samples from the radio
  static int find_beacon(const std::complex<int16_t>* raw_samples,
                         size_t check_window);

  static ssize_t find_beacon_avx(
      const std::complex<int16_t>* raw_samples,
      const std::vector<std::complex<float>>& match_samples,
      size_t check_window, float corr_scale,
      BeaconPick pick = BeaconPick::kFirstCrossing);
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
  // x86 and aarch64. num_threads=0 => auto (env SOUNDER_CORR_THREADS, else 1).
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
