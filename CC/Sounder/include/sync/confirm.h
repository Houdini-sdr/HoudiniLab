/**
 * @file sync/confirm.h
 * @brief The liveness confirm on a claimed detection: the in-window SNR guard.
 *
 * Moved from receiver.cc's beaconSnrDb unchanged. On this bench a real beacon
 * measures 45 to 48 dB and the noise-window artifact class that crosses the
 * correlation threshold measures ~0 dB (DEMO_VERIFICATION 4.25, 8.155), so a
 * floor separates them by orders of magnitude. The floor is a property of the
 * LINK AND THE WAVEFORM, not of the detector: a beacon 1.1 dB quieter than
 * the one the floor was set on is rejected wholesale at a level the louder one
 * clears (8.157). SyncConfig::validate says so when it can.
 *
 * NR's equivalent is the SSS decode at the PSS timing; a SequenceConfirm with
 * this interface is AP-68's job and slots in beside this one.
 */
#pragma once

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <sys/types.h>

namespace houdini {
namespace sync {

class SnrWindowGuard {
 public:
  /// @param floor_db   the SNR a detection must clear
  /// @param first_path_window  the detector's RESOLVED back window. The guard
  ///                   excluded either side of the core from BOTH sums is
  ///                   max(8, that): it MUST cover the first-path back window,
  ///                   because when first-path returns the direct path and a
  ///                   stronger echo sits a delay later, that echo lands
  ///                   outside the core span and, with a small guard, inside
  ///                   the noise sum -- a 1.4x echo 24 samples late then reads
  ///                   ~20 dB, the floor rejects, and resync escalates (8.151).
  SnrWindowGuard(double floor_db, size_t first_path_window)
      : floor_db_(floor_db), guard_(std::max<size_t>(8, first_path_window)) {}

  /// Energy of the presumed core [end_idx - core_len, end_idx) against the
  /// rest of the window, in dB. Returns -99 for an impossible span and +99 for
  /// a window with no noise samples left to compare against.
  double snrDb(const std::complex<int16_t>* w, size_t n, ssize_t end_idx,
               size_t core_len) const {
    const ssize_t lo = end_idx - static_cast<ssize_t>(core_len);
    if (lo < 0 || end_idx > static_cast<ssize_t>(n) || core_len == 0) return -99.0;
    const ssize_t g = static_cast<ssize_t>(guard_);
    double core = 0, rest = 0;
    size_t nrest = 0;
    for (size_t i = 0; i < n; ++i) {
      const double re = w[i].real(), im = w[i].imag();
      const double e = re * re + im * im;
      const ssize_t si = static_cast<ssize_t>(i);
      if (si >= lo && si < end_idx) {
        core += e;
      } else if (si < lo - g || si >= end_idx + g) {
        rest += e;
        ++nrest;
      }
    }
    if (nrest == 0 || rest <= 0.0) return 99.0;
    return 10.0 * std::log10((core / core_len) / (rest / nrest) + 1e-30);
  }

  bool accept(double snr_db) const { return snr_db >= floor_db_; }
  double floorDb() const { return floor_db_; }
  size_t guard() const { return guard_; }

 private:
  double floor_db_;
  size_t guard_;
};

}  // namespace sync
}  // namespace houdini
