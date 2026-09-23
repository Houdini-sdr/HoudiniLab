/**
 * @file houdini/tx_rx_boundary.h
 * @brief Where the mode-V rates meet the sounder (AP-79): the x2 interpolation
 *        of every TX burst at the radio boundary, and the sub-6 RX channel
 *        filter on the lanes that need it. Pure, so the test drives it without
 *        a radio; RadioHoudini owns one of each.
 *
 * TX. Every waveform is still built at sample_rate (122.88 = the tick), so
 * everything upstream of the radio (slot offsets, the 384-grid pad, the U-slot
 * offset) stays in ticks and simply doubles when the whole burst is
 * interpolated. That is exact only when a burst is written in ONE call with
 * zeros around its content, which the UE's seated bursts are (the slot's
 * 32-tick zero prefix and postfix cover the halfband's 6/5-sample margins).
 * Each burst is padded with zeros up to a whole 8-sample TX beat (4 ticks at
 * vld 2): the PL moves TX data in 8-sample beats (fpga lane, TDD_FRAMER_DESIGN
 * section 1c).
 *
 * SPECTRAL SHAPING (prefilter). At the sub-6 NCO any TX energy 40.2 to 61.4
 * MHz from the NCO lands, through the far ADC's real-sampling mirror, back ON
 * the channel (at 65.2 - f), so unshaped splatter from symbol and burst edges
 * caps the link: measured -33 dB for the band-limited beacon. With prefilter
 * on, every burst first passes the +-24 MHz channel filter at the tick rate
 * (72 dB down from +-40.2), then the halfband. The burst then needs
 * kPrefilterLead zeros ahead of its content and kPrefilterTail after it: the
 * slot's 32-tick prefix and postfix cover both.
 *
 * The UE re-sends the SAME burst every frame until its pad changes, so the
 * interpolated output is cached per channel and reused when the input content
 * is unchanged (a memcmp of the input, far cheaper than the filter). The cache
 * keys on CONTENT, not the buffer address: the caller rewrites its burst
 * buffers in place when the pad changes, so an address-keyed cache would
 * transmit a stale burst; the test builds that mutant.
 *
 * RX. Lane i of a combined stream is RX channel rx_channels[i]; the lanes
 * whose channel's mirror lands in the output (rf_plan) are filtered in place,
 * the others are left bit-exact.
 *
 * RENEW OPEN SOURCE LICENSE: http://renew-wireless.org/license
 */
#pragma once

#include <algorithm>
#include <complex>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <vector>

#include "dsp/band_filters.h"

namespace houdini {
namespace boundary {

using cs16 = std::complex<int16_t>;

/// Samples at the input rate, rounded up to a whole 8-sample TX beat after x2.
inline size_t beatPaddedInput(size_t n) { return (n + 3) / 4 * 4; }

class TxBurstInterpolator {
 public:
  enum class CacheKey { kContent, kAddress };  // kAddress exists only as the test's mutant

  /// Zero input samples a prefiltered burst needs before / after its content:
  /// the channel filter's half length plus the halfband's own margins.
  static size_t prefilterLead() { return dsp::ChannelFilter().halfLength() + dsp::HalfbandInterp2().contextBefore(); }
  static size_t prefilterTail() { return dsp::ChannelFilter().halfLength() + dsp::HalfbandInterp2().contextAfter(); }

  explicit TxBurstInterpolator(bool prefilter = false, CacheKey key = CacheKey::kContent)
      : prefilter_(prefilter), key_(key) {}

  struct Out {
    std::vector<const void*> buffs;  ///< one per channel, 2 x beatPaddedInput(n) samples
    size_t samples = 0;              ///< TX samples per channel
    size_t saturated = 0;            ///< I/Q components clipped this call (recomputed channels)
  };

  Out run(const void* const* buffs, size_t nch, size_t n) {
    if (lanes_.size() < nch) lanes_.resize(nch);
    Out o;
    const size_t np = beatPaddedInput(n);
    o.samples = 2 * np;
    for (size_t c = 0; c < nch; ++c) {
      // A null channel is one the caller does not write (the beacon load
      // passes nullptr for every non-beacon TX stream, and RadioSoapy::xmit
      // skips it): it stays null, is never read, and is never cached.
      if (buffs[c] == nullptr) {
        o.buffs.push_back(nullptr);
        continue;
      }
      Lane& L = lanes_[c];
      const auto* in = static_cast<const cs16*>(buffs[c]);
      const bool same = L.valid && L.n == n &&
                        (key_ == CacheKey::kAddress ? L.addr == buffs[c]
                                                    : std::memcmp(L.in.data(), in, n * sizeof(cs16)) == 0);
      if (same) {
        ++hits_;
      } else {
        ++misses_;
        L.in.assign(in, in + n);
        L.in.resize(np, cs16(0, 0));
        L.out.resize(2 * np);
        if (prefilter_) {
          fin_.resize(np);
          fmid_.resize(np);
          fout_.resize(2 * np);
          for (size_t k = 0; k < np; ++k)
            fin_[k] = {static_cast<float>(L.in[k].real()), static_cast<float>(L.in[k].imag())};
          chan_.run(fin_.data(), np, fmid_.data());
          hb_.run(fmid_.data(), np, fout_.data());
          o.saturated += dsp::HalfbandInterp2::quantize(fout_.data(), 2 * np, L.out.data());
        } else {
          o.saturated += hb_.runCs16(L.in.data(), np, L.out.data());
        }
        L.in.resize(n);  // the comparison copy is the caller's n samples
        L.n = n;
        L.addr = buffs[c];
        L.valid = true;
      }
      o.buffs.push_back(L.out.data());
    }
    return o;
  }

  size_t hits() const { return hits_; }
  size_t misses() const { return misses_; }

 private:
  struct Lane {
    std::vector<cs16> in, out;
    size_t n = 0;
    const void* addr = nullptr;
    bool valid = false;
  };
  bool prefilter_;
  CacheKey key_;
  dsp::HalfbandInterp2 hb_;
  dsp::ChannelFilter chan_;
  std::vector<std::complex<float>> fin_, fmid_, fout_;
  std::vector<Lane> lanes_;
  size_t hits_ = 0, misses_ = 0;
};

/// Which lanes of a combined RX stream get the channel filter: lane i is
/// rx_channels[i], and needs_filter(ch) says whether channel ch does.
template <class Pred>
std::vector<bool> laneFlags(const std::vector<size_t>& rx_channels, Pred needs_filter) {
  std::vector<bool> on;
  for (size_t ch : rx_channels) on.push_back(needs_filter(ch));
  return on;
}

class RxLaneFilters {
 public:
  explicit RxLaneFilters(std::vector<bool> lane_on) : on_(std::move(lane_on)) {}
  bool any() const {
    for (bool b : on_) if (b) return true;
    return false;
  }
  bool laneOn(size_t lane) const { return lane < on_.size() && on_[lane]; }

  /// Filter `n` samples starting at `start` of one lane's `capture` (cap_len
  /// samples, zero beyond it) into `dst`, using the capture around the slice
  /// as real context: bit-identical to filtering the whole capture and
  /// cutting the slice out, at the cost of the slice alone. The BS uses this
  /// on the slots it extracts from a continuous capture, which it could not
  /// afford to filter whole (AP-79).
  void filterSlice(const cs16* capture, size_t cap_len, size_t start, size_t n, cs16* dst) {
    if (start > cap_len || n > cap_len - start) throw std::invalid_argument("RxLaneFilters: slice outside the capture");
    const size_t h = f_.halfLength();
    const size_t lo = start > h ? start - h : 0;
    const size_t hi = std::min(cap_len, start + n + h);
    in_.resize(hi - lo);
    out_.resize(n);
    for (size_t k = lo; k < hi; ++k)
      in_[k - lo] = {static_cast<float>(capture[k].real()), static_cast<float>(capture[k].imag())};
    // The context buffer starts at `lo`, but zero lies at the CAPTURE's edges,
    // not the buffer's: the buffer is the capture itself wherever the filter
    // reaches (lo = max(0, start - h), hi = min(cap_len, start + n + h)).
    f_.runRange(in_.data(), hi - lo, start - lo, n, out_.data());
    dsp::HalfbandInterp2::quantize(out_.data(), n, dst);
  }
  /// Filter the flagged lanes of `buffs` (CS16, n samples each) in place.
  void apply(void* const* buffs, size_t nlanes, size_t n) {
    if (in_.size() < n) in_.resize(n);
    if (out_.size() < n) out_.resize(n);
    for (size_t l = 0; l < nlanes && l < on_.size(); ++l) {
      if (!on_[l]) continue;
      auto* p = static_cast<cs16*>(buffs[l]);
      for (size_t k = 0; k < n; ++k) in_[k] = {static_cast<float>(p[k].real()), static_cast<float>(p[k].imag())};
      f_.run(in_.data(), n, out_.data());
      dsp::HalfbandInterp2::quantize(out_.data(), n, p);
    }
  }

 private:
  std::vector<bool> on_;
  dsp::ChannelFilter f_;
  std::vector<std::complex<float>> in_, out_;
};

}  // namespace boundary
}  // namespace houdini
