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
 * PLACEMENT (AP-79 #6). The pad changes on almost every burst once the grid is
 * tracked, and at R3 (two 61440-sample slots) one interpolation costs 28-29 ms,
 * against a pad change every 1.6-2.7 frames. But a pad change only SHIFTS the
 * same content, and the filters are shift-invariant, so the content is
 * interpolated once (with a fixed lead of zeros) and each burst is that output
 * placed at twice the leading-zero offset: a memcmp and a memcpy. It is
 * BIT-exact, not merely close, when every non-zero output is computed on the
 * filters' interior path: the channel filter's edge path sums in a different
 * order. So the content is keyed with placeLead() zeros before it and computed
 * with as many appended after it, which is the filter on the zero-extended
 * signal; a burst with fewer leading zeros takes the full computation. The
 * test compares placement with that reference for every pad 0..383.
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

/// The caller's samples a write of `samples` delivered, from the TX samples
/// the radio took of its interpolated, beat-padded burst (`written` >= 0):
/// every input sample is two TX samples, and the beat padding is not the
/// caller's.
inline int inputSamplesWritten(int written, int samples) { return std::min(samples, written / 2); }

class TxBurstInterpolator {
 public:
  enum class CacheKey { kContent, kAddress };  // kAddress exists only as the test's mutant

  /// Zero input samples a prefiltered burst needs before / after its content:
  /// the channel filter's half length plus the halfband's own margins.
  static size_t prefilterLead() { return dsp::ChannelFilter().halfLength() + dsp::HalfbandInterp2().contextBefore(); }
  static size_t prefilterTail() { return dsp::ChannelFilter().halfLength() + dsp::HalfbandInterp2().contextAfter(); }

  explicit TxBurstInterpolator(bool prefilter = false, CacheKey key = CacheKey::kContent, bool place = true)
      : prefilter_(prefilter), key_(key), place_(place && key == CacheKey::kContent) {}

  /// Zeros placement needs before the content (and appends after it): every
  /// non-zero output of the channel filter (support +-half) must have its whole
  /// support inside the buffer, so 2 x half; the halfband is per-sample and
  /// shift-invariant.
  size_t placeLead() const { return prefilter_ ? 2 * chan_.halfLength() : hb_.contextBefore() + 1; }

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
      if (place_ && placeOne(L, in, n, np, o)) continue;
      // A placed lane's `in` is the placement key (shorter than n), not a
      // copy of the last burst: it never serves this path's comparison.
      const bool same = L.valid && !L.placed && L.n == n &&
                        (key_ == CacheKey::kAddress ? L.addr == buffs[c]
                                                    : std::memcmp(L.in.data(), in, n * sizeof(cs16)) == 0);
      L.placed = false;
      if (same) {
        ++hits_;
      } else {
        ++misses_;
        L.out.resize(2 * np);
        o.saturated += interpolate(in, n, np, L.in, L.out.data());
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
    // placement: `core` is the content's interpolation, `in` its input (from
    // placeLead() zeros before the content to the end), `at` the input offset
    // of that copy in the last burst placed, `np` that burst's padded length
    bool placed = false;
    std::vector<cs16> core;
    size_t at = 0, np = 0;
  };

  // Interpolate `in` (n samples, padded to np) into `out` (2 np samples).
  size_t interpolate(const cs16* in, size_t n, size_t np, std::vector<cs16>& scratch, cs16* out) {
    scratch.assign(in, in + n);
    scratch.resize(np, cs16(0, 0));
    if (prefilter_) {
      fin_.resize(np);
      fmid_.resize(np);
      fout_.resize(2 * np);
      for (size_t k = 0; k < np; ++k)
        fin_[k] = {static_cast<float>(scratch[k].real()), static_cast<float>(scratch[k].imag())};
      chan_.run(fin_.data(), np, fmid_.data());
      hb_.run(fmid_.data(), np, fout_.data());
      return dsp::HalfbandInterp2::quantize(fout_.data(), 2 * np, out);
    }
    return hb_.runCs16(scratch.data(), np, out);
  }

  // The placement path; false when this burst lacks the zero margins.
  bool placeOne(Lane& L, const cs16* in, size_t n, size_t np, Out& o) {
    const size_t lead = placeLead();
    size_t z = 0;
    while (z < n && in[z] == cs16(0, 0)) ++z;
    size_t e = n;
    while (e > z && in[e - 1] == cs16(0, 0)) --e;
    if (z == n || z < lead) return false;
    // The key and the core are [lead zeros | content], without the burst's
    // own trailing zeros; the core is computed with `lead` zeros of our own
    // after the content, so its every output is on the interior path.
    const size_t at = z - lead, m = e - at;
    const bool same = L.valid && L.placed && L.in.size() == m &&
                      std::memcmp(L.in.data(), in + at, m * sizeof(cs16)) == 0;
    if (same && L.at == at && L.np == np) {
      ++hits_;  // the identical burst: its placed output stands
      o.buffs.push_back(L.out.data());
      return true;
    }
    if (same) {
      ++hits_;
    } else {
      ++misses_;
      const size_t mp = beatPaddedInput(m + lead);
      L.core.resize(2 * mp);
      o.saturated += interpolate(in + at, m, mp, scratch_, L.core.data());
      L.in.assign(in + at, in + e);
    }
    // Place: zeros, then the core at twice the offset, cut at the burst's
    // padded end exactly where the full computation's output ends.
    L.out.assign(2 * np, cs16(0, 0));
    const size_t keep = std::min(L.core.size(), 2 * np - 2 * at);
    std::memcpy(L.out.data() + 2 * at, L.core.data(), keep * sizeof(cs16));
    L.at = at;
    L.np = np;
    L.n = n;
    L.addr = in;
    L.valid = true;
    L.placed = true;
    o.buffs.push_back(L.out.data());
    return true;
  }

  bool prefilter_;
  CacheKey key_;
  bool place_;
  std::vector<cs16> scratch_;
  dsp::HalfbandInterp2 hb_;
  dsp::ChannelFilter chan_;
  std::vector<std::complex<float>> fin_, fmid_, fout_;
  std::vector<Lane> lanes_;
  size_t hits_ = 0, misses_ = 0;
};

/// The beacon replay image's lead, in ticks: the prefiltered interpolator's
/// lead rounded up to the 4-tick grid, so the strobe offset 384 - lead stays
/// on it.
inline size_t beaconReplayLead() { return (TxBurstInterpolator::prefilterLead() + 3) / 4 * 4; }

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
