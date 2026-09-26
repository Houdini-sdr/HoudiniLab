/**
 * @file houdini/rx_packet.h
 * @brief The packet size that tiles a slot exactly: a UDP packet that
 *        divides the slot, so a slot is a whole number of packets, RX and TX
 *        alike (the host plugin sizes TX packets from the same MTU, and the
 *        TX slot is twice the RX slot's samples).
 *
 * The device derives its RX packet from the link MTU given at make()
 * (HOUDINI_MTU): the largest whole number of 512-bit beats (8 words of 64 bits,
 * 16 samples of sc16) that fits after the 58 bytes of Eth, IP, UDP and framer
 * header (SoapyHoudiniSDR shared/houdini_streaming_math.h, DeriveFrameWords).
 * The default MTU 8192 gives 2032 samples, 30.24 per 61440-sample slot.
 */
#pragma once

#include <cstddef>

namespace houdini {
namespace rxpkt {

constexpr size_t kBeatSamples = 16;        ///< 8 words x 2 sc16 samples
constexpr size_t kOverheadBytes = 58;      ///< Eth 14 + IP 20 + UDP 8 + framer 16
constexpr size_t kBytesPerSample = 4;      ///< sc16
constexpr size_t kDefaultMtu = 8192;

/// The largest packet (samples) that is a whole number of beats, divides the
/// slot exactly and fits the default MTU; 0 when none does.
inline size_t samplesForSlot(size_t slot_samples, size_t mtu = kDefaultMtu) {
  if (slot_samples == 0 || mtu <= kOverheadBytes) return 0;
  const size_t max_s = ((mtu - kOverheadBytes) / kBytesPerSample) / kBeatSamples * kBeatSamples;
  for (size_t s = max_s; s >= kBeatSamples; s -= kBeatSamples)
    if (slot_samples % s == 0) return s;
  return 0;
}

/// The HOUDINI_MTU that makes the device derive exactly `samples` per packet.
inline size_t mtuFor(size_t samples) { return samples * kBytesPerSample + kOverheadBytes; }

/// The device's derivation (DeriveFrameWords), in samples, for a given MTU.
inline size_t deviceSamples(size_t mtu) {
  if (mtu < kOverheadBytes + kBeatSamples * kBytesPerSample) return 0;
  return (mtu - kOverheadBytes) / kBytesPerSample / kBeatSamples * kBeatSamples;
}

/// samplesForSlot, but only when it keeps at least 3/4 of the default MTU's
/// packet; otherwise 0 (the default). A slot with no large divisor (4096 ->
/// 1024) would double the packet rate and halve the TX bank's depth in time
/// for nothing.
inline size_t tiledPacketOrDefault(size_t slot_samples) {
  const size_t s = samplesForSlot(slot_samples);
  return (s * 4 >= deviceSamples(kDefaultMtu) * 3) ? s : 0;
}

}  // namespace rxpkt
}  // namespace houdini
