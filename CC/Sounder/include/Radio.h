/** @file Radio.h
  * @brief The radio seam: one abstract radio, a value of parameters instead of
  *        a configuration pointer, and a factory that is the only place that
  *        knows which backend a platform uses.
  *
  * Copyright (c) 2018-2022, Rice University
  * RENEW OPEN SOURCE LICENSE: http://renew-wireless.org/license
  *
  * docs/RADIO_PLATFORM_SEAM.md. The shape follows Agora's radio layer where
  * that layer is rigorous (an abstract radio, a runtime factory over
  * backends) and departs from it where it is not: the base class here is
  * NARROW -- streams, time, device facts -- takes a RadioParams value rather
  * than the whole Config, has one receive form, and carries no other
  * platform's framer hooks. What a platform has and another does not (the
  * transmit time grid, the receive gap ledger, a hardware trigger, an AGC)
  * is a capability a backend reports, never a no-op default that silently
  * passes.
  *
  * Backends: RadioSoapy (Iris and SoapyUHD, the SoapySDR plumbing),
  * RadioHoudini (RadioSoapy plus the Houdini stream arguments, the
  * pre-stream rates, the drain and gap ledger, the TDD grid), and, when
  * built with USE_UHD, the native UHD radio (seam step S3).
*/
#ifndef RADIO_H_
#define RADIO_H_

#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <memory>
#include <string>
#include <vector>

#include "sync/sync_config.h"  // houdini::sync::Platform

namespace SoapySDR {
class Device;
}

/// What a radio needs to open and set itself up. Filled by the radio sets
/// from Config; the radio never sees Config.
struct RadioParams {
  std::string id;             ///< Iris serial, UHD address, or the Houdini board IP
  std::string label;          ///< how logs name it ("BS <id>", "UE <id>")
  std::string remote_port;    ///< Houdini: the SoapyRemote port on the board
  std::vector<size_t> channels;
  double rate_hz = 0.0;
  double nco_hz = 0.0;        ///< Houdini: the mixer NCO; Iris: the BB frequency
  double rf_freq_hz = 0.0;    ///< Iris/UHD: the RF tune
  double bw_filter_hz = 0.0;  ///< Iris: the analog filter
  bool single_gain = false;   ///< Iris: one unified gain per direction
  double rx_freq_offset_hz = 0.0;  ///< deliberate detune for CFO validation (AP-33)
  double tx_freq_offset_hz = 0.0;
  // Houdini stream facts.
  int rx_local_port = 10002;  ///< host UDP port the RX stream binds
  std::string tx_mode = "stream";  ///< "replay" (the BS beacon RAM) or "stream" (the UE)
  bool tdd = false;           ///< the driver's TDD tick anchor for the UE pilot
  bool mts = true;            ///< multi-tile sync on every stream (AP-23)
  std::string timeout = "1000000";
};

class Radio {
 public:
  enum class Type { kSoapyIris, kSoapyUhd, kSoapyHoudini, kUhdNative };

  /// The one place that knows which backend a type is. Throws
  /// std::invalid_argument for a type this build has no backend for.
  static std::unique_ptr<Radio> create(Type type, const RadioParams& params);
  static const char* name(Type type);

  // (radioTypeFor(const Config&) in Radio.h below: the config's radio_type and
  // the build's Soapy-vs-UHD choice decide the backend.)

  virtual ~Radio() = default;

  // ---- device facts -------------------------------------------------------
  virtual Type type() const = 0;
  virtual houdini::sync::Platform platform() const = 0;
  const RadioParams& params() const { return params_; }
  /// Print what the device reports it is running at (rates, gains, antennas
  /// where they exist) and register the node for the version-skew check.
  virtual void printSettings() const = 0;

  // ---- capabilities: what this backend HAS, never a silent no-op ----------
  /// A hardware trigger and correlator block (Iris): the base station measures
  /// sync delays against it and the client waits on it.
  virtual bool hasHardwareTrigger() const = 0;
  /// An AGC block to configure (Iris).
  virtual bool hasAgc() const = 0;
  /// The transmit time this backend wants for a burst the sounder schedules at
  /// `frame_ticks` (sample ticks at `rate_hz`): the plain conversion for a
  /// backend without a grid; the Houdini backend adds its tick advance and
  /// snaps to the TDD window grid it accepts.
  virtual long long txTimeNs(long long frame_ticks, double rate_hz, bool tdd_pilot,
                             long long advance_ticks) const = 0;
  /// Samples zero-padded into the window the LAST recv() filled (0 = clean, and
  /// 0 always for a backend without a gap ledger). See AP-10.
  virtual size_t lastPadSamples() const { return 0; }
  /// Stream-relative sample position after the last recv, for a backend that
  /// records gap extents against that axis.
  virtual int64_t rxSamplePos() const { return 0; }
  /// Drain queued asynchronous TX status events; the count that indicated a
  /// problem. 0 for a backend whose stream reports none.
  virtual int drainTxStatus() { return 0; }

  // ---- streams ------------------------------------------------------------
  virtual void setup(int ch, double rxgain, double txgain) = 0;
  virtual int recv(void* const* buffs, int samples, long long& frameTime) = 0;
  virtual int activateRecv(long long rxTime = 0, size_t numSamps = 0, int flags = 0) = 0;
  virtual void deactivateRecv() = 0;
  virtual int xmit(const void* const* buffs, int samples, int flags, long long& frameTime) = 0;
  virtual void activateXmit() = 0;
  virtual void deactivateXmit() = 0;
  virtual int getTriggers() const = 0;
  virtual void drain_buffers(std::vector<void*> buffs, int symSamp) = 0;
  virtual void reset_DATA_clk_domain() = 0;

  /// The SoapySDR device behind a Soapy backend, nullptr otherwise. Used by
  /// the base-station framer code that still programs the Iris TDD block
  /// directly; seam step S2 moves those callers into the framer objects.
  virtual SoapySDR::Device* RawDev() const { return nullptr; }

 protected:
  explicit Radio(const RadioParams& params) : params_(params) {}
  RadioParams params_;
};

class Config;
/// The backend a configuration asks for: radio_type "houdini" is the Houdini
/// backend; otherwise the build's Soapy flavour (Iris, or UHD through Soapy
/// when built with RADIO_TYPE=SOAPY_UHD).
Radio::Type radioTypeFor(const Config& cfg);

#endif  // RADIO_H_
