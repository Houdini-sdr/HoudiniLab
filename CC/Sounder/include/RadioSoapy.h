/** @file RadioSoapy.h
  * @brief The SoapySDR backend: the Iris radio (and SoapyUHD when built for
  *        it). Today's Radio class minus the Houdini branch, which lives in
  *        RadioHoudini on top of this.
  *
  * Copyright (c) 2018-2022, Rice University
  * RENEW OPEN SOURCE LICENSE: http://renew-wireless.org/license
*/
#ifndef RADIO_SOAPY_H_
#define RADIO_SOAPY_H_

#include <cstdint>
#include <vector>

#include "Radio.h"
#include "SoapySDR/Device.hpp"
#include "SoapySDR/Types.hpp"

class RadioSoapy : public Radio {
 public:
  /// Open an Iris (or SoapyUHD) device from its parameters.
  explicit RadioSoapy(const RadioParams& params);
  ~RadioSoapy() override;

  Type type() const override;
  houdini::sync::Platform platform() const override { return houdini::sync::Platform::kIrisUhd; }
  void printSettings() const override;
  bool hasHardwareTrigger() const override;
  bool hasAgc() const override;
  long long txTimeNs(long long frame_ticks, double rate_hz, bool tdd_pilot,
                     long long advance_ticks) const override;

  void setup(int ch, double rxgain, double txgain) override;
  int recv(void* const* buffs, int samples, long long& frameTime) override;
  int activateRecv(long long rxTime = 0, size_t numSamps = 0, int flags = 0) override;
  void deactivateRecv() override;
  int xmit(const void* const* buffs, int samples, int flags, long long& frameTime) override;
  void activateXmit() override;
  void deactivateXmit() override;
  int getTriggers() const override;
  int drainTxStatus() override;
  void drain_buffers(std::vector<void*> buffs, int symSamp) override;
  void reset_DATA_clk_domain() override;
  SoapySDR::Device* RawDev() const override { return dev_; }

 protected:
  /// The generic open: make the device, apply the pre-stream rates and tune
  /// (a backend that forbids a live rate change passes them non-zero; Iris
  /// passes 0 and sets them in setup()), inject the deliberate frequency
  /// offsets, open the streams. `houdini_streams` selects the Houdini stream
  /// order (TX first, with the MTS membership stream when ch0 is not a data
  /// channel).
  RadioSoapy(const RadioParams& params, const SoapySDR::Kwargs& args,
             const SoapySDR::Kwargs& rxStreamArgs, const SoapySDR::Kwargs& txStreamArgs,
             double preStreamRxRate, double preStreamTxRate, double preStreamFreq,
             bool houdini_streams);

  SoapySDR::Device* dev_ = nullptr;
  // nullptr NSDMI is load-bearing: the ctor's cleanup-and-rethrow reads
  // these before every setupStream has assigned them (second review 2.1 --
  // an indeterminate txs_ meant closeStream on a wild pointer on the
  // transient-board-wedge retry path).
  SoapySDR::Stream* rxs_ = nullptr;
  SoapySDR::Stream* txs_ = nullptr;
  // MTS membership helper: DAC tile 0 must be a GROUP MEMBER (not merely
  // powered), so a single-channel ch1 stream needs this never-activated
  // ch0 replay stream opened first (the canonical mts_check group shape).
  SoapySDR::Stream* aux_mts_txs_ = nullptr;
  size_t num_rx_ch_ = 1;

 private:
  bool tx_status_unsupported_ = false;  // stream reported no status surface: stop asking
  size_t tx_status_events_ = 0;         // cumulative problem events seen
  long long tx_status_log_ns_ = 0;      // warn throttle
};

#endif  // RADIO_SOAPY_H_
