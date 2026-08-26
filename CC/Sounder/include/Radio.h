/** @file Radio.h
  * @brief Declaration file for the Radio class.
  * 
  * Copyright (c) 2018-2022, Rice University 
  * RENEW OPEN SOURCE LICENSE: http://renew-wireless.org/license
*/
#ifndef RADIO_H_
#define RADIO_H_

#include <cstdlib>
#include <vector>

#include <cstdint>

#include "SoapySDR/Device.hpp"
#include "SoapySDR/Types.hpp"
#include "config.h"

class Radio {
 public:
  inline SoapySDR::Device* RawDev() const { return dev_; };

  // preStreamRxRate/preStreamTxRate/preStreamFreq (when non-zero) are applied
  // BEFORE the streams are opened -- required for backends (Houdini) that forbid
  // a live sample-rate change once a stream is open. RX and TX rates are set
  // independently because the Houdini BS beacon replays at the DAC max rate
  // while its RX runs at the sounder rate. preStreamTxRate < 0 means "use the
  // device's maximum TX sample rate" (Houdini replay). 0 leaves a rate to
  // dev_init (Iris path unchanged).
  Radio(const SoapySDR::Kwargs& args, const char soapyFmt[],
        const std::vector<size_t>& channels,
        const SoapySDR::Kwargs& rxStreamArgs = SoapySDR::Kwargs(),
        const SoapySDR::Kwargs& txStreamArgs = SoapySDR::Kwargs(),
        double preStreamRxRate = 0.0, double preStreamTxRate = 0.0,
        double preStreamFreq = 0.0);
  ~Radio(void);
  int recv(void* const* buffs, int samples, long long& frameTime);
  int activateRecv(const long long rxTime = 0, const size_t numSamps = 0,
                   int flags = 0);
  void deactivateRecv(void);
  int xmit(const void* const* buffs, int samples, int flags,
           long long& frameTime);
  void activateXmit(void);
  void deactivateXmit(void);
  int getTriggers(void) const;
  void drain_buffers(std::vector<void*> buffs, int symSamp);

  void reset_DATA_clk_domain(void);
  void dev_init(Config* _cfg, int ch, double rxgain, double txgain);

 private:
  int recvHoudini(void* const* buffs, int samples, long long& frameTime);

  SoapySDR::Device* dev_;
  SoapySDR::Stream* rxs_;
  SoapySDR::Stream* txs_;
  bool houdini_ = false;
  size_t num_rx_ch_ = 1;
  // Sample-gap awareness (Houdini UDP RX). recvHoudini() detects a dropped-packet gap
  // spliced mid-window (see grid tracker) and zero-pads it so post-gap samples keep
  // their offset; the extent is pushed to the process-wide RxGapSink (rx_gap_sink.h)
  // for the recorder's /Data/Gaps table.
  double rx_rate_ = 0.0;        // cached RX sample rate for the grid tracker
  int64_t rx_sample_pos_ = 0;  // absolute samples emitted across recvHoudini calls
};

#endif  // RADIO_H_