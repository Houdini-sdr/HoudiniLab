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
        double preStreamFreq = 0.0, double rxFreqOffset = 0.0,
        double txFreqOffset = 0.0);
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
  // Samples zero-padded into the window the LAST recv() filled (0 = clean). Valid
  // until the next recv() on this radio. Lets a consumer tell a window that carries
  // inserted zeros from one that is all real samples, which the CSI/view path needs
  // and which the /Data/Gaps table alone cannot answer (it is drained only by the
  // HDF5 writer, and viewing mode writes no file). See AP-10.
  size_t lastPadSamples(void) const { return last_pad_samples_; }
  // Drain queued asynchronous TX status events and report how many indicated a
  // problem. writeStream returning the full count only means the burst was ACCEPTED;
  // a burst sent late or dropped for being off-grid shows up only here. The driver
  // merges device-side events for live TX streams (tx_mode=stream), so on the fine
  // TDD grid this is the difference between a silent phase jump and a logged one.
  // Latches off if the stream does not support status. See AP-10.
  int drainTxStatus(void);
  void drain_buffers(std::vector<void*> buffs, int symSamp);

  void reset_DATA_clk_domain(void);
  void dev_init(Config* _cfg, int ch, double rxgain, double txgain);

 private:
  int recvHoudini(void* const* buffs, int samples, long long& frameTime);

  SoapySDR::Device* dev_;
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
  bool houdini_ = false;
  size_t num_rx_ch_ = 1;
  // Sample-gap awareness (Houdini UDP RX). recvHoudini() detects a dropped-packet gap
  // spliced mid-window (see grid tracker) and zero-pads it so post-gap samples keep
  // their offset; the extent is pushed to the process-wide RxGapSink (rx_gap_sink.h)
  // for the recorder's /Data/Gaps table.
  double rx_rate_ = 0.0;        // cached RX sample rate for the grid tracker
  int64_t rx_sample_pos_ = 0;  // absolute samples emitted across recvHoudini calls

 public:
  // Stream-relative sample position after the last recv, for callers that
  // record gap extents against the same axis recvHoudini uses.
  int64_t rxSamplePos() const { return rx_sample_pos_; }

 private:
  size_t last_pad_samples_ = 0;  // zeros inserted into the last window (lastPadSamples)
  bool tx_status_unsupported_ = false;  // stream reported no status surface: stop asking
  size_t tx_status_events_ = 0;         // cumulative problem events seen
  long long tx_status_log_ns_ = 0;      // warn throttle
};

#endif  // RADIO_H_