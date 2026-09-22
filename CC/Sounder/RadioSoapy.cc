/** @file RadioSoapy.cc
  * @brief The SoapySDR backend (Iris, and SoapyUHD when built for it): the
  *        plumbing every Soapy radio shares. Today's Radio.cc with the Houdini
  *        branch moved to RadioHoudini.cc.
  *
  * Copyright (c) 2018-2022, Rice University
  * RENEW OPEN SOURCE LICENSE: http://renew-wireless.org/license
*/
#include "include/RadioSoapy.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <vector>

#include "SoapySDR/Errors.hpp"
#include "SoapySDR/Formats.hpp"
#include "SoapySDR/Time.hpp"
#include "include/logger.h"
#include "include/macros.h"

void RadioSoapy::setup(int ch, double rxgain, double txgain) {
  SoapySDR::Kwargs info = dev_->getHardwareInfo();

  dev_->setSampleRate(SOAPY_SDR_RX, ch, params_.rate_hz);
  dev_->setSampleRate(SOAPY_SDR_TX, ch, params_.rate_hz);

  // these params are sufficient to set before DC offset and IQ imbalance calibration
  if (!isUhd()) {
    dev_->setAntenna(SOAPY_SDR_RX, ch, "TRX");
    dev_->setBandwidth(SOAPY_SDR_RX, ch, params_.bw_filter_hz);
    dev_->setBandwidth(SOAPY_SDR_TX, ch, params_.bw_filter_hz);
    dev_->setFrequency(SOAPY_SDR_RX, ch, "BB", params_.nco_hz);
    dev_->setFrequency(SOAPY_SDR_TX, ch, "BB", params_.nco_hz);
  } else {
    MLPD_INFO("Init USRP channel: %d\n", ch);
    dev_->setAntenna(SOAPY_SDR_TX, ch, "TX/RX");
    dev_->setAntenna(SOAPY_SDR_RX, ch, "RX2");  // or "TX/RX"
    dev_->setFrequency(SOAPY_SDR_RX, ch, "BB", 0);
    dev_->setFrequency(SOAPY_SDR_TX, ch, "BB", 0);
  }

  dev_->setFrequency(SOAPY_SDR_RX, ch, "RF", params_.rf_freq_hz);
  dev_->setFrequency(SOAPY_SDR_TX, ch, "RF", params_.rf_freq_hz);
  if (!isUhd()) {
    // Unified gains for both lime and frontend
    if (params_.single_gain) {
      dev_->setGain(SOAPY_SDR_RX, ch,
                    rxgain);  // w/CBRS 3.6GHz [0:105], 2.5GHZ [0:108]
      dev_->setGain(SOAPY_SDR_TX, ch,
                    txgain);  // w/CBRS 3.6GHz [0:105], 2.5GHZ [0:105]
      MLPD_INFO("Tx gain: %lf, Rx gain: %lf\n", dev_->getGain(SOAPY_SDR_TX, ch),
                dev_->getGain(SOAPY_SDR_RX, ch));
    } else {
      if (info["frontend"].find("CBRS") != std::string::npos) {
        if (params_.rf_freq_hz > 3e9) {
          dev_->setGain(SOAPY_SDR_RX, ch, "ATTN", -6);  //[-18,0]
        } else if ((params_.rf_freq_hz > 2e9) &&
                   (params_.rf_freq_hz < 3e9)) {
          dev_->setGain(SOAPY_SDR_RX, ch, "ATTN", -18);  //[-18,0]
        } else {
          dev_->setGain(SOAPY_SDR_RX, ch, "ATTN", -12);  //[-18,0]
        }
        dev_->setGain(SOAPY_SDR_RX, ch, "LNA2", 17);  //[0,17]
      } else if (info["frontend"].find("UHF") != std::string::npos) {
        dev_->setGain(SOAPY_SDR_RX, ch, "ATTN1", -6);  //[-18,0]
        dev_->setGain(SOAPY_SDR_RX, ch, "ATTN2", -6);  //[-18,0]
      }
      dev_->setGain(
          SOAPY_SDR_RX, ch, "LNA",
          std::min(30.0, rxgain));  // w/CBRS 3.6GHz [0:105], 2.5GHZ [0:108]
      dev_->setGain(SOAPY_SDR_RX, ch, "TIA", 0);
      dev_->setGain(SOAPY_SDR_RX, ch, "PGA", 0);

      if (info["frontend"].find("CBRS") != std::string::npos) {
        dev_->setGain(SOAPY_SDR_TX, ch, "ATTN", -6);  //[-18,0] by 3
        dev_->setGain(SOAPY_SDR_TX, ch, "PA2", 0);    //[0|15]
      }
      if (info["frontend"].find("DEV") != std::string::npos) {
        dev_->setGain(SOAPY_SDR_TX, ch, "PAD", txgain);
      } else {
        dev_->setGain(
            SOAPY_SDR_TX, ch, "PAD",
            std::min(42.0, txgain));  // w/CBRS 3.6GHz [0:105], 2.5GHZ [0:105]
      }
      dev_->setGain(SOAPY_SDR_TX, ch, "IAMP", 0);
    }
  } else {
    dev_->setGain(SOAPY_SDR_RX, ch, "PGA0", std::min(31.5, rxgain));
    dev_->setGain(SOAPY_SDR_TX, ch, "PGA0", std::min(31.5, txgain));
  }

  // DC Offset for Iris
  if (!isUhd()) {
    dev_->setDCOffsetMode(SOAPY_SDR_RX, ch, true);
    dev_->writeSetting("RESET_DATA_LOGIC", "");
  }
}


void RadioSoapy::drain_buffers(std::vector<void*> buffs, int symSamp) {
  /*
     *  "Drain" rx buffers during initialization
     *  Input:
     *      buffs   - Vector to which we will write received IQ samples
     *      symSamp - Number of samples
     *
     *  Output:
     *      None
     */
  long long frameTime = 0;
  int flags = 0, r = 0;
  [[maybe_unused]] int i = 0;
  while (r != -1) {
    r = dev_->readStream(rxs_, buffs.data(), symSamp, flags, frameTime, 0);
    i++;
  }
  MLPD_TRACE("Number of reads needed to drain: %d\n", i);
}


RadioSoapy::RadioSoapy(const RadioParams& params, Type type, const SoapySDR::Kwargs& args,
                       const SoapySDR::Kwargs& rxStreamArgs,
                       const SoapySDR::Kwargs& txStreamArgs, double preStreamRxRate,
                       double preStreamTxRate, double preStreamFreq,
                       bool houdini_streams,
                       const std::function<void(SoapySDR::Device&)>& preStream)
    : Radio(params), type_(type) {
  const char* soapyFmt = SOAPY_SDR_CS16;
  // TX and RX may use different channel sets (see RadioParams). Rate/NCO are set
  // per direction on that direction's channels; streams open per direction too.
  const std::vector<size_t>& tx_channels = params.tx_channels;
  const std::vector<size_t>& rx_channels = params.rx_channels;
  const double rxFreqOffset = params.rx_freq_offset_hz;
  const double txFreqOffset = params.tx_freq_offset_hz;
  dev_ = SoapySDR::Device::make(args);
  if (dev_ == nullptr) {
    throw std::invalid_argument("error making SoapySDR::Device\n");
  }

  /* Moved to dev_init function (seems to fix the rate issue)
    for (auto ch : channels) {
        dev_->setSampleRate(SOAPY_SDR_RX, ch, rate);
        dev_->setSampleRate(SOAPY_SDR_TX, ch, rate);
    }*/
  // Backends that forbid live rate changes (Houdini) must have the rate and
  // NCO set BEFORE the stream opens; the Iris path passes 0 and keeps setting
  // these in dev_init (post-setupStream) as before. RX/TX rates are independent;
  // a negative preStreamTxRate is a sentinel for "use the device max TX rate"
  // (the replay RAM plays at that rate and the RFDC interpolates to the DAC) --
  // but the BS beacon now passes the app rate, so no host upsampling is needed.
  // A backend that owns its whole pre-stream configuration (Houdini mode V,
  // AP-79) runs it here, between make() and the first setupStream, in place
  // of the one-rate, one-NCO block below. A throw releases the device first,
  // for the same reason the stream setup below does (Opus review M12).
  if (preStream) {
    try {
      preStream(*dev_);
    } catch (...) {
      SoapySDR::Device::unmake(dev_);
      dev_ = nullptr;
      throw;
    }
  }
  if (!preStream && preStreamRxRate > 0.0) {
    for (auto ch : rx_channels) {
      dev_->setSampleRate(SOAPY_SDR_RX, ch, preStreamRxRate);
    }
  }
  if (!preStream && preStreamTxRate != 0.0) {
    double tx_rate = preStreamTxRate;
    if (tx_rate < 0.0) {  // sentinel: use the device max TX rate (replay)
      const auto tr = dev_->listSampleRates(
          SOAPY_SDR_TX, tx_channels.empty() ? 0 : tx_channels.front());
      tx_rate = tr.empty() ? 0.0 : *std::max_element(tr.begin(), tr.end());
    }
    if (tx_rate > 0.0) {
      for (auto ch : tx_channels) {
        dev_->setSampleRate(SOAPY_SDR_TX, ch, tx_rate);
      }
    }
  }
  if (!preStream && preStreamFreq > 0.0) {
    // rx/txFreqOffset DELIBERATELY detune this radio to inject a known carrier
    // offset (AP-33/AP-34 validation): with both boards on the shared 10 MHz
    // reference there is no natural CFO, so the only way to confirm the beacon
    // estimator's SIGN and SCALE is to impose one. Detuning RX alone gives pure
    // carrier offset on the received beacon with ZERO sample-timing drift,
    // which is exactly the term estimateCFO() should report and nothing else.
    // Detuning TX instead moves what the BS sees. Normally both are 0.
    const double rx_f = preStreamFreq + rxFreqOffset;
    const double tx_f = preStreamFreq + txFreqOffset;
    if (rxFreqOffset != 0.0 || txFreqOffset != 0.0) {
      MLPD_WARN(
          "Radio: DELIBERATE frequency offset in effect -- RX %+.1f Hz, TX "
          "%+.1f Hz off the %.6f MHz NCO. This is a test injection; results "
          "are NOT nominal.\n",
          rxFreqOffset, txFreqOffset, preStreamFreq / 1e6);
    }
    for (auto ch : rx_channels) {
      dev_->setFrequency(SOAPY_SDR_RX, ch, rx_f);
    }
    for (auto ch : tx_channels) {
      dev_->setFrequency(SOAPY_SDR_TX, ch, tx_f);
    }
    // Read the NCO BACK. setFrequency returning is not evidence the hardware
    // holds it, and a later stream-setup call could quietly restore nominal --
    // which would make an injection experiment silently measure nothing and
    // publish the null as a result (the SH-338 class).
    if (rxFreqOffset != 0.0 || txFreqOffset != 0.0) {
      for (auto ch : rx_channels) {
        MLPD_WARN("Radio: NCO readback RX ch%zu -- %.3f Hz (wanted %.3f)\n", ch,
                  dev_->getFrequency(SOAPY_SDR_RX, ch), rx_f);
      }
      for (auto ch : tx_channels) {
        MLPD_WARN("Radio: NCO readback TX ch%zu -- %.3f Hz (wanted %.3f)\n", ch,
                  dev_->getFrequency(SOAPY_SDR_TX, ch), tx_f);
      }
    }

  }
  // Houdini SoapyHoudiniSDR needs per-stream args (RX host port, TX replay/stream
  // mode) and its stream ORDER; Iris/UHD ignore an empty Kwargs.
  if (houdini_streams) {
    // MTS group order (AP-23, fail-loud contract): DAC tile 0 must be a
    // MEMBER of the group (PG269 -- it hosts the analog SYSREF receiver and
    // is the DAC RefTile), so when the data TX channel is not ch0, open a
    // never-activated ch0 replay stream first purely for membership; then
    // the data TX stream; then RX (whose setup runs the accumulated sync).
    const bool want_mts = txStreamArgs.count("mts") != 0u &&
                          txStreamArgs.at("mts") == "true";
    const bool has_ch0 =
        std::find(tx_channels.begin(), tx_channels.end(), 0u) != tx_channels.end();
    // A throw from any setup below escapes the constructor, so ~Radio never
    // runs: release what this ctor already owns (the aux stream, then the
    // device) before rethrowing, or the in-process radio-open retry finds
    // the device still held by a half-built attempt (Opus review M12).
    try {
      // The ADC half of the MTS rule (software lane, M2): the group needs an
      // RX member on ADC tile 0. The planned nodes have one (RX ch0); a node
      // that omits it is refused here, naming the fix, rather than left to a
      // sync that fails or lands unsynced. Skipped when the driver does not
      // report tiles.
      if (want_mts && !rx_channels.empty()) {
        bool tile0 = false, reported = false;
        for (auto ch : rx_channels) {
          const auto info = dev_->getChannelInfo(SOAPY_SDR_RX, ch);
          const auto it = info.find("rfdc_tile_index");
          if (it == info.end()) continue;
          reported = true;
          tile0 = tile0 || it->second == "0";
        }
        if (reported && !tile0) {
          throw std::invalid_argument(
              "MTS needs an RX channel on ADC tile 0 (channel A or B); add one "
              "to rx_channel / ue_rx_channel");
        }
      }
      if (want_mts && !has_ch0) {
        SoapySDR::Kwargs aux;
        aux["tx_mode"] = "replay";
        aux["mts"] = "true";
        aux_mts_txs_ = dev_->setupStream(SOAPY_SDR_TX, soapyFmt, {0}, aux);
      }
      // SH-235: the Houdini driver rejects a multi-channel TX stream on both
      // modes (replay beacon and live pilot). Open one single-channel TX stream
      // per channel; xmit routes each channel's buffer to its own stream.
      for (auto ch : tx_channels) {
        tx_streams_.push_back(
            dev_->setupStream(SOAPY_SDR_TX, soapyFmt, {ch}, txStreamArgs));
      }
      // One combined RX stream over the RX channels. Since SH-142/SH-159 landed
      // the driver activates a >1-channel RX stream and readStream fills buffs[i]
      // per channel, sample-aligned with one timestamp -- the same shape Iris
      // uses. RX channels may differ from TX (e.g. an RX-only converter).
      rxs_ = dev_->setupStream(SOAPY_SDR_RX, soapyFmt, rx_channels, rxStreamArgs);
    } catch (...) {
      for (auto* s : tx_streams_)
        if (s != nullptr) dev_->closeStream(s);
      tx_streams_.clear();
      if (rxs_ != nullptr) dev_->closeStream(rxs_);
      rxs_ = nullptr;
      if (aux_mts_txs_ != nullptr) dev_->closeStream(aux_mts_txs_);
      SoapySDR::Device::unmake(dev_);
      throw;
    }
  } else {
    // Iris/UHD: one multi-channel RX stream; one multi-channel TX stream.
    rxs_ = dev_->setupStream(SOAPY_SDR_RX, soapyFmt, rx_channels, rxStreamArgs);
    tx_streams_.push_back(
        dev_->setupStream(SOAPY_SDR_TX, soapyFmt, tx_channels, txStreamArgs));
  }

  const std::string driver =
      (args.count("driver") != 0u) ? args.at("driver") : std::string();
  num_rx_ch_ = rx_channels.empty() ? 1 : rx_channels.size();

  // RESET_DATA_LOGIC is an Iris-only setting; Houdini/UHD don't implement it.
  if (!isUhd() && driver == "iris") {
    reset_DATA_clk_domain();
  }
}


RadioSoapy::RadioSoapy(const RadioParams& params, Type type)
    : RadioSoapy(params, type, [&params, type] {
        SoapySDR::Kwargs args;
        if (type == Type::kSoapyUhd) {
          args["driver"] = "uhd";
          args["addr"] = params.id;
          std::cout << "Init radio (uhd): " << args["addr"] << std::endl;
        } else {
          args["driver"] = "iris";
          args["serial"] = params.id;
        }
        args["timeout"] = params.timeout;
        return args;
      }(),
      SoapySDR::Kwargs(), SoapySDR::Kwargs(), 0.0, 0.0, 0.0, false) {}

bool RadioSoapy::hasHardwareTrigger() const { return !isUhd(); }
bool RadioSoapy::hasAgc() const { return !isUhd(); }

long long RadioSoapy::txTimeNs(long long frame_ticks, double rate_hz, bool /*tdd_pilot*/,
                               long long /*advance_ticks*/) const {
  return SoapySDR::ticksToTimeNs(frame_ticks, rate_hz);
}

void RadioSoapy::printSettings() const {
  // Iris/UHD diagnostics (Houdini overrides this); TX and RX use the same set
  // there, so the RX channel list covers both directions' reports.
  const auto& channels = params_.rx_channels;
  std::cout << params_.label << ": Front end " << dev_->getHardwareInfo()["frontend"]
            << std::endl;
  for (auto ch : channels) {
    if (ch < dev_->getNumChannels(SOAPY_SDR_RX)) {
      printf("RX Channel %zu\n", ch);
      printf("Actual RX sample rate: %fMSps...\n", (dev_->getSampleRate(SOAPY_SDR_RX, ch) / 1e6));
      printf("Actual RX frequency: %fGHz...\n", (dev_->getFrequency(SOAPY_SDR_RX, ch) / 1e9));
      printf("Actual RX gain: %f...\n", (dev_->getGain(SOAPY_SDR_RX, ch)));
      if (!isUhd()) {
        printf("Actual RX LNA gain: %f...\n", (dev_->getGain(SOAPY_SDR_RX, ch, "LNA")));
        printf("Actual RX PGA gain: %f...\n", (dev_->getGain(SOAPY_SDR_RX, ch, "PGA")));
        printf("Actual RX TIA gain: %f...\n", (dev_->getGain(SOAPY_SDR_RX, ch, "TIA")));
        if (dev_->getHardwareInfo()["frontend"].find("CBRS") != std::string::npos) {
          printf("Actual RX LNA1 gain: %f...\n", (dev_->getGain(SOAPY_SDR_RX, ch, "LNA1")));
          printf("Actual RX LNA2 gain: %f...\n", (dev_->getGain(SOAPY_SDR_RX, ch, "LNA2")));
        }
      }
      printf("Actual RX bandwidth: %fM...\n", (dev_->getBandwidth(SOAPY_SDR_RX, ch) / 1e6));
      printf("Actual RX antenna: %s...\n", (dev_->getAntenna(SOAPY_SDR_RX, ch).c_str()));
    }
  }
  for (auto ch : channels) {
    if (ch < dev_->getNumChannels(SOAPY_SDR_TX)) {
      printf("TX Channel %zu\n", ch);
      printf("Actual TX sample rate: %fMSps...\n", (dev_->getSampleRate(SOAPY_SDR_TX, ch) / 1e6));
      printf("Actual TX frequency: %fGHz...\n", (dev_->getFrequency(SOAPY_SDR_TX, ch) / 1e9));
      printf("Actual TX gain: %f...\n", (dev_->getGain(SOAPY_SDR_TX, ch)));
      if (!isUhd()) {
        printf("Actual TX PAD gain: %f...\n", (dev_->getGain(SOAPY_SDR_TX, ch, "PAD")));
        printf("Actual TX IAMP gain: %f...\n", (dev_->getGain(SOAPY_SDR_TX, ch, "IAMP")));
        if (dev_->getHardwareInfo()["frontend"].find("CBRS") != std::string::npos) {
          printf("Actual TX PA1 gain: %f...\n", (dev_->getGain(SOAPY_SDR_TX, ch, "PA1")));
          printf("Actual TX PA2 gain: %f...\n", (dev_->getGain(SOAPY_SDR_TX, ch, "PA2")));
          printf("Actual TX PA3 gain: %f...\n", (dev_->getGain(SOAPY_SDR_TX, ch, "PA3")));
        }
      }
      printf("Actual TX bandwidth: %fM...\n", (dev_->getBandwidth(SOAPY_SDR_TX, ch) / 1e6));
      printf("Actual TX antenna: %s...\n", (dev_->getAntenna(SOAPY_SDR_TX, ch).c_str()));
    }
  }
  std::cout << std::endl;
}

RadioSoapy::~RadioSoapy() {
  // Qualified: a derived class's overrides are gone by the time a base
  // destructor runs, so say which versions are meant.
  RadioSoapy::deactivateRecv();
  RadioSoapy::deactivateXmit();
  if (aux_mts_txs_ != nullptr) {
    dev_->closeStream(aux_mts_txs_);
    aux_mts_txs_ = nullptr;
  }
  dev_->closeStream(rxs_);
  rxs_ = nullptr;
  for (auto* s : tx_streams_)
    if (s != nullptr) dev_->closeStream(s);
  tx_streams_.clear();
  SoapySDR::Device::unmake(dev_);
  dev_ = nullptr;
}


int RadioSoapy::drainTxStatus() {
  if (tx_status_unsupported_ || tx_streams_.empty()) return 0;
  int problems = 0;
  // Poll every TX stream: per-channel Houdini streams each carry their own
  // status queue, so draining only one would miss a late/dropped burst on the
  // others. Each stream keeps the original 32-read bound.
  for (size_t si = 0; si < tx_streams_.size(); ++si) {
    auto* txs = tx_streams_[si];
    if (txs == nullptr) continue;
    for (int i = 0; i < 32; ++i) {  // bounded so a hot queue cannot stall the caller
      size_t chan_mask = 0;
      int flags = 0;
      long long t = 0;
      const int st = dev_->readStreamStatus(txs, chan_mask, flags, t, 0);
      if (st == SOAPY_SDR_TIMEOUT) break;  // nothing queued: the normal case
      if (st == SOAPY_SDR_NOT_SUPPORTED) {
        tx_status_unsupported_ = true;
        return problems;  // the surface is absent device-wide, stop asking
      }
      if (st == 0) continue;  // a benign event (e.g. an end-of-burst ack)
      ++problems;
      ++tx_status_events_;
      const long long now =
          std::chrono::duration_cast<std::chrono::nanoseconds>(
              std::chrono::steady_clock::now().time_since_epoch())
              .count();
      if (now - tx_status_log_ns_ > 5000000000LL) {  // at most one line per 5 s
        tx_status_log_ns_ = now;
        // AP-78 forensics: attribute the event to its per-channel stream and
        // dump the driver's per-bank counters (late/under/drops/played), which
        // discriminate pacing (late) from starvation (under) from overfeeding
        // (drops) -- the aggregate text alone cannot. TX_BANK_STATUS carries the
        // real totals even when a readStreamStatus counter saturates at 0xFFFF.
        std::string bank;
        try {
          bank = dev_->readSetting("TX_BANK_STATUS");
        } catch (...) {
          bank = "<readSetting failed>";
        }
        MLPD_WARN(
            "TX status: %zu problem event(s), latest %s (code %d) on "
            "tx_stream[%zu] at %lld ns. TX_BANK_STATUS=%s\n",
            tx_status_events_, SoapySDR::errToStr(st), st, si, t, bank.c_str());
      }
    }
  }
  return problems;
}


int RadioSoapy::recv(void* const* buffs, int samples, long long& frameTime) {
  int flags(0);
  int r = dev_->readStream(rxs_, buffs, samples, flags, frameTime, 1000000);
  if (r < 0) {
    MLPD_ERROR("Time: %lld, readStream error: %d - %s, flags: %d\n", frameTime,
               r, SoapySDR::errToStr(r), flags);
    MLPD_TRACE("Samples: %d, Frame time: %lld\n", samples, frameTime);
  } else if (r < samples) {
    MLPD_WARN(
        "Time: %lld, readStream returned less than requested "
        "samples: %d : %d, flags: %d\n",
        frameTime, r, samples, flags);
  }

  return r;
}


int RadioSoapy::activateRecv(long long rxTime, size_t numSamps, int flags) {
  int soapyFlags[] = {0, SOAPY_SDR_HAS_TIME,
                      SOAPY_SDR_HAS_TIME | SOAPY_SDR_END_BURST,
                      SOAPY_SDR_WAIT_TRIGGER | SOAPY_SDR_END_BURST};
  int flag_args = soapyFlags[flags];
  // for USRP device start rx stream UHD_INIT_TIME_SEC sec in the future
  if (!isUhd()) {
    return dev_->activateStream(rxs_, flag_args, rxTime, numSamps);
  } else {
    return dev_->activateStream(rxs_, SOAPY_SDR_HAS_TIME,
                                UHD_INIT_TIME_SEC * 1e9, 0);
  }
}

void RadioSoapy::deactivateRecv(void) { dev_->deactivateStream(rxs_); }

int RadioSoapy::xmit(const void* const* buffs, int samples, int flags,
                long long& frameTime) {
  int soapyFlags[] = {0, SOAPY_SDR_HAS_TIME,
                      SOAPY_SDR_HAS_TIME | SOAPY_SDR_END_BURST,
                      SOAPY_SDR_WAIT_TRIGGER | SOAPY_SDR_END_BURST};
  int flag_args = soapyFlags[flags];
  if (tx_streams_.empty()) return 0;
  // One multi-channel stream (Iris/UHD, or a single channel): write the whole
  // per-channel buffer array in one call, exactly as before.
  if (tx_streams_.size() == 1) {
    int r = dev_->writeStream(tx_streams_.front(), buffs, samples, flag_args,
                              frameTime, 1000000);
    if (r != samples) {
      std::cerr << "unexpected writeStream error " << SoapySDR::errToStr(r)
                << std::endl;
    }
    return r;
  }
  // Houdini per-channel streams (SH-235): buffs[i] belongs to channel i, so
  // write each to its own single-channel stream at the SAME timed start. Both
  // channels share the board's clock, so one frameTime seats them on the same
  // TDD grid. Return the first short/failed write so the caller's BAD-Write
  // check still fires.
  int ret = samples;
  // AP-78 diag: the second-written per-channel stream's burst can miss its tick
  // (arrives late -> the bank never starts -> zero-fill), while the first-written
  // one makes the deadline. HOUDINI_TX_REVERSE flips the write order so we can
  // tell an order-dependent margin (the dead lane follows the order) from a
  // stream-specific fault (the dead lane stays put).
  static const bool tx_reverse = getenv("HOUDINI_TX_REVERSE") != nullptr;
  const size_t nstreams = tx_streams_.size();
  // AP-78 step-2 diag: the real per-burst host lead is frameTime - device clock
  // at the moment of the write. One getHardwareTime RPC per xmit (env-gated,
  // throttled) -- both streams write within ~us of this, so it is the base lead
  // they share; if it is under the driver's ~500 us threshold the second write
  // misses. Not in the shipped loop (the RPC is ~0.1-1.5 ms).
  static const bool margin_log = getenv("HOUDINI_TX_MARGIN") != nullptr;
  if (margin_log) {
    static long long last_margin_ns = 0;
    const long long nowns =
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch())
            .count();
    if (nowns - last_margin_ns > 2000000000LL) {
      last_margin_ns = nowns;
      long long hw = -1;
      try {
        hw = dev_->getHardwareTime();
      } catch (...) {
      }
      MLPD_WARN(
          "TX margin: frameTime=%lld ns hw_time=%lld ns lead=%.1f us "
          "(nstreams=%zu)\n",
          frameTime, hw, (hw >= 0 ? (frameTime - hw) / 1000.0 : 0.0), nstreams);
    }
  }
  for (size_t k = 0; k < nstreams; ++k) {
    const size_t i = tx_reverse ? (nstreams - 1 - k) : k;
    // A null channel buffer means "nothing on this channel this write" -- the BS
    // beacon is single-antenna, so it passes its samples only on the beacon
    // channel and nullptr on the others; fanning it to every channel would fill
    // (and, if armed, be refused on) a channel that is not the beacon's.
    if (buffs[i] == nullptr) continue;
    long long ft = frameTime;  // writeStream may advance its copy; keep ours
    // writeStream takes flags by REFERENCE and CLEARS the consumed bits
    // (HAS_TIME/END_BURST) in place. flag_args must therefore be copied PER
    // STREAM, exactly like ft above -- otherwise the first stream's write zeroes
    // the flags and every later stream is written with 0x0 (no HAS_TIME), so its
    // burst is never anchored to its tick, the bank never activates, and it
    // zero-fills. That was the whole dead-second-antenna bug (AP-78): the driver
    // DIAG showed ch0 flags=0x6 but ch1 flags=0x0 for the same pilot.
    int fl = flag_args;
    const void* one[1] = {buffs[i]};
    int r = dev_->writeStream(tx_streams_[i], one, samples, fl, ft, 1000000);
    if (r != samples) {
      std::cerr << "unexpected writeStream error (ch " << i << ") "
                << SoapySDR::errToStr(r) << std::endl;
      if (ret == samples) ret = r;
    }
  }
  return ret;
}

void RadioSoapy::activateXmit(void) {
  // for USRP device start tx stream UHD_INIT_TIME_SEC sec in the future
  for (auto* txs : tx_streams_) {
    if (!isUhd()) {
      dev_->activateStream(txs);
    } else {
      dev_->activateStream(txs, SOAPY_SDR_HAS_TIME, UHD_INIT_TIME_SEC * 1e9, 0);
    }
  }
}

void RadioSoapy::deactivateXmit(void) {
  for (auto* txs : tx_streams_) dev_->deactivateStream(txs);
}

int RadioSoapy::getTriggers(void) const {
  return std::stoi(dev_->readSetting("TRIGGER_COUNT"));
}

void RadioSoapy::reset_DATA_clk_domain(void) {
  dev_->writeSetting("RESET_DATA_LOGIC", "");
}
