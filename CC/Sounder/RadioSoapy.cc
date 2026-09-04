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
                       bool houdini_streams)
    : Radio(params), type_(type) {
  const char* soapyFmt = SOAPY_SDR_CS16;
  const std::vector<size_t>& channels = params.channels;
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
  if (preStreamRxRate > 0.0) {
    for (auto ch : channels) {
      dev_->setSampleRate(SOAPY_SDR_RX, ch, preStreamRxRate);
    }
  }
  if (preStreamTxRate != 0.0) {
    double tx_rate = preStreamTxRate;
    if (tx_rate < 0.0) {  // sentinel: use the device max TX rate (replay)
      const auto tr = dev_->listSampleRates(
          SOAPY_SDR_TX, channels.empty() ? 0 : channels.front());
      tx_rate = tr.empty() ? 0.0 : *std::max_element(tr.begin(), tr.end());
    }
    if (tx_rate > 0.0) {
      for (auto ch : channels) {
        dev_->setSampleRate(SOAPY_SDR_TX, ch, tx_rate);
      }
    }
  }
  if (preStreamFreq > 0.0) {
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
    for (auto ch : channels) {
      dev_->setFrequency(SOAPY_SDR_RX, ch, rx_f);
      dev_->setFrequency(SOAPY_SDR_TX, ch, tx_f);
    }
    // Read the NCO BACK. setFrequency returning is not evidence the hardware
    // holds it, and a later stream-setup call could quietly restore nominal --
    // which would make an injection experiment silently measure nothing and
    // publish the null as a result (the SH-338 class).
    if (rxFreqOffset != 0.0 || txFreqOffset != 0.0) {
      for (auto ch : channels) {
        MLPD_WARN(
            "Radio: NCO readback ch%zu -- RX %.3f Hz, TX %.3f Hz (wanted "
            "%.3f / %.3f)\n",
            ch, dev_->getFrequency(SOAPY_SDR_RX, ch),
            dev_->getFrequency(SOAPY_SDR_TX, ch), rx_f, tx_f);
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
        std::find(channels.begin(), channels.end(), 0u) != channels.end();
    // A throw from any setup below escapes the constructor, so ~Radio never
    // runs: release what this ctor already owns (the aux stream, then the
    // device) before rethrowing, or the in-process radio-open retry finds
    // the device still held by a half-built attempt (Opus review M12).
    try {
      if (want_mts && !has_ch0) {
        SoapySDR::Kwargs aux;
        aux["tx_mode"] = "replay";
        aux["mts"] = "true";
        aux_mts_txs_ = dev_->setupStream(SOAPY_SDR_TX, soapyFmt, {0}, aux);
      }
      txs_ = dev_->setupStream(SOAPY_SDR_TX, soapyFmt, channels, txStreamArgs);
      rxs_ = dev_->setupStream(SOAPY_SDR_RX, soapyFmt, channels, rxStreamArgs);
    } catch (...) {
      if (txs_ != nullptr) dev_->closeStream(txs_);
      if (aux_mts_txs_ != nullptr) dev_->closeStream(aux_mts_txs_);
      SoapySDR::Device::unmake(dev_);
      throw;
    }
  } else {
    rxs_ = dev_->setupStream(SOAPY_SDR_RX, soapyFmt, channels, rxStreamArgs);
    txs_ = dev_->setupStream(SOAPY_SDR_TX, soapyFmt, channels, txStreamArgs);
  }

  const std::string driver =
      (args.count("driver") != 0u) ? args.at("driver") : std::string();
  num_rx_ch_ = channels.empty() ? 1 : channels.size();

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
  const auto& channels = params_.channels;
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
  dev_->closeStream(txs_);
  txs_ = nullptr;
  SoapySDR::Device::unmake(dev_);
  dev_ = nullptr;
}


int RadioSoapy::drainTxStatus() {
  if (tx_status_unsupported_ || txs_ == nullptr) return 0;
  int problems = 0;
  for (int i = 0; i < 32; ++i) {  // bounded so a hot queue cannot stall the caller
    size_t chan_mask = 0;
    int flags = 0;
    long long t = 0;
    const int st = dev_->readStreamStatus(txs_, chan_mask, flags, t, 0);
    if (st == SOAPY_SDR_TIMEOUT) break;  // nothing queued: the normal case
    if (st == SOAPY_SDR_NOT_SUPPORTED) {
      tx_status_unsupported_ = true;
      break;
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
      MLPD_WARN(
          "TX status: %zu problem event(s), latest %s at %lld ns. A burst the "
          "driver accepted was sent late or dropped; on the TDD grid that shows "
          "up as a phase jump, not as a write error.\n",
          tx_status_events_, SoapySDR::errToStr(st), t);
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
  int r =
      dev_->writeStream(txs_, buffs, samples, flag_args, frameTime, 1000000);
  if (r != samples) {
    std::cerr << "unexpected writeStream error " << SoapySDR::errToStr(r)
              << std::endl;
  }
  return (r);
}

void RadioSoapy::activateXmit(void) {
  // for USRP device start tx stream UHD_INIT_TIME_SEC sec in the future
  if (!isUhd()) {
    dev_->activateStream(txs_);
  } else {
    dev_->activateStream(txs_, SOAPY_SDR_HAS_TIME, UHD_INIT_TIME_SEC * 1e9, 0);
  }
}

void RadioSoapy::deactivateXmit(void) { dev_->deactivateStream(txs_); }

int RadioSoapy::getTriggers(void) const {
  return std::stoi(dev_->readSetting("TRIGGER_COUNT"));
}

void RadioSoapy::reset_DATA_clk_domain(void) {
  dev_->writeSetting("RESET_DATA_LOGIC", "");
}
