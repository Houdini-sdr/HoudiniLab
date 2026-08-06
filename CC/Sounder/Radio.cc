/** @file Radio.cc
  * @brief Defination file for the Radio class.
  *
  * Copyright (c) 2018-2022, Rice University
  * RENEW OPEN SOURCE LICENSE: http://renew-wireless.org/license
  * ----------------------------------------------------------
  * Initialize and Configure an SDR
  * ----------------------------------------------------------
*/
#include "include/Radio.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <vector>

#include "SoapySDR/Errors.hpp"
#include "include/logger.h"
#include "include/macros.h"

void Radio::dev_init(Config* _cfg, int ch, double rxgain, double txgain) {
  SoapySDR::Kwargs info = dev_->getHardwareInfo();

  // Houdini RFSoC: the mixer NCO is the only tuning knob and there is no
  // antenna/analog-bandwidth/gain/DC-offset stage to program. Rate + NCO were
  // already applied in the ctor (before setupStream, since Houdini forbids a
  // live rate change), so nothing to do here but report.
  if (_cfg->is_houdini()) {
    MLPD_INFO("Houdini channel %d: rate %.2f MSPS, NCO %.2f MHz\n", ch,
              dev_->getSampleRate(SOAPY_SDR_RX, ch) / 1e6,
              dev_->getFrequency(SOAPY_SDR_RX, ch) / 1e6);
    (void)rxgain;
    (void)txgain;
    return;
  }

  dev_->setSampleRate(SOAPY_SDR_RX, ch, _cfg->rate());
  dev_->setSampleRate(SOAPY_SDR_TX, ch, _cfg->rate());

  // these params are sufficient to set before DC offset and IQ imbalance calibration
  if (!kUseSoapyUHD) {
    dev_->setAntenna(SOAPY_SDR_RX, ch, "TRX");
    dev_->setBandwidth(SOAPY_SDR_RX, ch, _cfg->bw_filter());
    dev_->setBandwidth(SOAPY_SDR_TX, ch, _cfg->bw_filter());
    dev_->setFrequency(SOAPY_SDR_RX, ch, "BB", _cfg->nco());
    dev_->setFrequency(SOAPY_SDR_TX, ch, "BB", _cfg->nco());
  } else {
    MLPD_INFO("Init USRP channel: %d\n", ch);
    dev_->setAntenna(SOAPY_SDR_TX, ch, "TX/RX");
    dev_->setAntenna(SOAPY_SDR_RX, ch, "RX2");  // or "TX/RX"
    dev_->setFrequency(SOAPY_SDR_RX, ch, "BB", 0);
    dev_->setFrequency(SOAPY_SDR_TX, ch, "BB", 0);
  }

  dev_->setFrequency(SOAPY_SDR_RX, ch, "RF", _cfg->radio_rf_freq());
  dev_->setFrequency(SOAPY_SDR_TX, ch, "RF", _cfg->radio_rf_freq());
  if (kUseSoapyUHD == false) {
    // Unified gains for both lime and frontend
    if (_cfg->single_gain()) {
      dev_->setGain(SOAPY_SDR_RX, ch,
                    rxgain);  // w/CBRS 3.6GHz [0:105], 2.5GHZ [0:108]
      dev_->setGain(SOAPY_SDR_TX, ch,
                    txgain);  // w/CBRS 3.6GHz [0:105], 2.5GHZ [0:105]
      MLPD_INFO("Tx gain: %lf, Rx gain: %lf\n", dev_->getGain(SOAPY_SDR_TX, ch),
                dev_->getGain(SOAPY_SDR_RX, ch));
    } else {
      if (info["frontend"].find("CBRS") != std::string::npos) {
        if (_cfg->radio_rf_freq() > 3e9) {
          dev_->setGain(SOAPY_SDR_RX, ch, "ATTN", -6);  //[-18,0]
        } else if ((_cfg->radio_rf_freq() > 2e9) &&
                   (_cfg->radio_rf_freq() < 3e9)) {
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
  if (!kUseSoapyUHD) {
    dev_->setDCOffsetMode(SOAPY_SDR_RX, ch, true);
    dev_->writeSetting("RESET_DATA_LOGIC", "");
  }
}

void Radio::drain_buffers(std::vector<void*> buffs, int symSamp) {
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
  int flags = 0, r = 0, i = 0;
  while (r != -1) {
    r = dev_->readStream(rxs_, buffs.data(), symSamp, flags, frameTime, 0);
    i++;
  }
  MLPD_TRACE("Number of reads needed to drain: %d\n", i);
}

Radio::Radio(const SoapySDR::Kwargs& args, const char soapyFmt[],
             const std::vector<size_t>& channels,
             const SoapySDR::Kwargs& rxStreamArgs,
             const SoapySDR::Kwargs& txStreamArgs, double preStreamRxRate,
             double preStreamTxRate, double preStreamFreq) {
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
    for (auto ch : channels) {
      dev_->setFrequency(SOAPY_SDR_RX, ch, preStreamFreq);
      dev_->setFrequency(SOAPY_SDR_TX, ch, preStreamFreq);
    }
  }
  // Houdini SoapyHoudiniSDR needs per-stream args (RX host port, TX replay/stream
  // mode); Iris/UHD ignore an empty Kwargs, so this is backend-agnostic.
  rxs_ = dev_->setupStream(SOAPY_SDR_RX, soapyFmt, channels, rxStreamArgs);
  txs_ = dev_->setupStream(SOAPY_SDR_TX, soapyFmt, channels, txStreamArgs);

  const std::string driver =
      (args.count("driver") != 0u) ? args.at("driver") : std::string();
  houdini_ = (driver == "houdinisdr");
  num_rx_ch_ = channels.empty() ? 1 : channels.size();

  // RESET_DATA_LOGIC is an Iris-only setting; Houdini/UHD don't implement it.
  if (!kUseSoapyUHD && driver == "iris") {
    reset_DATA_clk_domain();
  }
}

Radio::~Radio(void) {
  deactivateRecv();
  deactivateXmit();
  dev_->closeStream(rxs_);
  rxs_ = nullptr;
  dev_->closeStream(txs_);
  txs_ = nullptr;
  SoapySDR::Device::unmake(dev_);
  dev_ = nullptr;
}

// SoapyHoudiniSDR delivers ~1 MTU (~1016 samples) per readStream and lets the
// host socket buffer a backlog while the caller is busy (e.g. running
// find_beacon between windows), so a single readStream can neither fill a
// multi-thousand-sample sync window nor guarantee it is contiguous. Drain any
// stale backlog non-blocking, then accumulate a fresh, contiguous window --
// this is the in-radio equivalent of the client_sync_cuda drain-before-frame.
int Radio::recvHoudini(void* const* buffs, int samples, long long& frameTime) {
  constexpr size_t kBytesPerSamp = 4;  // CS16 = 2 x int16
  static thread_local std::vector<uint8_t> junk;
  const size_t drain_samps = 16384;
  if (junk.size() < drain_samps * kBytesPerSamp * num_rx_ch_)
    junk.resize(drain_samps * kBytesPerSamp * num_rx_ch_);
  std::vector<void*> jb(num_rx_ch_);
  for (size_t c = 0; c < num_rx_ch_; c++)
    jb[c] = junk.data() + c * drain_samps * kBytesPerSamp;
  int jf = 0;
  long long jt = 0;
  while (dev_->readStream(rxs_, jb.data(), drain_samps, jf, jt, 0) > 0) {
  }

  std::vector<void*> cur(num_rx_ch_);
  for (size_t c = 0; c < num_rx_ch_; c++) cur[c] = buffs[c];
  int got = 0;
  while (got < samples) {
    int flags = 0;
    long long t = 0;
    int r =
        dev_->readStream(rxs_, cur.data(), samples - got, flags, t, 1000000);
    if (r <= 0) {
      if (got == 0) return r;
      break;
    }
    if (got == 0) frameTime = t;
    got += r;
    for (size_t c = 0; c < num_rx_ch_; c++)
      cur[c] = static_cast<uint8_t*>(cur[c]) + r * kBytesPerSamp;
  }
  if ((getenv("HOUDINI_CL_RX_DEBUG") != nullptr ||
       getenv("HOUDINI_DUMP_WIN") != nullptr) &&
      got > 0 && buffs[0] != nullptr) {
    const int16_t* p = static_cast<const int16_t*>(buffs[0]);
    double s = 0;
    int amax = 0;
    for (int k = 0; k < got * 2; ++k) {
      s += double(p[k]) * p[k];
      amax = std::max(amax, std::abs((int)p[k]));
    }
    const double rms = std::sqrt(s / (got * 2));
    if (getenv("HOUDINI_CL_RX_DEBUG") != nullptr) {
      static std::atomic<int> cnt{0};
      if ((cnt.fetch_add(1) % 40) == 0)
        MLPD_INFO("Houdini client RX dbg: got=%d rms=%.2f absmax=%d\n", got,
                  rms, amax);
    }
    // Dump the first strong (beacon-present) window for offline correlation.
    if (getenv("HOUDINI_DUMP_WIN") != nullptr && rms > 100.0) {
      static std::atomic<bool> done{false};
      bool expected = false;
      if (done.compare_exchange_strong(expected, true)) {
        FILE* f = std::fopen("/tmp/cl_win.bin", "wb");
        if (f) {
          std::fwrite(p, sizeof(int16_t), static_cast<size_t>(got) * 2, f);
          std::fclose(f);
          MLPD_INFO("Dumped client beacon window rms=%.1f got=%d -> /tmp/cl_win.bin\n",
                    rms, got);
        }
      }
    }
  }
  return got;
}

int Radio::recvTddWindow(void* const* buffs, int samples, long long start_ns) {
  // The Houdini TDD framer gates the ADC to the scheduled rx_gate symbol; a
  // HAS_TIME activate arms a finite burst of `samples` at that symbol's tick.
  // Read one packet per call and STOP on END_BURST -- the burst may end a few
  // samples short of the request, and waiting the full timeout for the missing
  // tail would stall ~1 s per window (mirrors the working run_burst read loop).
  constexpr size_t kBytesPerSamp = 4;  // CS16
  const int flags = SOAPY_SDR_HAS_TIME | SOAPY_SDR_END_BURST;
  const bool dbg = std::getenv("HOUDINI_TDD_TIMING") != nullptr;
  using clk = std::chrono::steady_clock;
  auto us = [](clk::duration d) {
    return std::chrono::duration_cast<std::chrono::microseconds>(d).count();
  };
  const auto t0 = clk::now();
  int rc = dev_->activateStream(rxs_, flags, start_ns,
                                static_cast<size_t>(samples));
  const auto t_act = clk::now();
  if (rc != 0) {
    MLPD_ERROR("recvTddWindow: activateStream rc=%d (%s)\n", rc,
               SoapySDR::errToStr(rc));
    dev_->deactivateStream(rxs_);
    return rc;
  }
  std::vector<void*> cur(num_rx_ch_);
  for (size_t c = 0; c < num_rx_ch_; c++) cur[c] = buffs[c];
  int got = 0;
  int idle = 0;
  int nreads = 0;
  long long first_read_us = -1;
  while (got < samples) {
    int f = 0;
    long long t = 0;
    const auto tr = clk::now();
    int r = dev_->readStream(rxs_, cur.data(), samples - got, f, t, 200000);
    ++nreads;
    if (first_read_us < 0) first_read_us = us(clk::now() - tr);
    if (r > 0) {
      got += r;
      for (size_t c = 0; c < num_rx_ch_; c++)
        cur[c] = static_cast<uint8_t*>(cur[c]) + r * kBytesPerSamp;
      if ((f & SOAPY_SDR_END_BURST) != 0) break;  // burst done
      idle = 0;
    } else if (r == SOAPY_SDR_TIMEOUT) {
      if (++idle >= 3) break;  // no data for ~0.6 s -> give up on this window
    } else {
      break;  // hard error
    }
  }
  const auto t_read = clk::now();
  dev_->deactivateStream(rxs_);
  const auto t_deact = clk::now();
  if (dbg) {
    MLPD_INFO("recvTddWindow timing: activate=%lldus firstRead=%lldus "
              "readLoop=%lldus deactivate=%lldus total=%lldus got=%d/%d nreads=%d\n",
              us(t_act - t0), first_read_us, us(t_read - t_act),
              us(t_deact - t_read), us(t_deact - t0), got, samples, nreads);
  }
  return got;
}

int Radio::recv(void* const* buffs, int samples, long long& frameTime) {
  if (houdini_) return recvHoudini(buffs, samples, frameTime);
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

int Radio::activateRecv(const long long rxTime, const size_t numSamps,
                        int flags) {
  int soapyFlags[] = {0, SOAPY_SDR_HAS_TIME,
                      SOAPY_SDR_HAS_TIME | SOAPY_SDR_END_BURST,
                      SOAPY_SDR_WAIT_TRIGGER | SOAPY_SDR_END_BURST};
  int flag_args = soapyFlags[flags];
  // for USRP device start rx stream UHD_INIT_TIME_SEC sec in the future
  if (!kUseSoapyUHD) {
    return dev_->activateStream(rxs_, flag_args, rxTime, numSamps);
  } else {
    return dev_->activateStream(rxs_, SOAPY_SDR_HAS_TIME,
                                UHD_INIT_TIME_SEC * 1e9, 0);
  }
}

void Radio::deactivateRecv(void) { dev_->deactivateStream(rxs_); }

int Radio::xmit(const void* const* buffs, int samples, int flags,
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

void Radio::activateXmit(void) {
  // for USRP device start tx stream UHD_INIT_TIME_SEC sec in the future
  if (!kUseSoapyUHD) {
    dev_->activateStream(txs_);
  } else {
    dev_->activateStream(txs_, SOAPY_SDR_HAS_TIME, UHD_INIT_TIME_SEC * 1e9, 0);
  }
}

void Radio::deactivateXmit(void) { dev_->deactivateStream(txs_); }

int Radio::getTriggers(void) const {
  return std::stoi(dev_->readSetting("TRIGGER_COUNT"));
}

void Radio::reset_DATA_clk_domain(void) {
  dev_->writeSetting("RESET_DATA_LOGIC", "");
}
