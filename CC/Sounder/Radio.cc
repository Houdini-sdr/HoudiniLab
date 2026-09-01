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
#include "include/rx_gap_sink.h"       // RxGapSink (UDP gap -> /Data/Gaps bridge)
#include "include/rx_recorder_grid.h"  // TimeGridTracker

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
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
  int flags = 0, r = 0;
  [[maybe_unused]] int i = 0;
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
             double preStreamTxRate, double preStreamFreq, double rxFreqOffset,
             double txFreqOffset) {
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
  }
  // Houdini SoapyHoudiniSDR needs per-stream args (RX host port, TX replay/stream
  // mode); Iris/UHD ignore an empty Kwargs, so this is backend-agnostic.
  if (args.count("driver") && args.at("driver") == "houdinisdr") {
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
  // Split drain from read: the loop spends 93% of an iteration inside the 30
  // radioRx calls it makes per frame (29 of them purely to throw the slot
  // away), at ~880 us each, and the fix differs depending on whether that cost
  // is the drain loop or the read itself. HOUDINI_LOOP_PROFILE reports both.
  // Its OWN knob: HOUDINI_LOOP_PROFILE counts loop ITERATIONS while this
  // counts radioRx CALLS, and coalescing changes the ratio between them from
  // ~30:1 to ~2:1. One shared setting would silently report two different
  // scales, which is a hazard given how much of this branch's evidence rests
  // on those numbers being comparable.
  static const size_t rx_profile_every = [] {
    const char* e = getenv("HOUDINI_RX_PROFILE");
    return e != nullptr ? static_cast<size_t>(atol(e)) : 0;
  }();
  static thread_local double p_drain = 0, p_read = 0;
  static thread_local size_t p_calls = 0, p_chunks = 0, p_drained = 0;
  const auto p_t0 = std::chrono::steady_clock::now();
  int drained_chunks = 0, drained_samps = 0;
  int dr = 0;
  while ((dr = dev_->readStream(rxs_, jb.data(), drain_samps, jf, jt, 0)) > 0) {
    ++drained_chunks;
    drained_samps += dr;
  }
  const auto p_t1 = std::chrono::steady_clock::now();

  // A dropped UDP packet splices a gap between two reads of THIS window. Detect it
  // from each read's own timestamp (the window used to keep only the first read's
  // time and concatenate the rest as if contiguous -- silently mis-aligning every
  // post-gap sample, which corrupts the correlation window / CSI). A per-window
  // TimeGridTracker compares where each read's samples land vs. where its stamp says
  // they belong; a gap is zero-padded so post-gap samples stay on their true offset,
  // and the extent is logged (absolute RX sample position) for the /Data/Gaps table.
  if (rx_profile_every > 0) {
    p_drain += std::chrono::duration<double, std::micro>(p_t1 - p_t0).count();
    p_chunks += static_cast<size_t>(drained_chunks);
    p_drained += static_cast<size_t>(drained_samps);
  }
  if (rx_rate_ == 0.0) rx_rate_ = dev_->getSampleRate(SOAPY_SDR_RX, 0);
  Sounder::TimeGridTracker grid(rx_rate_);
  std::vector<void*> cur(num_rx_ch_);
  int got = 0;
  size_t padded = 0;  // zeros inserted into THIS window (see lastPadSamples)
  last_pad_samples_ = 0;  // cleared up front so an early return can't leave a stale count
  while (got < samples) {
    for (size_t c = 0; c < num_rx_ch_; c++)
      cur[c] = static_cast<uint8_t*>(buffs[c]) +
               static_cast<size_t>(got) * kBytesPerSamp;
    int flags = 0;
    long long t = 0;
    int r =
        dev_->readStream(rxs_, cur.data(), samples - got, flags, t, 1000000);
    if (r <= 0) {
      if (got == 0) return r;
      break;
    }
    if (got == 0) frameTime = t;  // first (grid-anchoring) read stamps the window
    size_t pad = 0;
    if (rx_rate_ > 0.0 && (flags & SOAPY_SDR_HAS_TIME) != 0) {
      const Sounder::GridCheck gc = grid.onStamp(t, got);
      pad = std::min(gc.pad_samples, static_cast<size_t>(samples - got));
    } else {
      // No usable stamp, so this read is spliced onto the previous one with no
      // continuity check: precisely the corruption the grid tracker exists to
      // prevent. HOUDINI_PROTOCOL stamps every packet, so on a conformant device
      // this cannot fire; if it does, the guarantee is gone and the window is a
      // guess. Say so rather than degrading silently (AP-10).
      static std::atomic<int> unstamped{0};
      const int n_unstamped = unstamped.fetch_add(1);
      // Braces are load-bearing: MLPD_WARN expands to several statements, so an
      // unbraced guard would gate only the header and print the body every read.
      if ((n_unstamped % 200) == 0) {
        MLPD_WARN(
            "RX read without a usable timestamp (rate=%.0f, flags=0x%x), count "
            "%d: splicing with NO gap check, so this window's timing is not "
            "guaranteed.\n",
            rx_rate_, flags, n_unstamped + 1);
      }
    }
    if (pad > 0) {
      // The r samples just read belong at got+pad: shift them forward and zero-fill
      // the gap so the window stays sample-exact (one gap can't time-shift the rest).
      const size_t keep = std::min(static_cast<size_t>(r),
                                   static_cast<size_t>(samples - got) - pad);
      for (size_t c = 0; c < num_rx_ch_; c++) {
        uint8_t* d = static_cast<uint8_t*>(buffs[c]) +
                     static_cast<size_t>(got) * kBytesPerSamp;
        std::memmove(d + pad * kBytesPerSamp, d, keep * kBytesPerSamp);
        std::memset(d, 0, pad * kBytesPerSamp);
      }
      Sounder::RxGapSink::instance().push({rx_sample_pos_ + got,
                                           static_cast<int64_t>(pad),
                                           Sounder::kGapTimeJump});
      padded += pad;
      got += static_cast<int>(pad + keep);
    } else {
      got += r;
    }
  }
  rx_sample_pos_ += got;
  last_pad_samples_ = padded;
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
      if ((cnt.fetch_add(1) % 40) == 0) {  // braces load-bearing: MLPD_INFO
        MLPD_INFO("Houdini client RX dbg: got=%d rms=%.2f absmax=%d\n", got,
                  rms, amax);                 // is a multi-statement macro
      }
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
  if (rx_profile_every > 0) {
    const auto p_t2 = std::chrono::steady_clock::now();
    p_read += std::chrono::duration<double, std::micro>(p_t2 - p_t1).count();
    if (++p_calls >= rx_profile_every) {
      MLPD_INFO(
          "RX PROFILE over %zu radioRx calls: drain %.0f us (%.1f chunks, "
          "%.0f samples) + read %.0f us = %.0f us/call\n",
          p_calls, p_drain / p_calls, 1.0 * p_chunks / p_calls,
          1.0 * p_drained / p_calls, p_read / p_calls,
          (p_drain + p_read) / p_calls);
      p_drain = p_read = 0;
      p_calls = p_chunks = p_drained = 0;
    }
  }
  return got;
}

int Radio::drainTxStatus(void) {
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

int Radio::recv(void* const* buffs, int samples, long long& frameTime) {
  if (houdini_) return recvHoudini(buffs, samples, frameTime);
  // Non-Houdini path has no gap detection, so it reports no padding rather than
  // leaving a stale count from an earlier call visible to lastPadSamples().
  last_pad_samples_ = 0;
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
