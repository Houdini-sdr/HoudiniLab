/*
 Copyright (c) 2018-2022, Rice University
 RENEW OPEN SOURCE LICENSE: http://renew-wireless.org/license
 
---------------------------------------------------------------------
 Reads configuration parameters from file 
---------------------------------------------------------------------
*/

#include "houdini/rx_packet.h"
#include "include/config.h"
#include "include/run_options.h"

#include <cerrno>
#include <cstring>
#include <cstdio>
#include <cstdlib>
#include <optional>
#include <random>

#include "houdini/rf_plan.h"
#include "houdini/tx_rx_boundary.h"
#include "sync/beacon_shapes.h"
#include "sync/detector.h"
#include "include/comms-lib.h"
#include "include/constants.h"
#include "include/logger.h"
#include "include/macros.h"
#include "include/utils.h"
#include "nlohmann/json.hpp"
using json = nlohmann::json;

static size_t kFpgaTxRamSize = 4096;
static size_t kMaxSupportedFFTSize = 4096;  // AP-79: the 5G-like fft 4096
static size_t kMinSupportedFFTSize = 64;
static size_t kMaxSupportedCPSize = 512;  // AP-79: CP 288 at fft 4096

Config::Config(const std::string& jsonfile, const std::string& directory,
               const bool bs_only, const bool client_only, const bool calibrate)
    : directory_(directory) {
  std::string conf_str;
  Utils::loadTDDConfig(jsonfile, conf_str);
  // Enable comments in json file
  const auto tddConf = json::parse(conf_str, nullptr, true, true);
  // The sync block, loaded and validated as ONE struct with provenance. Throws
  // std::invalid_argument on an unknown key or an out-of-range value, which
  // main.cc reports as a configuration error rather than a crash. Its
  // sentinels are resolved, and the record printed, once the beacon shape is
  // built below.
  {
    // The library never sees the config FILE: the block and the legacy
    // top-level keys are handed to it explicitly.
    std::optional<std::string> sync_block;
    if (tddConf.contains("sync") && !tddConf["sync"].is_null()) {
      sync_block = tddConf["sync"].dump();
    }
    std::optional<std::string> legacy_type;
    if (tddConf.contains("beacon_type")) {
      if (!tddConf["beacon_type"].is_string()) {
        throw std::invalid_argument("beacon_type: expects a string");
      }
      legacy_type = tddConf["beacon_type"].get<std::string>();
    }
    sync_ = houdini::sync::SyncConfig::load(sync_block, legacy_type);
    // The legacy corr_scale arrays are adopted below, once they are parsed
    // with their own fallbacks (an absent key is 1, as it always was).
  }
  std::stringstream ss;
  ss << "  Config: " << tddConf << "\n" << std::endl;
  MLPD_INFO("\nInput config:\n\n%s", ss.str().c_str());
  ss.str(std::string());
  ss.clear();

  if (bs_only && client_only == true) {
    MLPD_ERROR("Client-Only and BS-Only can't both be enabled!\n");
    exit(1);
  }

  internal_measurement_ = tddConf.value("internal_measurement", false);
  // Diagnostic only (AP-73 / SH-348): skip the TX_CLEAR pulse in the Houdini
  // framer's teardown ladder so the replay-RAM reload after an abort can be
  // measured without it. Never set in a shipped config; the framer warns
  // loudly when it is.
  diag_skip_tx_clear_ = tddConf.value("houdini_diag_skip_tx_clear", false);
  /* Used for internal measurements. If internal_measurement is enabled,
       the default is to use a reference node for reciprocity calibration.
       Users have the option of "disabling" the reference node to get a full
       matrix (send pilot from all base station boards and receive on all
       base station boards). The guard interval multiplier extends the number
       of G's in a schedule (between pilots).
     */

  ref_node_enable_ = tddConf.value("reference_node_enable", true);
  guard_mult_ = tddConf.value("meas_guard_interval_mult", 1);

  sample_cal_en_ = tddConf.value("calibrate_digital", false);
  imbalance_cal_en_ = tddConf.value("calibrate_analog", false);

  num_bs_sdrs_all_ = 0;
  num_bs_antennas_all_ = 0;
  num_cl_sdrs_ = 0;

  // `channel` is the single knob that sets BOTH directions. TX and RX may be
  // given independently via `tx_channel` / `rx_channel` (and the UE's
  // `ue_tx_channel` / `ue_rx_channel`), each defaulting to `channel`. This lets
  // a node transmit on one channel set and receive on another, required where a
  // converter is RX-only (the RFSoC4x2 has 2 DACs but 4 ADCs) or the TX and RX
  // antenna counts differ.
  // Each spec is letters A-D (any subset, in order); the count feeds the
  // recorder/antenna accounting (RX) and the stream setup (per direction).
  const auto valid_ch = [](const std::string& s) {
    return !s.empty() && !Utils::strToChannels(s).empty();
  };
  bs_channel_ = tddConf.value("channel", "A");
  bs_tx_channel_ = tddConf.value("tx_channel", bs_channel_);
  bs_rx_channel_ = tddConf.value("rx_channel", bs_channel_);
  if (!valid_ch(bs_tx_channel_) || !valid_ch(bs_rx_channel_)) {
    throw std::invalid_argument(
        "error channel config: tx_channel/rx_channel (or channel) must be "
        "letters A-D\n");
  }
  bs_tx_ch_ = Utils::strToChannels(bs_tx_channel_).size();
  bs_rx_ch_ = Utils::strToChannels(bs_rx_channel_).size();
  bs_sdr_ch_ = bs_rx_ch_;  // legacy: the recorded-antenna (RX) count per BS SDR

  cl_channel_ = tddConf.value("ue_channel", "A");
  cl_tx_channel_ = tddConf.value("ue_tx_channel", cl_channel_);
  cl_rx_channel_ = tddConf.value("ue_rx_channel", cl_channel_);
  if (!valid_ch(cl_tx_channel_) || !valid_ch(cl_rx_channel_)) {
    throw std::invalid_argument(
        "error channel config: ue_tx_channel/ue_rx_channel (or ue_channel) "
        "must be letters A-D\n");
  }
  cl_tx_ch_ = Utils::strToChannels(cl_tx_channel_).size();
  cl_rx_ch_ = Utils::strToChannels(cl_rx_channel_).size();
  cl_sdr_ch_ = cl_rx_ch_;  // legacy: the recorded-antenna (RX) count per UE SDR

  // --topology names another file than the config's (e.g. the demo venue's).
  const std::string serials_file = Sounder::runOptions().topology.empty()
                                       ? tddConf.value("serial_file", "./files/topology.json")
                                       : Sounder::runOptions().topology;
  loadTopology(serials_file, bs_only, client_only, calibrate);
  std::cout << "Topology: "
            << "\n"
            << " Number of cells: " << num_cells_ << "\n"
            << " Number of BS sdrs in cell 0: " << n_bs_sdrs_.at(0) << "\n"
            << " Client SDRs: " << num_cl_sdrs_ << std::endl;

  static const int kMaxTxGainBS = 81;
  // common (BaseStation config overrides these)
  freq_ = tddConf.value("frequency", 2.5e9);
  rate_ = tddConf.value("sample_rate", 5e6);
  nco_ = tddConf.value("nco_frequency", 0.75 * rate_);
  // AP-79 mode V, all optional (absent = one rate, one NCO, as before). The TX
  // stream may run at twice the RX/tick rate: every TX waveform is still built
  // at sample_rate and doubled by the x2 interpolator at the radio boundary
  // (dsp/band_filters.h), so no other ratio is accepted.
  tx_rate_ = tddConf.value("tx_sample_rate", rate_);
  if (tx_rate_ != rate_ && tx_rate_ != 2.0 * rate_) {
    throw std::invalid_argument(
        "tx_sample_rate must equal sample_rate or twice it (the x2 TX "
        "interpolator is the only one built)");
  }
  adc_fs_hz_ = tddConf.value("rfdc_adc_fs_mhz", 0.0) * 1e6;
  dac_fs_hz_ = tddConf.value("rfdc_dac_fs_mhz", 0.0) * 1e6;
  if ((adc_fs_hz_ > 0.0) != (dac_fs_hz_ > 0.0)) {
    throw std::invalid_argument(
        "rfdc_adc_fs_mhz and rfdc_dac_fs_mhz are set together or not at all");
  }
  // Only the NCO is per channel (the config stays common; only what must
  // differ is split); zone, calibration mode, inverse sinc and the RX filter
  // are derived from it (houdini/rf_plan.h).
  if (tddConf.contains("channel_nco_frequency")) {
    const auto& m = tddConf["channel_nco_frequency"];
    if (!m.is_object()) {
      throw std::invalid_argument(
          "channel_nco_frequency must be an object of channel letter -> Hz");
    }
    for (auto it = m.begin(); it != m.end(); ++it) {
      const auto chs = Utils::strToChannels(it.key());
      if (chs.size() != 1 || !it.value().is_number()) {
        throw std::invalid_argument("channel_nco_frequency: key \"" + it.key() +
                                    "\" must be ONE letter A-D with a value in Hz");
      }
      channel_nco_[chs.front()] = it.value().get<double>();
    }
  }
  houdini_tx_gain_db_ = tddConf.value("houdini_tx_gain_db",
                                      std::numeric_limits<double>::quiet_NaN());
  houdini_rx_gain_db_ = tddConf.value("houdini_rx_gain_db",
                                      std::numeric_limits<double>::quiet_NaN());
  if (adc_fs_hz_ > 0.0 && tddConf.value("radio_type", "iris") != std::string("houdini")) {
    throw std::invalid_argument("rfdc_*_fs_mhz (mode V) is Houdini-only");
  }
  if (adc_fs_hz_ > 0.0 && (tddConf.value("fft_size", 0) <= 0 ||
                           tddConf.value("ofdm_data_num", 0) <= 0)) {
    // The per-channel plan is checked against the waveform's occupied band.
    throw std::invalid_argument(
        "mode V (rfdc_*_fs_mhz) needs fft_size and ofdm_data_num: the channel "
        "plan is checked against the band the waveform occupies");
  }
  if (!(adc_fs_hz_ > 0.0) &&
      (tddConf.contains("channel_nco_frequency") || tddConf.contains("houdini_tx_gain_db") ||
       tddConf.contains("houdini_rx_gain_db"))) {
    // These are applied only by the mode-V bring-up; without it they would
    // silently do nothing (every channel on nco_frequency, make()'s gains).
    throw std::invalid_argument(
        "channel_nco_frequency and houdini_tx/rx_gain_db need the mode-V converter "
        "plan (rfdc_adc_fs_mhz / rfdc_dac_fs_mhz); without it they are not applied");
  }
  if (tx_rate_ != rate_ && !(adc_fs_hz_ > 0.0)) {
    throw std::invalid_argument(
        "tx_sample_rate != sample_rate needs the mode-V converter plan "
        "(rfdc_adc_fs_mhz / rfdc_dac_fs_mhz); without it the TX rate is not applied");
  }
  symbol_per_slot_ = tddConf.value("ofdm_symbol_per_slot", 1);
  fft_size_ = tddConf.value("fft_size", 0);
  cp_size_ = tddConf.value("cp_size", 0);
  dl_pilots_en_ = tddConf.value("enable_dl_pilots", false);
  prefix_ = tddConf.value("ofdm_tx_zero_prefix", 0);
  if (adc_fs_hz_ > 0.0 && tx_rate_ != rate_) {
    // The TX interpolator (RadioHoudini) needs zeros around every burst's
    // content, on either lane path (prefiltered, or the wide halfband of
    // AP-85), and the slot's zero prefix/postfix are what provide them; too
    // few and the filter's ramps are cut off at the buffer edge, bringing back
    // the splatter the prefilter exists to remove.
    const int need_pre = static_cast<int>(houdini::boundary::TxBurstInterpolator::maxLead());
    const int need_post = static_cast<int>(houdini::boundary::TxBurstInterpolator::maxTail());
    if (tddConf.value("ofdm_tx_zero_prefix", 0) < need_pre ||
        tddConf.value("ofdm_tx_zero_postfix", 0) < need_post) {
      throw std::invalid_argument(
          "mode V with tx_sample_rate = 2 x sample_rate needs ofdm_tx_zero_prefix >= " +
          std::to_string(need_pre) + " and ofdm_tx_zero_postfix >= " + std::to_string(need_post) +
          " (the TX prefilter and interpolator margins)");
    }
  }
  postfix_ = tddConf.value("ofdm_tx_zero_postfix", 0);
  symbol_data_subcarrier_num_ = tddConf.value("ofdm_data_num", fft_size_);
  // AP-85: a tone count per channel, keyed like channel_nco_frequency (the
  // X-band at 270 RB beside the sub-6 at 133); absent, every channel carries
  // ofdm_data_num. Mode V only: its per-channel plan and TX lanes are what
  // apply a per-channel width.
  if (tddConf.contains("channel_ofdm_data_num")) {
    const auto& m = tddConf["channel_ofdm_data_num"];
    if (!(adc_fs_hz_ > 0.0)) {
      throw std::invalid_argument(
          "channel_ofdm_data_num needs the mode-V converter plan (rfdc_adc_fs_mhz / "
          "rfdc_dac_fs_mhz); without it a per-channel width is not applied");
    }
    if (!m.is_object()) {
      throw std::invalid_argument("channel_ofdm_data_num must be an object of channel letter -> tones");
    }
    if (fft_size_ == Consts::kFftSize_80211) {
      throw std::invalid_argument("channel_ofdm_data_num needs a Zadoff-Chu pilot (fft_size other than 64)");
    }
    for (auto it = m.begin(); it != m.end(); ++it) {
      const auto chs = Utils::strToChannels(it.key());
      // Whole RBs (12 tones): the pilot tones sit at each RB's centre, and
      // the ZC generator reads past its sequence for an odd count.
      if (chs.size() != 1 || !it.value().is_number_unsigned() || it.value().get<size_t>() == 0 ||
          it.value().get<size_t>() % 12 != 0 || it.value().get<size_t>() > fft_size_) {
        throw std::invalid_argument("channel_ofdm_data_num: key \"" + it.key() +
                                    "\" must be ONE letter A-D with a whole number of RBs (a multiple of 12 "
                                    "tones) up to fft_size");
      }
      channel_data_num_[chs.front()] = it.value().get<size_t>();
    }
  }
  if (adc_fs_hz_ > 0.0) {
    // Every channel's band inside the RX decimator's passband, which is also
    // the wide TX halfband's design edge (dsp/band_filters.h).
    const double max_half = houdini::rfplan::Rules{}.decim_pass_frac * rate_;
    auto half = [this](size_t n) { return static_cast<double>(n) * rate_ / static_cast<double>(fft_size_) / 2.0; };
    std::map<size_t, size_t> tones;  // every channel either node opens, and every override
    for (const auto* spec : {&bs_tx_channel_, &bs_rx_channel_, &cl_tx_channel_, &cl_rx_channel_})
      for (const size_t ch : Utils::strToChannels(*spec)) tones[ch] = symbol_data_subcarrier_num_;
    for (const auto& [ch, n] : channel_data_num_) tones[ch] = n;
    for (const auto& [ch, n] : tones) {
      if (half(n) > max_half) {
        throw std::invalid_argument("channel " + std::string(1, static_cast<char>('A' + ch)) + ": " +
                                    std::to_string(n) + " tones occupy +-" + std::to_string(half(n) / 1e6) +
                                    " MHz, beyond the decimator's passband +-" + std::to_string(max_half / 1e6) +
                                    " MHz (0.4 x sample_rate)");
      }
    }
    // Channels on one NCO are one band (in mode V the NCO is the band's
    // centre), so they carry one tone count: this is what ties the UE's TX
    // B to the BS's RX C, and a file that widens one and not the other would
    // have the BS estimate the X-band against the wrong pilot.
    auto nco_of = [this](size_t ch) {
      const auto it = channel_nco_.find(ch);
      return it != channel_nco_.end() ? it->second : nco_;
    };
    for (const auto& [a, na] : tones)
      for (const auto& [b, nb] : tones)
        if (a < b && nco_of(a) == nco_of(b) && na != nb) {
          throw std::invalid_argument(
              "channels " + std::string(1, static_cast<char>('A' + a)) + " and " +
              std::string(1, static_cast<char>('A' + b)) + " share the NCO " + std::to_string(nco_of(a) / 1e6) +
              " MHz but carry " + std::to_string(na) + " and " + std::to_string(nb) +
              " tones; set both in channel_ofdm_data_num");
        }
  }
  pilot_seq_ = tddConf.value("pilot_seq", "lts");
  data_mod_ = tddConf.value("modulation", "QPSK");
  single_gain_ = tddConf.value("single_gain", true);

  if (tddConf.value("tx_gain_a", 20) > kMaxTxGainBS) {
    std::string msg = "ERROR: BaseStation ChanA - Maximum TX gain value is ";
    msg += std::to_string(kMaxTxGainBS);
    throw std::invalid_argument(msg);
  } else {
    tx_gain_.push_back(tddConf.value("tx_gain_a", 20));
  }

  if (tddConf.value("tx_gain_b", 20) > kMaxTxGainBS) {
    std::string msg = "ERROR: BaseStation ChanB - Maximum TX gain value is ";
    msg += std::to_string(kMaxTxGainBS);
    throw std::invalid_argument(msg);
  } else {
    tx_gain_.push_back(tddConf.value("tx_gain_b", 20));
  }

  rx_gain_.push_back(tddConf.value("rx_gain_a", 20));
  rx_gain_.push_back(tddConf.value("rx_gain_b", 20));
  cal_tx_gain_.push_back(tddConf.value("cal_tx_gain_a", tx_gain_.at(0)));
  cal_tx_gain_.push_back(tddConf.value("cal_tx_gain_b", tx_gain_.at(1)));
  tx_gain_.shrink_to_fit();
  rx_gain_.shrink_to_fit();
  cal_tx_gain_.shrink_to_fit();

  beam_sweep_ = tddConf.value("beamsweep", false);
  // WHICH beacon waveform. Parsed here because this is where tddConf lives;
  // genPilots builds from it. Unknown names throw there rather than falling
  // back, so a typo cannot quietly ship the default beacon.
  // sync.beacon.type, or the legacy top-level "beacon_type" (SyncConfig
  // accepts either and refuses the two disagreeing).
  beacon_type_ = sync_.beacon.type;
  beacon_ant_ = tddConf.value("beacon_antenna", 0);
  beacon_radio_ = beacon_ant_ / bs_sdr_ch_;
  beacon_ch_ = beacon_ant_ % bs_sdr_ch_;
  max_frame_ = tddConf.value("max_frame", 0);
  // --max_frame overrides it (the live-CSI dashboard runs the sounder about
  // indefinitely in viewing mode, where max_frame would stop the BS loop).
  if (Sounder::runOptions().max_frame >= 0) max_frame_ = static_cast<size_t>(Sounder::runOptions().max_frame);
  bs_hw_framer_ = tddConf.value("bs_hw_framer", true);
  // AP-87: the BS receives only its rx slots (the real TDD pattern and the
  // device's SH-347 slots mode); checked below once the slot size is known.
  bs_rx_slots_ = tddConf.value("bs_rx_slots", false);
  // The BS removes each lane's pilot-measured carrier offset before the FFT
  // (houdini/pre_cfo.h), not only per symbol after it.
  bs_cfo_pre_fft_ = tddConf.value("bs_cfo_pre_fft", false);

  // Load/Build BS and Client SDRs' Schedules
  bs_array_frames_.resize(num_cells_);
  if (internal_measurement_ == true) {
    genBsSchedule(ref_node_enable_ ? CALIB_STAR_TOPO : CALIB_FULLY_CONN);
    genClientSchedule(ref_node_enable_ ? CALIB_STAR_TOPO : CALIB_FULLY_CONN);
  } else {
    auto jBsFrames = tddConf.value("frame_schedule", json::array());
    std::vector<std::string> frames;
    frames.assign(jBsFrames.begin(), jBsFrames.end());
    assert(frames.size() == num_cells_);
    for (size_t cell_id = 0; cell_id < num_cells_; cell_id++) {
      bs_array_frames_.at(cell_id).resize(n_bs_sdrs_.at(cell_id),
                                          frames.at(cell_id));
    }
    genBsSchedule(dl_pilots_en_ ? DL_SOUNDING : USER_INPUT);
    // read commons from client json config
    if (client_serial_present_ == false) {
      const size_t ref_cell_id = 0;
      num_cl_antennas_ =
          std::count(bs_array_frames_.at(ref_cell_id).at(0).begin(),
                     bs_array_frames_.at(ref_cell_id).at(0).end(), 'P');
      num_cl_sdrs_ = num_cl_antennas_ / cl_sdr_ch_;
    }
    if (dl_pilots_en_ == true) {
      genClientSchedule(DL_SOUNDING);
    } else {
      if (tddConf.find("ue_frame_schedule") == tddConf.end()) {
        genClientSchedule(USER_INPUT);
      } else {
        auto jClFrames = tddConf.value("ue_frame_schedule", json::array());
        cl_frames_.assign(jClFrames.begin(), jClFrames.end());
        assert(cl_frames_.size() == num_cl_sdrs_);
        for (size_t i = 0; i < num_cl_sdrs_; i++) {
          std::cout << "Client " << i << " schedule: " << cl_frames_.at(i)
                    << std::endl;
        }
      }
    }
  }
  cl_pilot_slots_ = Utils::loadSlots(cl_frames_, 'P');
  cl_ul_slots_ = Utils::loadSlots(cl_frames_, 'U');
  cl_dl_slots_ = Utils::loadSlots(cl_frames_, 'D');

  std::cout << "Slots: " << slot_per_frame_ << "\n"
            << " Pilots: " << pilot_slot_per_frame_ << "\n"
            << " Noise: " << noise_slot_per_frame_ << "\n"
            << " UL Slots: " << ul_slot_per_frame_ << "\n"
            << " DL Slots: " << dl_slot_per_frame_ << "\n"
            << " Client SDRs: " << num_cl_sdrs_ << std::endl;

  // Clients
  cl_data_mod_ = tddConf.value("ue_modulation", "QPSK");

  cl_agc_en_ = tddConf.value("agc_en", false);
  cl_agc_gain_init_ = tddConf.value("agc_gain_init", 70);  // 0 to 108
  cl_power_ramp_ = tddConf.value("ue_power_ramp", false);
  cl_power_ramp_lo_ = tddConf.value("ue_ramp_min_gain", 10);
  cl_power_ramp_hi_ = tddConf.value("ue_ramp_max_gain", 42);
  frame_mode_ = tddConf.value("frame_mode", "continuous_resync");
  hw_framer_ = tddConf.value("ue_hw_framer", false);
  // AP-86: the X-band RF front end (XUD1A + ADTR1107) is attached. Each node
  // holds its board STATIC for the session (BS rx, UE tx; the mode V bring-up,
  // step 1b). The UE must never arm a TDD schedule then: the device's arm gate
  // refuses it under a static source with a guarded channel.
  xband_frontend_static_ = tddConf.value("xband_frontend_static", false);
  if (xband_frontend_static_ && !mode_v()) {
    throw std::invalid_argument("xband_frontend_static needs the mode-V converter plan (rfdc_adc_fs_mhz / rfdc_dac_fs_mhz)");
  }
  if (xband_frontend_static_ && hw_framer_) {
    throw std::invalid_argument("xband_frontend_static: ue_hw_framer must be false (the UE must not arm a TDD schedule while its board is held static)");
  }
  radio_type_ = tddConf.value("radio_type", "iris");
  remote_port_ = tddConf.value("remote_port", "55132");
  ue_tdd_pilot_ = tddConf.value("ue_tdd_pilot", false);
  ue_tx_advance_ticks_ = tddConf.value("ue_tx_advance_ticks", 0);
  // The driver accepts TX anchors only on the 384-tick grid, so this knob is
  // quantized: values below 192 vanish in the round-to-nearest, larger ones
  // jump whole grid steps. Fine seating comes from the zero-padded burst
  // composition; leave this at 0 unless you know why not.
  if (ue_tx_advance_ticks_ != 0 && ue_tx_advance_ticks_ % 384 != 0) {
    MLPD_WARN(
        "ue_tx_advance_ticks=%lld is not a multiple of 384 ticks and will be "
        "quantized by the TX anchor grid\n",
        static_cast<long long>(ue_tx_advance_ticks_));
  }
  ue_pilot_horizon_ = tddConf.value("ue_pilot_horizon", 0);
  auto tx_advance = tddConf.value("tx_advance", json::array());
  if (tx_advance.empty() == true) {
    // Houdini default = the measured wired-bench calibration (AP-19,
    // DEMO_VERIFICATION.md 4.35); Iris keeps its historical 250.
    tx_advance_.resize(num_cl_sdrs_, is_houdini() ? 247 : 250);
  } else {
    if (client_present_ && tx_advance.size() != num_cl_sdrs_) {
      MLPD_ERROR("tx_advance size must be same as the number of clients!\n");
      exit(1);
    }
    tx_advance_.assign(tx_advance.begin(), tx_advance.end());
  }
  auto corr_scale = tddConf.value("corr_scale", json::array());
  if (corr_scale.empty() == true) {
    corr_scale_.resize(num_cl_sdrs_, 1);
  } else {
    if (client_present_ && corr_scale.size() != num_cl_sdrs_) {
      MLPD_ERROR("corr_scale size must match the number of clients!\n");
      exit(1);
    }
    corr_scale_.assign(corr_scale.begin(), corr_scale.end());
  }
  // Acquisition gets its own threshold. The re-sync path deliberately RELAXES on
  // every retry (corr_scale + resync_retry_cnt) because there getting a lock back
  // beats stalling; acquisition is the opposite case, because the frame anchor it
  // produces is what every slot boundary in the frame is measured from. On the
  // bench, beacon windows peak at a ratio of 0.31 to 4.2 while a corr_scale of 100
  // puts the bar at 0.01, low enough for sidelobes to cross first
  // (DEMO_VERIFICATION.md 4.14). Defaults to corr_scale when unset, so this is
  // inert until a config asks for it.
  auto corr_scale_init = tddConf.value("corr_scale_init", json::array());
  if (corr_scale_init.empty() == true) {
    corr_scale_init_ = corr_scale_;
  } else {
    if (client_present_ && corr_scale_init.size() != num_cl_sdrs_) {
      MLPD_ERROR("corr_scale_init size must match the number of clients!\n");
      exit(1);
    }
    corr_scale_init_.assign(corr_scale_init.begin(), corr_scale_init.end());
  }
  // The detection bars the library records and the receiver applies for a
  // single client: the first client's, with the sounder's fallbacks (an
  // absent corr_scale is 1, an absent corr_scale_init is corr_scale), marked
  // json or derived accordingly. With several clients the arrays apply.
  // A config with no clients has an empty corr_scale_ (BS-only, calibration,
  // beam sweep): the sounder's fallback of 1 applies, marked derived, per
  // array, so an explicit corr_scale_init in such a file is still recorded.
  {
    const double cs = corr_scale_.empty() ? 1.0 : static_cast<double>(corr_scale_.at(0));
    const double csi =
        corr_scale_init_.empty() ? cs : static_cast<double>(corr_scale_init_.at(0));
    sync_.adoptLegacyThreshold(cs, csi, !corr_scale.empty(), !corr_scale_init.empty());
  }
  ul_data_frame_num_ = tddConf.value("ul_data_frame_num", 1);
  dl_data_frame_num_ = tddConf.value("dl_data_frame_num", 1);

  // Help verify whether gain exceeds max value
  struct compare {
    const int key_;
    compare(int const& i) : key_(i) {}
    bool operator()(int const& i) { return (i > key_); }
  };
  cl_txgain_vec_.resize(2);
  cl_rxgain_vec_.resize(2);
  auto jClTxgainA_vec = tddConf.value("ue_tx_gain_a", json::array());
  cl_txgain_vec_.at(0).assign(jClTxgainA_vec.begin(), jClTxgainA_vec.end());
  auto jClRxgainA_vec = tddConf.value("ue_rx_gain_a", json::array());
  cl_rxgain_vec_.at(0).assign(jClRxgainA_vec.begin(), jClRxgainA_vec.end());
  auto jClTxgainB_vec = tddConf.value("ue_tx_gain_b", json::array());
  cl_txgain_vec_.at(1).assign(jClTxgainB_vec.begin(), jClTxgainB_vec.end());
  auto jClRxgainB_vec = tddConf.value("ue_rx_gain_b", json::array());
  cl_rxgain_vec_.at(1).assign(jClRxgainB_vec.begin(), jClRxgainB_vec.end());

  max_tx_gain_ue_ = tddConf.value("maxTxGainUE", 109);
  compare find_guilty(max_tx_gain_ue_);
  if (std::any_of(cl_txgain_vec_.at(0).begin(), cl_txgain_vec_.at(0).end(),
                  find_guilty)) {
    std::string msg = "ERROR: UE ChanA - Maximum TX gain value is ";
    msg += std::to_string(max_tx_gain_ue_);
    throw std::invalid_argument(msg);
  }
  if (std::any_of(cl_txgain_vec_.at(1).begin(), cl_txgain_vec_.at(1).end(),
                  find_guilty)) {
    std::string msg = "ERROR: UE ChanB - Maximum TX gain value is ";
    msg += std::to_string(max_tx_gain_ue_);
    throw std::invalid_argument(msg);
  }

  bw_filter_ = rate_ + 2 * nco_;
  radio_rf_freq_ = freq_ - nco_;
  ofdm_symbol_size_ = fft_size_ + cp_size_;
  slot_samp_size_ = symbol_per_slot_ * ofdm_symbol_size_;
  samps_per_slot_ = slot_samp_size_ + prefix_ + postfix_;
  if (bs_rx_slots_ && !bs_hw_framer_) {
    throw std::invalid_argument("bs_rx_slots needs bs_hw_framer (the native TDD framer arms the pattern)");
  }
  if (is_houdini() && bs_hw_framer_ && houdini::rxpkt::framerPacket(samps_per_slot_, rate_) > 0 &&
      houdini::rxpkt::tiledPacketOrDefault(samps_per_slot_) == 0) {
    // The tiling packet is smaller than 3/4 of the default one: the host's
    // receive load rises with the packet rate. Allowed (the device refuses a
    // packet that does not tile, SH-488), and said once.
    const size_t pkt = houdini::rxpkt::framerPacket(samps_per_slot_, rate_);
    MLPD_WARN("the BS's TDD framer packet is %zu samples (the default is %zu): %.1fx the packet rate, so more host "
              "receive load; a slot with a larger divisor avoids it\n",
              pkt, houdini::rxpkt::deviceSamples(houdini::rxpkt::kDefaultMtu),
              static_cast<double>(houdini::rxpkt::deviceSamples(houdini::rxpkt::kDefaultMtu)) / pkt);
  }
  if (is_houdini() && bs_hw_framer_ && houdini::rxpkt::framerPacket(samps_per_slot_, rate_) == 0) {
    throw std::invalid_argument("the BS's TDD framer needs packets that tile the slot (the device refuses a TDD RX "
                                "packet that does not divide the slot or spans under " +
                                std::to_string(houdini::rxpkt::kMinFramerTicks) + " ticks, SH-488); a " +
                                std::to_string(samps_per_slot_) + "-sample slot at " + std::to_string(rate_ / 1e6) +
                                " MSPS has no such packet");
  }
  // The Houdini TDD framer cuts ONE pilot slot per frame: it keeps the
  // schedule's 'P' as the pilot and centres the burst search on it, so a
  // second 'P' sits where the search assumes silence and misplaces every cut,
  // unflagged. With bs_rx_slots the beacon strobe is armed on the 'B' slot, so
  // a schedule without one plays no beacon.
  if (is_houdini() && bs_hw_framer_ && !internal_measurement_) {
    for (const auto& cell : bs_array_frames_) {
      for (const auto& sched : cell) {
        const auto np = std::count(sched.begin(), sched.end(), 'P');
        if (np != 1) {
          throw std::invalid_argument("frame_schedule '" + sched + "': the Houdini TDD framer cuts exactly one pilot "
                                      "slot ('P') per frame; this schedule has " + std::to_string(np));
        }
        if (bs_rx_slots_ && sched.find('B') == std::string::npos) {
          throw std::invalid_argument("frame_schedule '" + sched + "': bs_rx_slots arms the beacon strobe on the "
                                      "schedule's 'B' slot, and this schedule has none");
        }
      }
    }
  }
  assert((internal_measurement_ && num_cl_antennas_ == 0) || (dl_pilots_en_) ||
         (num_cl_sdrs_ > 0 && slot_per_frame_ == cl_frames_.at(0).size()));

  ul_data_slot_present_ =
      (internal_measurement_ == false) &&
      ((bs_present_ == true && (ul_slots_.at(0).empty() == false)) ||
       (client_present_ == true && cl_ul_slots_.at(0).empty() == false));

  dl_data_slot_present_ =
      (internal_measurement_ == false) &&
      ((bs_present_ == true && dl_slots_.empty() == false &&
        (dl_slots_.at(0).empty() == false)) ||
       (client_present_ == true && cl_dl_slots_.at(0).empty() == false));

  tx_scale_ = tddConf.value("tx_scale", 0);
  this->genPilots();

  this->loadULData();
  this->loadDLData();

  bool recording =
      pilot_slot_per_frame_ + ul_slot_per_frame_ + dl_slot_per_frame_ > 0;
  if (recording == true) {
    // set trace file path
    time_t now = time(0);
    tm* ltm = localtime(&now);
    std::string filename;
    if (internal_measurement_ && num_cl_antennas_ == 0) {
      filename =
          directory_ + "/trace-internal-meas-" +
          std::to_string(1900 + ltm->tm_year) + "-" +
          std::to_string(1 + ltm->tm_mon) + "-" + std::to_string(ltm->tm_mday) +
          "-" + std::to_string(ltm->tm_hour) + "-" +
          std::to_string(ltm->tm_min) + "-" + std::to_string(ltm->tm_sec) +
          "_" + std::to_string(num_cells_) + "_" +
          std::to_string(num_bs_antennas_all_) + ".hdf5";
    } else if (internal_measurement_ && num_cl_antennas_ > 0) {
      filename =
          directory_ + "/trace-reciprocity-" +
          std::to_string(1900 + ltm->tm_year) + "-" +
          std::to_string(1 + ltm->tm_mon) + "-" + std::to_string(ltm->tm_mday) +
          "-" + std::to_string(ltm->tm_hour) + "-" +
          std::to_string(ltm->tm_min) + "-" + std::to_string(ltm->tm_sec) +
          "_" + std::to_string(num_cells_) + "_" +
          std::to_string(num_bs_antennas_all_) + "x" +
          std::to_string(num_cl_antennas_) + ".hdf5";
    } else {
      std::string ul_present_str = (ul_data_slot_present_ ? "uplink-" : "");
      std::string dl_present_str = (dl_data_slot_present_ ? "downlink-" : "");
      filename =
          directory_ + "/trace-" + ul_present_str + dl_present_str +
          std::to_string(1900 + ltm->tm_year) + "-" +
          std::to_string(1 + ltm->tm_mon) + "-" + std::to_string(ltm->tm_mday) +
          "-" + std::to_string(ltm->tm_hour) + "-" +
          std::to_string(ltm->tm_min) + "-" + std::to_string(ltm->tm_sec) +
          "_" + std::to_string(num_cells_) + "_" +
          std::to_string(num_bs_antennas_all_) + "x" +
          std::to_string(num_cl_antennas_) + ".hdf5";
    }
    trace_file_ = tddConf.value("trace_file", filename);
    recorder_thread_num_ = tddConf.value(
        "recorder_thread",
        bs_present_ || (client_present_ && cl_dl_slots_.at(0).size() > 0)
            ? RECORDER_THREAD_NUM
            : 0);
    reader_thread_num_ = (client_present_ && ul_slot_per_frame_ > 0) +
                         (bs_present_ && dl_slot_per_frame_ > 0);
  } else {
    recorder_thread_num_ = 0;
    reader_thread_num_ = 0;
  }

  // Multi-threading settings
  unsigned num_cores = this->getCoreCount();
  MLPD_INFO("Cores found %u ... \n", num_cores);
  if (bs_present_ == true &&
      pilot_slot_per_frame_ + ul_slot_per_frame_ + dl_slot_per_frame_ > 0) {
    bs_rx_thread_num_ =
        (num_cores >= (2 * RX_THREAD_NUM))
            ? std::min(RX_THREAD_NUM, static_cast<int>(num_bs_sdrs_all_))
            : 1;
    if (internal_measurement_ == true && ref_node_enable_ == true) {
      bs_rx_thread_num_ = 2;
    }
  } else {
    bs_rx_thread_num_ = 0;
  }

  if (client_present_ == true && cl_dl_slots_.at(0).size() > 0) {
    cl_rx_thread_num_ = num_cl_sdrs_;
  } else {
    cl_rx_thread_num_ = 0;
  }

  core_alloc_ = num_cores >= (1 + recorder_thread_num_ + reader_thread_num_ +
                              bs_rx_thread_num_ + num_cl_sdrs_);

  if (core_alloc_ == true) {
    if (bs_present_ == true) {
      MLPD_INFO("Allocating %zu cores to receive threads ... \n",
                bs_rx_thread_num_);
      MLPD_INFO("Allocating %zu cores to record threads ... \n",
                recorder_thread_num_);
      MLPD_INFO("Allocating %zu cores to read threads ... \n",
                reader_thread_num_);
    }
    if (client_present_ == true) {
      MLPD_INFO("Allocating %zu cores to client threads ... \n", num_cl_sdrs_);
    }
  }

  tx_frame_delta_ =
      std::ceil(TIME_DELTA_MS / (1e3 * this->getFrameDurationSec()));
  std::printf(
      "Config: %zu BS, %zu BS radios (total), %zu UE antennas, %zu pilot "
      "symbols per "
      "frame,\n\t%zu uplink data symbols per frame, %zu downlink data "
      "symbols per frame,\n\t%zu OFDM subcarriers (%zu data subcarriers), "
      "modulation %s, frame time %.3f usec \n",
      num_cells_, num_bs_sdrs_all_, num_cl_antennas_, pilot_slot_per_frame_,
      ul_slot_per_frame_, dl_slot_per_frame_, fft_size_,
      symbol_data_subcarrier_num_, data_mod_.c_str(),
      this->getFrameDurationSec() * 1e6);
  std::printf(
      "Thread Config: %zu BS receive threads, %zu Client receive "
      "threads, %zu recording threads, %zu reading thread\n",
      bs_rx_thread_num_, num_cl_sdrs_, recorder_thread_num_,
      reader_thread_num_);
  running_.store(true);
}

void Config::loadTopology(std::string serials_file, const bool bs_only,
                          const bool client_only, const bool calibrate) {
  // Load serials file (loads hub, sdr, and rrh serials)
  std::string serials_str;
  Utils::loadTDDConfig(serials_file, serials_str);
  std::stringstream ss;
  if (serials_str.empty() == false) {
    const auto j_serials = json::parse(serials_str, nullptr, true, true);
    if (j_serials.find("BaseStations") != j_serials.end()) {
      json j_bs_serials;
      ss << j_serials.value("BaseStations", j_bs_serials);
      j_bs_serials = json::parse(ss);
      ss.str(std::string());
      ss.clear();
      num_cells_ = j_bs_serials.size();
      bs_sdr_ids_.resize(num_cells_);
      calib_ids_.resize(num_cells_);
      n_bs_sdrs_.resize(num_cells_);
      n_bs_antennas_.resize(num_cells_);

      for (size_t i = 0; i < num_cells_; i++) {
        json serials_conf;
        std::string cell_str = "BS" + std::to_string(i);
        ss << j_bs_serials.value(cell_str, serials_conf);
        serials_conf = json::parse(ss);
        ss.str(std::string());
        ss.clear();

        auto hub_serial = serials_conf.value("hub", "");
        hub_ids_.push_back(hub_serial);
        auto sdr_serials = serials_conf.value("sdr", json::array());
        bs_sdr_ids_.at(i).assign(sdr_serials.begin(), sdr_serials.end());

        // Append calibration node
        if ((internal_measurement_ == true && ref_node_enable_ == true) ||
            (calibrate && sample_cal_en_ == true)) {
          calib_ids_.at(i) = serials_conf.value("reference", "");
          if (calib_ids_.at(i).empty()) {
            MLPD_ERROR("No calibration node ID found in topology file!\n");
            exit(1);
          }
          std::cout << "Calibration Node: " << calib_ids_.at(i) << std::endl;
          bs_sdr_ids_.at(i).push_back(calib_ids_.at(i));
        }

        n_bs_sdrs_.at(i) = bs_sdr_ids_.at(i).size();
        n_bs_antennas_.at(i) = bs_sdr_ch_ * n_bs_sdrs_.at(i);
        num_bs_sdrs_all_ += n_bs_sdrs_.at(i);
        num_bs_antennas_all_ += n_bs_antennas_.at(i);
        cal_ref_sdr_id_ = n_bs_sdrs_.at(i) > 0 ? n_bs_sdrs_.at(i) - 1 : 0;
        MLPD_INFO(
            "Loading devices - cell %zu, sdrs %zu, antennas: %zu, "
            "total bs srds: %zu\n",
            i, n_bs_sdrs_.at(i), n_bs_antennas_.at(i), num_bs_sdrs_all_);
      }
      // Print Topology
      std::cout << "Topology: " << std::endl;
      for (size_t i = 0; i < bs_sdr_ids_.size(); i++) {
        std::cout << "BS" + std::to_string(i) + " Hub:"
                  << (hub_ids_.empty() == false ? hub_ids_.at(i) : "")
                  << std::endl;
        for (size_t j = 0; j < bs_sdr_ids_.at(i).size(); j++) {
          std::cout << " \t- " << bs_sdr_ids_.at(i).at(j) << std::endl;
        }
      }

      // Array with cummulative sum of SDRs in cells
      n_bs_sdrs_agg_.resize(num_cells_ + 1);
      n_bs_sdrs_agg_.at(0) = 0;  //n_bs_sdrs_[0];
      for (size_t i = 0; i < num_cells_; i++) {
        n_bs_sdrs_agg_.at(i + 1) = n_bs_sdrs_agg_.at(i) + n_bs_sdrs_.at(i);
      }

    } else {
      num_cells_ = 0;
      bs_present_ = false;
      if (internal_measurement_ == true) {
        MLPD_ERROR("No BS devices are present in the serial file!");
        exit(1);
      }
    }

    // read client serials
    client_serial_present_ = (j_serials.find("Clients") != j_serials.end());
    if (client_serial_present_) {
      json j_ue_serials;
      ss << j_serials.value("Clients", j_ue_serials);
      j_ue_serials = json::parse(ss);
      ss.str(std::string());
      ss.clear();
      auto ue_serials = j_ue_serials.value("sdr", json::array());
      cl_sdr_ids_.assign(ue_serials.begin(), ue_serials.end());
      num_cl_sdrs_ = cl_sdr_ids_.size();
      num_cl_antennas_ = num_cl_sdrs_ * cl_sdr_ch_;
    } else {
      num_cl_sdrs_ = num_cl_antennas_ = 0;
    }

    client_present_ = !bs_only && client_serial_present_ && num_cl_sdrs_ > 0;
    bs_present_ = !client_only && num_bs_sdrs_all_ > 0;
  } else {
    std::cout << "Serial file empty! Exitting.." << std::endl;
    exit(1);
  }
}

void Config::genBsSchedule(BsSchedType type) {
  size_t num_channels = bs_channel_.size();
  switch (type) {
    case CALIB_STAR_TOPO:
      for (size_t c = 0; c < num_cells_; c++) {
        cal_ref_sdr_id_ = n_bs_sdrs_[c] > 0 ? n_bs_sdrs_[c] - 1 : 0;
        bs_array_frames_[c].resize(n_bs_sdrs_[c]);
        size_t beacon_slot = 0;
        if (num_cl_antennas_ > 0)
          beacon_slot = 1;  // add a "B" in the front for UE sync
        size_t frame_length =
            beacon_slot + n_bs_antennas_[c] + num_cl_antennas_;
        bs_array_frames_[c][cal_ref_sdr_id_] = std::string(frame_length, 'G');
        bs_array_frames_[c][cal_ref_sdr_id_].replace(
            beacon_slot + num_channels * cal_ref_sdr_id_, 1, "P");

        for (size_t i = 0; i < n_bs_sdrs_[c]; i++) {
          if (i != cal_ref_sdr_id_) {
            bs_array_frames_[c][i] = std::string(frame_length, 'G');
            if (num_cl_antennas_ > 0) bs_array_frames_[c][i].replace(0, 1, "B");
            for (size_t ch = 0; ch < num_channels; ch++) {
              bs_array_frames_[c][i].replace(
                  beacon_slot + i * num_channels + ch, 1, "P");
              bs_array_frames_[c][cal_ref_sdr_id_].replace(
                  beacon_slot + num_channels * i + ch, 1, "R");
            }
            bs_array_frames_[c][i].replace(
                beacon_slot + num_channels * cal_ref_sdr_id_, 1, "R");
            for (size_t p = 0; p < num_cl_antennas_; p++)
              bs_array_frames_[c][i].replace(
                  beacon_slot + num_channels * n_bs_sdrs_[c] + p, 1, "R");
          }
        }
      }
      break;
    case CALIB_FULLY_CONN:
      // For full matrix measurements (all bs nodes transmit and receive)
      for (size_t c = 0; c < num_cells_; c++) {
        cal_ref_sdr_id_ = n_bs_sdrs_[c] > 0 ? n_bs_sdrs_[c] - 1 : 0;
        bs_array_frames_[c].resize(n_bs_sdrs_[c]);
        size_t frame_length = num_channels * n_bs_sdrs_[c] * guard_mult_;
        for (size_t i = 0; i < n_bs_sdrs_[c]; i++) {
          bs_array_frames_[c][i] = std::string(frame_length, 'G');
        }
        for (size_t i = 0; i < n_bs_sdrs_[c]; i++) {
          for (size_t ch = 0; ch < num_channels; ch++) {
            bs_array_frames_[c][i].replace(guard_mult_ * i * num_channels + ch,
                                           1, "P");
          }
          for (size_t k = 0; k < n_bs_sdrs_[c]; k++) {
            if (i != k) {
              for (size_t ch = 0; ch < num_channels; ch++) {
                bs_array_frames_[c][k].replace(
                    guard_mult_ * i * num_channels + ch, 1, "R");
              }
            }
          }
        }
      }
      break;
    case DL_SOUNDING:
      for (size_t cell_id = 0; cell_id < num_cells_; cell_id++) {
        auto& bs_array_frame = bs_array_frames_.at(cell_id);
        const auto& num_cell_bs_antennas = n_bs_antennas_.at(cell_id);
        const auto& num_cell_bs_radios = n_bs_sdrs_.at(cell_id);
        bs_array_frame.resize(num_cell_bs_radios);
        std::cout << "BS antennas...." << num_cell_bs_antennas << std::endl;
        std::cout << "BS radios......" << num_cell_bs_radios << std::endl;

        // If downlink pilots enabled
        const size_t beacon_slot = 0;

        const size_t num_uplink_pilots = num_cl_antennas_;
        std::cout << "UE antennas...." << num_uplink_pilots << std::endl;
        //Produces 1 less than value
        const size_t num_guard_after_beacon = 3;
        const size_t num_guard_after_ul_pilots = 2;
        const size_t num_guard_after_dl_pilots = 2;
        const size_t num_dl_pilots_per_ant = 1;
        const size_t frame_length =
            num_guard_after_beacon + num_uplink_pilots +
            num_guard_after_ul_pilots +
            (num_cell_bs_antennas * num_dl_pilots_per_ant) +
            num_guard_after_dl_pilots;

        for (size_t i = 0; i < num_cell_bs_radios; i++) {
          auto& frame = bs_array_frame.at(i);
          frame = std::string(frame_length, 'G');
          //Add beacon
          frame.at(beacon_slot) = 'B';
          // Add uplink pilots (1 per Ue antenna)
          for (size_t uplink_pilot = 0; uplink_pilot < num_uplink_pilots;
               uplink_pilot++) {
            frame.at(beacon_slot + num_guard_after_beacon + uplink_pilot) = 'P';
          }
          // Add downlink pilots (1 per bs antenna)
          for (size_t ch = 0; ch < num_channels; ch++) {
            frame.at(beacon_slot + num_guard_after_beacon + num_uplink_pilots +
                     num_guard_after_ul_pilots + ((i * num_channels) + ch)) =
                'D';
          }
          std::cout << frame << std::endl;
        }
      }
      break;
    default:
      break;
  }
  for (size_t c = 0; c < num_cells_; c++) {
    for (std::string const& s : bs_array_frames_[c])
      std::cout << s << std::endl;
  }
  const size_t ref_cell_id = 0;
  // Assume all BS nodes will transmit the same number of downlink/uplink pilots, etc. Grab the first one
  // TODO: Extend to multi-cell
  pilot_slots_ = Utils::loadSlots(bs_array_frames_.at(ref_cell_id), 'P');
  noise_slots_ = Utils::loadSlots(bs_array_frames_.at(ref_cell_id), 'N');
  ul_slots_ = Utils::loadSlots(bs_array_frames_.at(ref_cell_id), 'U');
  dl_slots_ = Utils::loadSlots(bs_array_frames_.at(ref_cell_id), 'D');
  slot_per_frame_ = bs_array_frames_.at(ref_cell_id).at(0).size();
  pilot_slot_per_frame_ = pilot_slots_.at(ref_cell_id).size();
  noise_slot_per_frame_ = noise_slots_.at(ref_cell_id).size();
  ul_slot_per_frame_ = ul_slots_.at(ref_cell_id).size();
  dl_slot_per_frame_ = dl_slots_.at(ref_cell_id).size();
}

void Config::genClientSchedule(BsSchedType type) {
  const size_t ref_cell_id = 0;
  size_t num_channels = bs_channel_.size();
  switch (type) {
    case CALIB_STAR_TOPO:
      // Two pilots (up/down) plus additional user pilots
      pilot_slot_per_frame_ = 2 + num_cl_antennas_;
      if (num_cl_antennas_ > 0) {
        cl_frames_.resize(num_cl_sdrs_);
        std::string empty_frame = std::string(slot_per_frame_, 'G');
        for (size_t i = 0; i < num_cl_sdrs_; i++) {
          cl_frames_.at(i) = empty_frame;
          for (size_t n = 0; n < n_bs_sdrs_.at(ref_cell_id); n++) {
            if (n != cal_ref_sdr_id_) {
              for (size_t ch = 0; ch < num_channels; ch++) {
                size_t slot = 1 + n * num_channels + ch;
                std::cout << "Replacing slot " << slot << std::endl;
                // downlink symbol for UEs
                cl_frames_.at(i).replace(slot, 1, "D");
              }
            }
          }
          for (size_t ch = 0; ch < cl_sdr_ch_; ch++) {
            size_t slot = 1 + n_bs_antennas_[ref_cell_id] + cl_sdr_ch_ * i + ch;
            cl_frames_.at(i).replace(slot, 1, "P");
          }
          std::cout << "Client " << i << " schedule: " << cl_frames_.at(i)
                    << std::endl;
        }
      }
      break;
    case CALIB_FULLY_CONN:
      if (num_cl_antennas_ > 0) {
        std::cout << "Client Schedule is not supported!" << std::endl;
        exit(1);
      }
      pilot_slot_per_frame_ = n_bs_antennas_.at(ref_cell_id);
      break;
    case DL_SOUNDING: {
      cl_frames_.resize(num_cl_sdrs_);
      std::string empty_frame = std::string(slot_per_frame_, 'G');
      //Include all the D's
      for (const auto& dl_slot_sdr : dl_slots_) {
        for (const auto& dl_ind : dl_slot_sdr) {
          empty_frame.at(dl_ind) = 'D';
        }
      }
      //Include all the U's
      const size_t ref_sdr_idx = 0;
      for (const auto& ul_ind : ul_slots_.at(ref_sdr_idx)) {
        empty_frame.at(ul_ind) = 'U';
      }

      //Add the per sdr ul pilot schedule
      for (size_t i = 0; i < num_cl_sdrs_; i++) {
        cl_frames_.at(i) = empty_frame;
        //Look at the P index array.
        const auto& ul_pilots = pilot_slots_.at(ref_sdr_idx);
        size_t cl_ant_num = cl_sdr_ch_ * i;
        size_t ul_pilot_idx = 0;
        for (const auto& ul_pilot : ul_pilots) {
          if (ul_pilot_idx == cl_ant_num) {
            cl_frames_.at(i).at(ul_pilot) = 'P';
            cl_ant_num++;
            //Exit when the last pilot channel has been found
            if (cl_ant_num >= (cl_sdr_ch_ * (i + 1))) {
              break;
            }
          }
          ul_pilot_idx++;
        }

        std::cout << "Client " << i << " schedule: " << cl_frames_.at(i)
                  << std::endl;
      }
      break;
    }
    default:
      cl_frames_.resize(num_cl_sdrs_);
      for (size_t i = 0; i < cl_frames_.size(); i++) {
        cl_frames_.at(i) = bs_array_frames_.at(ref_cell_id).at(0);
        size_t frame_len = bs_array_frames_.at(ref_cell_id).at(0).size();
        for (size_t s = 0; s < frame_len; s++) {
          char c = cl_frames_.at(i).at(s);
          if (c == 'B') {
            // Dummy RX used in PHY scheduler
            cl_frames_.at(i).replace(s, 1, "G");
          } else if (c == 'P' and
                     ((cl_sdr_ch_ == 1 and pilot_slots_.at(0).at(i) != s) or
                      (cl_sdr_ch_ == 2 and
                       (pilot_slots_.at(0).at(2 * i) != s and
                        pilot_slots_.at(0).at(i * 2 + 1) != s)))) {
            cl_frames_.at(i).replace(s, 1, "G");
          } else if (c != 'P' && c != 'U' && c != 'D') {
            cl_frames_.at(i).replace(s, 1, "G");
          }
        }
        std::cout << "Client " << i << " schedule: " << cl_frames_.at(i)
                  << std::endl;
      }
  }
}

void Config::genPilots() {
  std::vector<std::complex<int16_t>> prefix_zpad(prefix_, 0);
  std::vector<std::complex<int16_t>> postfix_zpad(postfix_, 0);

  // Compose the beacon slot. WHICH beacon is a config choice (sync.beacon.type,
  // or the top-level beacon_type), and every candidate is defined once in
  // include/sync/beacon_shapes.h: the same header the offline geometry test and
  // the bench probes build from, so the waveform this transmits is
  // sample-for-sample the waveform they measured, and the bench and the build
  // cannot disagree about a beacon.
  //
  // The default is `legacy` (beacon_geometry_test pins its recipe bit for bit):
  // the reference every result in DEMO_VERIFICATION.md before the shape choice
  // was taken against (8.111-8.116 compare the shapes).
  srand(time(NULL));
  // The configured beacon as ONE object (sync/beacon_shape.h): waveform,
  // replica, field geometry and the index convention every consumer rests
  // on. An unknown name throws, naming the valid ones: a typo that quietly
  // ships the old beacon is exactly the failure this parameter exists to
  // make visible.
  houdini::sync::Numerology num;
  num.rate_hz = rate();
  num.samps_per_slot = samps_per_slot();
  num.samps_per_frame = samps_per_frame();
  num.prefix_samples = static_cast<size_t>(prefix_);
  shape_ = std::make_unique<houdini::sync::BeaconShape>(
      houdini::sync::BeaconShape::make(beacon_type_, platform(), num));
  const houdini::sync::BeaconShape& shape = *shape_;
  if (mode_v()) {
    // AP-79: in mode V the sub-6 RX lanes are filtered to +-24 MHz and the TX
    // is prefiltered to it, so a beacon wider than that is cut on both ends
    // while the detector still correlates against its full-band replica.
    const double half = shape.occupiedHalfBwHz();
    const double pass = houdini::rfplan::Rules{}.filter_pass_hz;
    if (!(half > 0.0) || half > pass) {
      throw std::invalid_argument(
          "mode V needs a band-limited beacon inside +-" + std::to_string(pass / 1e6) +
          " MHz (sync.beacon.type nr_pss_bl); " + beacon_type_ + " occupies " +
          (half > 0.0 ? "+-" + std::to_string(half / 1e6) + " MHz" : std::string("the whole output")));
    }
  }

  // NOTE THE REPLICA'S SCALE IS LOAD-BEARING, and do not "tidy" it to unit
  // power. The power-ratio form's test is `corr_scale * |gc|^2|gc_lag|^2 >
  // sum|gc|^2`, 4th order against 2nd, so scaling the replica by k scales the
  // decision ratio by k^2 (the normalised forms divide the scale out). Every
  // detector ratio in DEMO_VERIFICATION.md 8.112 was measured with the replica
  // exactly as it comes out of beacon_shapes, and renormalising it would move
  // every power-form threshold without touching a threshold.
  auto gold_ifft_ci16 = Utils::cfloat_to_cint16(shape.replica());
  gold_cf32_.assign(shape.replica().begin(), shape.replica().end());
  // The sentinels resolve against the shape and the slot layout now that both
  // are known; what is printed here is the configuration actually used.
  if (!shape.numerologyHeld()) {
    MLPD_WARN(
        "beacon_type %s: %.6g MSPS has no whole power-of-two symbol size for the NR "
        "subcarrier spacing (%.0f kHz); the shipped 128-point symbols are built "
        "instead, so the spacing on the air is %.1f kHz\n",
        beacon_type_.c_str(), rate() / 1e6, num.scs_hz / 1e3, rate() / 128.0 / 1e3);
  }
  houdini::sync::ResolveContext rctx;
  rctx.replica_len = shape.replicaLen();
  rctx.prefix_samples = static_cast<double>(prefix_);
  rctx.platform = platform();
  rctx.single_copy_replica = shape.singleCopy();
  rctx.clients = num_cl_sdrs_;
  rctx.backend_applies_config = houdini::sync::Detector::backendAppliesConfigByDefault();
  sync_.resolve(rctx);
  CommsLib::setCorrelatorThreads(static_cast<unsigned>(sync_.detector.corr_threads));
  MLPD_INFO("%s", sync_.describe().c_str());
  std::cout << "Beacon: type " << beacon_type_ << ", core " << shape.coreLen()
            << " samples, matched field " << shape.replicaReps() << " x "
            << shape.replicaLen() << " at offset "
            << shape.replicaOff() << " (beacon end = index + "
            << shape.replicaTail() << ")"
            << (shape.guardLen() ? " (cyclic guard " : " (no guard")
            << (shape.guardLen()
                    ? std::to_string(shape.guardLen()) + ")"
                    : ")")
            << ", PAPR " << shape.paprDb() << " dB" << std::endl;

  if (Sounder::runOptions().dump_gold) {  // the exact find_beacon match
    FILE* f = std::fopen(Utils::dumpPath("gold.bin").c_str(), "wb");
    if (f == nullptr) {
      MLPD_WARN("--dump_gold: cannot open %s (%s)\n", Utils::dumpPath("gold.bin").c_str(),
                std::strerror(errno));
    }
    if (f) {
      for (const auto& c : gold_cf32_) {
        float v[2] = {c.real(), c.imag()};
        std::fwrite(v, sizeof(float), 2, f);
      }
      std::fclose(f);
      std::printf("Dumped gold_cf32 (%zu samp) to %s\n", gold_cf32_.size(),
                  Utils::dumpPath("gold.bin").c_str());
    }
  }

  // A detector index hundreds of samples early on a strong link is the
  // earliest-crossing pick locking onto the STS preamble (16-periodic, so
  // lag-128 self-coherent), not the beacon's cyclic guard: `legacy_guard`
  // measures the same index as every other shape under
  // CommsLib::BeaconPick::kTargetedArgmax (comms-lib.h; BACKLOG AP-34).
  beacon_ci16_ = Utils::cfloat_to_cint16(shape.core());
  beacon_size_ = beacon_ci16_.size();
  if (Sounder::runOptions().dump_gold) {
    // The beacon core for the CONFIGURED shape (pre-prefix, unconjugated, at
    // the shape's own scale) -- what buildHoudiniBeacon conjugates and scales
    // into the replay RAM. Lets offline tools (tests/demo-verify) construct the
    // exact TX waveform. Length is beacon_size(), NOT a constant: 496 for
    // legacy, the shape's own for the others.
    FILE* f = std::fopen(Utils::dumpPath("beacon_core.bin").c_str(), "wb");
    if (f == nullptr) {
      MLPD_WARN("--dump_gold: cannot open %s (%s)\n",
                Utils::dumpPath("beacon_core.bin").c_str(), std::strerror(errno));
    }
    if (f) {
      std::fwrite(beacon_ci16_.data(), sizeof(std::complex<int16_t>),
                  beacon_ci16_.size(), f);
      std::fclose(f);
      std::printf("Dumped beacon core (%zu samp ci16) to %s\n", beacon_ci16_.size(),
                  Utils::dumpPath("beacon_core.bin").c_str());
    }
  }

  if (slot_samp_size_ < beacon_size_) {
    std::string msg = "Minimum supported slot_samp_size is ";
    msg += std::to_string(beacon_size_);
    msg += ". Current slot_samp_size is " + std::to_string(slot_samp_size_);
    throw std::invalid_argument(msg);
  }

  beacon_ = Utils::cint16_to_uint32(beacon_ci16_, false, "QI");
  coeffs_ = Utils::cint16_to_uint32(gold_ifft_ci16, true, "QI");

  std::vector<std::complex<int16_t>> post_beacon_zpad(
      slot_samp_size_ - beacon_size_, 0);
  beacon_ci16_.insert(beacon_ci16_.begin(), prefix_zpad.begin(),
                      prefix_zpad.end());
  beacon_ci16_.insert(beacon_ci16_.end(), post_beacon_zpad.begin(),
                      post_beacon_zpad.end());
  beacon_ci16_.insert(beacon_ci16_.end(), postfix_zpad.begin(),
                      postfix_zpad.end());

  neg_beacon_ci16_.resize(beacon_ci16_.size());
  for (size_t i = 0; i < beacon_ci16_.size(); i++) {
    neg_beacon_ci16_.at(i) = std::complex<int16_t>(0, 0) - beacon_ci16_.at(i);
  }

  // compose pilot slot
  // Refuse rather than clamp: a silent clamp leaves ofdm_data_num, the slot
  // length and every derived bandwidth describing a waveform that is not the
  // one generated (fft 4096 clamped to 2048 doubles the computed occupancy).
  if (fft_size_ > kMaxSupportedFFTSize) {
    throw std::invalid_argument("fft_size " + std::to_string(fft_size_) + " above " +
                                std::to_string(kMaxSupportedFFTSize));
  }
  // Below the floor, samps_per_slot_ was already sized from the configured
  // fft_size while the pilot is built at 64 points, so the pilot-slot length
  // check in buildBand refuses the config (if the slot-size check above has
  // not already).
  if (fft_size_ < kMinSupportedFFTSize) {
    fft_size_ = kMinSupportedFFTSize;
    std::cout << "Unsupported fft size! Setting fft size to "
              << kMinSupportedFFTSize << "..." << std::endl;
  }
  if (cp_size_ > kMaxSupportedCPSize) {
    throw std::invalid_argument("cp_size " + std::to_string(cp_size_) + " above " +
                                std::to_string(kMaxSupportedCPSize));
  }


  // The pilot and the UE data slot, per band (AP-85). Band 0 is ofdm_data_num
  // and the single-band accessors read it (pilot_ci16(), data_ind(), ...); a
  // channel with a channel_ofdm_data_num entry carries the band of its tone
  // count. One construction serves every band, so a config without the key
  // builds band 0 alone.
  ue_data_mod_order_ = (cl_data_mod_ == "QAM64")   ? 6
                       : (cl_data_mod_ == "QAM16") ? 4
                                                   : 2;  // bits/symbol (QPSK)
  const float tx_scale_cfg = tx_scale_;  // 0 = each band derives its own
  bands_.clear();
  band_of_channel_.clear();
  bands_.push_back(buildBand(symbol_data_subcarrier_num_, tx_scale_cfg));
  symbol_data_subcarrier_num_ = bands_.at(0).data_num;  // 52 at fft 64 (802.11)
  tx_scale_ = bands_.at(0).tx_scale;

  pilot_ = Utils::cint16_to_uint32(bands_.at(0).pilot_ci16, false, "QI");

  // Pad to the Iris FPGA TX_RAM (4096 words), only when the pilot is shorter: a
  // longer slot (the 5G-like numerology's 61440 samples) is left as it is.
  // Houdini does not use this RAM image.
  if (pilot_.size() < kFpgaTxRamSize) pilot_.resize(kFpgaTxRamSize, 0);
#if DEBUG_PRINT
  for (size_t j = 0; j < bands_.at(0).pilot_ci16.size(); j++) {
    std::cout << "Pilot[" << j << "]: \t " << bands_.at(0).pilot_ci16.at(j) << std::endl;
  }
#endif

  for (const auto& [ch, n] : channel_data_num_) {
    size_t b = 0;
    while (b < bands_.size() && bands_.at(b).data_num != n) ++b;
    if (b == bands_.size()) {
      std::printf("Channel %c band: %zu tones (+-%.3f MHz)\n", static_cast<char>('A' + ch), n,
                  static_cast<double>(n) * rate_ / static_cast<double>(fft_size_) / 2.0 / 1e6);
      bands_.push_back(buildBand(n, tx_scale_cfg));
    }
    band_of_channel_[ch] = b;
  }
}

OfdmBand Config::buildBand(size_t data_num, float tx_scale_cfg) const {
  std::vector<std::complex<int16_t>> prefix_zpad(prefix_, 0);
  std::vector<std::complex<int16_t>> postfix_zpad(postfix_, 0);
  OfdmBand band;
  band.data_num = data_num;
  if (fft_size_ == 64) {
    band.pilot_sym_f = CommsLib::getSequence(CommsLib::LTS_SEQ_F);
    band.pilot_sym_t = CommsLib::getSequence(CommsLib::LTS_SEQ);
    band.data_num = Consts::kNumMappedSubcarriers_80211;
  } else {  // Construct Zadoff-Chu-based pilot
    // The ZC tones on the data_num CENTRE subcarriers of an fft_size grid (the
    // placement getDataSc uses), and the symbol the IFFT of that grid at
    // fft_size. Do not take the generator's time-domain symbol: it pads its
    // tones to the next power of two ABOVE data_num, which equals fft_size only
    // when data_num does (at fft 256 with 96 tones that is a 128-point symbol,
    // with the wrong slot length and twice the tone spacing).
    const auto zc = CommsLib::getSequence(CommsLib::LTE_ZADOFF_CHU_F, data_num);
    const size_t padded = zc.at(0).size();
    const size_t lead = (padded - data_num) / 2;  // the generator's own centring
    const size_t start = (fft_size_ - data_num) / 2;
    if (data_num > fft_size_) {
      throw std::invalid_argument("ofdm_data_num " + std::to_string(data_num) + " exceeds fft_size " +
                                  std::to_string(fft_size_));
    }
    band.pilot_sym_f.assign(2, std::vector<float>(fft_size_, 0.0f));
    std::vector<std::complex<float>> grid(fft_size_, std::complex<float>(0.0f, 0.0f));
    for (size_t i = 0; i < data_num; ++i) {
      band.pilot_sym_f[0][start + i] = zc[0][lead + i];
      band.pilot_sym_f[1][start + i] = zc[1][lead + i];
      grid[start + i] = std::complex<float>(zc[0][lead + i], zc[1][lead + i]);
    }
    const auto t = CommsLib::IFFT(grid, static_cast<int>(fft_size_), 1.f / static_cast<float>(fft_size_), false, true);
    band.pilot_sym_t.assign(2, std::vector<float>(fft_size_, 0.0f));
    for (size_t i = 0; i < fft_size_; ++i) {
      band.pilot_sym_t[0][i] = t[i].real();
      band.pilot_sym_t[1][i] = t[i].imag();
    }
  }

  auto iq_tmp_ci16 = Utils::float_to_cint16(band.pilot_sym_t);
  auto iq_cf = Utils::cint16_to_cfloat(iq_tmp_ci16);
  float max_amp = 0;
  for (size_t i = 0; i < iq_cf.size(); i++) {
    float this_amp = std::abs(iq_cf.at(i));
    if (this_amp > max_amp) max_amp = this_amp;
  }
  std::printf("Max pilot amplitude = %.2f\n", max_amp);
  // Amplitude backoff: houdini targets ~1/2 FS; its data slot is separately
  // normalized below to the same realized peak, and cfloat_to_cint16
  // saturates. Iris/UHD keep x4: their file-based UL data (data_generator.cc)
  // inherits tx_scale with NO peak normalization, so halving the backoff there
  // would clip real PAPR. Each band is normalized on its own (AP-85).
  const float ofdm_pwr_scale_lin = is_houdini() ? 2.0f : 4.0f;
  band.tx_scale = tx_scale_cfg;
  if (band.tx_scale == 0) {
    band.tx_scale = 1 / (ofdm_pwr_scale_lin * max_amp);
  }
  for (size_t i = 0; i < iq_cf.size(); i++) {
    iq_cf.at(i) *= band.tx_scale;
  }
  auto iq_ci16 = Utils::cfloat_to_cint16(iq_cf);
  // copy the CP via a temp: inserting a container's own tail into its front
  // is UB ([sequence.reqmts])
  {
    std::vector<std::complex<int16_t>> cp_tmp(iq_ci16.end() - cp_size_,
                                              iq_ci16.end());
    iq_ci16.insert(iq_ci16.begin(), cp_tmp.begin(), cp_tmp.end());
  }

  band.pilot_ci16.insert(band.pilot_ci16.begin(), prefix_zpad.begin(), prefix_zpad.end());
  for (size_t i = 0; i < symbol_per_slot_; i++)
    band.pilot_ci16.insert(band.pilot_ci16.end(), iq_ci16.begin(), iq_ci16.end());
  band.pilot_ci16.insert(band.pilot_ci16.end(), postfix_zpad.begin(), postfix_zpad.end());
  // Every consumer (the UE's burst composition copies samps_per_slot samples
  // from it, the BS's CSI windows fft_size bodies against pilot_sym_f) takes
  // the pilot slot to be exactly one slot long: refuse anything else here,
  // where the numbers are known, rather than on the rig.
  if (band.pilot_ci16.size() != samps_per_slot_) {
    throw std::invalid_argument("pilot slot is " + std::to_string(band.pilot_ci16.size()) + " samples, the slot " +
                                std::to_string(samps_per_slot_) + " (fft_size " + std::to_string(fft_size_) +
                                ", cp_size " + std::to_string(cp_size_) + ")");
  }

  band.data_ind = CommsLib::getDataSc(fft_size_, band.data_num);
  band.pilot_sc_ind = CommsLib::getPilotScIndex(fft_size_, band.data_num);
  if (fft_size_ == Consts::kFftSize_80211) {
    band.pilot_sc = CommsLib::getPilotScValue(fft_size_, band.data_num);
  } else {
    // The data symbols' pilot tones carry the PILOT's own values at those
    // subcarriers: the ZC grid built above (DC-centred, unit magnitude, the
    // placement the data symbol uses). Not getPilotScValue: it reads an FFT of
    // the generator's power-of-two time sequence at natural bins, which at
    // fft != ofdm_data_num lands mostly between its spectral lobes (near-zero
    // pilot tones, so the BS's timing fit and phase fix would run on noise).
    for (const size_t k : band.pilot_sc_ind)
      band.pilot_sc.push_back(std::complex<float>(band.pilot_sym_f.at(0).at(k), band.pilot_sym_f.at(1).at(k)));
  }

  // UE uplink-data slot (symbol U): a DISTINCT random modulated OFDM symbol per
  // symbol slot (so the BS tells it from the identical-symbol pilot by
  // self-similarity), demodulable offline (plot_hdf5.py) against the reference
  // loadULData writes:
  //   data subcarriers  <- modulated symbols (modulate() takes SYMBOL INDICES 0..M-1,
  //                        NOT bits), and
  //   pilot subcarriers <- the known OFDM pilot values (for per-symbol phase tracking).
  // ue_data_f keeps the freq-domain reference to write ul_data_f_*.bin.
  // Every band draws from the same fixed seed; the BS separates the bands by
  // channel, so nothing couples them (AP-85: independent data per band).
  const int mod_alph = 1 << ue_data_mod_order_;  // 4 / 16 / 64
  const size_t n_data = band.data_ind.size();
  std::mt19937 rng(0xC0FFEE);  // fixed seed -> reproducible constellation
  band.ue_data_ci16.insert(band.ue_data_ci16.end(), prefix_zpad.begin(), prefix_zpad.end());
  // Two passes: build every symbol's time-domain float first and find the
  // slot's global peak, then scale the whole slot so its REALIZED peak equals
  // the pilot's (tx_scale x max_amp), so both exercise the same DAC range (the
  // pilot's tx_scale alone leaves the data at its raw OFDM PAPR, about twice
  // the pilot peak).
  std::vector<std::vector<std::complex<float>>> data_syms_t;
  float data_gmax = 0.0f;
  for (size_t sym = 0; sym < symbol_per_slot_; ++sym) {
    std::vector<uint8_t> syms_in(n_data);
    for (auto& v : syms_in) v = static_cast<uint8_t>(rng() % mod_alph);
    auto mod_data = CommsLib::modulate(syms_in, ue_data_mod_order_);
    std::vector<std::complex<float>> ofdm_sym(fft_size_, {0.0f, 0.0f});  // DC-centered
    for (size_t j = 0; j < n_data && j < mod_data.size(); ++j)
      ofdm_sym[band.data_ind.at(j)] = mod_data[j];
    for (size_t c = 0; c < band.pilot_sc.size(); ++c)  // OFDM pilot subcarriers
      ofdm_sym[band.pilot_sc_ind.at(c)] = band.pilot_sc.at(c);
    band.ue_data_f.insert(band.ue_data_f.end(), ofdm_sym.begin(), ofdm_sym.end());
    auto data_t = CommsLib::IFFT(ofdm_sym, fft_size_, 1.0f / fft_size_, false, true);
    for (const auto& v : data_t) data_gmax = std::max(data_gmax, std::abs(v));
    data_syms_t.push_back(std::move(data_t));
  }
  const float pilot_peak_f = band.tx_scale * max_amp;  // the pilot's realized peak
  const float dscale = (data_gmax > 0.0f) ? pilot_peak_f / data_gmax
                                          : ((band.tx_scale > 0.0f) ? band.tx_scale : 0.5f);
  for (auto& data_t : data_syms_t) {
    for (auto& v : data_t) v *= dscale;
    auto data_iq = Utils::cfloat_to_cint16(data_t);
    {  // CP via a temp (same UB note as the pilot's CP insert above)
      std::vector<std::complex<int16_t>> cp_tmp(data_iq.end() - cp_size_,
                                                data_iq.end());
      data_iq.insert(data_iq.begin(), cp_tmp.begin(), cp_tmp.end());
    }
    band.ue_data_ci16.insert(band.ue_data_ci16.end(), data_iq.begin(), data_iq.end());
  }
  band.ue_data_ci16.insert(band.ue_data_ci16.end(), postfix_zpad.begin(), postfix_zpad.end());

  // Report the realized TX peaks in DAC counts, the waveform's digital level
  // (the analog gains are houdini_tx_gain_db / houdini_rx_gain_db, mode V
  // only). The data slot is normalized to the pilot's realized peak above;
  // print both so any regression in that equalization is visible.
  auto peak_counts = [](const std::vector<std::complex<int16_t>>& v) {
    int p = 0;
    for (const auto& s : v)
      p = std::max({p, std::abs((int)s.real()), std::abs((int)s.imag())});
    return p;
  };
  const int pilot_pk = peak_counts(band.pilot_ci16);
  const int data_pk = peak_counts(band.ue_data_ci16);
  std::printf(
      "TX peaks (int16 counts): pilot %d (%.1f%% FS), UE data %d (%.1f%% FS), "
      "tx_scale %.4f\n",
      pilot_pk, 100.0 * pilot_pk / 32768.0, data_pk,
      100.0 * data_pk / 32768.0, band.tx_scale);
  return band;
}

void Config::loadULData() {
  // compose data slot
  if (ul_data_slot_present_) {
    // For now, we're reading one frame worth of data
    for (size_t i = 0; i < num_cl_sdrs_; i++) {
      std::string filename_tag = cl_data_mod_ + "_" +
                                 std::to_string(symbol_data_subcarrier_num_) +
                                 "_" + std::to_string(fft_size_) + "_" +
                                 std::to_string(symbol_per_slot_) + "_" +
                                 std::to_string(cl_ul_slots_[i].size()) + "_" +
                                 std::to_string(ul_data_frame_num_) + "_" +
                                 cl_channel_ + "_" + std::to_string(i) + ".bin";

      std::string filename_ul_data_f =
          directory_ + "/ul_data_f_" + filename_tag;
      ul_tx_fd_data_files_.push_back("ul_data_f_" + filename_tag);
      std::string filename_ul_data_t =
          directory_ + "/ul_data_t_" + filename_tag;
      ul_tx_td_data_files_.push_back(filename_ul_data_t);

      // Houdini transmits the in-process UE data slot (band 0's ue_data_f), so write its
      // freq-domain reference to the ul_data_f_*.bin that TX_FD_DATA_FILENAMES points
      // to -- plot_hdf5.py needs it to demodulate (the file-based DataGenerator path
      // is bypassed on Houdini). Layout matches the reader: [frame][slot][ch][sym]
      // [fft] interleaved f32 I/Q; ue_data_f is one frame/slot/ch (sym x fft).
      const auto& ue_data_f = bands_.at(0).ue_data_f;
      if (is_houdini() && !ue_data_f.empty()) {
        FILE* fp = std::fopen(filename_ul_data_f.c_str(), "wb");
        if (fp != nullptr) {
          for (const auto& v : ue_data_f) {
            const float re = v.real(), im = v.imag();
            std::fwrite(&re, sizeof(float), 1, fp);
            std::fwrite(&im, sizeof(float), 1, fp);
          }
          std::fclose(fp);
          MLPD_INFO("Wrote UE UL freq-domain reference (%zu complex) to %s\n",
                    ue_data_f.size(), filename_ul_data_f.c_str());
        } else {
          MLPD_WARN("Could not write UL reference %s (plot_hdf5 demod unavailable)\n",
                    filename_ul_data_f.c_str());
        }
      }
    }
  }
}

void Config::loadDLData() {
  // compose data slot
  if (dl_data_slot_present_) {
    // For now, we're reading one frame worth of data
    for (size_t i = 0; i < num_bs_sdrs_all_; i++) {
      std::string filename_tag =
          data_mod_ + "_" + std::to_string(symbol_data_subcarrier_num_) + "_" +
          std::to_string(fft_size_) + "_" + std::to_string(symbol_per_slot_) +
          "_" + std::to_string(dl_slot_per_frame_) + "_" +
          std::to_string(dl_data_frame_num_) + "_" + bs_channel_ + "_" +
          std::to_string(i) + ".bin";

      std::string filename_dl_data_f =
          directory_ + "/dl_data_f_" + filename_tag;
      dl_tx_fd_data_files_.push_back("dl_data_f_" + filename_tag);

      std::string filename_dl_data_t =
          directory_ + "/dl_data_t_" + filename_tag;
      dl_tx_td_data_files_.push_back(filename_dl_data_t);
    }
  }
}

const OfdmBand& Config::bsRxBand(size_t ant) const {
  const auto chs = Utils::strToChannels(bs_rx_channel_);
  return band(chs.at(ant % chs.size()));
}

const OfdmBand& Config::ueTxBand(size_t lane) const {
  return band(Utils::strToChannels(cl_tx_channel_).at(lane));
}

std::map<size_t, double> Config::channel_half_bw_hz(void) const {
  std::map<size_t, double> m;
  for (const auto& [ch, n] : channel_data_num_)
    m[ch] = static_cast<double>(n) * rate_ / static_cast<double>(fft_size_) / 2.0;
  return m;
}

size_t Config::getNumBsSdrs() {
  size_t sdr_num;
  /* Total number of Sdrs across cells */
  if (this->bs_present_ == false) {
    sdr_num = 1;
  } else {
    sdr_num = num_bs_sdrs_all_;
    for (size_t i = 0; i < num_cells_; i++) {
      if (internal_measurement_ == true && ref_node_enable_ == true) sdr_num--;
    }
  }
  return sdr_num;
}

size_t Config::getTotNumAntennas() {
  size_t ret;
  if (this->bs_present_ == false) {
    ret = 0;
  } else {
    ret = this->getNumBsSdrs() * bs_channel_.length();
  }
  return ret;
}

size_t Config::getNumRecordedSdrs() {
  size_t ret = 0;
  /* Total number of antennas across cells */
  if (this->bs_present_ == true) {
    ret += getNumBsSdrs();
  }
  if (this->client_present_ == true) {
    // Only consider clients that have 'D' in their schedule
    for (size_t i = 0; i < cl_dl_slots_.size(); i++) {
      if (cl_dl_slots_.at(i).size() > 0) ret++;
    }
  }
  return ret;
}

Config::~Config() {}

int Config::getClientId(size_t radio_id, size_t slot_id) {
  // TODO: Consider cell_id
  std::vector<size_t>::iterator it;
  it = find(pilot_slots_.at(radio_id).begin(), pilot_slots_.at(radio_id).end(),
            slot_id);
  if (it != pilot_slots_.at(radio_id).end()) {
    return (int)(it - pilot_slots_.at(radio_id).begin());
  }
  return -1;
}

int Config::getNoiseSlotIndex(size_t radio_id, size_t slot_id) {
  // TODO: Consider cell_id
  std::vector<size_t>::iterator it;
  it = find(noise_slots_.at(radio_id).begin(), noise_slots_.at(radio_id).end(),
            slot_id);
  if (it != noise_slots_.at(radio_id).end())
    return (int)(it - noise_slots_.at(radio_id).begin());
  return -1;
}

int Config::getUlSlotIndex(size_t radio_id, size_t slot_id) {
  // TODO: Consider cell_id
  std::vector<size_t>::iterator it;
  it = find(ul_slots_.at(radio_id).begin(), ul_slots_.at(radio_id).end(),
            slot_id);
  if (it != ul_slots_.at(radio_id).end()) {
    return (int)(it - ul_slots_.at(radio_id).begin());
  }
  return -1;
}

int Config::getDlSlotIndex(size_t radio_id, size_t slot_id) {
  std::vector<size_t>::iterator it;
  it = find(cl_dl_slots_.at(radio_id).begin(), cl_dl_slots_.at(radio_id).end(),
            slot_id);
  if (it != cl_dl_slots_.at(radio_id).end())
    return (int)(it - cl_dl_slots_.at(radio_id).begin());
  return -1;
}

bool Config::isPilot(size_t cell_id, size_t radio_id, size_t slot_id) {
  try {
    return bs_array_frames_.at(cell_id).at(radio_id).at(slot_id) == 'P';
  } catch (const std::out_of_range&) {
    return false;
  }
}

bool Config::isNoise(size_t cell_id, size_t radio_id, size_t slot_id) {
  try {
    return bs_array_frames_.at(cell_id).at(radio_id).at(slot_id) == 'N';
  } catch (const std::out_of_range&) {
    return false;
  }
}

bool Config::isUlData(size_t cell_id, size_t radio_id, size_t slot_id) {
  try {
    return bs_array_frames_.at(cell_id).at(radio_id).at(slot_id) == 'U';
  } catch (const std::out_of_range&) {
    return false;
  }
}

bool Config::isDlData(size_t radio_id, size_t slot_id) {
  try {
    return cl_frames_.at(radio_id).at(slot_id) == 'D';
  } catch (const std::out_of_range&) {
    return false;
  }
}

unsigned Config::getCoreCount() {
  unsigned n_cores = std::thread::hardware_concurrency();
#if DEBUG_PRINT
  std::cout << "number of CPU cores " << std::to_string(n_cores) << std::endl;
#endif
  return n_cores;
}

extern "C" {
__attribute__((visibility("default"))) Config* Config_new(char* filename,
                                                          char* storepath,
                                                          bool bs_only,
                                                          bool client_only,
                                                          bool calibrate) {
  Config* cfg =
      new Config(filename, storepath, bs_only, client_only, calibrate);
  return cfg;
}
}
