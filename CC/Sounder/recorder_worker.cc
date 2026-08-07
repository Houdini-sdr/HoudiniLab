/*
 Copyright (c) 2018-2022, Rice University
 RENEW OPEN SOURCE LICENSE: http://renew-wireless.org/license

----------------------------------------------------------------------
 Class to handle writting data to an hdf5 file
---------------------------------------------------------------------
*/

#include "include/recorder_worker.h"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <string>

#include "include/logger.h"
#include "include/macros.h"
#include "include/utils.h"

namespace Sounder {

// Parse HOUDINI_CSI_UDP ("host:port"), open a connected UDP socket, precompute the
// DC-centered freq-domain pilot reference + a DC-centered DFT matrix, and set the
// per-antenna send throttle. Enables view mode when the env is present.
void RecorderWorker::initCsi(void) {
  const char* dst = std::getenv("HOUDINI_CSI_UDP");
  if (dst == nullptr) return;
  std::string s(dst);
  const auto colon = s.find(':');
  const std::string host = (colon == std::string::npos) ? "127.0.0.1"
                                                        : s.substr(0, colon);
  const int port =
      (colon == std::string::npos) ? 9999 : std::atoi(s.substr(colon + 1).c_str());
  csi_sock_ = ::socket(AF_INET, SOCK_DGRAM, 0);
  if (csi_sock_ < 0) {
    MLPD_ERROR("CSI view: socket() failed, view mode disabled\n");
    return;
  }
  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(static_cast<uint16_t>(port));
  if (::inet_pton(AF_INET, host.c_str(), &addr.sin_addr) != 1 ||
      ::connect(csi_sock_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) {
    MLPD_ERROR("CSI view: bad dest %s:%d, view mode disabled\n", host.c_str(),
               port);
    ::close(csi_sock_);
    csi_sock_ = -1;
    return;
  }
  const int N = static_cast<int>(cfg_->fft_size());
  auto& pf = cfg_->pilot_sym_f();
  pilot_ref_.resize(N);
  for (int k = 0; k < N; ++k)
    pilot_ref_[k] = {pf.at(0).at(k), pf.at(1).at(k)};  // DC-centered
  // Xs[k] = sum_n x[n] * dft_[k*N+n] gives the DC-centered spectrum directly
  // (natural bin (k+N/2)%N), matching the DC-centered pilot reference.
  dft_.resize(static_cast<size_t>(N) * N);
  for (int k = 0; k < N; ++k) {
    const int m = (k + N / 2) % N;
    for (int n = 0; n < N; ++n) {
      const double ph = -2.0 * M_PI * m * n / N;
      dft_[static_cast<size_t>(k) * N + n] = {static_cast<float>(std::cos(ph)),
                                              static_cast<float>(std::sin(ph))};
    }
  }
  double fps = 30.0;
  if (const char* f = std::getenv("HOUDINI_CSI_FPS")) fps = std::max(0.5, atof(f));
  csi_throttle_ns_ = 1e9 / fps;
  view_mode_ = true;
  MLPD_INFO("CSI view mode: streaming to %s:%d (%d subcarriers, ~%.0f fps/ant)\n",
            host.c_str(), port, N, fps);
}

// Energy leading edge of a slot (first index where the sliding-64 power crosses
// 15% of the slot peak). Equals `prefix` when the slot is perfectly aligned, but
// tolerates a small residual misalignment so the OFDM symbol windows land on the
// real symbol boundaries (a whole-symbol offset otherwise causes inter-symbol
// interference -> spread constellation / bad CSI).
int RecorderWorker::slotEnergyStart(const short* d, int slot) const {
  std::vector<double> cs(static_cast<size_t>(slot) + 1, 0.0);
  for (int i = 0; i < slot; ++i) {
    const double re = d[2 * i], im = d[2 * i + 1];
    cs[i + 1] = cs[i] + re * re + im * im;
  }
  double peak = 0.0;
  for (int i = 0; i + 64 <= slot; ++i)
    peak = std::max(peak, cs[i + 64] - cs[i]);
  const double thr = 0.15 * peak;
  for (int i = 0; i + 64 <= slot; ++i)
    if (cs[i + 64] - cs[i] > thr) return i;
  return this->cfg_->prefix();
}

// DC-centered FFT of the fft-size symbol body starting at sample `base` in `d`.
std::vector<std::complex<float>> RecorderWorker::symbolFft(const short* d,
                                                          int base) const {
  const int N = static_cast<int>(cfg_->fft_size());
  std::vector<std::complex<float>> out(N);
  for (int k = 0; k < N; ++k) {
    const std::complex<float>* row = &dft_[static_cast<size_t>(k) * N];
    std::complex<float> acc(0.0f, 0.0f);
    for (int n = 0; n < N; ++n) {
      const std::complex<float> x(static_cast<float>(d[2 * (base + n)]),
                                  static_cast<float>(d[2 * (base + n) + 1]));
      acc += x * row[n];
    }
    out[k] = acc;
  }
  return out;
}

// Route a received slot: pilot -> CSI (+ cache H); uplink data -> constellation.
void RecorderWorker::streamCsi(Packet* pkt, NodeType node_type) {
  const size_t num_channels = cfg_->bs_channel().size();
  const size_t radio_id = pkt->ant_id / num_channels;
  const bool is_pilot =
      cfg_->internal_measurement()
          ? (node_type == kBS)
          : cfg_->isPilot(pkt->cell_id, radio_id, pkt->slot_id);
  if (is_pilot) {
    sendCsi(pkt);
  } else if (cfg_->isUlData(pkt->cell_id, radio_id, pkt->slot_id)) {
    sendConstellation(pkt);
  }
}

// Pilot slot -> channel estimate H[k] (DC-centered), cached per antenna + streamed.
void RecorderWorker::sendCsi(Packet* pkt) {
  const int N = static_cast<int>(cfg_->fft_size());
  const int cp = static_cast<int>(cfg_->cp_size());
  const int nsym = static_cast<int>(cfg_->symbol_per_slot());
  const int slot = static_cast<int>(cfg_->samps_per_slot());
  const short* d = pkt->data;
  const int es = slotEnergyStart(d, slot);  // actual symbol-0 start (robust align)
  int s0 = nsym / 8, s1 = nsym - nsym / 8;
  if (s1 <= s0) { s0 = 0; s1 = nsym; }
  std::vector<std::complex<float>> hacc(N, {0.0f, 0.0f});
  int used = 0;
  for (int sym = s0; sym < s1; ++sym) {
    const int base = es + sym * (cp + N) + cp;
    if (base + N > slot) break;
    auto F = symbolFft(d, base);
    for (int k = 0; k < N; ++k) hacc[k] += F[k] * std::conj(pilot_ref_[k]);
    ++used;
  }
  // Cache H per antenna (always -- keeps it fresh for equalizing this ant's data).
  auto& H = csi_h_[pkt->ant_id];
  H.assign(N, {0.0f, 0.0f});
  for (int k = 0; k < N; ++k) {
    const float pw = std::norm(pilot_ref_[k]);
    if (pw > 1e-6f && used > 0) H[k] = hacc[k] / (static_cast<float>(used) * pw);
  }
  // Throttle the CSI datagram (H is cached above regardless).
  const long long now =
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::steady_clock::now().time_since_epoch())
          .count();
  auto it = csi_last_ns_.find(pkt->ant_id);
  if (it != csi_last_ns_.end() &&
      (now - it->second) < static_cast<long long>(csi_throttle_ns_))
    return;
  csi_last_ns_[pkt->ant_id] = now;
  // [magic 'CSI1'][frame][ant][num_sc][rate][H re,im]*N
  std::vector<uint8_t> buf(20 + static_cast<size_t>(8) * N);
  const uint32_t magic = 0x43534931u, fr = pkt->frame_id, an = pkt->ant_id,
                 nsc = static_cast<uint32_t>(N);
  const float rate = static_cast<float>(cfg_->rate());
  std::memcpy(&buf[0], &magic, 4);
  std::memcpy(&buf[4], &fr, 4);
  std::memcpy(&buf[8], &an, 4);
  std::memcpy(&buf[12], &nsc, 4);
  std::memcpy(&buf[16], &rate, 4);
  for (int k = 0; k < N; ++k) {
    const float re = H[k].real(), im = H[k].imag();
    std::memcpy(&buf[20 + 8 * k], &re, 4);
    std::memcpy(&buf[24 + 8 * k], &im, 4);
  }
  (void)::send(csi_sock_, buf.data(), buf.size(), 0);
}

// Uplink-data slot -> equalize with the cached H and stream the constellation.
void RecorderWorker::sendConstellation(Packet* pkt) {
  auto hit = csi_h_.find(pkt->ant_id);
  if (hit == csi_h_.end() || hit->second.empty()) return;  // no CSI yet
  const long long now =
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::steady_clock::now().time_since_epoch())
          .count();
  auto it = cns_last_ns_.find(pkt->ant_id);
  if (it != cns_last_ns_.end() &&
      (now - it->second) < static_cast<long long>(csi_throttle_ns_))
    return;
  cns_last_ns_[pkt->ant_id] = now;

  const std::vector<std::complex<float>>& H = hit->second;
  const int N = static_cast<int>(cfg_->fft_size());
  const int cp = static_cast<int>(cfg_->cp_size());
  const int nsym = static_cast<int>(cfg_->symbol_per_slot());
  const int slot = static_cast<int>(cfg_->samps_per_slot());
  const short* d = pkt->data;
  const int es = slotEnergyStart(d, slot);  // actual symbol-0 start (robust align)
  const auto& data_ind = cfg_->data_ind();
  // One-shot raw dump for offline analysis: [N cp prefix nsym ndata i32]
  // [H re,im f32]*N [data_ind i32]*ndata [U slot re,im i16]*slot.
  if (std::getenv("HOUDINI_CSI_DUMP") != nullptr) {
    static std::atomic<bool> dumped{false};
    bool exp = false;
    if (dumped.compare_exchange_strong(exp, true)) {
      FILE* f = std::fopen("/tmp/cns_dump.bin", "wb");
      if (f) {
        const int32_t hdr[5] = {N, cp, es, nsym,
                                static_cast<int32_t>(data_ind.size())};
        std::fwrite(hdr, sizeof(int32_t), 5, f);
        for (int k = 0; k < N; ++k) {
          const float re = H[k].real(), im = H[k].imag();
          std::fwrite(&re, 4, 1, f);
          std::fwrite(&im, 4, 1, f);
        }
        for (size_t j = 0; j < data_ind.size(); ++j) {
          const int32_t di = static_cast<int32_t>(data_ind[j]);
          std::fwrite(&di, 4, 1, f);
        }
        std::fwrite(d, sizeof(short), static_cast<size_t>(slot) * 2, f);
        std::fclose(f);
        MLPD_INFO("CSI dump written to /tmp/cns_dump.bin\n");
      }
    }
  }
  // Skip deep-fade data subcarriers: zero-forcing (X=Y/H) amplifies noise where
  // |H| is small, which blows up the constellation. Keep only subcarriers with
  // |H| >= 0.4 x median|H| (over the data subcarriers).
  std::vector<float> habs;
  habs.reserve(data_ind.size());
  for (size_t j = 0; j < data_ind.size(); ++j)
    habs.push_back(std::abs(H[data_ind[j]]));
  std::vector<float> tmp = habs;
  std::nth_element(tmp.begin(), tmp.begin() + tmp.size() / 2, tmp.end());
  const float hmed = tmp.empty() ? 0.0f : tmp[tmp.size() / 2];
  const float hmin = 0.4f * hmed;
  const int mod_ord = cfg_->ue_data_mod_order();
  const size_t kMaxPts = 600;
  std::vector<std::complex<float>> pts;
  pts.reserve(kMaxPts);
  int s0 = nsym / 8, s1 = nsym - nsym / 8;
  if (s1 <= s0) { s0 = 0; s1 = nsym; }
  double psum = 0.0;
  for (int sym = s0; sym < s1 && pts.size() < kMaxPts; ++sym) {
    const int base = es + sym * (cp + N) + cp;
    if (base + N > slot) break;
    auto Y = symbolFft(d, base);
    // Equalize this symbol's data subcarriers.
    std::vector<std::complex<float>> sx;
    sx.reserve(data_ind.size());
    std::complex<double> acc4(0.0, 0.0);  // 4th-power sum for blind QPSK CPE
    for (size_t j = 0; j < data_ind.size(); ++j) {
      const size_t k = data_ind[j];
      const std::complex<float> h = H[k];
      if (std::abs(h) < hmin || std::norm(h) < 1e-9f) continue;  // deep fade
      const std::complex<float> x = Y[k] / h;  // zero-forcing equalizer
      sx.push_back(x);
      const std::complex<double> xd(x.real(), x.imag());
      acc4 += xd * xd * xd * xd;
    }
    // Per-symbol common-phase correction. Residual phase/CFO rotates each OFDM
    // symbol; the pilot CSI (averaged over the slot) can't track it, so estimate
    // it blind per symbol via the QPSK 4th-power (data cancels: ideal X^4 -> angle
    // pi) and de-rotate. Only for QPSK; higher-order QAM needs pilot subcarriers.
    std::complex<float> derot(1.0f, 0.0f);
    if (mod_ord == 2 && std::abs(acc4) > 0.0) {
      const double cpe = (std::arg(acc4) - M_PI) / 4.0;
      derot = std::complex<float>(std::cos(cpe), -std::sin(cpe));
    }
    for (auto& x : sx) {
      if (pts.size() >= kMaxPts) break;
      const std::complex<float> xr = x * derot;
      psum += std::norm(xr);
      pts.push_back(xr);
    }
  }
  if (pts.empty()) return;
  // Normalize to unit average power so the ideal alphabet is fixed in the GUI.
  const float scale = static_cast<float>(1.0 / std::sqrt(psum / pts.size() + 1e-12));
  // [magic 'CNS1'][frame][ant][num_pts][mod_order][X re,im]*num_pts
  std::vector<uint8_t> buf(20 + 8 * pts.size());
  const uint32_t magic = 0x434E5331u, fr = pkt->frame_id, an = pkt->ant_id,
                 npt = static_cast<uint32_t>(pts.size()),
                 mod = static_cast<uint32_t>(cfg_->ue_data_mod_order());
  std::memcpy(&buf[0], &magic, 4);
  std::memcpy(&buf[4], &fr, 4);
  std::memcpy(&buf[8], &an, 4);
  std::memcpy(&buf[12], &npt, 4);
  std::memcpy(&buf[16], &mod, 4);
  for (size_t j = 0; j < pts.size(); ++j) {
    const float re = pts[j].real() * scale, im = pts[j].imag() * scale;
    std::memcpy(&buf[20 + 8 * j], &re, 4);
    std::memcpy(&buf[24 + 8 * j], &im, 4);
  }
  (void)::send(csi_sock_, buf.data(), buf.size(), 0);
}

RecorderWorker::RecorderWorker(Config* in_cfg, size_t antenna_offset,
                               size_t num_antennas)
    : cfg_(in_cfg) {
  antenna_offset_ = antenna_offset;
  num_antennas_ = num_antennas;
  unsigned int end_antenna = (this->antenna_offset_ + this->num_antennas_) - 1;

  this->hdf5_name_ = this->cfg_->trace_file();
  size_t found_index = this->hdf5_name_.find_last_of('.');
  std::string append = "_" + std::to_string(this->antenna_offset_) + "_" +
                       std::to_string(end_antenna);
  this->hdf5_name_.insert(found_index, append);
}

RecorderWorker::~RecorderWorker() { this->finalize(); }

void RecorderWorker::init(void) {
  this->initCsi();
  if (this->view_mode_) return;  // viewing mode streams CSI, writes no HDF5
  this->hdf5_ = new Hdf5Lib(this->hdf5_name_, "Data");
  // Write Atrributes
  // ******* COMMON ******** //
  // TX/RX Frequencyfile
  this->hdf5_->write_attribute("FREQ", this->cfg_->freq());

  // BW
  this->hdf5_->write_attribute("RATE", this->cfg_->rate());

  // Number of samples for prefix (padding)
  this->hdf5_->write_attribute("PREFIX_LEN", this->cfg_->prefix());

  // Number of samples for postfix (padding)
  this->hdf5_->write_attribute("POSTFIX_LEN", this->cfg_->postfix());

  // Number of samples on each symbol including prefix and postfix
  this->hdf5_->write_attribute("SLOT_SAMP_LEN", this->cfg_->samps_per_slot());

  // Size of FFT
  this->hdf5_->write_attribute("FFT_SIZE", this->cfg_->fft_size());

  // Number of data subcarriers in ofdm symbols
  this->hdf5_->write_attribute("DATA_SUBCARRIER_NUM",
                               this->cfg_->symbol_data_subcarrier_num());

  // Length of cyclic prefix
  this->hdf5_->write_attribute("CP_LEN", this->cfg_->cp_size());

  // Downlink Pilots Enabled Flag
  this->hdf5_->write_attribute("DL_PILOTS_EN", this->cfg_->dl_pilots_en());

  // Beacon sequence type (string)
  this->hdf5_->write_attribute("BEACON_SEQ_TYPE", this->cfg_->beacon_seq());

  // Pilot sequence type (string)
  this->hdf5_->write_attribute("PILOT_SEQ_TYPE", this->cfg_->pilot_seq());

  // ******* Base Station ******** //
  // Hub IDs (vec of strings)
  this->hdf5_->write_attribute("BS_HUB_ID", this->cfg_->hub_ids());

  // BS SDR IDs
  // *** first, how many boards in each cell? ***
  std::vector<std::string> bs_sdr_num_per_cell(this->cfg_->bs_sdr_ids().size());
  for (size_t i = 0; i < bs_sdr_num_per_cell.size(); ++i) {
    bs_sdr_num_per_cell[i] =
        std::to_string(this->cfg_->bs_sdr_ids().at(i).size());
  }
  this->hdf5_->write_attribute("BS_SDR_NUM_PER_CELL", bs_sdr_num_per_cell);

  // *** second, reshape matrix into vector ***
  std::vector<std::string> bs_sdr_id;
  for (auto&& v : this->cfg_->bs_sdr_ids()) {
    bs_sdr_id.insert(bs_sdr_id.end(), v.begin(), v.end());
  }
  this->hdf5_->write_attribute("BS_SDR_ID", bs_sdr_id);

  // Number of Base Station Cells
  this->hdf5_->write_attribute("BS_NUM_CELLS", this->cfg_->num_cells());

  // How many RF channels per Iris board are enabled ("single" or "dual")
  this->hdf5_->write_attribute("BS_CH_PER_RADIO",
                               this->cfg_->bs_channel().length());

  // Frame schedule (string vector)
  // TODO: This should change to matrix when we go to multi-cell
  this->hdf5_->write_attribute("BS_FRAME_SCHED",
                               this->cfg_->bs_array_frames().at(0));

  // RX Gain RF channel A
  this->hdf5_->write_attribute("BS_RX_GAIN_A", this->cfg_->rx_gain().at(0));

  // TX Gain RF channel A
  this->hdf5_->write_attribute("BS_TX_GAIN_A", this->cfg_->tx_gain().at(0));

  // RX Gain RF channel B
  this->hdf5_->write_attribute("BS_RX_GAIN_B", this->cfg_->rx_gain().at(1));

  // TX Gain RF channel B
  this->hdf5_->write_attribute("BS_TX_GAIN_B", this->cfg_->tx_gain().at(1));

  // Beamsweep (true or false)
  this->hdf5_->write_attribute("BS_BEAMSWEEP",
                               this->cfg_->beam_sweep() ? 1 : 0);

  // Beacon Antenna
  this->hdf5_->write_attribute("BS_BEACON_ANT", this->cfg_->beacon_ant());

  // Number of antennas on Base Station (per cell)
  std::vector<std::string> bs_ant_num_per_cell(this->cfg_->bs_sdr_ids().size());
  for (size_t i = 0; i < bs_ant_num_per_cell.size(); ++i) {
    bs_ant_num_per_cell[i] =
        std::to_string(this->cfg_->bs_sdr_ids().at(i).size() *
                       this->cfg_->bs_channel().length());
  }
  this->hdf5_->write_attribute("BS_ANT_NUM_PER_CELL", bs_ant_num_per_cell);

  //If the antennas are non consective this will be an issue.
  this->hdf5_->write_attribute("ANT_OFFSET", this->antenna_offset_);
  this->hdf5_->write_attribute("ANT_NUM", this->num_antennas_);
  this->hdf5_->write_attribute("ANT_TOTAL", this->cfg_->getTotNumAntennas());

  // Number of symbols in a frame
  this->hdf5_->write_attribute("BS_FRAME_LEN", this->cfg_->slot_per_frame());

  // Number of uplink symbols per frame
  this->hdf5_->write_attribute("UL_SLOTS", this->cfg_->ul_slot_per_frame());

  // Reciprocal Calibration Mode
  bool reciprocity_cal =
      this->cfg_->internal_measurement() && this->cfg_->ref_node_enable();
  this->hdf5_->write_attribute("RECIPROCAL_CALIB", reciprocity_cal ? 1 : 0);

  // All combinations of TX/RX boards in the base station
  bool full_matrix_meas =
      this->cfg_->internal_measurement() && !this->cfg_->ref_node_enable();
  this->hdf5_->write_attribute("FULL_MATRIX_MEAS", full_matrix_meas ? 1 : 0);

  // ******* Clients ******** //
  // Freq. Domain Pilot symbols
  std::vector<double> split_vec_pilot_f(2 *
                                        this->cfg_->pilot_sym_f().at(0).size());
  for (size_t i = 0; i < this->cfg_->pilot_sym_f().at(0).size(); i++) {
    split_vec_pilot_f[2 * i + 0] = this->cfg_->pilot_sym_f().at(0).at(i);
    split_vec_pilot_f[2 * i + 1] = this->cfg_->pilot_sym_f().at(1).at(i);
  }
  this->hdf5_->write_attribute("OFDM_PILOT_F", split_vec_pilot_f);

  // Time Domain Pilot symbols
  std::vector<double> split_vec_pilot(2 *
                                      this->cfg_->pilot_sym_t().at(0).size());
  for (size_t i = 0; i < this->cfg_->pilot_sym_t().at(0).size(); i++) {
    split_vec_pilot[2 * i + 0] = this->cfg_->pilot_sym_t().at(0).at(i);
    split_vec_pilot[2 * i + 1] = this->cfg_->pilot_sym_t().at(1).at(i);
  }
  this->hdf5_->write_attribute("OFDM_PILOT", split_vec_pilot);

  // Number of Pilots
  this->hdf5_->write_attribute("PILOT_NUM", this->cfg_->pilot_slot_per_frame());

  // Data subcarriers
  if (this->cfg_->data_ind().size() > 0)
    this->hdf5_->write_attribute("OFDM_DATA_SC", this->cfg_->data_ind());

  // Pilot subcarriers (indexes)
  if (this->cfg_->pilot_sc_ind().size() > 0)
    this->hdf5_->write_attribute("OFDM_PILOT_SC", this->cfg_->pilot_sc_ind());
  if (this->cfg_->pilot_sc().size() > 0)
    this->hdf5_->write_attribute("OFDM_PILOT_SC_VALS", this->cfg_->pilot_sc());

  // Number of Client Antennas
  this->hdf5_->write_attribute("CL_NUM", this->cfg_->num_cl_antennas());

  // Data modulation
  this->hdf5_->write_attribute("CL_MODULATION", this->cfg_->cl_data_mod());

  if (this->cfg_->internal_measurement() == false ||
      this->cfg_->num_cl_antennas() > 0) {
    // Client antenna polarization
    this->hdf5_->write_attribute("CL_CH_PER_RADIO", this->cfg_->cl_sdr_ch());

    // Client AGC enable flag
    this->hdf5_->write_attribute("CL_AGC_EN", this->cfg_->cl_agc_en() ? 1 : 0);

    // RX Gain RF channel A
    this->hdf5_->write_attribute("CL_RX_GAIN_A",
                                 this->cfg_->cl_rxgain_vec().at(0));

    // TX Gain RF channel A
    this->hdf5_->write_attribute("CL_TX_GAIN_A",
                                 this->cfg_->cl_txgain_vec().at(0));

    // RX Gain RF channel B
    this->hdf5_->write_attribute("CL_RX_GAIN_B",
                                 this->cfg_->cl_rxgain_vec().at(1));

    // TX Gain RF channel B
    this->hdf5_->write_attribute("CL_TX_GAIN_B",
                                 this->cfg_->cl_txgain_vec().at(1));

    // Client frame schedule (vec of strings)
    this->hdf5_->write_attribute("CL_FRAME_SCHED", this->cfg_->cl_frames());

    // Set of client SDR IDs (vec of strings)
    this->hdf5_->write_attribute("CL_SDR_ID", this->cfg_->cl_sdr_ids());
  }

  if (this->cfg_->ul_data_slot_present()) {
    // Number of frames for UL data recorded in bit source files
    this->hdf5_->write_attribute("UL_DATA_FRAME_NUM",
                                 this->cfg_->ul_data_frame_num());

    // Names of Files including uplink tx frequency-domain data
    if (this->cfg_->ul_tx_fd_data_files().size() > 0) {
      this->hdf5_->write_attribute("TX_FD_DATA_FILENAMES",
                                   this->cfg_->ul_tx_fd_data_files());
    }
  }
  // ********************* //

  // dataset dimension
  hsize_t IQ = 2 * this->cfg_->samps_per_slot();
  std::array<hsize_t, kDsDimsNum> cdims = {
      1, 1, 1, 1, IQ};  // recording chunk size, TODO: optimize size

  if (this->cfg_->bs_rx_thread_num() > 0 &&
      this->cfg_->pilot_slot_per_frame() > 0) {
    // pilots
    datasets.push_back("Pilot_Samples");
    std::array<hsize_t, kDsDimsNum> dims_pilot = {
        MAX_FRAME_INC, this->cfg_->num_cells(),
        this->cfg_->pilot_slot_per_frame(), this->num_antennas_, IQ};
    this->hdf5_->createDataset(datasets.back(), dims_pilot, cdims);
  }
  if (this->cfg_->noise_slot_per_frame() > 0) {
    // noise
    datasets.push_back("Noise_Samples");
    std::array<hsize_t, kDsDimsNum> dims_noise = {
        MAX_FRAME_INC, this->cfg_->num_cells(),
        this->cfg_->noise_slot_per_frame(), this->num_antennas_, IQ};
    this->hdf5_->createDataset(datasets.back(), dims_noise, cdims);
  }

  if (this->cfg_->bs_rx_thread_num() > 0 &&
      this->cfg_->ul_slot_per_frame() > 0) {
    // UL data
    datasets.push_back("UplinkData");
    std::array<hsize_t, kDsDimsNum> dims_ul_data = {
        MAX_FRAME_INC, this->cfg_->num_cells(), this->cfg_->ul_slot_per_frame(),
        this->num_antennas_, IQ};
    this->hdf5_->createDataset(datasets.back(), dims_ul_data, cdims);
  }

  if (this->cfg_->cl_rx_thread_num() > 0 &&
      cfg_->cl_dl_slots().at(0).empty() == false) {
    // DL
    datasets.push_back("DownlinkData");
    std::array<hsize_t, kDsDimsNum> dims_dl_data = {
        MAX_FRAME_INC, this->cfg_->num_cells(),
        this->cfg_->cl_dl_slots().at(0).size(), this->cfg_->num_cl_antennas(),
        IQ};
    this->hdf5_->createDataset(datasets.back(), dims_dl_data, cdims);
  }

  this->hdf5_->setTargetPrimaryDimSize(MAX_FRAME_INC);
  this->hdf5_->setMaxPrimaryDimSize(cfg_->max_frame());
  this->hdf5_->openDataset();
}

void RecorderWorker::finalize(void) {
  if (this->csi_sock_ >= 0) {
    ::close(this->csi_sock_);
    this->csi_sock_ = -1;
  }
  if (this->hdf5_ != nullptr) {
    this->hdf5_->closeDataset();
    this->hdf5_->closeFile();
  }
}

void RecorderWorker::record(int tid, Packet* pkt, NodeType node_type) {
  (void)tid;
  if (this->view_mode_) {  // viewing mode: compute + stream CSI, no HDF5
    this->streamCsi(pkt, node_type);
    return;
  }
  /* TODO: remove TEMP check */
  size_t end_antenna = (this->antenna_offset_ + this->num_antennas_) - 1;
  size_t num_channels = this->cfg_->bs_channel().size();

  if ((pkt->ant_id < this->antenna_offset_) || (pkt->ant_id > end_antenna)) {
    MLPD_ERROR("Antenna id is not within range of this recorder %d, %zu:%zu",
               pkt->ant_id, this->antenna_offset_, end_antenna);
  }
  assert((pkt->ant_id >= this->antenna_offset_) &&
         (pkt->ant_id <= end_antenna));

  //Generates a ton of messages
  //MLPD_TRACE( "Tid: %d -- frame_id %u, antenna: %u\n", tid, pkt->frame_id, pkt->ant_id);

  if (kDebugPrint) {
    std::printf(
        "record            frame %d, symbol %d, cell %d, ant %d "
        "samples: %d "
        "%d %d %d %d %d %d %d ....\n",
        pkt->frame_id, pkt->slot_id, pkt->cell_id, pkt->ant_id, pkt->data[1],
        pkt->data[2], pkt->data[3], pkt->data[4], pkt->data[5], pkt->data[6],
        pkt->data[7], pkt->data[8]);
  }
  hsize_t IQ = 2 * this->cfg_->samps_per_slot();
  if ((this->cfg_->max_frame()) != 0 &&
      (pkt->frame_id > this->cfg_->max_frame())) {
    this->hdf5_->closeDataset();
    MLPD_TRACE("Closing file due to frame id %d : %zu max\n", pkt->frame_id,
               this->cfg_->max_frame());
  } else {
    // Update the max frame number.
    // Note that the 'frame_id' might be out of order.
    this->max_frame_number_ = this->hdf5_->getTargetPrimaryDimSize();
    if (pkt->frame_id >= this->max_frame_number_) {
      // Open the hdf5 file if we haven't.
      this->hdf5_->closeDataset();
      this->hdf5_->openDataset();
      this->hdf5_->setTargetPrimaryDimSize(this->max_frame_number_ +
                                           MAX_FRAME_INC);
    }

    uint32_t antenna_index = pkt->ant_id - this->antenna_offset_;
    const size_t radio_id = pkt->ant_id / num_channels;
    const size_t cell_id = pkt->cell_id;
    const size_t slot_id = pkt->slot_id;
    std::array<hsize_t, kDsDimsNum> hdfoffset = {pkt->frame_id, cell_id, 0,
                                                 antenna_index, 0};
    std::array<hsize_t, kDsDimsNum> count = {1, 1, 1, 1, IQ};
    if (this->cfg_->internal_measurement() == true) {
      if (node_type == kClient) {
        this->hdf5_->extendDataset(std::string("DownlinkData"), pkt->frame_id);
        hdfoffset[kDsDimSymbol] = this->cfg_->getDlSlotIndex(radio_id, slot_id);
        this->hdf5_->writeDataset(std::string("DownlinkData"), hdfoffset, count,
                                  pkt->data);
      } else {
        this->hdf5_->extendDataset(std::string("Pilot_Samples"), pkt->frame_id);
        hdfoffset[kDsDimSymbol] = slot_id;
        this->hdf5_->writeDataset(std::string("Pilot_Samples"), hdfoffset,
                                  count, pkt->data);
      }
    } else if (this->cfg_->isPilot(cell_id, radio_id, slot_id) == true) {
      this->hdf5_->extendDataset(std::string("Pilot_Samples"), pkt->frame_id);
      hdfoffset[kDsDimSymbol] = this->cfg_->getClientId(radio_id, slot_id);
      this->hdf5_->writeDataset(std::string("Pilot_Samples"), hdfoffset, count,
                                pkt->data);
    } else if (this->cfg_->isUlData(cell_id, radio_id, slot_id) == true) {
      this->hdf5_->extendDataset(std::string("UplinkData"), pkt->frame_id);
      hdfoffset[kDsDimSymbol] = this->cfg_->getUlSlotIndex(radio_id, slot_id);
      this->hdf5_->writeDataset(std::string("UplinkData"), hdfoffset, count,
                                pkt->data);

    } else if (this->cfg_->isDlData(radio_id, slot_id) == true) {
      this->hdf5_->extendDataset(std::string("DownlinkData"), pkt->frame_id);
      hdfoffset[kDsDimSymbol] = this->cfg_->getDlSlotIndex(radio_id, slot_id);
      this->hdf5_->writeDataset(std::string("DownlinkData"), hdfoffset, count,
                                pkt->data);
    } else if (this->cfg_->isNoise(cell_id, radio_id, slot_id) == true) {
      this->hdf5_->extendDataset(std::string("Noise_Samples"), pkt->frame_id);
      hdfoffset[kDsDimSymbol] =
          this->cfg_->getNoiseSlotIndex(radio_id, slot_id);
      this->hdf5_->writeDataset(std::string("Noise_Samples"), hdfoffset, count,
                                pkt->data);
    }
  } /* End else */
}
};  //End namespace Sounder
