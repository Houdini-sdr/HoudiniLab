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
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <string>

#include "include/logger.h"
#include "include/macros.h"
#include "include/rx_gap_sink.h"  // RxGapSink -> /Data/Gaps at finalize
#include "include/utils.h"

namespace Sounder {

// Raw-ADC envelope panel. kAdcCols columns cover the whole slot however long it is;
// 250 matches the panel width, so one column is one pixel and nothing is thrown away
// that the display could have shown.
static constexpr int kAdcCols = 250;
// Full scale for the int16 sample format this code uses everywhere (see utils.cc).
// A sample within 1% of the rail is counted as clipped. NOTE this is the rail of the
// SAMPLE FORMAT, not necessarily of the converter: if the RFSoC delivers a 14-bit
// sample that is not left-justified, the true rail is lower and this count stays 0
// while the input is in fact clipping. The peak is reported alongside for exactly
// that reason -- a peak that sits on the same value every frame IS the rail,
// whatever value it reads.
static constexpr int32_t kAdcFullScale = 32767;
static constexpr int32_t kAdcClip = (kAdcFullScale * 99) / 100;

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
  rx_conj_ = cfg_->is_houdini();  // undo the R2C mixer's spectral inversion (RFSoC only)
  if (std::getenv("HOUDINI_RX_NOCONJ")) rx_conj_ = false;  // A/B override (before/after)
  // Symbol-0 start: default the FFT window HALF A CP earlier than the nominal prefix.
  // The cyclic-prefix guard is one-sided -- a window placed early (within the CP) is a
  // valid circular shift (pure phase, recoverable), but one placed even 1 sample LATE
  // pulls the next symbol into the FFT = ISI (unrecoverable). The nominal prefix sits
  // right at that cliff edge, so beacon-relock jitter routinely tips runs into ISI
  // (measured: es=prefix 19.8% EVM -> es=prefix-CP/2 3.2% on a window-ISI run). Backing
  // off CP/2 centers the window in the guard for two-sided jitter margin. Still fully
  // manual: HOUDINI_CSI_SYM_START overrides (an int, or "auto" for the energy-edge detector).
  csi_sym_start_ = static_cast<int>(cfg_->prefix()) -
                   static_cast<int>(cfg_->cp_size()) / 2;
  if (const char* s = std::getenv("HOUDINI_CSI_SYM_START"))
    csi_sym_start_ = (std::string(s) == "auto") ? -1 : std::atoi(s);
  // Per-frame pilot-vs-data timing re-align (Houdini framer jitter). Default on for Houdini.
  csi_timing_fix_ = cfg_->is_houdini();
  if (std::getenv("HOUDINI_CSI_NO_TIMING_FIX")) csi_timing_fix_ = false;
  view_mode_ = true;
  MLPD_INFO("CSI view mode: streaming to %s:%d (%d subcarriers, ~%.0f fps/ant, rx_conj=%d, "
            "sym_start=%s, timing_fix=%d)\n", host.c_str(), port, N, fps, rx_conj_ ? 1 : 0,
            csi_sym_start_ >= 0 ? std::to_string(csi_sym_start_).c_str() : "auto",
            csi_timing_fix_ ? 1 : 0);
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

// Symbol-0 start for a received slot: the fixed csi_sym_start_ (manual, the default)
// when >= 0, else the opt-in energy-edge auto-detector.
int RecorderWorker::symStart(const short* d, int slot) const {
  return csi_sym_start_ >= 0 ? csi_sym_start_ : slotEnergyStart(d, slot);
}

// DC-centered FFT of the fft-size symbol body starting at sample `base` in `d`.
std::vector<std::complex<float>> RecorderWorker::symbolFft(const short* d,
                                                          int base) const {
  const int N = static_cast<int>(cfg_->fft_size());
  const float qs = rx_conj_ ? -1.0f : 1.0f;  // conjugate RX (undo R2C spectral inversion)
  std::vector<std::complex<float>> out(N);
  for (int k = 0; k < N; ++k) {
    const std::complex<float>* row = &dft_[static_cast<size_t>(k) * N];
    std::complex<float> acc(0.0f, 0.0f);
    for (int n = 0; n < N; ++n) {
      const std::complex<float> x(static_cast<float>(d[2 * (base + n)]),
                                  qs * static_cast<float>(d[2 * (base + n) + 1]));
      acc += x * row[n];
    }
    out[k] = acc;
  }
  return out;
}

// Route a received slot: pilot -> CSI (+ cache H); uplink data -> constellation.
void RecorderWorker::streamCsi(Packet* pkt, NodeType node_type) {
  // A dropped RX packet is covered by inserted zeros so the window keeps its true
  // timing. Those zeros are not signal: an FFT over them yields a wrong H, and since
  // H is cached and reused to equalize later data slots, accepting one would smear
  // the view until the next clean pilot. Recording mode can afford to keep the
  // samples because it writes the extents to /Data/Gaps; viewing mode has no such
  // record, so it drops the slot instead of rendering something untrue (AP-10).
  if (pkt->rx_pad > 0) {
    csi_slots_dropped_++;
    const long long now =
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch())
            .count();
    if (now - csi_drop_log_ns_ > 5000000000LL) {  // at most one line per 5 s
      csi_drop_log_ns_ = now;
      MLPD_WARN(
          "CSI view: dropped %zu slot(s) with RX gaps (latest %u padded samples, "
          "ant %u). The display is stale, not wrong; the link is losing packets.\n",
          csi_slots_dropped_, pkt->rx_pad, pkt->ant_id);
    }
    return;
  }
  sendAdc(pkt);  // raw-ADC envelope, before any of the routing below
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

// Any received slot -> raw-ADC min/max envelope, for spotting converter saturation.
//
// The envelope, not a decimated copy of the samples: decimation picks every Mth
// sample, so a slot that clips for a handful of samples can decimate to a trace that
// never touches full scale, which is the exact failure this panel exists to catch.
// Column c carries the min and max of every sample it covers, so one clipped sample
// pins its column to the rail and cannot be missed. The whole slot is covered in
// kAdcCols columns regardless of how long it is.
void RecorderWorker::sendAdc(Packet* pkt) {
  const int slot = static_cast<int>(cfg_->samps_per_slot());
  if (slot <= 0) return;
  const long long now =
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::steady_clock::now().time_since_epoch())
          .count();
  auto it = adc_last_ns_.find(pkt->ant_id);
  if (it != adc_last_ns_.end() &&
      (now - it->second) < static_cast<long long>(csi_throttle_ns_))
    return;
  adc_last_ns_[pkt->ant_id] = now;

  const int cols = std::min(kAdcCols, slot);
  const short* d = pkt->data;
  std::vector<int16_t> env(static_cast<size_t>(4) * cols);
  // Full-scale counts are taken over the WHOLE slot, not the plotted envelope, so the
  // clipped-sample number stays exact however wide the panel is.
  int32_t peak = 0;
  uint32_t clipped = 0;
  for (int c = 0; c < cols; ++c) {
    const int from = static_cast<int>(static_cast<int64_t>(c) * slot / cols);
    const int to = static_cast<int>(static_cast<int64_t>(c + 1) * slot / cols);
    int16_t imin = INT16_MAX, imax = INT16_MIN, qmin = INT16_MAX, qmax = INT16_MIN;
    for (int n = from; n < to; ++n) {
      const short si = d[2 * n], sq = d[2 * n + 1];
      imin = std::min(imin, static_cast<int16_t>(si));
      imax = std::max(imax, static_cast<int16_t>(si));
      qmin = std::min(qmin, static_cast<int16_t>(sq));
      qmax = std::max(qmax, static_cast<int16_t>(sq));
      const int32_t ai = std::abs(static_cast<int32_t>(si));
      const int32_t aq = std::abs(static_cast<int32_t>(sq));
      peak = std::max(peak, std::max(ai, aq));
      if (ai >= kAdcClip || aq >= kAdcClip) ++clipped;
    }
    env[4 * c + 0] = imin;
    env[4 * c + 1] = imax;
    env[4 * c + 2] = qmin;
    env[4 * c + 3] = qmax;
  }

  // [magic 'ADC1'][frame][ant][cols][samps][rate][peak][clipped][Imin,Imax,Qmin,Qmax]*cols
  std::vector<uint8_t> buf(32 + static_cast<size_t>(8) * cols);
  const uint32_t magic = 0x41444331u, fr = pkt->frame_id, an = pkt->ant_id,
                 nc = static_cast<uint32_t>(cols),
                 ns = static_cast<uint32_t>(slot),
                 pk = static_cast<uint32_t>(peak);
  const float rate = static_cast<float>(cfg_->rate());
  std::memcpy(&buf[0], &magic, 4);
  std::memcpy(&buf[4], &fr, 4);
  std::memcpy(&buf[8], &an, 4);
  std::memcpy(&buf[12], &nc, 4);
  std::memcpy(&buf[16], &ns, 4);
  std::memcpy(&buf[20], &rate, 4);
  std::memcpy(&buf[24], &pk, 4);
  std::memcpy(&buf[28], &clipped, 4);
  std::memcpy(&buf[32], env.data(), static_cast<size_t>(8) * cols);
  (void)::send(csi_sock_, buf.data(), buf.size(), 0);
}

// Pilot slot -> channel estimate H[k] (DC-centered), cached per antenna + streamed.
void RecorderWorker::sendCsi(Packet* pkt) {
  const int N = static_cast<int>(cfg_->fft_size());
  const int cp = static_cast<int>(cfg_->cp_size());
  const int nsym = static_cast<int>(cfg_->symbol_per_slot());
  const int slot = static_cast<int>(cfg_->samps_per_slot());
  const short* d = pkt->data;
  const int es = symStart(d, slot);  // symbol-0 start (fixed prefix by default; sym_start knob)
  int s0 = nsym / 8, s1 = nsym - nsym / 8;
  if (s1 <= s0) { s0 = 0; s1 = nsym; }
  std::vector<std::complex<float>> hacc(N, {0.0f, 0.0f});
  std::vector<float> pacc(N, 0.0f);  // incoherent power, for the repeat coherence below
  int used = 0;
  for (int sym = s0; sym < s1; ++sym) {
    const int base = es + sym * (cp + N) + cp;
    if (base + N > slot) break;
    auto F = symbolFft(d, base);
    for (int k = 0; k < N; ++k) {
      const std::complex<float> g = F[k] * std::conj(pilot_ref_[k]);
      hacc[k] += g;
      pacc[k] += std::norm(g);
    }
    ++used;
  }
  // Cache H per antenna (always -- keeps it fresh for equalizing this ant's data).
  auto& H = csi_h_[pkt->ant_id];
  H.assign(N, {0.0f, 0.0f});
  for (int k = 0; k < N; ++k) {
    const float pw = std::norm(pilot_ref_[k]);
    if (pw > 1e-6f && used > 0) H[k] = hacc[k] / (static_cast<float>(used) * pw);
  }
  // Per-subcarrier REPEAT COHERENCE, |mean of the per-symbol estimates|^2 over their
  // mean power. This slot already carries `used` repetitions of the same pilot and we
  // were averaging them and throwing the spread away. 1.0 means every repetition
  // agreed, so H[k] is signal; low means they disagreed, so H[k] is noise wearing a
  // channel's clothes and the magnitude and phase at that subcarrier mean nothing.
  //
  // Cauchy-Schwarz bounds the ratio to [0,1], but the floor for PURE NOISE is 1/used,
  // not 0 (measured: 8 reps -> 0.128, 32 reps -> 0.030). The dashboard draws that
  // floor as a reference line, because 0.2 out of 1.0 sounds poor and at 6 reps is
  // indistinguishable from noise. It is only a measurement at all when used >= 2:
  // a single term makes the ratio exactly 1.0 however bad the noise, which is why
  // `used` goes on the wire instead of being assumed.
  std::vector<float> qual(N, 0.0f);
  for (int k = 0; k < N; ++k)
    if (pacc[k] > 0.0f && used > 0)
      qual[k] = std::min(
          1.0f, std::norm(hacc[k]) / (static_cast<float>(used) * pacc[k]));
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
  // [magic 'CSI2'][frame][ant][num_sc][rate][reps][H re,im]*N[quality]*N
  // 'CSI2' supersedes 'CSI1' (same layout without reps and quality). The dashboard
  // still accepts 'CSI1', so a backend running ahead of an un-rebuilt sounder shows
  // everything but the quality panel rather than showing nothing.
  std::vector<uint8_t> buf(24 + static_cast<size_t>(12) * N);
  const uint32_t magic = 0x43534932u, fr = pkt->frame_id, an = pkt->ant_id,
                 nsc = static_cast<uint32_t>(N),
                 reps = static_cast<uint32_t>(used);
  const float rate = static_cast<float>(cfg_->rate());
  std::memcpy(&buf[0], &magic, 4);
  std::memcpy(&buf[4], &fr, 4);
  std::memcpy(&buf[8], &an, 4);
  std::memcpy(&buf[12], &nsc, 4);
  std::memcpy(&buf[16], &rate, 4);
  std::memcpy(&buf[20], &reps, 4);
  for (int k = 0; k < N; ++k) {
    const float re = H[k].real(), im = H[k].imag();
    std::memcpy(&buf[24 + 8 * k], &re, 4);
    std::memcpy(&buf[28 + 8 * k], &im, 4);
  }
  for (int k = 0; k < N; ++k)
    std::memcpy(&buf[24 + 8 * N + 4 * k], &qual[k], 4);
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
  const int es = symStart(d, slot);  // symbol-0 start (fixed prefix by default; sym_start knob)
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
  int s0 = nsym / 8, s1 = nsym - nsym / 8;
  if (s1 <= s0) { s0 = 0; s1 = nsym; }
  // Pre-FFT the middle data symbols once (reused by the timing search + the constellation).
  std::vector<std::vector<std::complex<float>>> Ys;
  for (int sym = s0; sym < s1; ++sym) {
    const int base = es + sym * (cp + N) + cp;
    if (base + N > slot) break;
    Ys.push_back(symbolFft(d, base));
  }
  // Pilot-vs-data timing re-align (Houdini): an unstable beacon re-lock leaves the pilot
  // ~1 sample off the data, which ramps H and rings the otherwise-fine data. A pilot
  // re-align of r samples == a linear phase ramp exp(j 2pi (k-N/2) r / N) on H. Pick the
  // integer r whose equalized constellation is tightest (blind 4th-power). |H|
  // (the deep-fade gate) is phase-invariant, so hmin is unchanged. The 4th-power is valid
  // for any square QAM (QPSK/16/64-QAM: E[X^4] real -> arg pi), so gate on mod_ord 2/4/6.
  std::vector<std::complex<float>> Hc(H.begin(), H.end());
  if (csi_timing_fix_ && (mod_ord == 2 || mod_ord == 4 || mod_ord == 6) && !Ys.empty()) {
    double best_score = -1.0;
    int best_r = 0;
    for (int r = -2; r <= 2; ++r) {
      std::complex<double> s4(0.0, 0.0);
      double pwr = 0.0;
      for (const auto& Y : Ys)
        for (size_t j = 0; j < data_ind.size(); ++j) {
          const size_t k = data_ind[j];
          if (std::abs(H[k]) < hmin) continue;
          const double ang = 2.0 * M_PI * (static_cast<double>(k) - N / 2.0) * r / N;
          const std::complex<double> hr =
              std::complex<double>(H[k]) *
              std::complex<double>(std::cos(ang), std::sin(ang));
          const std::complex<double> x = std::complex<double>(Y[k]) / hr;
          const std::complex<double> x2 = x * x;
          s4 += x2 * x2;
          pwr += std::norm(x);
        }
      const double score = (pwr > 0.0) ? std::abs(s4) / (pwr * pwr) : -1.0;
      if (score > best_score) { best_score = score; best_r = r; }
    }
    if (best_r != 0)
      for (int k = 0; k < N; ++k) {
        const double ang = 2.0 * M_PI * (static_cast<double>(k) - N / 2.0) * best_r / N;
        Hc[k] *= std::complex<float>(static_cast<float>(std::cos(ang)),
                                     static_cast<float>(std::sin(ang)));
      }
  }
  std::vector<std::complex<float>> pts;
  pts.reserve(kMaxPts);
  for (size_t si = 0; si < Ys.size() && pts.size() < kMaxPts; ++si) {
    const auto& Y = Ys[si];
    for (size_t j = 0; j < data_ind.size() && pts.size() < kMaxPts; ++j) {
      const size_t k = data_ind[j];
      const std::complex<float> h = Hc[k];
      if (std::abs(h) < hmin || std::norm(h) < 1e-9f) continue;  // deep fade
      pts.push_back(Y[k] / h);  // zero-forcing equalizer
    }
  }
  if (pts.empty()) return;
  // Global common-phase de-rotation. A fixed phase offset between the pilot CSI and
  // the data slot leaves the whole constellation rotated off the axes. Estimate it
  // blind over ALL points via the 4th-power (data cancels: ideal X^4 -> angle pi) --
  // robust, unlike a per-symbol estimate (~48 points is too few to lock). Valid for any
  // square QAM (QPSK/16/64-QAM all have E[X^4] real-negative), so gate on mod_ord 2/4/6.
  if (mod_ord == 2 || mod_ord == 4 || mod_ord == 6) {
    std::complex<double> acc4(0.0, 0.0);
    for (const auto& x : pts) {
      const std::complex<double> xd(x.real(), x.imag());
      acc4 += xd * xd * xd * xd;
    }
    if (std::abs(acc4) > 0.0) {
      const double th = (std::arg(acc4) - M_PI) / 4.0;
      const std::complex<float> derot(std::cos(th), -std::sin(th));
      for (auto& x : pts) x *= derot;
    }
  }
  double psum = 0.0;
  for (const auto& x : pts) psum += std::norm(x);
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
    // Emit /Data/Gaps: the UDP sample gaps the RX path detected + zero-padded this
    // capture (Houdini only). The sink is process-wide, so drain it once here, before
    // the file closes. start_time_ns is relative to the RX stream start (0-anchored);
    // the parser tools (gap_forensics.py) key off the gap sizes + spacing, not an
    // absolute wall-clock. Single receiving stream assumed (see rx_gap_sink.h).
    if (this->cfg_->is_houdini()) {
      const std::vector<Sounder::GapExtent> gaps =
          Sounder::RxGapSink::instance().drain();
      if (gaps.empty() == false) {
        const double rate = this->cfg_->rate();
        std::vector<int64_t> table;
        table.reserve(gaps.size() * 4);
        int64_t untrusted = 0;
        for (const auto& g : gaps) {
          table.push_back(g.start_sample);
          table.push_back(g.n_samples);
          table.push_back(
              static_cast<int64_t>(Sounder::sampleToNs(g.start_sample, rate)));
          table.push_back(g.cause);
          if (g.n_samples > 0) untrusted += g.n_samples;
        }
        this->hdf5_->writeTableInt64("Gaps", 4, table);
        this->hdf5_->write_attribute(
            "GAP_COLUMNS",
            std::string("start_sample,n_samples,start_time_ns,"
                        "cause(0=time_jump,1=host_ring,2=write_error,"
                        "3=backward,4=resync)"));
        this->hdf5_->write_attribute("TOTAL_UNTRUSTED_SAMPLES",
                                     static_cast<double>(untrusted));
        MLPD_INFO("Recorder: /Data/Gaps -- %zu gap(s), %lld untrusted samples\n",
                  gaps.size(), static_cast<long long>(untrusted));
      }
    }
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
