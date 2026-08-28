/*
 Copyright (c) 2018-2020
 RENEW OPEN SOURCE LICENSE: http://renew-wireless.org/license
 
----------------------------------------------------------------------
 Class to handle writting data to an hdf5 file
---------------------------------------------------------------------
*/
#ifndef SOUNDER_RECORDER_WORKER_H_
#define SOUNDER_RECORDER_WORKER_H_

#include <complex>
#include <cstdint>
#include <unordered_map>
#include <vector>

#include "config.h"
#include "hdf5_lib.h"
#include "receiver.h"

namespace Sounder {
class RecorderWorker {
 public:
  RecorderWorker(Config* in_cfg, size_t antenna_offset, size_t num_antennas);
  ~RecorderWorker();

  void init(void);
  void finalize(void);
  void record(int tid, Packet* pkt, NodeType node_type);

  inline size_t num_antennas(void) { return num_antennas_; }
  inline size_t antenna_offset(void) { return antenna_offset_; }

 private:
  Config* cfg_;
  H5std_string hdf5_name_;
  Hdf5Lib* hdf5_ = nullptr;
  std::vector<std::string> datasets;

  size_t max_frame_number_;

  size_t antenna_offset_;
  size_t num_antennas_;

  // --- Viewing mode (HOUDINI_CSI_UDP=host:port set): compute per-antenna CSI from
  // each received pilot (pilot-agnostic -- uses the config's freq-domain reference,
  // so LTS / Zadoff-Chu / any pilot works) and stream it to the GUI over UDP INSTEAD
  // of writing HDF5. One datagram per (frame, antenna); the GUI scales to whatever
  // antennas appear. ---
  bool view_mode_ = false;
  // Houdini RFSoC only: the matched-NCO R2C RX mixer delivers baseband CONJUGATED
  // (a +f tone returns at -f -- same inversion buildHoudiniBeacon pre-conjugates the
  // TX beacon to cancel). Sync uses raw samples, but CSI/constellation must undo it,
  // else H[k] lands on the mirror subcarrier (N-k) and the constellation scrambles.
  bool rx_conj_ = false;
  int csi_sock_ = -1;
  std::vector<std::complex<float>> pilot_ref_;  // DC-centered freq-domain pilot
  std::vector<std::complex<float>> dft_;        // NxN DC-centered DFT coefficients
  double csi_throttle_ns_ = 0.0;                // per-antenna min send interval
  // OFDM symbol-0 start within a received slot. Default = the nominal prefix (a fixed,
  // manually-tunable offset via HOUDINI_CSI_SYM_START); the energy-edge auto-detector
  // slotEnergyStart() is opt-in only (HOUDINI_CSI_SYM_START=auto) because its 15%
  // threshold can mis-trigger on pre-symbol leakage and mis-align the FFT windows.
  int csi_sym_start_ = -1;
  // Houdini: unstable beacon re-locks leave the pilot slot ~1 sample off the data on
  // ~40% of frames, ramping H and ringing the (otherwise-fine) data. Per constellation
  // frame, pick the integer pilot re-align (a ramp on the cached H) that maximizes the
  // QPSK 4th-power concentration. On by default for is_houdini; HOUDINI_CSI_NO_TIMING_FIX.
  bool csi_timing_fix_ = false;
  std::unordered_map<uint32_t, long long> csi_last_ns_;   // CSI (pilot) send timer
  std::unordered_map<uint32_t, long long> cns_last_ns_;   // constellation send timer
  std::unordered_map<uint32_t, long long> adc_last_ns_;   // raw-ADC envelope send timer
  // Slots refused because the RX path had zero-padded a dropped-packet gap into them
  // (AP-10), plus a throttle so the warning cannot flood a lossy run.
  size_t csi_slots_dropped_ = 0;
  long long csi_drop_log_ns_ = 0;
  // Latest channel estimate H[k] per antenna (DC-centered), cached from the pilot
  // slot and used to equalize that antenna's uplink-data (U) slot.
  std::unordered_map<uint32_t, std::vector<std::complex<float>>> csi_h_;
  void initCsi(void);
  void streamCsi(Packet* pkt, NodeType node_type);   // routes pilot vs uplink data
  int slotEnergyStart(const short* d, int slot) const;   // opt-in energy-edge auto-detect
  int symStart(const short* d, int slot) const;          // fixed csi_sym_start_ (or auto)
  std::vector<std::complex<float>> symbolFft(const short* d, int base) const;
  void sendCsi(Packet* pkt);                          // pilot -> CSI + cache H
  void sendConstellation(Packet* pkt);                // uplink data -> equalize
  void sendAdc(Packet* pkt);                          // any slot -> raw-ADC envelope
};
}; /* End namespace Sounder */

#endif /* SOUNDER_RECORDER_WORKER_H_ */
