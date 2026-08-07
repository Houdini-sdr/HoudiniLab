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
  int csi_sock_ = -1;
  std::vector<std::complex<float>> pilot_ref_;  // DC-centered freq-domain pilot
  std::vector<std::complex<float>> dft_;        // NxN DC-centered DFT coefficients
  double csi_throttle_ns_ = 0.0;                // per-antenna min send interval
  std::unordered_map<uint32_t, long long> csi_last_ns_;   // CSI (pilot) send timer
  std::unordered_map<uint32_t, long long> cns_last_ns_;   // constellation send timer
  // Latest channel estimate H[k] per antenna (DC-centered), cached from the pilot
  // slot and used to equalize that antenna's uplink-data (U) slot.
  std::unordered_map<uint32_t, std::vector<std::complex<float>>> csi_h_;
  void initCsi(void);
  void streamCsi(Packet* pkt, NodeType node_type);   // routes pilot vs uplink data
  std::vector<std::complex<float>> symbolFft(const short* d, int base) const;
  void sendCsi(Packet* pkt);                          // pilot -> CSI + cache H
  void sendConstellation(Packet* pkt);                // uplink data -> equalize
};
}; /* End namespace Sounder */

#endif /* SOUNDER_RECORDER_WORKER_H_ */
