/*
 Copyright (c) 2018-2026, Rice University
 RENEW OPEN SOURCE LICENSE: http://renew-wireless.org/license

----------------------------------------------------------------------
 Lean configuration for the rx-recorder tool: one SoapySDR device
 (Houdini SDR by default), a continuous RX stream, a fixed-duration
 capture to an HDF5 file. Parsed from a small JSON file -- see
 files/rx-record.json for the reference example.
---------------------------------------------------------------------
*/
#ifndef SOUNDER_RX_RECORDER_CONFIG_H_
#define SOUNDER_RX_RECORDER_CONFIG_H_

#include <cstddef>
#include <map>
#include <string>
#include <vector>

namespace Sounder {

class RxRecorderConfig {
 public:
  RxRecorderConfig(const std::string& json_file, const std::string& storepath);

  // Device::make() kwargs. "driver" defaults to "houdinisdr" when absent.
  inline const std::map<std::string, std::string>& device_args(void) const {
    return device_args_;
  }
  // setupStream() kwargs, forwarded verbatim (ring_bytes, cpu_affinity, ...).
  inline const std::map<std::string, std::string>& stream_args(void) const {
    return stream_args_;
  }
  inline const std::vector<size_t>& channels(void) const { return channels_; }

  // Requested sample rate in Hz; 0 = keep the device's current rate.
  inline double rate(void) const { return rate_; }
  // RF tune frequency in Hz (fine NCO on Houdini); only applied when set.
  inline bool has_freq(void) const { return has_freq_; }
  inline double freq(void) const { return freq_; }
  inline bool has_gain(void) const { return has_gain_; }
  inline double gain(void) const { return gain_; }
  // Antenna name; empty = leave the device default.
  inline const std::string& antenna(void) const { return antenna_; }

  inline double duration_sec(void) const { return duration_sec_; }
  // Samples (CS16 elements) per recorded slot == HDF5 row.
  inline size_t samps_per_slot(void) const { return samps_per_slot_; }
  // RX ring: number of slot-sized packets buffered between the RX loop
  // and the HDF5 writer thread.
  inline size_t buffer_slots(void) const { return buffer_slots_; }
  inline long rx_timeout_us(void) const { return rx_timeout_us_; }
  inline const std::string& output_file(void) const { return output_file_; }

  inline size_t getPacketDataLength(void) const {
    return (2 * samps_per_slot_ * sizeof(short));
  }

 private:
  std::map<std::string, std::string> device_args_;
  std::map<std::string, std::string> stream_args_;
  std::vector<size_t> channels_;
  double rate_;
  bool has_freq_;
  double freq_;
  bool has_gain_;
  double gain_;
  std::string antenna_;
  double duration_sec_;
  size_t samps_per_slot_;
  size_t buffer_slots_;
  long rx_timeout_us_;
  std::string output_file_;
};

}; /* End namespace Sounder */

#endif /* SOUNDER_RX_RECORDER_CONFIG_H_ */
