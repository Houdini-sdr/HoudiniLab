/*
 Copyright (c) 2018-2026, Rice University
 RENEW OPEN SOURCE LICENSE: http://renew-wireless.org/license

----------------------------------------------------------------------
 HDF5 sink for the rx-recorder tool: one extendable "Samples" dataset
 of raw CS16 slots plus capture metadata attributes.
---------------------------------------------------------------------
*/
#ifndef SOUNDER_RX_RECORDER_WORKER_H_
#define SOUNDER_RX_RECORDER_WORKER_H_

#include <memory>
#include <string>
#include <vector>

#include "hdf5_lib.h"
#include "recorder_worker_interface.h"
#include "rx_recorder_config.h"
#include "rx_recorder_grid.h"

namespace Sounder {

// Live-device facts gathered by the app after configuring the radio, so
// the file records what the hardware actually did, not what was asked.
struct RxCaptureMeta {
  std::string hardware_key;
  std::string hardware_info;
  double actual_rate = 0.0;
  double actual_freq = 0.0;
  double actual_gain = 0.0;
  std::string antenna;
  size_t total_slots = 0;
};

class RxRecorderWorker : public RecorderWorkerInterface {
 public:
  RxRecorderWorker(const RxRecorderConfig* cfg, const RxCaptureMeta& meta);
  ~RxRecorderWorker() override;

  void init(void) override;
  void finalize(void) override;
  void record(int tid, Packet* pkt, NodeType node_type) override;

  // Capture-start times, written as file attributes at finalize.
  // Call from the capture thread BEFORE dispatching the recorder Stop
  // event — the queue's release/acquire ordering makes the writes visible
  // to the writer thread's finalize.
  //   hw_time_ns:      getHardwareTime() right after activateStream
  //                    (approximate: excludes in-flight latency).
  //   first_sample_ns: timeNs of the first stamped readStream — the exact
  //                    hardware time of file sample 0.
  void setStartTimes(long long hw_time_ns, long long first_sample_ns,
                     bool has_first_sample);

  // Untrusted-sample extents observed by the capture thread (stream gaps,
  // host-ring drops). Same ordering contract as setStartTimes: call before
  // the recorder Stop event. Merged with this worker's own write-error
  // extents into the /Data/Gaps table at finalize.
  void setGapExtents(const std::vector<GapExtent>& extents);

  inline size_t num_antennas(void) const override {
    return cfg_->channels().size();
  }
  inline size_t antenna_offset(void) const override { return 0; }

 private:
  void writeGapTable(void);

  const RxRecorderConfig* cfg_;
  RxCaptureMeta meta_;
  std::unique_ptr<Hdf5Lib> hdf5_;
  long long start_hw_time_ns_ = 0;
  long long first_sample_time_ns_ = 0;
  bool has_first_sample_time_ = false;
  // Two owners, merged at finalize: capture_extents_ is written once by the
  // capture thread (setGapExtents, before the Stop event); write_error_extents_
  // only by the writer thread in record(). Never the same vector -- record()
  // can still be draining queued slots when setGapExtents runs.
  std::vector<GapExtent> capture_extents_;
  std::vector<GapExtent> write_error_extents_;
  size_t slots_written_ = 0;
  size_t write_errors_ = 0;
  bool finalized_ = false;
};

}; /* End namespace Sounder */

#endif /* SOUNDER_RX_RECORDER_WORKER_H_ */
