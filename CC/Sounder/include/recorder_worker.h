/*
 Copyright (c) 2018-2026
 RENEW OPEN SOURCE LICENSE: http://renew-wireless.org/license
 
----------------------------------------------------------------------
 Class to handle writting data to an hdf5 file
---------------------------------------------------------------------
*/
#ifndef SOUNDER_RECORDER_WORKER_H_
#define SOUNDER_RECORDER_WORKER_H_

#include "config.h"
#include "hdf5_lib.h"
#include "recorder_worker_interface.h"

namespace Sounder {
class RecorderWorker : public RecorderWorkerInterface {
 public:
  RecorderWorker(Config* in_cfg, size_t antenna_offset, size_t num_antennas);
  ~RecorderWorker() override;

  void init(void) override;
  void finalize(void) override;
  void record(int tid, Packet* pkt, NodeType node_type) override;

  inline size_t num_antennas(void) const override { return num_antennas_; }
  inline size_t antenna_offset(void) const override { return antenna_offset_; }

 private:
  Config* cfg_;
  H5std_string hdf5_name_;
  Hdf5Lib* hdf5_;
  std::vector<std::string> datasets;

  size_t max_frame_number_;

  size_t antenna_offset_;
  size_t num_antennas_;
};
}; /* End namespace Sounder */

#endif /* SOUNDER_RECORDER_WORKER_H_ */
