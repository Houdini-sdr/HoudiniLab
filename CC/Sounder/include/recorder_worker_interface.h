/*
 Copyright (c) 2018-2026, Rice University
 RENEW OPEN SOURCE LICENSE: http://renew-wireless.org/license

----------------------------------------------------------------------
 Interface implemented by recorder workers so RecorderThread can drive
 any packet sink (sounder HDF5 worker, rx-recorder worker, ...)
---------------------------------------------------------------------
*/
#ifndef SOUNDER_RECORDER_WORKER_INTERFACE_H_
#define SOUNDER_RECORDER_WORKER_INTERFACE_H_

#include <cstddef>

#include "macros.h"

namespace Sounder {

class RecorderWorkerInterface {
 public:
  virtual ~RecorderWorkerInterface() = default;

  virtual void init(void) = 0;
  virtual void finalize(void) = 0;
  virtual void record(int tid, Packet* pkt, NodeType node_type) = 0;

  virtual size_t num_antennas(void) const = 0;
  virtual size_t antenna_offset(void) const = 0;
};

}; /* End namespace Sounder */

#endif /* SOUNDER_RECORDER_WORKER_INTERFACE_H_ */
