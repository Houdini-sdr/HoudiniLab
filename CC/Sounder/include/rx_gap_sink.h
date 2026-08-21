/*
 Copyright (c) 2018-2026, Rice University
 RENEW OPEN SOURCE LICENSE: http://renew-wireless.org/license

----------------------------------------------------------------------
 Process-wide sink that bridges RX-side sample-gap detection to the
 recorder that writes the HDF5. Radio::recvHoudini() (RX thread) detects
 a dropped-UDP-packet gap spliced into a receive window and pushes the
 extent here; RecorderWorker::finalize() (recorder thread) drains it and
 writes the file's /Data/Gaps table. The two subsystems are otherwise
 disconnected -- the scheduler owns the radios, the recorder owns the
 file -- so a small shared sink is the least-invasive bridge.

 SINGLE-STREAM assumption: start_sample is in one radio's cumulative-RX-
 sample coordinate. With more than one receiving radio the extents would
 need an antenna column (the rx-recorder tool this is ported from is
 likewise single-stream).
---------------------------------------------------------------------
*/
#ifndef SOUNDER_RX_GAP_SINK_H_
#define SOUNDER_RX_GAP_SINK_H_

#include <algorithm>
#include <mutex>
#include <vector>

#include "rx_recorder_grid.h"  // GapExtent

namespace Sounder {

class RxGapSink {
 public:
  static RxGapSink& instance(void) {
    static RxGapSink sink;
    return sink;
  }

  void push(const GapExtent& g) {
    std::lock_guard<std::mutex> lock(mtx_);
    // Bound memory: view mode detects+pushes gaps but never drains (no HDF5), so cap
    // the backlog. Gaps are rare (dropped UDP packets), so this never trips in practice.
    if (gaps_.size() < kMaxGaps) gaps_.push_back(g);
  }

  // Move out the accumulated gaps (sorted by start_sample) and clear the sink.
  std::vector<GapExtent> drain(void) {
    std::lock_guard<std::mutex> lock(mtx_);
    std::vector<GapExtent> out;
    out.swap(gaps_);
    std::sort(out.begin(), out.end(),
              [](const GapExtent& a, const GapExtent& b) {
                return a.start_sample < b.start_sample;
              });
    return out;
  }

 private:
  RxGapSink(void) = default;
  static constexpr size_t kMaxGaps = 1u << 20;  // ~24 MB worst case
  std::mutex mtx_;
  std::vector<GapExtent> gaps_;
};

}  // namespace Sounder

#endif  // SOUNDER_RX_GAP_SINK_H_
