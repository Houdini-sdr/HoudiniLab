/*
 Copyright (c) 2018-2026, Rice University
 RENEW OPEN SOURCE LICENSE: http://renew-wireless.org/license

----------------------------------------------------------------------
 Implementation of RxRecorderWorker (rx-recorder HDF5 sink)
---------------------------------------------------------------------
*/
#include "include/rx_recorder_worker.h"

#include <algorithm>

#include "include/logger.h"

namespace Sounder {

static const std::string kSamplesDataset = "Samples";

RxRecorderWorker::RxRecorderWorker(const RxRecorderConfig* cfg,
                                   const RxCaptureMeta& meta)
    : cfg_(cfg), meta_(meta) {}

RxRecorderWorker::~RxRecorderWorker() { this->finalize(); }

void RxRecorderWorker::init(void) {
  // H5 exceptions here propagate: init runs on the capture thread (inside
  // the RecorderThread constructor) where main()'s handlers report them.
  hdf5_ = std::make_unique<Hdf5Lib>(cfg_->output_file(), "Data");

  hdf5_->write_attribute("FREQ", meta_.actual_freq);
  hdf5_->write_attribute("RATE", meta_.actual_rate);
  hdf5_->write_attribute("GAIN", meta_.actual_gain);
  hdf5_->write_attribute("ANTENNA", meta_.antenna);
  hdf5_->write_attribute("CHANNELS", cfg_->channels());
  hdf5_->write_attribute("SLOT_SAMP_LEN", cfg_->samps_per_slot());
  hdf5_->write_attribute("DURATION_SEC_REQUESTED", cfg_->duration_sec());
  hdf5_->write_attribute("FORMAT", std::string("CS16"));
  hdf5_->write_attribute("HW_KEY", meta_.hardware_key);
  hdf5_->write_attribute("HW_INFO", meta_.hardware_info);
  hdf5_->write_attribute("READ_PATH", meta_.read_path);
  hdf5_->write_attribute("GAPS_EXACT", meta_.gaps_exact ? 1 : 0);
  hdf5_->write_attribute("STREAM_ARGS", meta_.stream_args);
  if (meta_.tx_replay_enabled == true) {
    hdf5_->write_attribute("TX_REPLAY_FREQ_ACTUAL", meta_.tx_replay_freq_actual);
    hdf5_->write_attribute("TX_REPLAY_AMP", meta_.tx_replay_amp);
    hdf5_->write_attribute("TX_REPLAY_CHANNEL", meta_.tx_replay_channel);
  }

  const hsize_t IQ = 2 * cfg_->samps_per_slot();
  const std::array<hsize_t, kDsDimsNum> total_dims = {MAX_FRAME_INC, 1, 1,
                                                      this->num_antennas(), IQ};
  const std::array<hsize_t, kDsDimsNum> chunk_dims = {1, 1, 1, 1, IQ};
  hdf5_->createDataset(kSamplesDataset, total_dims, chunk_dims);

  hdf5_->setTargetPrimaryDimSize(MAX_FRAME_INC);
  hdf5_->setMaxPrimaryDimSize(meta_.total_slots);
  hdf5_->openDataset();
}

void RxRecorderWorker::setStartTimes(long long hw_time_ns,
                                     long long first_sample_ns,
                                     bool has_first_sample) {
  start_hw_time_ns_ = hw_time_ns;
  first_sample_time_ns_ = first_sample_ns;
  has_first_sample_time_ = has_first_sample;
}

void RxRecorderWorker::setGapExtents(const std::vector<GapExtent>& extents) {
  capture_extents_ = extents;
}

void RxRecorderWorker::writeGapTable(void) {
  std::vector<GapExtent> gap_extents = capture_extents_;
  gap_extents.insert(gap_extents.end(), write_error_extents_.begin(),
                     write_error_extents_.end());
  std::sort(gap_extents.begin(), gap_extents.end(),
            [](const GapExtent& a, const GapExtent& b) {
              return a.start_sample < b.start_sample;
            });

  // Columns: start_sample, n_samples, start_time_ns, cause. Time is the
  // grid time of the extent's first sample; 0 when no time anchor exists.
  const double rate = meta_.actual_rate;
  std::vector<int64_t> rows;
  rows.reserve(gap_extents.size() * 4);
  for (const GapExtent& g : gap_extents) {
    const int64_t start_time_ns =
        has_first_sample_time_
            ? first_sample_time_ns_ + Sounder::sampleToNs(g.start_sample, rate)
            : 0;
    rows.push_back(g.start_sample);
    rows.push_back(g.n_samples);
    rows.push_back(start_time_ns);
    rows.push_back(g.cause);
  }
  hdf5_->writeTableInt64("Gaps", 4, rows);
  hdf5_->write_attribute(
      "GAP_COLUMNS",
      std::string(
          "start_sample,n_samples,start_time_ns,"
          "cause(0=time_jump,1=host_ring,2=write_error,3=backward,4=resync)"));

  // Union length: extents can overlap (a stream gap detected while a
  // host-ring drop was already discarding the same slot).
  int64_t untrusted = 0;
  int64_t cover_end = -1;
  for (const GapExtent& g : gap_extents) {
    if (g.n_samples <= 0) {
      continue;  // anomaly markers
    }
    const int64_t s = std::max(g.start_sample, cover_end);
    const int64_t e = g.start_sample + g.n_samples;
    if (e > s) {
      untrusted += e - s;
      cover_end = e;
    }
  }
  // double (not the size_t overload, which truncates to uint32): exact to
  // 2^53 samples, far past any capture, and convenient for readers.
  hdf5_->write_attribute("TOTAL_UNTRUSTED_SAMPLES",
                         static_cast<double>(untrusted));
}

void RxRecorderWorker::finalize(void) {
  if ((hdf5_ == nullptr) || (finalized_ == true)) {
    return;
  }
  finalized_ = true;
  try {
    // Dataset rows grow in MAX_FRAME_INC windows, so the shape alone
    // overstates the capture; this attribute is the exact written count.
    hdf5_->write_attribute("SLOTS_RECORDED", slots_written_);
    hdf5_->write_attribute("WRITE_ERRORS", write_errors_);
    // int64 ns does not fit the numeric attribute overloads (the size_t
    // one truncates to uint32); strings keep full precision. Written
    // BEFORE the gap table: the timing anchor must not be lost to a
    // gap-table write failure.
    hdf5_->write_attribute("START_HW_TIME_NS",
                           std::to_string(start_hw_time_ns_));
    if (has_first_sample_time_ == true) {
      hdf5_->write_attribute("FIRST_SAMPLE_TIME_NS",
                             std::to_string(first_sample_time_ns_));
    }
    // Isolated: a Gaps failure must still reach the managed close below.
    try {
      this->writeGapTable();
    } catch (H5::Exception& e) {
      MLPD_ERROR("HDF5 gap table write failed: %s\n", e.getCDetailMsg());
    }
    hdf5_->closeDataset();
    hdf5_->closeFile();
  } catch (H5::Exception& e) {
    MLPD_ERROR("HDF5 finalize failed: %s\n", e.getCDetailMsg());
  }
  if (write_errors_ > 0) {
    MLPD_ERROR("HDF5 slot write errors during capture: %zu\n", write_errors_);
  }
}

void RxRecorderWorker::record(int tid, Packet* pkt, NodeType node_type) {
  (void)tid;
  (void)node_type;

  // The capture loop bounds frame_id < total_slots; a packet beyond that
  // would wedge the extend loop below (extent is capped at total_slots+1).
  if (pkt->frame_id >= meta_.total_slots) {
    MLPD_ERROR("record: frame id %u out of range (%zu), dropping\n",
               pkt->frame_id, meta_.total_slots);
    write_errors_++;
    return;
  }

  try {
    // Grow the dataset in MAX_FRAME_INC windows, mirroring RecorderWorker —
    // but jump-aware: a long host-drop run can advance frame_id by more
    // than one window, and extendDataset() grows at most kDsExtendStep
    // rows per call, so both are looped until they cover frame_id.
    const size_t target_frames = hdf5_->getTargetPrimaryDimSize();
    if (pkt->frame_id >= target_frames) {
      hdf5_->closeDataset();
      hdf5_->openDataset();
      size_t new_target = target_frames;
      while (pkt->frame_id >= new_target) {
        new_target += MAX_FRAME_INC;
      }
      hdf5_->setTargetPrimaryDimSize(new_target);
    }
    while (hdf5_->extendDataset(kSamplesDataset, pkt->frame_id) == true) {
    }

    const hsize_t IQ = 2 * cfg_->samps_per_slot();
    const std::array<hsize_t, kDsDimsNum> offset = {pkt->frame_id, 0, 0,
                                                    pkt->ant_id, 0};
    const std::array<hsize_t, kDsDimsNum> count = {1, 1, 1, 1, IQ};
    hdf5_->writeDataset(kSamplesDataset, offset, count, pkt->data);
    slots_written_++;
  } catch (H5::Exception& e) {
    // Contain HDF5 failures to the slot: an escape from the writer thread
    // would std::terminate the whole capture. Reported via WRITE_ERRORS
    // and as a /Data/Gaps extent (the row's content is not trustworthy).
    write_errors_++;
    const int64_t n = static_cast<int64_t>(cfg_->samps_per_slot());
    write_error_extents_.push_back(
        {static_cast<int64_t>(pkt->frame_id) * n, n, kGapWriteError});
    MLPD_ERROR("HDF5 write failed for slot %u: %s\n", pkt->frame_id,
               e.getCDetailMsg());
  }
}

}; /* End namespace Sounder */
