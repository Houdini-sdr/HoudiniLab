// Synthetic end-to-end test of the rx-recorder pipeline (no radio):
// RecorderThread -> RxRecorderWorker -> Hdf5Lib, with the ring-slot claim
// protocol exactly as rx_recorder_main.cc uses it. Crosses the
// MAX_FRAME_INC dataset-growth window, simulates a long host-drop run
// (frame_id jump > kDsExtendStep=400, the review finding-2 scenario),
// then re-opens the file and verifies attributes, dims, and payloads.
#include <H5Cpp.h>

#include <atomic>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <vector>

#include "include/macros.h"
#include "include/recorder_thread.h"
#include "include/rx_recorder_config.h"
#include "include/rx_recorder_worker.h"

static constexpr size_t kSamps = 64;    // samples per slot
static constexpr size_t kSlots = 3000;  // crosses MAX_FRAME_INC=2000
static constexpr size_t kRingSlots = 8;
// Simulated host-drop run: frames [kDropStart, kDropEnd) are never recorded.
// 600 > kDsExtendStep(400) and the run crosses the 2000 growth window.
static constexpr size_t kDropStart = 1900;
static constexpr size_t kDropEnd = 2500;
static constexpr long long kHwTimeNs = 1234567890123LL;
static constexpr long long kFirstSampleNs = 987654321987LL;

#define CHECK(cond)                                                        \
  do {                                                                     \
    if (!(cond)) {                                                         \
      std::fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
      return 1;                                                            \
    }                                                                      \
  } while (0)

static short expected_sample(size_t frame, size_t k) {
  return static_cast<short>((frame * 7 + k) & 0x7FFF);
}

static bool frame_dropped(size_t frame) {
  return (frame >= kDropStart) && (frame < kDropEnd);
}

int main() {
  const char* conf_path = "/tmp/rx_test_conf.json";
  const char* out_path = "/tmp/rx_test_capture.h5";
  {
    std::ofstream conf(conf_path);
    conf << "{ \"channels\": [0], \"duration_sec\": 1.0, "
         << "\"samps_per_slot\": " << kSamps << ", "
         << "\"buffer_slots\": " << kRingSlots << ", "
         << "\"direct_rx\": \"require\", "
         << "\"tx_replay\": { \"freq\": 20e6, \"channel\": 1 }, "
         << "\"output_file\": \"" << out_path << "\" }";
  }
  Sounder::RxRecorderConfig cfg(conf_path, "/tmp");
  CHECK(cfg.direct_rx() == "require");
  CHECK(cfg.has_tx_replay() == true);
  CHECK(cfg.tx_replay_freq() == 20e6);
  CHECK(cfg.tx_replay_amp() == 0.25);     // default
  CHECK(cfg.tx_replay_channel() == 1);
  CHECK(cfg.tx_replay_n_addrs() == 4096);  // default = full BRAM depth

  Sounder::RxCaptureMeta meta;
  meta.hardware_key = "TESTHW";
  meta.hardware_info = "driver=synthetic";
  meta.actual_rate = 1e6;
  meta.actual_freq = 123e6;
  meta.actual_gain = 7.5;
  meta.antenna = "RX0";
  meta.total_slots = kSlots;

  const size_t packet_length = sizeof(Packet) + cfg.getPacketDataLength();
  SampleBuffer ring;
  ring.buffer.resize(kRingSlots * packet_length);
  const size_t intsize = sizeof(std::atomic_int);
  const size_t arraysize = (kRingSlots + intsize - 1) / intsize;
  ring.pkt_buf_inuse = new std::atomic_int[arraysize];
  std::fill_n(ring.pkt_buf_inuse, arraysize, 0);

  size_t sent = 0;
  {
    auto worker = std::make_unique<Sounder::RxRecorderWorker>(&cfg, meta);
    Sounder::RxRecorderWorker* worker_raw = worker.get();
    Sounder::RecorderThread recorder(std::move(worker),
                                     cfg.getPacketDataLength(), 0, -1,
                                     kRingSlots * 2, true);
    recorder.Start();

    for (size_t frame = 0; frame < kSlots; frame++) {
      if (frame_dropped(frame)) continue;  // simulated ring-full drop
      const size_t cursor = frame % kRingSlots;
      // Spin until the writer freed this slot (backpressure, no drops here).
      while (sample_buf_try_claim(ring.pkt_buf_inuse, cursor) == false) {
      }
      Packet* pkt = reinterpret_cast<Packet*>(ring.buffer.data() +
                                              cursor * packet_length);
      new (pkt) Packet(frame, 0, 0, 0);
      for (size_t k = 0; k < 2 * kSamps; k++) {
        pkt->data[k] = expected_sample(frame, k);
      }
      Event_data event;
      event.event_type = kTaskRecord;
      event.node_type = kBS;
      event.frame_id = pkt->frame_id;
      event.slot_id = 0;
      event.ant_id = 0;
      event.offset = cursor;
      event.buff_size = kRingSlots;
      event.buffer = &ring;
      CHECK(recorder.DispatchWork(event) == true);
      sent++;
    }
    worker_raw->setStartTimes(kHwTimeNs, kFirstSampleNs, true);
    // The simulated drop run as the capture thread would report it.
    std::vector<Sounder::GapExtent> extents;
    extents.push_back({static_cast<int64_t>(kDropStart * kSamps),
                       static_cast<int64_t>((kDropEnd - kDropStart) * kSamps),
                       Sounder::kGapHostRing});
    extents.push_back({100, 50, Sounder::kGapTimeJump});
    worker_raw->setGapExtents(extents);
    recorder.Stop();
  }  // ~RecorderThread joins, drains the queue, finalizes the HDF5 file
  delete[] ring.pkt_buf_inuse;

  // ---- Verify the file --------------------------------------------------
  H5::H5File file(out_path, H5F_ACC_RDONLY);
  H5::Group group = file.openGroup("/Data");

  double rate = 0;
  group.openAttribute("RATE").read(H5::PredType::NATIVE_DOUBLE, &rate);
  CHECK(rate == 1e6);

  unsigned int slots_recorded = 0;
  group.openAttribute("SLOTS_RECORDED")
      .read(H5::PredType::NATIVE_UINT, &slots_recorded);
  std::printf("SLOTS_RECORDED = %u (sent %zu)\n", slots_recorded, sent);
  CHECK(slots_recorded == sent);

  unsigned int write_errors = 99;
  group.openAttribute("WRITE_ERRORS")
      .read(H5::PredType::NATIVE_UINT, &write_errors);
  CHECK(write_errors == 0);

  H5::StrType strdatatype(H5::PredType::C_S1, H5T_VARIABLE);
  std::string hw_time, first_ns;
  group.openAttribute("START_HW_TIME_NS").read(strdatatype, hw_time);
  group.openAttribute("FIRST_SAMPLE_TIME_NS").read(strdatatype, first_ns);
  std::printf("START_HW_TIME_NS = %s, FIRST_SAMPLE_TIME_NS = %s\n",
              hw_time.c_str(), first_ns.c_str());
  CHECK(hw_time == std::to_string(kHwTimeNs));
  CHECK(first_ns == std::to_string(kFirstSampleNs));

  H5::DataSet ds = file.openDataSet("/Data/Samples");
  H5::DataSpace space = ds.getSpace();
  hsize_t dims[5];
  CHECK(space.getSimpleExtentNdims() == 5);
  space.getSimpleExtentDims(dims);
  std::printf("Samples dims = {%llu, %llu, %llu, %llu, %llu}\n",
              (unsigned long long)dims[0], (unsigned long long)dims[1],
              (unsigned long long)dims[2], (unsigned long long)dims[3],
              (unsigned long long)dims[4]);
  CHECK(dims[0] >= kSlots);      // all recorded rows present
  CHECK(dims[0] <= kSlots + 1);  // capped by max+1, no runaway growth
  CHECK(dims[3] == 1);
  CHECK(dims[4] == 2 * kSamps);

  // Spot-check payloads: growth boundary, both edges of the drop gap
  // (recorded frames must survive; dropped rows must be zeros), ends.
  const size_t check_frames[] = {0,          1,        1899, 1900, 2400,
                                 2499,       2500,     2501, 1999, 2000,
                                 kSlots - 1, kSlots / 2};
  for (size_t frame : check_frames) {
    hsize_t offset[5] = {frame, 0, 0, 0, 0};
    hsize_t count[5] = {1, 1, 1, 1, 2 * kSamps};
    H5::DataSpace filespace(ds.getSpace());
    filespace.selectHyperslab(H5S_SELECT_SET, count, offset);
    H5::DataSpace memspace(5, count);
    std::vector<short> row(2 * kSamps);
    ds.read(row.data(), H5::PredType::NATIVE_INT16, memspace, filespace);
    for (size_t k = 0; k < 2 * kSamps; k++) {
      const short want = frame_dropped(frame) ? 0 : expected_sample(frame, k);
      if (row[k] != want) {
        std::fprintf(stderr, "FAIL: frame %zu sample %zu: got %d want %d\n",
                     frame, k, row[k], want);
        return 1;
      }
    }
  }

  // ---- Gap table ----------------------------------------------------
  H5::DataSet gaps = file.openDataSet("/Data/Gaps");
  hsize_t gdims[2];
  CHECK(gaps.getSpace().getSimpleExtentDims(gdims) == 2);
  std::printf("Gaps dims = {%llu, %llu}\n", (unsigned long long)gdims[0],
              (unsigned long long)gdims[1]);
  CHECK(gdims[0] == 2 && gdims[1] == 4);
  std::vector<int64_t> grows(gdims[0] * gdims[1]);
  gaps.read(grows.data(), H5::PredType::NATIVE_INT64);
  // Sorted by start: the injected time-jump extent first.
  CHECK(grows[0] == 100 && grows[1] == 50);
  CHECK(grows[3] == 0);  // cause kGapTimeJump
  CHECK(grows[4] == (int64_t)(kDropStart * kSamps));
  CHECK(grows[5] == (int64_t)((kDropEnd - kDropStart) * kSamps));
  CHECK(grows[7] == 1);  // cause kGapHostRing
  // start_time_ns column = t0 + start/rate
  const long long want_t = kFirstSampleNs +
      (long long)std::llround(100 * 1e9 / 1e6);
  CHECK(grows[2] == want_t);
  double untrusted = -1;
  group.openAttribute("TOTAL_UNTRUSTED_SAMPLES")
      .read(H5::PredType::NATIVE_DOUBLE, &untrusted);
  std::printf("TOTAL_UNTRUSTED_SAMPLES = %.0f\n", untrusted);
  CHECK(untrusted == 50.0 + (kDropEnd - kDropStart) * kSamps);

  std::printf(
      "PASS: %zu slots recorded, %zu-frame drop gap crossed cleanly, "
      "payloads verified\n",
      sent, kDropEnd - kDropStart);
  return 0;
}
