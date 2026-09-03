/*
 Copyright (c) 2018-2022, Rice University
 RENEW OPEN SOURCE LICENSE: http://renew-wireless.org/license
 
----------------------------------------------------------
 Handles received samples from massive-mimo base station 
----------------------------------------------------------
*/

#include "include/receiver.h"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <climits>
#include <limits>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <cstdlib>
#include <random>

#include "SoapySDR/Time.hpp"
#include "include/comms-lib.h"
#include "include/logger.h"
#include "include/macros.h"
#include "include/node_version.h"
#include "sync/grid_tracker.h"
#include "sync/resync_policy.h"
#include "sync/sync_geometry.h"
#include "include/utils.h"

//Default to detect the beacon on first channel
static constexpr size_t kSyncDetectChannel = 0;
static constexpr float kBeaconDetectWindowScaler = 2.33f;
// Beacon core geometry, mirrored from Config::genBeacon (config.cc): 15 reps of
// STS(16) then 2 reps of gold(128). The estimator (sync/cfo_estimator.h) correlates BOTH structures
// and guards on the total at runtime.
// The legacy beacon's layout, kept ONLY as the reference the sync-geometry
// constants are written against. The CFO estimator no longer uses them: it
// reads the configured shape's geometry from Config, because `beacon_type`
// now selects between four beacons (include/sync/beacon_shapes.h).
// The beacon-CFO log line is throttled by sync.cfo.log_every (1 in N); the
// panel gets every sample regardless. 1 makes it dense, for a calibration run.

// Where the beacon END sits relative to the slot-0 start, per the
// TRANSMITTED layout -- the constant the UE subtracts from sync_index to
// derive its slot grid. Houdini: the strobe burst is stamped at
// window_open + 384 ticks (BaseRadioSet kTddGridTicks) with NO prefix, so
// the core ends at slot_start + 384 + beacon_size. Iris: [prefix][beacon]
// at the slot head. The old code used the Iris constant on Houdini too,
// baking a 256-sample model error into the UE grid that tx_advance then had
// to absorb (DEMO_VERIFICATION.md 4.28/4.29). The residual after this fix is
// pure pipeline/path latency (~1 us class, measured ~122 samples on-board),
// which is exactly what tx_advance / ue_tx_advance_ticks calibrate.
// (The 384-tick strobe offset is houdini::sync::kHoudiniStrobeOffsetTicks.)

// The detector's threshold form and pick rule are configuration now
// (sync.detector.threshold / sync.detector.pick, with HOUDINI_BEACON_THRESH and
// HOUDINI_BEACON_PICK as logged overrides while allow_env_overrides holds) and
// are resolved ONCE into sync_detector_ in the constructor. The reasoning that
// used to live here -- why first-crossing false-locks on a strong link, why the
// power-ratio form is a different test at every level, why a single-copy
// replica forces the coherence form -- is in sync/detector.h,
// CommsLib::BeaconPick / BeaconThresh and DEMO_VERIFICATION 8.138-8.154.
// The expected beacon end in a slot-aligned window is
// config_->shape().expectedEndOffset(): the strobe offset + core on Houdini,
// core + prefix on Iris/UHD. One definition, in sync/beacon_shape.h.

// The in-window SNR confirm is sync::SnrWindowGuard (sync/confirm.h), built
// once in the constructor with the configured floor and a guard that covers the
// first-path back window (8.151). Reached through sync_guard_.

// Every tunable of the sync path is a sync.* knob (sync/sync_config.h): JSON
// first, environment as a logged override while allow_env_overrides holds.

pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t cond = PTHREAD_COND_INITIALIZER;

Receiver::Receiver(
    Config* config, moodycamel::ConcurrentQueue<Event_data>* in_queue,
    std::vector<moodycamel::ConcurrentQueue<Event_data>*> tx_queue,
    std::vector<moodycamel::ProducerToken*> tx_ptoks,
    std::vector<moodycamel::ConcurrentQueue<Event_data>*> cl_tx_queue,
    std::vector<moodycamel::ProducerToken*> cl_tx_ptoks)
    : config_(config),
      message_queue_(in_queue),
      tx_queue_(tx_queue),
      tx_ptoks_(tx_ptoks),
      cl_tx_queue_(cl_tx_queue),
      cl_tx_ptoks_(cl_tx_ptoks) {
  /* initialize random seed: */
  srand(time(NULL));

  MLPD_TRACE("Receiver Construction - CL present: %d, BS Present: %d\n",
             config_->client_present(), config_->bs_present());
  try {
    if (config_->client_present()) client_radio_set_ = makeClientRadioSet(config_);
    if (config_->bs_present()) base_radio_set_ = makeBaseRadioSet(config_, false);
  } catch (std::exception& e) {
    throw ReceiverException(e.what());
  }

  // The sync library objects (docs/SYNC_LIBRARY_ARCHITECTURE.md, phase P1).
  // Built ONCE from the configured beacon shape and the validated sync block,
  // so a run cannot mix two detector populations and the log can say exactly
  // which rule produced its numbers.
  {
    const auto& sc = config_->sync();
    const auto& shape = config_->shape();
    sync_detector_ = std::make_unique<houdini::sync::Detector>(shape, sc.detector);
    // THE GUARD COVERS THE FIRST-PATH BACK WINDOW (8.151): one rule, from the
    // detector's resolved window (SnrWindowGuard::guardFor).
    sync_guard_ = std::make_unique<houdini::sync::SnrWindowGuard>(
        shape.coreLen(), sc.confirm.snr_floor_db,
        houdini::sync::SnrWindowGuard::guardFor(sync_detector_->firstPathWindow()));
    const houdini::sync::FieldGeometry& g = shape.geometry();
    // The matched-NCO R2C RX mixer delivers baseband CONJUGATED on Houdini, so
    // the estimator undoes the sign there (AP-30 verified the sign and scale).
    cfo_estimator_ = std::make_unique<houdini::sync::RepetitionPhaseEstimator>(
        g, sc.cfo.window_margin, config_->is_houdini());
    if (!g.usable()) {
      MLPD_WARN(
          "beacon CFO: beacon '%s' has no usable repeated field (core %d, fine "
          "%d x %d at %d) -- the beacon CFO reading will be NaN\n",
          config_->beacon_type().c_str(), g.core_len, g.fine_reps, g.fine_len,
          g.fine_off);
    }
  }

  MLPD_TRACE("Receiver Construction -- number radios %zu\n",
             config_->num_bs_sdrs_all());

  if (((this->base_radio_set_ != nullptr) &&
       (this->base_radio_set_->getRadioNotFound())) ||
      ((this->client_radio_set_ != nullptr) &&
       (this->client_radio_set_->getRadioNotFound()))) {
    if (this->base_radio_set_ != nullptr) {
      MLPD_WARN("Invalid Base Radio Setup: %d\n",
                this->base_radio_set_ == nullptr);
      this->base_radio_set_->radioStop();
      this->base_radio_set_.reset();
    }
    if (this->client_radio_set_ != nullptr) {
      MLPD_WARN("Invalid Client Radio Setup: %d\n",
                this->client_radio_set_ == nullptr);
      this->client_radio_set_->radioStop();
      this->client_radio_set_.reset();
    }
    throw ReceiverException("Invalid Radio Setup");
  }

  // Both radio sets are up: compare the stack every participating node is
  // running. A run split across mismatched gateware / device firmware / host
  // builds can fail as an RF or timing symptom, which is expensive to chase, so
  // print the versions once and WARN on any disagreement before we start.
  Sounder::NodeVersions::instance().checkAndWarn();

  this->initBuffers();
  MLPD_TRACE("Construction complete\n");
}

Receiver::~Receiver() {
  MLPD_TRACE("Radio Set cleanup, Base: %d, Client: %d\n",
             this->base_radio_set_ == nullptr,
             this->client_radio_set_ == nullptr);
  if (this->base_radio_set_ != nullptr) {
    this->base_radio_set_->radioStop();
    this->base_radio_set_.reset();
  }
  if (this->client_radio_set_ != nullptr) {
    this->client_radio_set_->radioStop();
    this->client_radio_set_.reset();
  }

  for (auto memory : zeros_) {
    free(memory);
  }
  zeros_.clear();
}

void Receiver::initBuffers() {
  size_t frameTimeLen = config_->samps_per_frame();
  txFrameDelta_ = config_->getTxFrameDelta();
  txTimeDelta_ = txFrameDelta_ * frameTimeLen;

  zeros_.resize(2);
  pilotbuffA_.resize(2);
  pilotbuffB_.resize(2);
  for (auto& memory : zeros_) {
    memory = calloc(config_->samps_per_slot(), sizeof(int16_t) * 2);
    if (memory == NULL) {
      throw std::runtime_error("Error allocating memory");
    }
  }
  pilotbuffA_.at(0) = config_->pilot_ci16().data();
  if (config_->cl_sdr_ch() == 2) {
    pilotbuffA_.at(1) = zeros_.at(0);
    pilotbuffB_.at(1) = config_->pilot_ci16().data();
    pilotbuffB_.at(0) = zeros_.at(1);
  }
  // Viewing-mode UE uplink-data slot buffer (ch A). Transmitted continuously in
  // the U slot alongside the pilot so the BS can equalize it and show the
  // constellation. Empty (0-length data()) when the config has no data slot.
  ue_databuffA_.resize(2);
  ue_databuffA_.at(0) = config_->ue_data_ci16().data();
  ue_databuffA_.at(1) = zeros_.at(0);
}

std::vector<pthread_t> Receiver::startClientThreads(SampleBuffer* rx_buffer,
                                                    SampleBuffer* tx_buffer,
                                                    unsigned in_core_id) {
  cl_tx_buffer_ = tx_buffer;
  std::vector<pthread_t> client_threads;
  if (config_->client_present() == true) {
    client_threads.resize(config_->num_cl_sdrs());
    for (unsigned int i = 0; i < config_->num_cl_sdrs(); i++) {
      pthread_t cl_thread_;
      // record the thread id
      ReceiverContext* context = new ReceiverContext;
      context->ptr = this;
      context->core_id = in_core_id;
      context->tid = i;
      context->buffer = rx_buffer;
      // start socket thread
      if (pthread_create(&cl_thread_, NULL, Receiver::clientTxRx_launch,
                         context) != 0) {
        MLPD_ERROR(
            "Socket client thread create failed in start client "
            "threads");
        throw std::runtime_error(
            "Socket client thread create failed "
            "in start client threads");
      }
      client_threads[i] = cl_thread_;
    }
  }
  return client_threads;
}

std::vector<pthread_t> Receiver::startRecvThreads(SampleBuffer* rx_buffer,
                                                  size_t n_rx_threads,
                                                  SampleBuffer* tx_buffer,
                                                  unsigned in_core_id) {
  assert(rx_buffer[0].buffer.size() != 0);
  thread_num_ = n_rx_threads;
  bs_tx_buffer_ = tx_buffer;
  std::vector<pthread_t> created_threads;
  created_threads.resize(this->thread_num_);
  for (size_t i = 0; i < this->thread_num_; i++) {
    // record the thread id
    ReceiverContext* context = new ReceiverContext;
    context->ptr = this;
    context->core_id = in_core_id;
    context->tid = i;
    context->buffer = rx_buffer;
    // start socket thread
    if (pthread_create(&created_threads.at(i), NULL, Receiver::loopRecv_launch,
                       context) != 0) {
      MLPD_ERROR("Socket recv thread create failed");
      throw std::runtime_error("Socket recv thread create failed");
    }
  }
  sleep(1);
  pthread_cond_broadcast(&cond);
  go();
  return created_threads;
}

void Receiver::completeRecvThreads(const std::vector<pthread_t>& recv_thread) {
  for (std::vector<pthread_t>::const_iterator it = recv_thread.begin();
       it != recv_thread.end(); ++it) {
    pthread_join(*it, NULL);
  }
}

void Receiver::go() {
  if (this->base_radio_set_ != NULL) {
    this->base_radio_set_->radioStart();  // hardware trigger
  }
}

void Receiver::baseTxBeacon(int radio_id, int cell, int frame_id,
                            long long base_time) {
  // prepare BS beacon in host buffer
  std::vector<void*> beaconbuff(2);
  if (config_->beam_sweep() == true) {
    size_t beacon_frame_slot = frame_id % config_->num_bs_antennas_all();
    for (size_t ch = 0; ch < config_->bs_sdr_ch(); ++ch) {
      size_t cell_radio_id = radio_id + config_->n_bs_sdrs_agg().at(cell);
      size_t cell_ant_id = cell_radio_id * config_->bs_sdr_ch();
      int hdmd = CommsLib::hadamard2(beacon_frame_slot, cell_ant_id);
      beaconbuff.at(ch) = hdmd == -1 ? config_->neg_beacon_ci16().data()
                                     : config_->beacon_ci16().data();
    }
  } else {
    if (config_->beacon_radio() == (size_t)radio_id) {
      size_t bcn_ch = config_->beacon_channel();
      beaconbuff.at(bcn_ch) = config_->beacon_ci16().data();
      if (config_->bs_sdr_ch() > 1) beaconbuff.at(1 - bcn_ch) = zeros_.at(0);
    } else {  // set both channels to zeros
      for (size_t ch = 0; ch < config_->bs_sdr_ch(); ++ch)
        beaconbuff.at(ch) = zeros_.at(ch);
    }
  }
  int r_tx;
  r_tx = this->base_radio_set_->radioTx(
      radio_id, cell, beaconbuff.data(), kStreamEndBurst,
      base_time);  // assume beacon is first slot

  if (r_tx != (int)config_->samps_per_slot())
    std::cerr << "BAD Transmit(" << r_tx << "/" << config_->samps_per_slot()
              << ") at Time " << base_time << ", frame count " << frame_id
              << std::endl;
}

int Receiver::baseTxData(int radio_id, int cell, int frame_id,
                         long long base_time) {
  int num_samps = config_->samps_per_slot();
  size_t packetLength = sizeof(Packet) + config_->getPacketDataLength();
  size_t tx_buffer_size = bs_tx_buffer_[radio_id].buffer.size() / packetLength;
  int flagsTxData;
  std::vector<void*> dl_txbuff(2);
  Event_data event;
  if (tx_queue_.at(radio_id)->try_dequeue_from_producer(*tx_ptoks_.at(radio_id),
                                                        event) == true) {
    assert(event.event_type == kEventTxSymbol);
    assert(event.ant_id == radio_id);
    size_t cur_offset = event.offset;
    long long txFrameTime =
        base_time + (event.frame_id - frame_id) * config_->samps_per_frame();
    if (config_->bs_hw_framer() == false)
      this->baseTxBeacon(radio_id, cell, event.frame_id, txFrameTime);
    for (size_t s = 0; s < config_->dl_slot_per_frame(); s++) {
      for (size_t ch = 0; ch < config_->bs_sdr_ch(); ++ch) {
        char* cur_ptr_buffer =
            bs_tx_buffer_[radio_id].buffer.data() + (cur_offset * packetLength);
        Packet* pkt = reinterpret_cast<Packet*>(cur_ptr_buffer);
        assert(pkt->ant_id == config_->bs_sdr_ch() * radio_id + ch);
        dl_txbuff.at(ch) = pkt->data;
        cur_offset = (cur_offset + 1) % tx_buffer_size;
      }
      long long txTime = 0;
      if (kUseSoapyUHD == true || kUsePureUHD == true ||
          config_->bs_hw_framer() == false) {
        txTime = txFrameTime +
                 config_->dl_slots().at(radio_id).at(s) * num_samps -
                 config_->tx_advance(radio_id);
      } else {
        txTime = ((size_t)event.frame_id << 32) |
                 (config_->dl_slots().at(radio_id).at(s) << 16);
      }
      if ((kUsePureUHD == true || kUseSoapyUHD == true) &&
          s < (config_->dl_slot_per_frame() - 1))
        flagsTxData = kStreamContinuous;  // HAS_TIME
      else
        flagsTxData = kStreamEndBurst;  // HAS_TIME & END_BURST, fixme
      int r;
      r = this->base_radio_set_->radioTx(radio_id, cell, dl_txbuff.data(),
                                         flagsTxData, txTime);

      if (r < num_samps) {
        MLPD_WARN("BAD Write: %d/%d\n", r, num_samps);
      }
    }
    bs_tx_buffer_[radio_id]
        .pkt_buf_inuse[event.frame_id % kSampleBufferFrameNum] = 0;
    return 0;
  }
  return -1;
}

void Receiver::notifyPacket(NodeType node_type, int frame_id, int slot_id,
                            int ant_id, int buff_size, int offset) {
  moodycamel::ProducerToken local_ptok(*message_queue_);
  Event_data new_frame;
  new_frame.event_type = kEventRxSymbol;
  new_frame.frame_id = frame_id;
  new_frame.slot_id = slot_id;
  new_frame.ant_id = ant_id;
  new_frame.node_type = node_type;
  new_frame.buff_size = buff_size;
  new_frame.offset = offset;
  if (message_queue_->enqueue(local_ptok, new_frame) == false) {
    MLPD_ERROR("New frame message enqueue failed\n");
    throw std::runtime_error("New frame message enqueue failed");
  }
}

void* Receiver::loopRecv_launch(void* in_context) {
  ReceiverContext* context = (ReceiverContext*)in_context;
  auto me = context->ptr;
  auto tid = context->tid;
  auto core_id = context->core_id;
  auto buffer = context->buffer;
  delete context;
  me->loopRecv(tid, core_id, buffer);
  return 0;
}

void Receiver::loopRecv(int tid, int core_id, SampleBuffer* rx_buffer) {
  if (config_->core_alloc() == true) {
    MLPD_INFO("Pinning rx thread %d to core %d\n", tid, core_id + tid);
    if (pin_to_core(core_id + tid) != 0) {
      MLPD_ERROR("Pin rx thread %d to core %d failed\n", tid, core_id + tid);
      throw std::runtime_error("Pin rx thread to core failed");
    }
  }

  // Use mutex to sychronize data receiving across threads
  if (config_->internal_measurement() ||
      ((config_->num_cl_sdrs() > 0) && (config_->num_bs_sdrs_all() > 0))) {
    pthread_mutex_lock(&mutex);
    MLPD_INFO("Recv Thread %d: waiting for release\n", tid);
    pthread_cond_wait(&cond, &mutex);
    pthread_mutex_unlock(&mutex);  // unlocking for all other threads
  }

  // use token to speed up
  moodycamel::ProducerToken local_ptok(*message_queue_);

  const size_t num_channels = config_->bs_channel().length();
  size_t packetLength = sizeof(Packet) + config_->getPacketDataLength();
  int buffer_chunk_size = rx_buffer[0].buffer.size() / packetLength;
  int bs_tx_buff_size = kSampleBufferFrameNum * config_->slot_per_frame();

  // handle two channels at each radio
  // this is assuming buffer_chunk_size is at least 2
  std::atomic_int* pkt_buf_inuse = rx_buffer[tid].pkt_buf_inuse;
  char* buffer = rx_buffer[tid].buffer.data();

  size_t num_radios = config_->num_bs_sdrs_all();  //config_->n_bs_sdrs()[0]
  std::vector<size_t> radio_ids_in_thread;
  if (config_->internal_measurement() && config_->ref_node_enable()) {
    if (tid == 0)
      radio_ids_in_thread.push_back(config_->cal_ref_sdr_id());
    else
      // FIXME: Does this work in multi-cell case?
      for (size_t it = 0; it < config_->num_bs_sdrs_all(); it++)
        if (it != config_->cal_ref_sdr_id()) radio_ids_in_thread.push_back(it);
  } else {
    size_t radio_start = (tid * num_radios) / thread_num_;
    size_t radio_end = ((tid + 1) * num_radios) / thread_num_;
    for (size_t it = radio_start; it < radio_end; it++)
      radio_ids_in_thread.push_back(it);
  }
  MLPD_INFO("Receiver thread %d has %zu radios\n", tid,
            radio_ids_in_thread.size());
  MLPD_TRACE(
      " -- %d - radio start: %zu, end: %zu, total radios %zu, thread: %zu\n",
      tid, radio_ids_in_thread.front(), radio_ids_in_thread.back(), num_radios,
      thread_num_);

  // prepare BS beacon in host buffer
  std::vector<void*> beaconbuff(2);
  void* zeroes_memory = calloc(config_->samps_per_slot(), sizeof(int16_t) * 2);

  if (zeroes_memory == NULL) {
    throw std::runtime_error("Memory allocation error");
  }

  MLPD_SYMBOL(
      "Process %d -- Loop Rx Allocated memory at: %p, approx size: %lu\n", tid,
      zeroes_memory, (sizeof(int16_t) * 2) * config_->samps_per_slot());
  beaconbuff.at(0u) = config_->beacon_ci16().data();
  beaconbuff.at(1u) = zeroes_memory;

  long long rxTimeBs(0);

  // read rx_offset to align the FPGA time of the BS
  // by performing dummy readStream()
  std::vector<std::complex<int16_t>> samp_buffer0(config_->samps_per_frame(),
                                                  0);
  std::vector<std::complex<int16_t>> samp_buffer1(config_->samps_per_frame(),
                                                  0);
  std::vector<void*> samp_buffer(2);
  samp_buffer[0] = samp_buffer0.data();
  if (num_channels == 2) samp_buffer[1] = samp_buffer1.data();
  // Scratch for the channel the reference antenna does NOT receive on. It has
  // to outlive the radioRx() call that writes into it: this used to be a
  // std::vector temporary, so samp[] held a dangling pointer and the radio
  // wrote into freed memory.
  std::vector<char> unused_channel_buffer(packetLength);

  int cell = 0;
  // for UHD device, the first pilot should not have an END_BURST flag
  if (kUseSoapyUHD == true || kUsePureUHD == true ||
      config_->bs_hw_framer() == false) {
    // For multi-USRP BS perform dummy radioRx to avoid initial late packets
    int bs_sync_ret = -1;
    MLPD_INFO("Sync BS host and FPGA timestamp for thread %d\n", tid);
    for (auto& it : radio_ids_in_thread) {
      // Find cell this USRP belongs to..,
      for (size_t i = 0; i <= config_->num_cells(); i++) {
        if (it < config_->n_bs_sdrs_agg().at(i)) {
          cell = i - 1;
          break;
        }
      }
      size_t radio_id = it - config_->n_bs_sdrs_agg().at(cell);
      bs_sync_ret = -1;
      while (bs_sync_ret < 0) {
        bs_sync_ret =
            this->base_radio_set_->radioRx(radio_id, cell, samp_buffer.data(),
                                           config_->samps_per_slot(), rxTimeBs);
      }
    }
  }

  int cursor = 0;
  size_t frame_id = 0;
  size_t slot_id = 0;
  size_t ant_id = 0;
  cell = 0;
  MLPD_INFO("Start BS main recv loop in thread %d\n", tid);
  while (config_->running() == true) {
    // Global updates of frame and slot IDs for USRPs
    if (kUseSoapyUHD == true || kUsePureUHD == true ||
        config_->bs_hw_framer() == false) {
      if (slot_id == config_->slot_per_frame()) {
        slot_id = 0;
        frame_id++;
      }
    }

    // Receive data
    for (auto& it : radio_ids_in_thread) {
      Packet* pkt[num_channels];
      void* samp[num_channels];

      // Find cell this board belongs to...
      for (size_t i = 0; i <= config_->num_cells(); i++) {
        if (it < config_->n_bs_sdrs_agg().at(i)) {
          cell = i - 1;
          break;
        }
      }

      size_t radio_id = it - config_->n_bs_sdrs_agg().at(cell);

      size_t num_packets =
          config_->internal_measurement() &&
                  radio_id == config_->cal_ref_sdr_id() &&
                  config_->ref_node_enable()
              ? 1
              : num_channels;  // receive only on one channel at the ref antenna

      // Set buffer status(es) to full; fail if full already
      for (size_t ch = 0; ch < num_packets; ++ch) {
        int bit = 1 << (cursor + ch) % sizeof(std::atomic_int);
        int offs = (cursor + ch) / sizeof(std::atomic_int);
        int old = std::atomic_fetch_or(&pkt_buf_inuse[offs], bit);  // now full
        // if buffer was full, exit
        if ((old & bit) != 0) {
          MLPD_ERROR("thread %d buffer full\n", tid);
          throw std::runtime_error("Thread %d buffer full\n");
        }
        // Reserved until marked empty by consumer
      }

      // Receive data into buffers
      for (size_t ch = 0; ch < num_packets; ++ch) {
        pkt[ch] = (Packet*)(buffer + (cursor + ch) * packetLength);
        samp[ch] = pkt[ch]->data;
      }
      if (num_packets != num_channels)
        samp[num_channels - 1] = unused_channel_buffer.data();

      assert(this->base_radio_set_ != NULL);

      ant_id = radio_id * num_channels;

      if (kUseSoapyUHD == true || kUsePureUHD == true ||
          config_->bs_hw_framer() == false) {
        int rx_len = config_->samps_per_slot();
        int r;

        // only write received pilot or data into samp
        // otherwise use samp_buffer as a dummy buffer
        if (config_->isPilot(cell, radio_id, slot_id) ||
            config_->isUlData(cell, radio_id, slot_id))
          r = this->base_radio_set_->radioRx(radio_id, cell, samp, rxTimeBs);
        else
          r = this->base_radio_set_->radioRx(radio_id, cell, samp_buffer.data(),
                                             rxTimeBs);

        if (r < 0) {
          config_->running(false);
          break;
        }
        if (r != rx_len) {
          std::cerr << "BAD Receive(" << r << "/" << rx_len << ") at Time "
                    << rxTimeBs << ", frame count " << frame_id << std::endl;
        }

        // schedule all TX slot
        if (slot_id == 0) {
          // schedule downlink slots
          if (config_->dl_data_slot_present() == true) {
            while (-1 != baseTxData(radio_id, cell, frame_id, rxTimeBs))
              ;
            this->notifyPacket(kBS, frame_id + this->txFrameDelta_, 0, radio_id,
                               bs_tx_buff_size);  // Notify new frame
          } else {
            this->baseTxBeacon(radio_id, cell, frame_id,
                               rxTimeBs + txTimeDelta_);
          }  // end if config_->dul_data_slot_present()
        }
        if (!config_->isPilot(cell, radio_id, slot_id) &&
            !config_->isUlData(cell, radio_id, slot_id)) {
          for (size_t ch = 0; ch < num_packets; ++ch) {
            const int bit = 1 << (cursor + ch) % sizeof(std::atomic_int);
            const int offs = (cursor + ch) / sizeof(std::atomic_int);
            const int old =
                std::atomic_fetch_and(&pkt_buf_inuse[offs], ~bit);  // now empty
            // if buffer was empty, exit
            if ((old & bit) != bit) {
              MLPD_ERROR("thread %d freed buffer when already free\n", tid);
              throw std::runtime_error("buffer empty during free\n");
            }
            // Reserved until marked empty by consumer
          }
          continue;
        }

      } else {
        long long frameTime = 0;
        const int rx_ret =
            this->base_radio_set_->radioRx(radio_id, cell, samp, frameTime);
        if (rx_ret < 0) {
          config_->running(false);
          break;
        }
        if (rx_ret == 0) {
          // No slot this round: the framer has no rx slots yet, or the read came
          // back too short to yield one. Either way buffs and frameTime were left
          // untouched, so publishing here would build a packet on an unset
          // frameTime (garbage frame/slot ids) over stale samples. Release the
          // reserved buffers and move on (AP-10).
          //
          // Say so, throttled. Dropping the packet is right, but doing it
          // SILENTLY turned a dead RX stream into a run that printed nothing at
          // all and looked alive -- worse to diagnose than the garbage packets
          // this replaced. Braces matter: MLPD_WARN is multi-statement.
          static std::atomic<long long> noslot{0};
          const long long n_noslot = noslot.fetch_add(1);
          if ((n_noslot % 2000) == 0) {
            MLPD_WARN(
                "BS recv: no slot from radio %zu (%lld slotless reads so "
                "far). Either the UE is not transmitting (pause, escalation, "
                "quiet-gate skip) or the RX stream is delivering nothing "
                "(check the BS opened and its data-plane egress).\n",
                radio_id, n_noslot + 1);
          }
          for (size_t ch = 0; ch < num_packets; ++ch) {
            const int bit = 1 << (cursor + ch) % sizeof(std::atomic_int);
            const int offs = (cursor + ch) / sizeof(std::atomic_int);
            const int old =
                std::atomic_fetch_and(&pkt_buf_inuse[offs], ~bit);  // now empty
            if ((old & bit) != bit) {
              MLPD_ERROR("thread %d freed buffer when already free\n", tid);
              throw std::runtime_error("buffer empty during free\n");
            }
          }
          continue;
        }

        frame_id = (size_t)(frameTime >> 32);
        slot_id = (size_t)((frameTime >> 16) & 0xFFFF);

        if (config_->internal_measurement() && config_->ref_node_enable()) {
          size_t beacon_slot = config_->num_cl_antennas() > 0 ? 1 : 0;
          if (radio_id == config_->cal_ref_sdr_id()) {
            ant_id = slot_id - beacon_slot;
            slot_id = 0;  // downlink reciprocal pilot
          } else {
            slot_id -= config_->cal_ref_sdr_id();  // uplink pilots
          }
        } else if (config_->internal_measurement() &&
                   !config_->ref_node_enable()) {
          // Mapping (compress schedule to eliminate Gs)
          size_t adv = int(slot_id / (config_->guard_mult() * num_channels));
          slot_id -= ((config_->guard_mult() - 1) * num_channels * adv);
        } else if (config_->getClientId(ant_id / num_channels, slot_id) ==
                   0) {  // first received pilot
          if (config_->dl_data_slot_present() == true) {
            while (-1 != baseTxData(radio_id, cell, frame_id, frameTime))
              ;
            this->notifyPacket(kBS, frame_id + this->txFrameDelta_, 0, radio_id,
                               bs_tx_buff_size);  // Notify new frame
          }
        }
      }

#if DEBUG_PRINT
      for (size_t ch = 0; ch < num_packets; ++ch) {
        printf(
            "receive thread %d, frame %zu, slot %zu, cell %zu, ant "
            "%zu samples: %d %d %d %d %d %d %d %d ...\n",
            tid, frame_id, slot_id, cell, ant_id + ch, pkt[ch]->data[1],
            pkt[ch]->data[2], pkt[ch]->data[3], pkt[ch]->data[4],
            pkt[ch]->data[5], pkt[ch]->data[6], pkt[ch]->data[7],
            pkt[ch]->data[8]);
      }
#endif

      // Zeros the RX path inserted to cover a dropped packet. Carried on the packet
      // so consumers that compute on the samples can refuse to trust them (AP-10).
      const uint32_t rx_pad = static_cast<uint32_t>(
          this->base_radio_set_->lastRxPadSamples(radio_id, cell));
      for (size_t ch = 0; ch < num_packets; ++ch) {
        // new (pkt[ch]) Packet(frame_id, slot_id, 0, ant_id + ch);
        new (pkt[ch]) Packet(frame_id, slot_id, cell, ant_id + ch, rx_pad);
        // push kEventRxSymbol event into the queue
        this->notifyPacket(kBS, frame_id, slot_id, ant_id + ch,
                           buffer_chunk_size, cursor + tid * buffer_chunk_size);
        cursor++;
        cursor %= buffer_chunk_size;
      }
    }

    // for UHD device update slot_id on host
    if (kUseSoapyUHD == true || kUsePureUHD == true ||
        config_->bs_hw_framer() == false) {
      slot_id++;
    }
  }
  MLPD_SYMBOL("Process %d -- Loop Rx Freed memory at: %p\n", tid,
              zeroes_memory);
  free(zeroes_memory);
}

void* Receiver::clientTxRx_launch(void* in_context) {
  ReceiverContext* context = (ReceiverContext*)in_context;
  auto me = context->ptr;
  auto tid = context->tid;
  auto core_id = context->core_id;
  auto buffer = context->buffer;
  delete context;
  if (me->config_->hw_framer())
    me->clientTxRx(tid);
  else
    me->clientSyncTxRx(tid, core_id, buffer);
  return 0;
}

void Receiver::clientTxRx(int tid) {
  size_t tx_slots = config_->cl_ul_slots().at(tid).size();
  size_t rxSyms = config_->cl_dl_slots().at(tid).size();
  int txStartSym = config_->cl_ul_slots().at(tid).empty()
                       ? 0
                       : config_->cl_ul_slots().at(tid).at(0);
  int NUM_SAMPS = config_->samps_per_slot();

  if (config_->core_alloc() == true) {
    int core =
        tid + 1 + config_->bs_rx_thread_num() + config_->recorder_thread_num();
    MLPD_INFO("Pinning client TxRx thread %d to core %d\n", tid, core);
    if (pin_to_core(core) != 0) {
      MLPD_ERROR("Pin client TxRx thread %d to core %d failed in client txrx\n",
                 tid, core);
      throw std::runtime_error(
          "Pin client TxRx thread to core failed in client txr");
    }
  }

  std::vector<std::complex<int16_t>> buffs(NUM_SAMPS, 0);
  std::vector<void*> rxbuff(2);
  rxbuff[0] = buffs.data();
  rxbuff[1] = buffs.data();

  std::vector<void*> ul_txbuff(2);
  for (size_t ch = 0; ch < config_->cl_sdr_ch(); ch++) {
    ul_txbuff.at(ch) =
        std::calloc(config_->samps_per_slot(), sizeof(int16_t) * 2);
  }
  //size_t slot_byte_size = config_->samps_per_slot() * sizeof(int16_t) * 2;
  //if (tx_slots > 0) {
  //  size_t txIndex = tid * config_->cl_sdr_ch();
  //  for (size_t ch = 0; ch < config_->cl_sdr_ch(); ch++) {
  //    std::memcpy(ul_txbuff.at(ch),
  //                config_->txdata_time_dom().at(txIndex + ch).data(),
  //                slot_byte_size);
  //  }
  //  MLPD_INFO("%zu uplink slots will be sent per frame...\n", tx_slots);
  //}

  int all_trigs = 0;
  struct timespec tv, tv2;
  clock_gettime(CLOCK_MONOTONIC, &tv);

  assert(client_radio_set_ != NULL);

  while (config_->running() == true) {
    clock_gettime(CLOCK_MONOTONIC, &tv2);
    double diff =
        ((tv2.tv_sec - tv.tv_sec) * 1e9 + (tv2.tv_nsec - tv.tv_nsec)) / 1e9;
    if ((config_->frame_mode() != "free_running") && (diff > 2)) {
      int total_trigs;
      total_trigs = client_radio_set_->triggers(tid);

      std::cout << "new triggers: " << total_trigs - all_trigs
                << ", total: " << total_trigs << std::endl;
      all_trigs = total_trigs;
      tv = tv2;
    }
    // receiver loop
    long long rxTime(0);
    long long txTime(0);
    long long firstRxTime(0);
    bool receiveErrors = false;
    for (size_t i = 0; i < rxSyms; i++) {
      int r;
      r = client_radio_set_->radioRx(tid, rxbuff.data(), NUM_SAMPS, rxTime);
      if (r == NUM_SAMPS) {
        if (i == 0) firstRxTime = rxTime;
      } else {
        std::cerr << "waiting for receive frames... " << std::endl;
        receiveErrors = true;
        break;
      }
    }
    if (receiveErrors == false) {
      // transmit loop
      txTime = firstRxTime & 0xFFFFFFFF00000000;
      txTime += ((long long)this->txFrameDelta_ << 32);
      txTime += ((long long)txStartSym << 16);
      //printf("rxTime %llx, txTime %llx \n", firstRxTime, txTime);
      for (size_t i = 0; i < tx_slots; i++) {
        int r;
        r = client_radio_set_->radioTx(tid, ul_txbuff.data(), NUM_SAMPS, 1,
                                       txTime);
        if (r == NUM_SAMPS) {
          txTime += 0x10000;
        }
      }
    }  // end receiveErrors == false)
  }    // end while config_->running() == true)
  for (size_t ch = 0; ch < config_->cl_sdr_ch(); ch++) {
    std::free(ul_txbuff.at(ch));
  }
}

void Receiver::clientTxPilots(size_t user_id, long long base_time,
                              double frame_period) {
  // for UHD device, the first pilot should not have an END_BURST flag
  int flags = (((kUsePureUHD == true || kUseSoapyUHD == true) &&
                (config_->cl_sdr_ch() == 2)))
                  ? kStreamContinuous
                  : kStreamEndBurst;
  int num_samps = config_->samps_per_slot();
  long long txTime = base_time +
                     config_->cl_pilot_slots().at(user_id).at(0) * num_samps -
                     config_->tx_advance(user_id);
  // Houdini pilot-only closed loop: the BS and UE loops are async and slower than
  // real-time (recvHoudini drains before it reads), so a single once-per-loop timed
  // pilot rarely lands in the frame the BS happens to arm its rx_gate on. With the
  // boards frequency-locked (CFO ~0, no drift) the pilot offset is stable, so emit
  // it on EVERY frame across a horizon ahead of real time -- then whichever frame
  // the BS listens on, a pilot is there. A per-thread cursor keeps the schedule
  // continuous and non-overlapping (each client tid runs this on one thread).
  // Viewing mode also sends an uplink DATA slot (U) each frame so the BS can
  // equalize it and render the constellation. It rides the same continuous burst,
  // offset from the pilot by (U_slot - P_slot) slots.
  const bool ul_present = !config_->cl_ul_slots().at(user_id).empty() &&
                          config_->ue_data_ci16().size() >= (size_t)num_samps;
  const long long ul_off =
      ul_present ? (static_cast<long long>(config_->cl_ul_slots().at(user_id).at(0)) -
                    static_cast<long long>(config_->cl_pilot_slots().at(user_id).at(0))) *
                       num_samps
                 : 0;
  static const int horizon_env = [] {
    const char* he = std::getenv("HOUDINI_PILOT_HORIZON");
    return he != nullptr ? std::atoi(he) : -1;
  }();
  const int horizon =
      horizon_env >= 0 ? horizon_env : config_->ue_pilot_horizon();
  if (horizon > 0 && config_->is_houdini() && config_->cl_sdr_ch() == 1) {
    // AP-31(c). This ladder used to step by samps_per_frame, on the assumption
    // stated in the comment above -- "with the boards frequency-locked (CFO ~0,
    // no drift) the pilot offset is stable". On free-running clocks it is not,
    // and because `cur` starts at max(pilot_cursor + frame, txTime) the cursor
    // wins that max for a whole horizon at a time, so a base_time riding the
    // tracked grid was being ignored for ~96 frames and the pilot walked at the
    // clock rate even with the UE's own sync loop locked (measured -1.58
    // samples per frame on 2026-09-01 with the tracker holding resid inside
    // +-68). Step by the tracked period instead, and index off txTime so the
    // rounding never accumulates: at 122881.0588 a per-step llround would
    // shed 0.0588 samples every frame, which is ~59 samples per second.
    const double frame_d = (frame_period > 0.0)
                               ? frame_period
                               : static_cast<double>(config_->samps_per_frame());
    const long long frame = llround(frame_d);
    // The driver only ACCEPTS burst anchors on the 384-tick / 3125 ns grid
    // (the finest ns-exact grid, TxTickAnchor SH-248), but a burst's INTERIOR
    // advances tick-exactly. So compose ONE burst per frame -- [front-pad
    // zeros | pilot slot | gap zeros | data slot] -- anchored at the grid
    // point floored below the desired start: the pad places the pilot to the
    // sample and the data rides at EXACTLY ul_off from it. Both snap draws
    // measured in ledger 4.44 (the +-192 per-run seat window and the bimodal
    // -128/+256 P->U differential) die here. [user 2026-08-30: "work around
    // the TX burst seam by zero padding".]
    //
    // The pad is NOT constant any more, and the comment that said it was
    // predated the tracker. The ladder steps by the TRACKED period (122881.05,
    // not the 122880 = 320*384 that divided the grid exactly), so `cur` advances
    // ~1 tick past the 384 grid per burst and `pad` changes on essentially every
    // one: the assign plus two memcpys run per scheduled frame rather than once
    // per run. Still correct, and measured cost is what decides whether it is
    // worth caching by pad (AP-54); do not re-assert the old property.
    constexpr long long kTddGridTicks = houdini::sync::kHoudiniStrobeOffsetTicks;
    thread_local long long pilot_cursor = 0;  // last-scheduled txTime (samples)
    thread_local std::vector<std::complex<int16_t>> burst;
    thread_local long long burst_pad = -1;
    // An anchor change must reach the WIRE: the cursor otherwise keeps
    // winning the max() below for ~horizon frames and the pilots stay on the
    // stale grid (Opus review finding 4). On an escalation re-anchor, resume
    // on the NEW grid at the first slot AFTER everything already queued:
    // jumping back to txTime would command times behind bursts the driver
    // already accepted (a late-start throw -> BAD Write -> a stalled cursor
    // retrying the same overlap forever, Opus review M2), and nothing here
    // can flush the driver's queue -- the stale-grid bursts simply drain
    // (up to ~horizon frames) while the new grid takes over behind them.
    const long long end = txTime + llround(horizon * frame_d);
    // Every burst is txTime + i * tracked_period for integer i, so the whole
    // ladder rides the tracked grid and no rounding accumulates along it.
    long long i0 = 0;
    if (pilot_cursor + frame > txTime) {
      i0 = static_cast<long long>(std::ceil(
          static_cast<double>(pilot_cursor + frame - txTime) / frame_d));
    }
    // There used to be a re-anchor flag set by the escalation and consumed
    // here. i0 above subsumes it: it ALWAYS resumes on the current grid at the
    // first index past what is already queued, which is exactly what the flag
    // triggered. It was kept for a while as write-only state, which is worse
    // than either keeping or removing it, because an atomic that nothing reads
    // still looks load-bearing to the next reader.
    int nsched = 0;
    const bool ul_fits = ul_present && ul_off >= num_samps;
    for (long long i = i0;; ++i) {
      const long long cur = txTime + llround(static_cast<double>(i) * frame_d);
      if (cur > end) break;
      const long long anchor = (cur / kTddGridTicks) * kTddGridTicks;
      const long long pad = cur - anchor;
      if (pad != burst_pad) {
        const size_t total = static_cast<size_t>(pad) + num_samps +
                             (ul_fits ? static_cast<size_t>(ul_off) : 0);
        // total kept even so the burst ends on a whole 2-sample TX unit; the
        // trailing zero does not move any signal.
        burst.assign(total + (total & 1), std::complex<int16_t>(0, 0));
        std::memcpy(burst.data() + pad, pilotbuffA_.at(0),
                    static_cast<size_t>(num_samps) * 4);
        if (ul_fits) {
          std::memcpy(burst.data() + pad + ul_off, ue_databuffA_.at(0),
                      static_cast<size_t>(num_samps) * 4);
        }
        burst_pad = pad;
      }
      long long tt = anchor;  // grid-exact, so radioTx's snap is a no-op
      const void* bufs[1] = {burst.data()};
      const int rr = client_radio_set_->radioTx(
          user_id, bufs, static_cast<int>(burst.size()), flags, tt);
      if (rr < static_cast<int>(burst.size())) {
        MLPD_WARN("BAD Write (burst @%lld): %d/%zu\n", cur, rr, burst.size());
        break;
      }
      pilot_cursor = cur;
      ++nsched;
    }
    // A full writeStream return only means the burst was ACCEPTED. Whether it
    // actually went out on its tick is asynchronous, and on the fine TDD grid a
    // late burst is exactly a phase jump with no other symptom. Drain here, after
    // scheduling, so the cost is once per horizon rather than per burst (AP-10).
    client_radio_set_->drainTxStatus(user_id);
    static const bool kUeTxDebug = std::getenv("HOUDINI_UE_TX_DEBUG") != nullptr;  // read once
    if (kUeTxDebug && nsched > 0) {
      MLPD_INFO("UE pilot burst: scheduled %d frames up to %lld (pad %lld)\n",
                nsched, pilot_cursor, burst_pad);
    }
    return;
  }
  int r;
  r = client_radio_set_->radioTx(user_id, pilotbuffA_.data(), num_samps, flags,
                                 txTime);

  if (r < num_samps) {
    MLPD_WARN("BAD Write: %d/%d\n", r, num_samps);
  }
  if (config_->cl_sdr_ch() == 2) {
    txTime = base_time +
             config_->cl_pilot_slots().at(user_id).at(1) * num_samps -
             config_->tx_advance(user_id);
    r = client_radio_set_->radioTx(user_id, pilotbuffB_.data(), num_samps,
                                   kStreamEndBurst, txTime);

    if (r < num_samps) {
      MLPD_WARN("BAD Write: %d/%d\n", r, num_samps);
    }
  }
}

int Receiver::clientTxData(int tid, int frame_id, long long base_time) {
  size_t tx_slots = config_->cl_ul_slots().at(tid).size();
  int num_samps = config_->samps_per_slot();
  size_t packetLength = sizeof(Packet) + config_->getPacketDataLength();
  size_t tx_buffer_size = cl_tx_buffer_[tid].buffer.size() / packetLength;
  int flagsTxUlData;
  std::vector<void*> ul_txbuff(2);
  Event_data event;
  if (cl_tx_queue_.at(tid)->try_dequeue_from_producer(*cl_tx_ptoks_.at(tid),
                                                      event) == true) {
    assert(event.event_type == kEventTxSymbol);
    assert(event.ant_id == tid);
    size_t cur_offset = event.offset;
    long long txFrameTime =
        base_time + (event.frame_id - frame_id) * config_->samps_per_frame();
    clientTxPilots(tid,
                   txFrameTime);  // assuming pilot is always sent before data

    for (size_t s = 0; s < tx_slots; s++) {
      for (size_t ch = 0; ch < config_->cl_sdr_ch(); ++ch) {
        char* cur_ptr_buffer =
            cl_tx_buffer_[tid].buffer.data() + (cur_offset * packetLength);
        Packet* pkt = reinterpret_cast<Packet*>(cur_ptr_buffer);
        assert(pkt->slot_id == config_->cl_ul_slots().at(tid).at(s));
        assert(pkt->ant_id == config_->cl_sdr_ch() * tid + ch);
        ul_txbuff.at(ch) = pkt->data;
        cur_offset = (cur_offset + 1) % tx_buffer_size;
      }
      long long txTime = txFrameTime +
                         config_->cl_ul_slots().at(tid).at(s) * num_samps -
                         config_->tx_advance(tid);
      if ((kUsePureUHD || kUseSoapyUHD) && s < (tx_slots - 1)) {
        flagsTxUlData = kStreamContinuous;  // HAS_TIME
      } else {
        flagsTxUlData = kStreamEndBurst;  // HAS_TIME & END_BURST, fixme
      }
      int r;
      r = client_radio_set_->radioTx(tid, ul_txbuff.data(), num_samps,
                                     flagsTxUlData, txTime);
      if (r < num_samps) {
        MLPD_WARN("BAD Write: %d/%d\n", r, num_samps);
      }
    }
    cl_tx_buffer_[tid].pkt_buf_inuse[event.frame_id % kSampleBufferFrameNum] =
        0;
    return 0;
  }
  return -1;
}

ssize_t Receiver::syncSearch(const std::complex<int16_t>* check_data,
                             size_t search_window, float corr_scale,
                             houdini::sync::PickRule pick,
                             houdini::sync::Detection* detection) {
  // One detector for BOTH search paths: acquisition and resync agree about what
  // the threshold means, which form the replica supports, and where the beacon
  // END sits relative to the correlator's index (the replica tail) -- all
  // resolved once in the constructor (sync/detector.h). The CUDA path is not
  // wired through the library yet: it still returns the first crossing and
  // needs the argmax-by-ratio change before it can serve acquisition.
  assert(search_window <= config_->samps_per_frame());
  // One detector for every backend: the CUDA correlator (USE_CUDA) is a
  // backend inside it, and the index convention is applied in one place.
  const char* kPath = sync_detector_->backendName();
  const houdini::sync::Detection det =
      sync_detector_->run(check_data, search_window, corr_scale, pick);
  const ssize_t sync_index = det.end_index;
  if (detection != nullptr) *detection = det;
  static const bool kSyncDebug = std::getenv("HOUDINI_SYNC_DEBUG") != nullptr;  // read once
  if (kSyncDebug) {
    static std::atomic<int> c{0};
    if ((c.fetch_add(1) % 20) == 0) {  // braces load-bearing (macro)
      MLPD_INFO("syncSearch[%s]: window=%zu corr_scale=%.3f gold=%zu pick=%d "
                "-> idx=%ld\n",
                kPath, search_window, corr_scale, config_->gold_cf32().size(),
                static_cast<int>(pick), sync_index);
    }
  }
  return sync_index;
}

// ---- SYN1: sync/CFO telemetry to the dashboard (AP-32) ---------------------
// The GUI socket in RecorderWorker is fed from the RECORDING path and carries
// per-antenna CSI; the sync state lives here in the client RX thread and has no
// route to it. Rather than plumb a queue across threads, this path opens its own
// connected UDP socket to the SAME destination (HOUDINI_CSI_UDP) and emits one
// small datagram per resync DETECTION.
//
// Per detection, never sampled at display cadence: detections run ~9/s, slower
// than the 30 fps display throttle, so resampling would alias exactly the thing
// the panel exists to show (the bug f8ba2b4 fixed on the retired H-stability
// strip). At ~9/s x 44 bytes this costs nothing.
static int syncTelemetrySock(void) {
  static const int fd = [] {
    const char* dst = std::getenv("HOUDINI_CSI_UDP");
    if (dst == nullptr) return -1;
    const std::string s(dst);
    const auto colon = s.find(':');
    const std::string host =
        (colon == std::string::npos) ? "127.0.0.1" : s.substr(0, colon);
    const int port = (colon == std::string::npos)
                         ? 9999
                         : std::atoi(s.substr(colon + 1).c_str());
    const int f = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (f < 0) return -1;
    sockaddr_in a{};
    a.sin_family = AF_INET;
    a.sin_port = htons(static_cast<uint16_t>(port));
    if (::inet_pton(AF_INET, host.c_str(), &a.sin_addr) != 1 ||
        ::connect(f, reinterpret_cast<sockaddr*>(&a), sizeof(a)) != 0) {
      ::close(f);
      return -1;
    }
    return f;
  }();
  return fd;
}

// State codes shared with csi_server.py's SYN1 parser. NOT_SYNCED is not sent:
// the page infers it from datagram staleness, the same idiom the antenna panels
// already use, because there is no detection to hang a datagram on while the
// acquisition loop is hunting.
enum SyncState : uint32_t {
  kSyncLocked = 1,       // beacon alive on the anchored grid
  kSyncHold = 2,         // one off-grid detection, deliberately NOT acted on
  kSyncEscalating = 3,   // anchor re-acquired; `shift` is the step applied
  kSyncWeak = 4,         // detected but under the SNR floor -- a WEAK beacon,
                         // which without this is indistinguishable from none
  kSyncReanchorFailed = 5  // escalation ran and re-acquisition did NOT confirm
};

// [magic 'SYN1'][frame u32][tid u32][state u32][resid i32][cfo_hz f32][snr f32]
// [shift i32][samps_per_frame u32][carrier_hz f32][scatter_tol u32]
// [cfo_beacon_hz f32] -- 48 bytes.
// The samps_per_frame / carrier / scatter_tol trio lets the panel convert to
// ppm and draw the accept/reject band without hardcoding config values into the
// page, which is exactly what breaks when a config changes.
//
// TWO CFO FIELDS, DELIBERATELY. `cfo_hz` is the TRACKED value (eps from the
// timing grid x carrier) -- accurate, and the one a correction should use.
// `cfo_beacon_hz` is the beacon phase estimator's own reading -- an independent
// instrument, precise but carrying a configuration-dependent bias (+280 to
// +1753 Hz across the campaign legs, DEMO_VERIFICATION 8.6). Carrying only the
// tracked value made the panel's "these two should agree" cross-check
// unresolvable, because the other figure on that line is the tracking RESIDUAL
// (~0.04 ppm) and not an independent measurement of the same thing. Sent as
// NaN on records with no fresh detection (escalation, re-anchor failure, weak);
// the page drops the field, not the datagram.
//
// `shift` is the schedule step actually applied, NOT fresh-minus-previous: both
// anchors are ABSOLUTE sample times taken k frames apart, so their difference is
// dominated by k*samps_per_frame elapsed time and overflows the int32 wire field
// after ~17.5 s of run. It is reduced modulo the TRACKED frame period and
// centred. It is nonzero on an escalation re-anchor AND on a LOCKED record,
// where the tracker's alpha half moves the schedule by kGridAlpha * resid.
static void sendSyncTelemetry(size_t frame, int tid, uint32_t state,
                              long long resid, double cfo_hz, double snr,
                              long long shift, uint32_t sfr, float carrier,
                              uint32_t scatter_tol, double cfo_beacon_hz) {
  const int fd = syncTelemetrySock();
  if (fd < 0) return;
  uint8_t buf[48];
  const uint32_t magic = 0x53594E31u, fr = static_cast<uint32_t>(frame),
                 ti = static_cast<uint32_t>(tid);
  const int32_t rs = static_cast<int32_t>(resid),
                sh = static_cast<int32_t>(shift);
  const float cf = static_cast<float>(cfo_hz), sn = static_cast<float>(snr);
  std::memcpy(buf + 0, &magic, 4);
  std::memcpy(buf + 4, &fr, 4);
  std::memcpy(buf + 8, &ti, 4);
  std::memcpy(buf + 12, &state, 4);
  std::memcpy(buf + 16, &rs, 4);
  std::memcpy(buf + 20, &cf, 4);
  std::memcpy(buf + 24, &sn, 4);
  std::memcpy(buf + 28, &sh, 4);
  std::memcpy(buf + 32, &sfr, 4);
  std::memcpy(buf + 36, &carrier, 4);
  std::memcpy(buf + 40, &scatter_tol, 4);
  const float cb = static_cast<float>(cfo_beacon_hz);
  std::memcpy(buf + 44, &cb, 4);
  (void)::send(fd, buf, sizeof(buf), 0);
}

// The beacon's own carrier-offset estimate is sync::RepetitionPhaseEstimator
// (sync/cfo_estimator.h): the two repetition correlations on the shape's
// geometry, NaN on every failure path, the Houdini mixer's conjugation undone.
// The window placement rules and their measured consequences (AP-39, 8.164) are
// documented there. `sync_index` is the beacon END, as everywhere in this file.

void Receiver::clientSyncTxRx(int tid, int core_id, SampleBuffer* rx_buffer) {
  if (config_->core_alloc() == true) {
    const int core = tid + core_id;

    MLPD_INFO("Pinning client synctxrx thread %d to core %d\n", tid, core);
    if (pin_to_core(core) != 0) {
      MLPD_ERROR("Pin client synctxrx thread %d to core %d failed\n", tid,
                 core);
      throw std::runtime_error("Failed to Pin client synctxrx thread to core");
    }
  }

  MLPD_INFO("Scheduling TX: %zu Frames (%lf ms) in the future!\n",
            this->txFrameDelta_,
            this->txFrameDelta_ * (1e3 * config_->getFrameDurationSec()));

  const size_t samples_per_slot = config_->samps_per_slot();
  const size_t num_rx_buffs = config_->cl_sdr_ch();
  std::vector<std::vector<std::complex<int16_t>>> samplemem(
      num_rx_buffs, std::vector<std::complex<int16_t>>(
                        samples_per_slot, std::complex<int16_t>(0, 0)));

  std::vector<void*> rxbuff;
  for (size_t ch = 0; ch < num_rx_buffs; ch++) {
    rxbuff.push_back(samplemem.at(ch).data());
  }

  const size_t ant_id = tid * config_->cl_sdr_ch();
  // use token to speed up
  moodycamel::ProducerToken local_ptok(*message_queue_);

  char* buffer = nullptr;
  std::atomic_int* pkt_buf_inuse = nullptr;
  int buffer_chunk_size = 0;
  int buffer_id = tid + config_->bs_rx_thread_num();
  size_t packetLength = sizeof(Packet) + config_->getPacketDataLength();
  if (config_->cl_dl_slots().at(0).empty() == false) {
    buffer_chunk_size = rx_buffer[buffer_id].buffer.size() / packetLength;
    // handle two channels at each radio
    // this is assuming buffer_chunk_size is at least 2
    pkt_buf_inuse = rx_buffer[buffer_id].pkt_buf_inuse;
    buffer = rx_buffer[buffer_id].buffer.data();
  }

  // tx_buffer info
  size_t tx_buffer_size = 0;
  if (config_->ul_data_slot_present() == true) {
    tx_buffer_size = cl_tx_buffer_[tid].buffer.size() / packetLength;
  }

  // For USRP clients skip UHD_INIT_TIME_SEC to avoid late packets
  if (kUsePureUHD == true || kUseSoapyUHD == true) {
    auto start_time = std::chrono::steady_clock::now();
    std::chrono::duration<float> elapsed_seconds{0.0};

    while (elapsed_seconds.count() < UHD_INIT_TIME_SEC) {
      long long ignore_time;
      client_radio_set_->radioRx(tid, rxbuff.data(), samples_per_slot,
                                 ignore_time);
      elapsed_seconds = std::chrono::duration_cast<std::chrono::seconds>(
          std::chrono::steady_clock::now() - start_time);
    }
    std::printf("Wait duration %3.2f seconds \n", elapsed_seconds.count());
  }

  //-------------------- New sync
  // ACQUISITION window = one whole frame. The beacon repeats every
  // samps_per_frame, so a full-frame read contains one with probability 1
  // (bar the ~0.4% that straddle the boundary, which simply retries at a new
  // phase) instead of the 7.4% a 2.33-slot window gets. Measured costs put the
  // optimum exactly here: expected time = F*a_read/W + F*(b_read+b_corr), so it
  // falls with W until the hit probability saturates at W = F, and reading
  // beyond a frame buys nothing. 15.5 ms -> 4.5 ms, and deterministic.
  // DERIVED BEFORE ACQUISITION, because acquisition needs it. The
  // acquisition gate is CLAMPED by the tracking gate (confirm <= scatter),
  // and houdiniAcquireAnchor used to re-derive its own copy with the
  // scatter tolerance hardcoded to the default -- so sweeping
  // HOUDINI_SCATTER_TOL_US, which the walkthrough documents and session-plan
  // leg 9 does, silently inverted the two gates and would have produced a
  // lock that escalates immediately, forever. One derivation, passed in.
  // DEFAULT LOWERED 8.3333 -> 2.0 us [user 2026-09-02, "a conservative value"].
  // 8.3333 us = 1024 samples was never derived from a measurement of this gate;
  // it dated from the pre-targeting era when find_beacon could anchor hundreds
  // of samples early. With targeted resync the measured detection residual is
  // -2 to +3 samples across ~20 runs, worst |resid| 6, so 1024 was ~170x the
  // worst case and the row AP-52 calls it "demote from control law to a tighter
  // outlier reject".
  //
  // 2.0 us = 246 samples is 41x the worst observed residual AND is the only
  // tighter value with silicon evidence: the AP-52 sweep ran it for 60 s with 0
  // escalations, 0 off-grid, the same residual spread, and 153 accepted
  // detections against the baseline's 91 -- because a tighter tolerance shrinks
  // kLead/kTail and therefore WIDENS the accept window, 42.2 % of the slot to
  // 80.2 %. Tighter still would be untested, which is the argument against it.
  const double kScatterTolUs = config_->sync().resync.scatter_tol_us;
  // The same argument that made kScatterTol a TIME (AP-40): what the
  // acquisition gate admits is detector scatter plus path, both properties of
  // the correlator and the cable measured in microseconds, so a fixed sample
  // count silently retunes it at every rate while the tracking gate scales.
  // 5.2083 us reproduces the old 640 samples exactly at 122.88 MSPS.
  const double kConfirmTolUs = config_->sync().resync.confirm_tol_us;
  // Resolved by Config: a quarter of the OFDM zero prefix unless configured.
  double sync_tol_samples = config_->sync().resync.sync_tol_samples;
  // DEFAULT LOWERED 1.0 -> 0.1 ppm [user 2026-09-02, "a conservative value,
  // maybe 10x"]. This is the assumed worst-case clock error AFTER tracking, and
  // it sets the cadence: 1.0 ppm gave a 260 ms resync, which AP-53(a) showed is
  // ~75-100x more often than the oscillator requires.
  //
  // What the measurement says, three 300 s captures with the binning artifact
  // fixed: the ADEV minimum sits at tau = 2 s, drift there is 0.46 / 0.58 /
  // 0.49 samples, and at tau = 20 s it is 18.1 / 23.5 / 20.3 against our
  // 32-sample budget. That implies an effective residual rate of 0.002 ppm at
  // 2 s and 0.008 ppm at 20 s -- so the 1.0 ppm assumption was 120-500x
  // pessimistic. 0.1 ppm gives a 2.6 s cadence and still leaves 12-50x margin
  // on the measured rate and ~45x on the 32-sample tolerance itself. The full
  // measured margin would be 0.01 ppm and a 26 s cadence; that is deliberately
  // NOT taken, because 20 s is where the ADEV data ends and beyond it we would
  // be extrapolating.
  double sync_residual_ppm = config_->sync().resync.residual_ppm;
  // Both inputs are validated: a zero or negative ppm makes the cadence
  // quotient infinite, and a config without `ofdm_tx_zero_prefix` gives a zero
  // tolerance and a resync attempt every single frame. Neither should degrade
  // quietly. (SyncConfig rejects NaN and inf and range-checks, AP-54/AP-56.)
  if (!(sync_tol_samples > 0.0) || !std::isfinite(sync_tol_samples)) {
    MLPD_WARN(
        "sync tolerance %.3f is not a positive finite sample count (config "
        "ofdm_tx_zero_prefix = %d?) -- falling back to 32\n",
        sync_tol_samples, config_->prefix());
    sync_tol_samples = 32.0;
  }
  if (!(sync_residual_ppm > 0.0) || !std::isfinite(sync_residual_ppm)) {
    MLPD_WARN("sync residual %.4f ppm is not positive finite -- using 1.0\n",
              sync_residual_ppm);
    sync_residual_ppm = 1.0;
  }
  // ONE derivation, shared with sync_geometry_test so the whole rate ladder is
  // checkable without a radio (AP-56). Everything below reads out of `geom`.
  const houdini::sync::SyncGeometry geom = houdini::sync::computeSyncGeometry(
      config_->rate(), static_cast<long long>(config_->samps_per_slot()),
      static_cast<long long>(config_->samps_per_frame()),
      static_cast<long long>(config_->shape().replicaLen()),
      static_cast<long long>(config_->shape().replicaTail()), kScatterTolUs,
      kConfirmTolUs, sync_tol_samples, sync_residual_ppm);
  const long long kScatterTol = geom.scatter_tol;
  if (geom.scatter_clamped) {
    MLPD_WARN(
        "scatter tolerance %.2f us exceeds what a %lld-sample slot can present "
        "while leaving a usable accept window; clamped to %lld samples "
        "(%.2f us)\n",
        kScatterTolUs, static_cast<long long>(config_->samps_per_slot()),
        geom.slot_cap, geom.slot_cap / config_->rate() * 1e6);
  }
  // State the resulting geometry at bring-up. This is the number that decides
  // whether the UE ever LOOKS for the beacon, and a zero here is the silent
  // failure, so it must be visible without the reader computing it.
  if (!config_->is_houdini()) {
    // The slice geometry drives the targeted resync, which only Houdini runs.
  } else if (!geom.usable) {
    MLPD_ERROR(
        "beacon accept window is %lld samples: the UE will never attempt a "
        "resync and will fly open loop with no telemetry. Lower "
        "sync.resync.scatter_tol_us.\n",
        geom.accept_window);
  } else {
    MLPD_INFO(
        "Beacon accept window %lld samples of a %zu-sample slot (%.1f%%), "
        "scatter tol %lld samples = %.2f us, confirm tol %lld samples\n",
        geom.accept_window, config_->samps_per_slot(),
        100.0 * geom.accept_window_frac, kScatterTol,
        kScatterTol / config_->rate() * 1e6, geom.confirm_tol);
  }

  const size_t beacon_detect_window = config_->is_houdini()
      ? config_->samps_per_frame()
      : static_cast<size_t>(static_cast<float>(config_->samps_per_slot()) *
                            kBeaconDetectWindowScaler);
  size_t sync_count = 0;
  constexpr size_t kTargetSyncCount = 2;
  assert(config_->samps_per_frame() >= beacon_detect_window);
  // Houdini acquisition anchor, set by the stamp-based confirm loop below and
  // consumed by the main loop (counted-sample alignment cannot survive
  // recvHoudini's drain, so the anchor is pure timestamp arithmetic).
  long long houdini_anchor = 0;
  bool houdini_anchored = false;
  // Seeded by the acquisition confirm (AP-31b bootstrap); nominal until then.
  double houdini_boot_period = static_cast<double>(config_->samps_per_frame());
  if (config_->is_houdini()) {
    houdini_anchored = houdiniAcquireAnchor(
        tid, beacon_detect_window, geom, houdini_anchor, &houdini_boot_period);
    if (!houdini_anchored && config_->running()) {
      throw std::runtime_error("beacon acquisition: no confirmed lock");
    }
  } else {
    while ((sync_count < kTargetSyncCount) && config_->running()) {
      const ssize_t sync_index = clientSyncBeacon(tid, beacon_detect_window);
      if (sync_index >= 0) {
        const ssize_t adjust = sync_index - config_->shape().expectedEndOffset();
        const size_t alignment_samples =
            config_->samps_per_frame() - beacon_detect_window;
        MLPD_INFO(
            "clientSyncTxRx [%d]: Beacon detected sync_index: %ld, rx sample "
            "offset: %ld, window %zu, samples in frame %zu, alignment removal "
            "%zu\n",
            tid, sync_index, adjust, beacon_detect_window,
            config_->samps_per_frame(), alignment_samples);

        //By definition alignment_samples + adjust must be > 0;
        if (static_cast<ssize_t>(alignment_samples) + adjust < 0) {
          throw std::runtime_error("Unexpected alignment");
        }
        clientAdjustRx(tid, alignment_samples + adjust);
        sync_count++;
      } else if (config_->running()) {
        MLPD_WARN(
            "clientSyncTxRx [%d]: Beacon could not be detected sync_index: "
            "%ld\n",
            tid, sync_index);
        throw std::runtime_error("rx sample offset is less than 0");
      }
    }
  }

  // Main client read/write loop.
  size_t frame_id = 0;
  size_t buffer_offset = 0;
  //sync on the first beacon after initial detection
  // AP-52 [user]: the escalation net was tuned against a grid that walked out
  // of the gate in ~1 s. Steered, it holds for MINUTES, so these numbers are
  // now absurdly conservative -- but the net stays, it is only retuned, and
  // retuning it wants an A/B on live silicon rather than a guess here. Knobs,
  // shipped at the values that were gated, so the default build is unchanged
  // and the sweep costs no rebuild.
  // The resync bar: the configured policy (sync.detector.corr_scale, relaxed
  // by one per retry) for a single client. With several clients the legacy
  // per-client array keeps its say, because the policy holds one value.
  const auto resyncScale = [this, tid](size_t retry) -> float {
    if (config_->num_cl_sdrs() > 1) {
      return config_->corr_scale(tid) + static_cast<float>(retry);
    }
    return static_cast<float>(
        config_->sync().detector.bar.relaxed(static_cast<int>(retry)));
  };
  size_t cfo_log_cnt = 0;  // throttles the beacon-CFO line
  const size_t kCfoLogEvery =
      static_cast<size_t>(config_->sync().cfo.log_every);
  // Liveness accept/reject half-width. Shipped ON THE WIRE so the panel draws
  // the band it actually illustrates rather than a hardcoded copy (AP-31
  // proposes retuning this, after which a page-side constant would silently lie).
  // AP-40: the scatter tolerance is a PHYSICAL quantity -- detector scatter,
  // cable and RF-chain delay -- so it belongs in TIME, not samples. As a sample
  // constant its physical meaning shrank every rung up the rate ladder (1024
  // samples is 8.33 us at 122.88 MSPS but 2.08 us at 491.52), tightening the
  // gate for no physical reason and making the loop twitchier at high rates.
  // The default is the 8.33 us that 1024 samples meant at the rate this was
  // tuned on, so behaviour here is unchanged and only the scaling is fixed.
  //
  // Note what does NOT move with it: the correlator run-up below is a property
  // of the 128-tap gold sequence and is genuinely a sample count, and the
  // resync PERIOD is already rate-invariant because the frame itself is defined
  // in samples (30 slots x 4096). Only this tolerance was wrong.
  // Single place that knows the wire's fixed fields, so no call site can forget
  // the geometry the page needs to convert to ppm.
  // WEAK is the only branch that does not clear `resync`, so it repeats at the
  // in-window ATTEMPT rate rather than once per resync period. Unthrottled it
  // evicts the whole LOCKED trace from the page's 240-deep history within
  // seconds of a weak beacon, destroying the context that makes it diagnosable.
  // Emit on the transition, then at most once a second.
  uint32_t last_emit_state = 0;
  long long weak_emit_ns = 0;
  auto emitSync = [&](uint32_t st, long long rs, double cfo_hz, double snr_db,
                      long long shift,
                      double cfo_beacon_hz =
                          std::numeric_limits<double>::quiet_NaN()) {
    if (st == kSyncWeak) {
      const long long nowns =
          std::chrono::duration_cast<std::chrono::nanoseconds>(
              std::chrono::steady_clock::now().time_since_epoch())
              .count();
      if (last_emit_state == kSyncWeak &&
          (nowns - weak_emit_ns) < 1000000000LL) {
        return;
      }
      weak_emit_ns = nowns;
    }
    last_emit_state = st;
    sendSyncTelemetry(frame_id, tid, st, rs, cfo_hz, snr_db, shift,
                      static_cast<uint32_t>(config_->samps_per_frame()),
                      static_cast<float>(config_->freq()),
                      static_cast<uint32_t>(kScatterTol), cfo_beacon_hz);
  };
  const double kGridAlpha = config_->sync().tracker.alpha;
  const double kGridBeta = config_->sync().tracker.beta;
  // Per-update slew limit on the rate estimate, distinct from the absolute
  // plausibility band further down: see the note at the update site for why
  // that band alone cannot reject an outlier.
  const double kGridStepPpm = config_->sync().tracker.step_ppm;
  // How often to LOOK at the beacon, derived rather than hardcoded (see
  // sync_geometry.h for the derivation and the test that covers it).
  //
  // The old form was `1e9 / (max_cfo_ppb * samps_per_frame)` with max_cfo = 100
  // ppb "For Iris", which decodes as "resync once the drift reaches ONE
  // sample". Both halves were wrong for this hardware: the tolerance is not one
  // sample but the slot's zero padding, and 100 ppb is neither our clock
  // (8520 ppb measured) nor a crystal spec (a +-25 ppm pair is 50000 ppb). The
  // real tolerance is the timing slack built into the slot: 48 symbols x (fft
  // 64 + cp 16) + prefix 128 + postfix 128 = 4096 = a slot, so the burst may
  // shift +-ofdm_tx_zero_prefix samples before OFDM content crosses the slot
  // boundary. A quarter of that is budgeted to inter-observation drift.
  //
  // The rate error is the RESIDUAL after tracking, not the raw offset:
  // measured 0.036 ppm against a raw 8.52 ppm. What bounds this cadence is not
  // accuracy at all, it is how long we are willing to not notice the beacon is
  // gone, and (on the read side) that each radioRx costs ~855 us fixed.
  //
  // TIMED, NOT COUNTED. The model is expressed in REAL frames while `frame_id`
  // counts LOOP ITERATIONS, and the loop runs slower than real time (measured
  // 412-746 iter/s against 1000 frames/s), so triggering on the counter
  // stretched the interval by that ratio and the grid drifted 1.3-2.4x further
  // than the tolerance it was derived from. Iris/UHD keeps its own frame-count
  // cadence: everything above is Houdini clock physics, and applying it there
  // unconditionally had moved those backends from 81 to 260 with nothing
  // measured about them to justify it.
  const double resync_interval_s = geom.resync_interval_s;
  const size_t resync_period = static_cast<size_t>(geom.resync_period_iters);
  // THE RESYNC STATE MACHINE (sync/resync_policy.h): when to look, the miss
  // budget, the exhausted episode, the off-grid hold and the escalation
  // triggers, with every transition covered by resync_policy_test. The loop
  // below reads its verdicts; nothing here counts anything itself.
  houdini::sync::ResyncPolicyConfig policy_cfg;
  policy_cfg.enabled = (config_->frame_mode() == "continuous_resync");
  policy_cfg.timed = config_->is_houdini();
  policy_cfg.interval_s = resync_interval_s;
  policy_cfg.period_frames = resync_period;
  policy_cfg.retry_max = static_cast<size_t>(config_->sync().resync.retry_max);
  policy_cfg.escalate_episodes = static_cast<size_t>(config_->sync().resync.escalate_episodes);
  policy_cfg.hold_offgrid = static_cast<size_t>(config_->sync().resync.hold_offgrid);
  houdini::sync::ResyncPolicy policy(policy_cfg, frame_id, std::chrono::steady_clock::now());
  if (config_->running() == true) {
    MLPD_INFO(
        "Start main client txrx loop... tid=%d with resync every %.1f ms "
        "(= %.0f real frames; Iris/UHD path uses %zu iterations), "
        "grid tracker alpha %.3f beta %.3f (0/0 = fixed-period grid), step "
        "limit %.3f ppm, bootstrap period %.4f (%+.4f samp/frame vs nominal); "
        "cadence derived from tol %.1f samples / %.3f ppm\n",
        tid, resync_interval_s * 1e3, std::floor(geom.resync_frames),
        resync_period,
        kGridAlpha, kGridBeta, kGridStepPpm, houdini_boot_period,
        houdini_boot_period - static_cast<double>(config_->samps_per_frame()),
        sync_tol_samples, sync_residual_ppm);
  }
  // AP-31 loop profile. The UE iterates SLOWER than real time -- measured
  // 205-417 iterations/s against 1000 frames/s, and the ratio moves ~2x with
  // HOUDINI_COALESCE_SLOTS, so quote the measurement not a single figure. That
  // is
  // WHY recvHoudini drains and therefore why the read lands at an arbitrary
  // frame phase and the beacon is only in the accept band ~1.4% of the time.
  // The drain is a symptom; this measures where the iteration actually goes so
  // the cause is traced rather than assumed. Four buckets, mean us per
  // iteration, logged every HOUDINI_LOOP_PROFILE iterations (0 = off).
  const size_t loop_profile_every = [] {
    const char* e = getenv("HOUDINI_LOOP_PROFILE");
    return e != nullptr ? static_cast<size_t>(atol(e)) : 0;
  }();
  using profile_clock = std::chrono::steady_clock;
  double prof_rx = 0, prof_sync = 0, prof_tx = 0, prof_slot = 0, prof_all = 0;
  size_t prof_n = 0, prof_sync_searched = 0;
  // Coalesce runs of discarded slots into one read (see the slot loop). ON by
  // default -- 11.4x on the measured iteration -- with HOUDINI_COALESCE_SLOTS=0
  // as the escape hatch back to per-slot reads for A/B.
  const bool coalesce_throwaway = [] {
    const char* e = getenv("HOUDINI_COALESCE_SLOTS");
    return (e == nullptr) || (atoi(e) != 0);
  }();
  std::vector<std::complex<int16_t>> throwaway;
  long long rx_beacon_time(0);
  //Always decreases the requested rx samples
  size_t beacon_adjust = 0;

  // Houdini: recvHoudini drains the FIFO then reads, so the per-frame RX read
  // timestamp (rx_beacon_time) is real-time accurate but at an ARBITRARY frame
  // phase -- the pilot TX time (rx_beacon_time + txTimeDelta) then jitters across
  // the whole frame and never seats in the BS rx_gate. With the boards
  // frequency-locked (shared 10 MHz ref) the frame period IS exactly
  // samps_per_frame, so we ANCHOR a beacon-locked frame start on each successful
  // (re)sync and, at pilot TX, SNAP the current read timestamp to that grid
  // (anchor + k*frame) -- see the clientTxPilots call below. (Iris keeps the raw
  // per-frame read timestamp -- its HW framer delivers frame-locked reads.)
  long long houdini_pilot_ref = houdini_anchor;      // from confirmed acquisition
  bool houdini_pilot_ref_valid = houdini_anchored;
  // AP-31 two-state grid tracker. The UE estimates the BS clock in its OWN
  // sample units as (ref, period) and derives EVERY prediction from it. The
  // old code fixed period at samps_per_frame -- "with the boards
  // frequency-locked the frame period IS exactly samps_per_frame" -- which is
  // true only on a shared reference. MEASURED on internal clocks 2026-09-01:
  // eps = -8.52 ppm, so the BS frame period is 122881.047 UE samples, the
  // anchored grid walks out of the +-kScatterTol gate in about 1.0 s, and the
  // UE re-acquires roughly once a second forever (DEMO_VERIFICATION 8.4).
  //
  // alpha-beta rather than per-detection correction: the arrival jitter is
  // 8-23 samples rms while the per-update drift is ~120 samples, so the SLOPE
  // is what carries information -- snapping to each detection would inject the
  // jitter straight into the TX schedule. ref is re-anchored to the newest
  // observation on every update, so the extrapolation distance stays one
  // update gap (~115 frames) instead of growing without bound from k = 0.
  double houdini_frame_period = houdini_boot_period;  // clamped just below
  // Physically plausible band for the BS frame period seen in UE samples. A
  // +-100 ppm bound is generous against any crystal pair (our measured pair is
  // 8.5 ppm) and exists only to stop a single bad observation from parking the
  // rate somewhere it can never recover from.
  const double kGridMaxPpm = config_->sync().tracker.max_ppm;
  const double kGridTrustPpm = config_->sync().tracker.trust_ppm;
  const double kGridPeriodLo =
      static_cast<double>(config_->samps_per_frame()) * (1.0 - kGridMaxPpm * 1e-6);
  const double kGridPeriodHi =
      static_cast<double>(config_->samps_per_frame()) * (1.0 + kGridMaxPpm * 1e-6);
  // The bootstrap is a wholesale write too, and acquisition has no band of its
  // own: a confirm accepted at small k with a large residual can hand back a
  // wildly implausible rate (resid 640 over k=4 is ~1300 ppm, 150x the real
  // offset). Clamp on the way in so no path installs a period the incremental
  // update would have rejected.
  houdini_frame_period =
      std::min(kGridPeriodHi, std::max(kGridPeriodLo, houdini_frame_period));
  // Count tracker updates so an escalation knows whether its own learned rate
  // is better than a fresh confirm-derived one (below).
  size_t houdini_grid_updates = 0;
  size_t houdini_grid_starved = 0;  // in-gate accepts with kf <= 0 (no update)
  size_t houdini_grid_innov_rej = 0;  // kalman arm: gated by its own innovation
  // AP-41. The estimator is swappable and DEFAULTS TO THE SHIPPED ALPHA-BETA,
  // so a gate run with nothing set exercises exactly the gated path. It
  // computes GAINS only; the state and its arithmetic stay here, so switching
  // arms cannot change the rounding on the arm that is already gated.
  //
  // The offline A/B (grid_tracker_test) predicts, on this bench's own numbers:
  // the two are identical at regular spacing, the kalman is ~2x better on the
  // irregular spacing we actually have (median 179 frames, range 10-831), and
  // the kalman WITHOUT its innovation gate is ~4x WORSE than alpha-beta when an
  // edge-of-gate detection lands inside the +-kScatterTol window, because
  // alpha-beta has the slew limit and a bare kalman has nothing. So the gate is
  // not an optional extra, it is what makes this arm worth running.
  // One owner of the values: the JSON tracker block, with the frame length
  // that turns step_ppm into a per-update limit in samples.
  const houdini::sync::TrackerConfig tracker_cfg(
      config_->sync().tracker, static_cast<double>(config_->samps_per_frame()));
  {
    // THE ACTIVE DETECTOR, ON EVERY RUN. The env overrides each warn when set,
    // so taking the same values BY DEFAULT logged nothing at all and a run's
    // log carried no record of which rule produced its numbers -- in a change
    // whose other half exists because a run's identity was not recorded.
    {
      MLPD_INFO(
          "Beacon detector [%s]: threshold %s, resync pick %s, acquisition pick "
          "%s (first-path back window %d samples, floor %.1f dB); SNR floor "
          "%.1f dB, SNR guard %zu samples; CFO guard %d margin %d\n",
          sync_detector_->backendName(), houdini::sync::name(sync_detector_->form()),
          houdini::sync::name(sync_detector_->pick()),
          houdini::sync::name(sync_detector_->pick()),
          sync_detector_->firstPathWindow(), sync_detector_->firstPathFloorDb(),
          sync_guard_->floorDb(), sync_guard_->guard(),
          config_->sync().cfo.index_guard, cfo_estimator_->margin());
    }
    if (!sync_detector_->backendAppliesConfig()) {
      MLPD_WARN(
          "Detector backend %s returns the first crossing under the power-ratio "
          "form; the configured detector.threshold / pick / first-path knobs "
          "above are NOT applied\n",
          sync_detector_->backendName());
    }
    if (config_->shape().singleCopy()) {
      MLPD_INFO(
          "Beacon replica is a single copy (%s): threshold form forced to "
          "coherence (plain matched filter, no repeat check); beacon end = "
          "detector index + %zu\n",
          config_->beacon_type().c_str(), config_->shape().replicaTail());
    }
    MLPD_INFO(
        "Grid tracker: %s (alpha %.3f beta %.3f step limit %.3f ppm; kalman "
        "R %.3f samp^2, q %.2g, innovation gate %.1f sigma)\n",
        (config_->sync().tracker.type == houdini::sync::TrackerType::kKalman) ? "KALMAN" : "alpha-beta", kGridAlpha, kGridBeta, kGridStepPpm,
        tracker_cfg.meas_var, tracker_cfg.rate_rw, tracker_cfg.innov_gate);
  }
  houdini::sync::GridTracker tracker;
  tracker.reset(tracker_cfg);
  // Frame-start grid point n frames after the tracked reference.
  auto houdiniGridStart = [&](long long n) {
    return houdini_pilot_ref + llround(static_cast<double>(n) *
                                       houdini_frame_period);
  };
  // Frames from the tracked reference to the grid point nearest t.
  auto houdiniGridIndex = [&](long long t) {
    return llround(static_cast<double>(t - houdini_pilot_ref) /
                   houdini_frame_period);
  };
  // Resync hold-off state [user 2026-08-30]: a large offset is applied only
  // after MORE THAN ONE consecutive consistent observation of it; a lone
  // large offset (artifact, scatter) is held, logged, and not applied.
  // AP-18 escalation [user]: give up on resync and return to the full
  // sliding-window acquisition when the anchored grid has plausibly lost the
  // beacon. Triggers: 2 CONSECUTIVE exhausted episodes OR >= 4 SNR-valid
  // detections held without agreeing with each other (incoherent state).
  // Under TARGETED resync an attempt only counts when the grid predicted the
  // full beacon inside the window (~2% of windows), so one exhausted episode
  // means ~100 predicted-position windows in a row failed to detect -- at a
  // healthy SNR that is not chance but a dead or moved beacon; two episodes
  // are pure confirmation (Opus review M4: the old ~4.7%-by-chance figure
  // described the pre-targeting whole-window search). Hold-off itself
  // already covers the beacon-MOVED case; this covers beacon-LOST.
  // How many consecutive off-grid detections before the beacon counts as
  // MOVED. One is scatter (ledger 4.18); the shipped rule is two. Steered, an
  // off-grid detection is far more surprising than it was, so this is the
  // other half of the AP-52 retune and it sweeps with the same A/B.
  const size_t beacon_detect_window_esc = static_cast<size_t>(
      static_cast<float>(config_->samps_per_slot()) *
      kBeaconDetectWindowScaler);
  auto houdiniEscalate = [&](const char* why) {
    MLPD_WARN(
        "Re-sync ESCALATION (%s) at frame %zu: returning to the full "
        "beacon acquisition, tid %d\n",
        why, frame_id, tid);
    long long fresh = 0;
    double fresh_period = houdini_frame_period;
    const long long prev_ref = houdini_pilot_ref;
    if (houdiniAcquireAnchor(tid, beacon_detect_window_esc, geom, fresh,
                             &fresh_period)) {
      // The LARGE schedule move. The tracker's alpha half moves it too, by up
      // to kGridAlpha * kScatterTol per accepted detection, and reports that
      // step on its own LOCKED record. Report the applied step so the panel can
      // mark it against the resid trace.
      // Both anchors are ABSOLUTE sample times k frames apart, so their raw
      // difference is dominated by elapsed time and overflows int32 in ~17.5 s.
      // The schedule step is that difference modulo the frame period, centred.
      // Fold on the TRACKED period, not the nominal one: the two differ by eps
      // per frame, so over the elapsed span the nominal fold accumulates
      // samps_per_frame * elapsed_frames * eps of pure error -- about 1044
      // samples per second of gap at the measured 8.5 ppm, which is larger
      // than every step this field is meant to show.
      long long step = 0;
      if (houdini_pilot_ref_valid) {
        const double per = houdini_frame_period > 0.0
                               ? houdini_frame_period
                               : static_cast<double>(config_->samps_per_frame());
        const double d = static_cast<double>(fresh - prev_ref);
        step = llround(d - per * std::round(d / per));
      }
      emitSync(kSyncEscalating, 0, 0.0, 0.0, step);
      // Escalation means the OFFSET was lost; the two oscillators did not
      // change when that happened. So keep a rate the tracker actually learned
      // (many observations, ~0.1%) and take the confirm's fresh one (1-5%)
      // only while the tracker has learned nothing at all -- which is exactly
      // the case an escalation-first run lands in.
      // Take the confirm's fresh rate when the tracker has learned nothing OR
      // when the two materially disagree. The confirm spans >= kRefineSpan real
      // frames and is good to ~0.04 ppm, comparable to the tracker's own
      // steady-state residual, so a disagreement beyond kGridTrustPpm means the
      // TRACKED value is the suspect one. Without this, a single bad kick is
      // permanent: escalation re-anchors the offset, keeps the bad period, and
      // the grid walks out again forever.
      // Clamp FIRST, then measure the disagreement, so the ppm printed below
      // is reproducible from the two periods on the same line. Computing it
      // from the unclamped value and printing the clamped one made the message
      // internally inconsistent whenever the clamp bit, which is reachable by
      // setting HOUDINI_ACQ_MAX_PPM above HOUDINI_GRID_MAX_PPM.
      fresh_period = std::min(kGridPeriodHi, std::max(kGridPeriodLo, fresh_period));
      const double disagree_ppm =
          std::fabs(fresh_period - houdini_frame_period) /
          static_cast<double>(config_->samps_per_frame()) * 1e6;
      if (houdini_grid_updates == 0 || disagree_ppm > kGridTrustPpm) {
        if (houdini_grid_updates != 0) {
          MLPD_WARN(
              "Re-sync ESCALATION: tracked period %.4f disagrees with the "
              "fresh confirm %.4f by %.3f ppm -- taking the confirm, tid %d\n",
              houdini_frame_period, fresh_period, disagree_ppm, tid);
        }
        houdini_frame_period = fresh_period;
        houdini_grid_updates = 0;
      }
      houdini_pilot_ref = fresh;
      houdini_pilot_ref_valid = true;
      // A re-anchor is a wholesale write of BOTH states, so the filter's
      // covariance must be re-inflated with them. Without this the kalman keeps
      // the confidence it earned before the anchor was lost and under-weights
      // every observation of the new one.
      tracker.reset(tracker_cfg);
      // The counter must be wiped WITH the state it describes. It is the
      // "tracker has learned nothing" test the branch above uses to decide
      // whether to trust a fresh confirm; leaving it set after resetting the
      // filter makes the NEXT escalation refuse a good confirm on the strength
      // of learning that no longer exists.
      houdini_grid_updates = 0;
      MLPD_INFO("Re-sync ESCALATION: re-anchored at %lld, tid %d\n", fresh,
                tid);
    } else if (config_->running()) {
      MLPD_WARN(
          "Re-sync ESCALATION: re-acquisition did not confirm; keeping the "
          "previous anchor, tid %d\n",
          tid);
      emitSync(kSyncReanchorFailed, 0, 0.0, 0.0, 0);
    }
    policy.reset(frame_id, std::chrono::steady_clock::now());
  };

  while (config_->running() == true) {
    if (config_->max_frame() > 0 && frame_id >= config_->max_frame()) {
      config_->running(false);
      break;
    }
    //Slot 0 / Beacon...
    const auto prof_t0 = loop_profile_every > 0 ? profile_clock::now() : profile_clock::time_point{};
    const int request_samples = samples_per_slot - beacon_adjust;
    const int rx_status = client_radio_set_->radioRx(
        tid, rxbuff.data(), request_samples, rx_beacon_time);
    beacon_adjust = 0;
    const auto prof_t1 = loop_profile_every > 0 ? profile_clock::now() : profile_clock::time_point{};
    if (rx_status < 0) {
      MLPD_ERROR("Rx status reporting error %d, exiting\n", rx_status);
      config_->running(false);
      break;
    }
    if (config_->ul_data_slot_present() == true && !config_->is_houdini()) {
      // Notify new frame (file-based UL data path; Houdini uses the self-contained
      // continuous P+U burst in clientTxPilots instead).
      this->notifyPacket(kClient, frame_id + this->txFrameDelta_, 0, tid,
                         tx_buffer_size);
    }

    // The clock is read only where the cadence is timed (Houdini).
    if (policy.due(frame_id, config_->is_houdini() ? std::chrono::steady_clock::now()
                                                   : std::chrono::steady_clock::time_point{})) {
      MLPD_TRACE("Enable resyncing at frame %zu\n", frame_id);
    }
    if (policy.looking()) {
      ssize_t sync_index = -1;
      bool resync_attempted = true;
      houdini::sync::Detection resync_det;  // the targeted search's evidence
      if (config_->is_houdini() && houdini_pilot_ref_valid) {
        // TARGETED liveness check: the anchored grid predicts exactly where
        // the beacon END lands in this (drained, random-phase) window, so
        // only attempt when it is inside (~1.4% of frames at kLead=1280 -- the others count
        // as NO attempt, so an exhausted episode really means "the beacon
        // was absent at its predicted spot ~100 times"), and search only
        // that neighborhood. A whole-window earliest-crossing search kept
        // losing the race to ~11 dB detections at the window edge (best
        // reading: the beacon itself straddling the edge with partial core
        // energy -- same-board TX coupling measured cold, ledger 4.40), so
        // an attempt only counts when the full beacon is predicted inside.
        // PREDICTIVE (AP-31b): extrapolate the grid by the ESTIMATED period to
        // the first beacon due at or after this window's start. The modulo
        // form this replaces folded on samps_per_frame, which silently assumed
        // period == nominal -- on free-running clocks that walks off the beacon
        // within a second and the tracker then never gets another observation.
        const long long beacon_end =
            static_cast<long long>(config_->shape().expectedEndOffset());
        const double n_due =
            std::ceil(static_cast<double>(rx_beacon_time - houdini_pilot_ref -
                                          beacon_end) /
                      houdini_frame_period);
        const long long off =
            houdiniGridStart(static_cast<long long>(n_due)) + beacon_end -
            rx_beacon_time;
        // The slice must be able to PRESENT every residual the liveness
        // gate can accept (+-kScatterTol = 1024) plus ~256 samples of
        // gold context for the correlator; a 700-sample lead left
        // residuals in [-1024,-444] undetectable (Opus review M3).
        // DERIVED from the tolerance rather than hand-tuned: the slice must be
        // able to PRESENT every residual the gate can accept, so a retune of
        // kScatterTol (AP-31) has to move these with it, or the panel draws a
        // band wider than the detector can ever fill.
        // A sample count by nature: the matched filter needs 2 gold lengths of
        // run-up regardless of how fast we sample, so this one does NOT scale
        // with the rate the way kScatterTol does.
        const long long kLead = geom.lead;
        const long long kTail = geom.tail;
        if (off >= kLead && off + kTail <= request_samples) {
          auto* base = reinterpret_cast<std::complex<int16_t>*>(
              rxbuff.at(kSyncDetectChannel));
          const ssize_t s0 = static_cast<ssize_t>(off - kLead);
          const ssize_t slice_len = kLead + kTail;
          prof_sync_searched++;
          // TARGETED slice: lead+tail is far shorter than the beacon copy
          // spacing of one full frame (122880 samples), so the strongest
          // crossing is unambiguous and the earliest one is the STS preamble.
          // See CommsLib::BeaconPick.
          //
          // HOUDINI_BEACON_PICK=first restores the pre-2026-09-02 rule ON THE
          // SAME BINARY. That is not a compatibility escape hatch, it is what
          // makes the fix gateable: PRE and POST on one build removes the
          // "different binary, different day" confound that a two-build gate
          // carries, and it is the only way this bench can show the OLD rule
          // failing at all. The bench runs below the level where kFirstCrossing
          // breaks, but the threshold test is `corr_scale * peak > energy`, so
          // raising corr_scale is arithmetically identical to raising the
          // received level -- and corr_scale is already a config knob. Sweep it
          // with this set to `first` and the false lock appears on silicon.
          const ssize_t idx = this->syncSearch(
              base + s0, static_cast<size_t>(slice_len),
              resyncScale(policy.retries()), sync_detector_->pick(), &resync_det);
          if (idx >= 0) sync_index = s0 + idx;
        } else {
          resync_attempted = false;  // beacon not due in this window
        }
      } else {
        // UNTARGETED re-sync: the whole read is searched, so it CAN hold more
        // than one beacon copy and kTargetedArgmax's precondition does not hold.
        // Left on kFirstCrossing, which is the shipped behaviour and is the
        // path Iris/UHD hardware takes -- hardware this bench cannot exercise.
        // It is not a good rule (the earliest crossing returned a sidelobe on 58
        // of 62 detections, and the STS plateau reaches it too), but changing it
        // would alter untestable behaviour to fix a path the Houdini link does
        // not use once its grid is valid. Tracked as its own item.
        sync_index = this->syncSearch(
            reinterpret_cast<std::complex<int16_t>*>(
                rxbuff.at(kSyncDetectChannel)),
            request_samples, resyncScale(policy.retries()),
            houdini::sync::PickRule::kFirstCrossing);
      }
      if (sync_index >= 0 && config_->is_houdini() &&
          houdini_pilot_ref_valid) {
        // Liveness verdict on the targeted detection: SNR floor first, then
        // the grid residual (alive within scatter / moved beyond it).
        const double snr = sync_guard_->snrDb(
            reinterpret_cast<std::complex<int16_t>*>(
                rxbuff.at(kSyncDetectChannel)),
            static_cast<size_t>(request_samples), sync_index);
        MLPD_INFO("Re-sync frame %zu: detection idx %ld snr %.1f dB, tid %d\n",
                  frame_id, sync_index, snr, tid);
        // Ledger 4.42 instrument: dump this window + the verdict inputs so the
        // offline analyzer can place the TRUE core by exact-waveform
        // correlation and recompute the SNR without the detector-index bias.
        const char* rwdir = getenv("HOUDINI_DUMP_RESYNC_WIN");
        if (rwdir != nullptr) {
          static std::atomic<int> rwn{0};
          const int wk = rwn.fetch_add(1);
          if (wk < 6) {
            char pb[512];
            snprintf(pb, sizeof(pb), "%s/resyncwin_%02d.bin", rwdir, wk);
            FILE* fb = fopen(pb, "wb");
            if (fb != nullptr) {
              fwrite(rxbuff.at(kSyncDetectChannel), sizeof(int16_t),
                     static_cast<size_t>(request_samples) * 2, fb);
              fclose(fb);
            }
            snprintf(pb, sizeof(pb), "%s/resyncwin_%02d.txt", rwdir, wk);
            FILE* fg = fopen(pb, "w");
            if (fg != nullptr) {
              // The settings this window was searched and judged under, so a
              // replay (golden_window_test) can assert it runs the same ones.
              fprintf(fg,
                      "n %d\nsync_index %zd\nsnr %.2f\nframe %zu\n"
                      "rx_beacon_time %lld\npilot_ref %lld\nbeacon_end %lld\n"
                      "corr_scale %.6g\nthresh %s\npick %s\nfirst_path_window %d\n"
                      "first_path_floor_db %.3f\nsnr_floor_db %.3f\n"
                      "snr_guard %zu\nreplica_tail %zu\nbeacon_type %s\n"
                      "statistic %.6g\nbar %.6g\n",
                      request_samples, sync_index, snr, frame_id,
                      rx_beacon_time, houdini_pilot_ref,
                      static_cast<long long>(config_->shape().expectedEndOffset()),
                      static_cast<double>(resyncScale(policy.retries())),
                      houdini::sync::name(sync_detector_->form()),
                      houdini::sync::name(sync_detector_->pick()),
                      sync_detector_->firstPathWindow(),
                      sync_detector_->firstPathFloorDb(), sync_guard_->floorDb(),
                      sync_guard_->guard(), sync_detector_->replicaTail(),
                      config_->beacon_type().c_str(), resync_det.statistic,
                      resync_det.bar);
              fclose(fg);
            }
          }
        }
        if (!sync_guard_->accept(snr)) {
          // Report it: without this a WEAK beacon is indistinguishable from no
          // beacon on the panel, and they call for different operator actions.
          emitSync(kSyncWeak, 0, 0.0, snr, 0);
          sync_index = -1;  // fall through to the miss path below
        } else {
          const long long abs_end = rx_beacon_time + sync_index;
          const long long beacon_end =
              static_cast<long long>(config_->shape().expectedEndOffset());
          const long long kf = houdiniGridIndex(abs_end - beacon_end);
          const long long resid =
              abs_end - (houdiniGridStart(kf) + beacon_end);
          // Beacon CFO on the SAME validated detection (~600 flops at ~9/s).
          // Reported beside resid because they are one oscillator error seen
          // two ways: the SLOPE of resid is the fractional rate error, and
          // cfo/carrier is that same fraction read off the carrier. They must
          // agree -- a disagreement means one instrument is wrong, which is the
          // whole reason both are measured (BACKLOG AP-30/AP-31).
          // AP-50 stage 1: give estimateCFO the TRACKER'S predicted beacon
          // end, not find_beacon's detected one, plus a small positive guard.
          //
          // The estimator's whole measured bias is one-sided window
          // misalignment: delta < 0 slides its first gold window back into the
          // STS and the STS x gold cross-correlation injects phase (+226 to
          // +2817 Hz modelled at true CFO = 0), while delta > 0 slides the
          // second window into the beacon's trailing ZEROS, where the terms
          // multiply by zero and cost only a little correlation energy.
          // find_beacon uses an earliest-crossing rule, so it biases early by
          // construction -- into the toxic side -- which is why every
          // disagreement measured on silicon was positive.
          //
          // pred_end = sync_index - resid by definition of resid, so the
          // tracker's estimate is free here. It replaces the detector's bias
          // and scatter with the tracker's own error (resid sd 0.63-0.70, max 3
          // measured), and the guard then biases what is left onto the harmless
          // side. Modelled: mean error +1487.5 Hz -> -31.8 Hz.
          const long long cfo_guard =
              static_cast<long long>(config_->sync().cfo.index_guard);
          long long cfo_index = sync_index - resid + cfo_guard;
          // BOTH bounds. An out-of-range index would otherwise cost the
          // estimate entirely (estimateCFO now returns NaN rather than a
          // fabricated zero), and the fallback below keeps a usable reading in
          // the very line whose job is to keep the disagreement visible. resid
          // is only bounded by half a frame at this
          // point (the +-kScatterTol gate is below), so the low side is
          // reachable on any off-grid detection: exactly when the reading
          // matters most.
          if (cfo_index > request_samples ||
              cfo_index < static_cast<long long>(config_->beacon_size())) {
            cfo_index = sync_index;  // fall back to the detector's own index
          }
          const float cfo_norm =
              cfo_estimator_->estimate(reinterpret_cast<std::complex<int16_t>*>(
                              rxbuff.at(kSyncDetectChannel)),
                          static_cast<size_t>(request_samples),
                          static_cast<int>(cfo_index));
          const double cfo_hz =
              static_cast<double>(cfo_norm) * config_->rate();
          const double cfo_ppm = (config_->freq() > 0.0)
                                     ? (cfo_hz / config_->freq()) * 1e6
                                     : 0.0;
          // CFO FROM THE TRACKED CLOCK (AP-31d). The sample clock and the RFDC
          // NCO are both derived from the one LMK PLL1 reference, so the
          // fractional error the grid tracker measures off TIMING is the same
          // fraction the carrier carries: eps = samps_per_frame/period - 1, and
          // the offset is eps * freq. That is the estimate to correct with:
          //   - the beacon phase estimator is precise but NOT accurate. On a
          //     link where the arrival ramp measures eps = 0 exactly it reads
          //     +353 Hz, and across the four campaign legs its error vs the
          //     timing truth ranged +280 to +1753 Hz with no consistent slope,
          //     so it is a configuration-dependent bias, not a scale factor to
          //     divide out (DEMO_VERIFICATION 8.6).
          //   - the timing channel agrees with a completely independent, RF-free
          //     hardware-clock ratio to <= 0.05 ppm, and the tracked residual is
          //     0.036 ppm = ~18 Hz at 500 MHz. SCOPE, ADDED 2026-09-02: that
          //     agreement was established on legs where eps was 7-8 ppm and it
          //     does NOT transfer to a sub-ppm pair. `hwtime_rate_probe` reads
          //     two host-referenced rates that each wander ~3 ppm between runs,
          //     so differencing them leaves ~+-0.26 ppm -- measured, two
          //     consecutive runs 0.87 ppm apart with a sign flip on a 0.25 ppm
          //     pair (DEMO_VERIFICATION 8.91). Do not cite it below ~1 ppm.
          // Both are logged so the disagreement stays visible rather than
          // becoming folklore. AP-34(b) is FIXED as of 2026-09-02: the ladder's
          // stage 3 now agrees with the timing channel to 0.02 ppm over four
          // paired runs, so the estimator's own bias is measurable rather than
          // merely known about (8.100, 8s).
          const double eps_tracked =
              (houdini_frame_period > 0.0)
                  ? (static_cast<double>(config_->samps_per_frame()) /
                         houdini_frame_period - 1.0)
                  : 0.0;
          const double cfo_tracked_hz = eps_tracked * config_->freq();
          if ((cfo_log_cnt++ % kCfoLogEvery) == 0) {
            MLPD_INFO(
                "Beacon CFO frame %zu: tracked %+.1f Hz (%+.4f ppm) | beacon "
                "%+.1f Hz (%+.3f ppm), delta %+.1f Hz, resid %+lld, "
                "snr %.1f dB, tid %d\n",
                frame_id, cfo_tracked_hz, eps_tracked * 1e6, cfo_hz, cfo_ppm,
                cfo_hz - cfo_tracked_hz, resid, snr, tid);
          }
          // Liveness model, not micro-correction: with locked clocks + MTS
          // the drift is ~0, while INDEPENDENT detections of the same beacon
          // scatter by +-hundreds of samples (the earliest-crossing/STS
          // class, ledger 4.18) -- a tight drift gate rejected every real
          // hit and the escalation churned ~every 1.7 s. A SNR-passing hit
          // within the scatter tolerance = beacon ALIVE on the anchored
          // grid, touch nothing; two consecutive hits beyond it = beacon
          // MOVED -> escalate straight to re-acquisition, whose confirm
          // loop is immune to the common detector bias.
          if (std::llabs(resid) <= kScatterTol) {
            long long applied_shift = 0;  // reported on the LOCKED record
            // Accepted observation: advance the tracked grid. The gate keeps
            // its old role as the alive/moved verdict AND becomes the tracker's
            // outlier reject -- a rejected detection updates nothing rather
            // than levering the rate estimate (AP-31).
            if (kf > 0 && tracker.update(kf, static_cast<double>(resid))) {
              // This IS a schedule move, up to kGridAlpha * kScatterTol = 512
              // samples in one step, and it used to be reported as shift = 0
              // while only the escalation's move was shown. Carry it out to
              // emitSync so the panel sees every move the UE makes.
              applied_shift = llround(tracker.shift());
              houdini_pilot_ref = houdiniGridStart(kf) + applied_shift;
              // The absolute band below is a PLAUSIBILITY bound and cannot
              // serve as the outlier reject on its own: it admits a single
              // edge-of-gate detection kicking the rate ~3.2 ppm at the shipped
              // settings, a third of the real 8.5 ppm offset and ~30x looser
              // than the kick its own note describes. Each arm bounds that its
              // own way -- alpha-beta with a per-update slew limit
              // (HOUDINI_GRID_STEP_PPM, 14x the measured 0.036 ppm residual, so
              // a normal ~0.003 ppm update is untouched), the kalman with an
              // innovation gate scaled by what it currently knows.
              houdini_frame_period += tracker.deltaPeriod();
              houdini_frame_period =
                  std::min(kGridPeriodHi, std::max(kGridPeriodLo,
                                                   houdini_frame_period));
              houdini_grid_updates++;
            } else if (kf > 0) {
              // Only the kalman arm reaches here: its innovation gate rejected
              // the observation. The detection is still ALIVE on the grid --
              // the outer gate said so, and every counter below still clears --
              // it simply does not inform the state.
              houdini_grid_innov_rej++;
              // Braces load-bearing: MLPD_WARN is three statements with no
              // do/while wrapper, so unbraced the throttle governs only the
              // header (the flood recorder_worker.cc already documented).
              if (houdini_grid_innov_rej == 1 ||
                  houdini_grid_innov_rej % 50 == 0) {
                MLPD_WARN(
                    "Re-sync frame %zu: innovation %.1f sigma exceeds the %.1f "
                    "sigma gate, tracker not updated (%zu so far, sigma_t %.2f "
                    "samp, sigma_rate %.4f samp/frame), tid %d\n",
                    frame_id, tracker.innovSigmas(), tracker_cfg.innov_gate,
                    houdini_grid_innov_rej, tracker.timeSigma(),
                    tracker.rateSigma(), tid);
              }
            } else {
              // No usable baseline, so the tracker learns nothing from this
              // detection -- while every counter below still clears and the
              // panel still reads LOCKED. A tracker starved this way is
              // otherwise invisible, so count it and say so. The two ways to
              // get here are different faults and are named separately:
              // kf == 0 is a detection inside the anchor's own frame, kf < 0 is
              // one landing BEFORE the tracked reference, which is reachable
              // just after a re-anchor and means something quite else.
              houdini_grid_starved++;
              // Braces are load-bearing: MLPD_WARN expands to three statements
              // with no do/while wrapper, so unbraced only the header is
              // throttled and the body prints on every pass (the flood this
              // repo already hit at recorder_worker.cc's HOUDINI_CSI_R_DEBUG).
              if (houdini_grid_starved == 1 ||
                  houdini_grid_starved % 100 == 0) {
                MLPD_WARN(
                    "Re-sync frame %zu: detection at kf = %lld (%s), so the "
                    "tracker did NOT update -- %zu such accepts so far against "
                    "%zu updates, tid %d\n",
                    frame_id, kf,
                    kf == 0 ? "inside the anchor's own frame"
                            : "BEFORE the tracked reference",
                    houdini_grid_starved, houdini_grid_updates, tid);
              }
            }
            policy.onAlive();
            MLPD_INFO(
                "Re-sync frame %zu: beacon alive on the anchored grid "
                "(resid %+lld within scatter, snr %.1f dB), tid %d\n",
                frame_id, resid, snr, tid);
            // The panel gets the TRACKED estimate: same units and field, the
            // accurate source (see the note above).
            emitSync(kSyncLocked, resid, cfo_tracked_hz, snr, applied_shift,
                     cfo_hz);
          } else {
            MLPD_WARN(
                "Re-sync frame %zu: off-grid detection %+lld (snr %.1f dB, "
                "pending %d) -- beacon possibly moved, tid %d\n",
                frame_id, resid, snr, policy.holdPending() ? 1 : 0, tid);
            emitSync(kSyncHold, resid, cfo_tracked_hz, snr, 0, cfo_hz);
            // One off-grid detection is held (scatter); hold_offgrid
            // consecutive ones mean the beacon moved (AP-52).
            if (policy.onOffGrid() == houdini::sync::ResyncAction::kEscalate) {
              houdiniEscalate("beacon moved");
              continue;  // rx_beacon_time is pre-hunt; restart the frame loop
            }
          }
        }
      } else if (sync_index >= 0) {
        const int new_rx_offset =
            static_cast<int>(sync_index - config_->shape().expectedEndOffset());
        //Adjust tx time
        rx_beacon_time += new_rx_offset;
        if (config_->is_houdini() && !houdini_pilot_ref_valid) {
          houdini_pilot_ref = rx_beacon_time;
          houdini_pilot_ref_valid = true;
        }
        policy.onAccept();
        MLPD_INFO(
            "Re-syncing success at frame %zu with offset: %d, after %zu tries, "
            "index: %ld, tid %d\n",
            frame_id, new_rx_offset, policy.retries() + 1, sync_index, tid);

        //Offset Alignment logic
        if (new_rx_offset < 0) {
          beacon_adjust = (-1 * new_rx_offset);
        } else if (new_rx_offset > 0) {
          const size_t discard_samples = new_rx_offset;
          //throw away samples to get back in alignment, could combine with the next beacon but would need bigger buffers
          clientAdjustRx(tid, discard_samples);
        }
      }
      if (sync_index < 0 && resync_attempted) {
        const houdini::sync::ResyncAction act =
            policy.onMiss(config_->is_houdini() && houdini_pilot_ref_valid);
        if (act == houdini::sync::ResyncAction::kExhausted ||
            act == houdini::sync::ResyncAction::kEscalate) {
          // Under recvHoudini's drain the per-frame slot-0 window carries
          // the beacon only a few percent of the time, so long miss runs
          // are NORMAL. The anchored grid keeps the pilots seated (drift
          // measured ~0), so log and retry next period instead of killing
          // the run; consecutive exhausted episodes escalate.
          MLPD_WARN(
              "Re-sync: %zu misses this period for client %d (successes "
              "%zu); anchored grid keeps flying, retrying next period "
              "(exhausted streak %zu)\n",
              policy.config().retry_max, tid, policy.successes(),
              policy.exhaustedStreak());
          if (act == houdini::sync::ResyncAction::kEscalate) {
            houdiniEscalate("episodes exhausted");
            continue;  // rx_beacon_time is pre-hunt; restart the frame loop
          }
        } else if (act == houdini::sync::ResyncAction::kStop) {
          // Iris/UHD path (on Houdini the anchor is always valid here:
          // acquisition either confirms or throws).
          MLPD_WARN(
              "Exceeded resync retry limit (%zu) for client %d reached "
              "after %zu resync successes at frame: %zu.  Stopping!\n",
              policy.config().retry_max, tid, policy.successes(), frame_id);
          config_->running(false);
          break;
        }
      }
    }
    const auto prof_t2 = loop_profile_every > 0 ? profile_clock::now() : profile_clock::time_point{};
    // schedule all TX slot
    // config_->tx_advance() needs calibration based on SDR model and sampling rate
    // Houdini always uses the continuous P(+U) burst below (clientTxPilots now
    // transmits the uplink-data slot too); the file-based clientTxData path is for
    // Iris/UHD.
    if (config_->ul_data_slot_present() == true && !config_->is_houdini()) {
      int tx_return = 0;
      while (tx_return >= 0) {
        tx_return = this->clientTxData(tid, frame_id, rx_beacon_time);
      }
    } else {
      if (config_->cl_pilot_slots().at(tid).size() > 0) {
        // Houdini: the per-frame read timestamp (rx_beacon_time) is real-time
        // accurate but at an ARBITRARY frame phase (recvHoudini drains then reads),
        // so SNAP it to the beacon-locked frame grid (anchor + k*frame) -- keeps
        // real-time tracking (the loop rate != real-time because of the drain) AND
        // a constant frame phase, so the pilot lands at the same BS-frame position.
        long long pilot_base = rx_beacon_time;
        if (config_->is_houdini() && houdini_pilot_ref_valid) {
          // Snap to the TRACKED grid, not a nominal-period one: on free-running
          // clocks a nominal snap drifts out of the BS rx_gate at the same
          // 1.047 samples per frame the beacon does.
          pilot_base = houdiniGridStart(houdiniGridIndex(rx_beacon_time));
        }
        this->clientTxPilots(tid, pilot_base + txTimeDelta_,
                             houdini_frame_period);
      }
    }  // end if config_->ul_data_slot_present()
    const auto prof_t3 = loop_profile_every > 0 ? profile_clock::now() : profile_clock::time_point{};

    //Beacon + Tx Complete, process the rest of the slots
    for (size_t slot_id = 1; slot_id < config_->slot_per_frame(); slot_id++) {
      int rx_data_status;
      long long rx_data_time;
      if (config_->isDlData(tid, slot_id)) {
        // Set buffer status(es) to full; fail if full already
        for (size_t ch = 0; ch < config_->cl_sdr_ch(); ++ch) {
          const int bit = 1 << (buffer_offset + ch) % sizeof(std::atomic_int);
          const int offs = (buffer_offset + ch) / sizeof(std::atomic_int);
          const int old =
              std::atomic_fetch_or(&pkt_buf_inuse[offs], bit);  // now full
          // if buffer was full, exit
          if ((old & bit) != 0) {
            MLPD_ERROR("thread %d buffer full\n", tid);
            throw std::runtime_error("Thread %d buffer full\n");
          }
          // Reserved until marked empty by consumer
        }

        // Receive data into buffers
        std::vector<Packet*> pkts(config_->cl_sdr_ch());
        std::vector<void*> dl_slot_samp(config_->cl_sdr_ch());
        for (size_t ch = 0; ch < config_->cl_sdr_ch(); ++ch) {
          pkts.at(ch) = reinterpret_cast<Packet*>(
              buffer + (buffer_offset + ch) * packetLength);
          dl_slot_samp.at(ch) = pkts.at(ch)->data;
        }

        rx_data_status = this->client_radio_set_->radioRx(
            tid, dl_slot_samp.data(), samples_per_slot, rx_data_time);
        for (size_t ch = 0; ch < config_->cl_sdr_ch(); ++ch) {
          new (pkts.at(ch)) Packet(frame_id, slot_id, 0, ant_id + ch);
          // push kEventRxSymbol event into the queue
          this->notifyPacket(kClient, frame_id, slot_id, ant_id + ch,
                             buffer_chunk_size,
                             buffer_offset + buffer_id * buffer_chunk_size);
          buffer_offset++;
          buffer_offset %= buffer_chunk_size;
        }
      } else if (coalesce_throwaway && config_->is_houdini()) {
        // Consume a RUN of consecutive discarded slots in ONE read.
        //
        // radioRx costs 855 us fixed + 0.0037 us/sample (measured), so 30 calls
        // per frame cost 25.4 ms of a 27.5 ms iteration while the samples
        // themselves cost 0.5 ms. Coalescing the run took the iteration to
        // 2.4 ms, 36 -> 417 iter/s, with the fronthaul untouched. The run is
        // computed from isDlData rather than assumed to be "everything after
        // slot 0", so a schedule that DOES carry DL slots still reads each of
        // them into its own buffer at its own offset.
        size_t run = 0;
        while (slot_id + run < config_->slot_per_frame() &&
               !config_->isDlData(tid, slot_id + run)) {
          ++run;
        }
        const size_t want = run * samples_per_slot;
        // ONE buffer PER CHANNEL: radioRx writes cl_sdr_ch() destinations, so a
        // 1-element array is an out-of-bounds write on any 2-channel client.
        if (throwaway.size() < want * num_rx_buffs) {
          throwaway.resize(want * num_rx_buffs);
        }
        std::vector<void*> tb(num_rx_buffs);
        for (size_t ch = 0; ch < num_rx_buffs; ++ch) {
          tb.at(ch) = throwaway.data() + ch * want;
        }
        const int got = this->client_radio_set_->radioRx(
            tid, tb.data(), static_cast<int>(want), rx_data_time);
        // Advance by the slots ACTUALLY consumed. A short read (rx_gap_break
        // truncates; ret=2032 against a 12288 request observed live) would
        // otherwise skip slots whose samples are still in the stream, putting
        // the slot index and the stream position permanently out of step.
        size_t whole = (got > 0) ? static_cast<size_t>(got) / samples_per_slot
                                 : 0;
        if (whole > run) whole = run;
        // Report the read as GOOD whenever whole slots were consumed and the
        // slot index was advanced to match, because that case is handled and
        // expected: rx_gap_break truncates (ret=2032 against a 12288 request,
        // observed live). Passing `got` through instead tripped the caller's
        // `!= samples_per_slot` check and printed BAD Receive(20480/4096) on a
        // path that had just done the right thing, at loop rate. A short read
        // still says so, below, with the numbers that describe it. Only a read
        // that yielded no whole slot at all is a genuine bad receive.
        rx_data_status = (whole >= 1) ? static_cast<int>(samples_per_slot) : got;
        // THROTTLED. Both of these sit in the per-slot RX loop, and the
        // condition they report (rx_gap_break truncating a read) is PERSISTENT
        // rather than one-shot -- ret=2032 against a 12288 request was observed
        // continuously, which at loop rate is ~400 lines/s. That is the flood
        // hazard the rest of this file guards against with exactly this idiom,
        // and I added these two without it.
        static std::atomic<size_t> short_cnt{0};
        const size_t sc = short_cnt.fetch_add(1);
        if (whole < run && got > 0 && (sc == 0 || sc % 200 == 0)) {
          MLPD_INFO(
              "coalesced throwaway short read %d/%zu at frame %zu slot %zu: "
              "%zu of %zu slots consumed, index advanced to match\n",
              got, want, frame_id, slot_id, whole, run);
        }
        if (whole > 1) slot_id += whole - 1;  // the for-loop's ++ takes one
        // The SUB-SLOT remainder is consumed from the stream and not accounted
        // for: after a short read of `got` we are `got % samples_per_slot`
        // samples into the next slot, and the slot index says we are at its
        // start. Harmless on today's schedule -- these are discarded slots, and
        // the Houdini sync path re-anchors from each read's OWN timestamp
        // rather than from an accumulated position -- but a schedule that
        // carries DL data slots would window them short by that remainder. Say
        // so when it happens rather than leaving it to be discovered by a
        // mis-decoded DL slot (AP row filed; the realignment needs a DL
        // schedule to validate against, and none exists yet).
        const size_t rem = (got > 0) ? static_cast<size_t>(got) % samples_per_slot : 0;
        if (rem != 0 && (sc == 0 || sc % 200 == 0)) {
          MLPD_WARN(
              "coalesced throwaway short read %d/%zu at frame %zu slot %zu: "
              "%zu sub-slot samples consumed and unaccounted, so the stream is "
              "that far into the next slot. Safe on this schedule (no DL data "
              "slots); NOT safe once there are.\n",
              got, want, frame_id, slot_id, rem);
        }
      } else {
        //Not dl data so we throw it away
        rx_data_status = this->client_radio_set_->radioRx(
            tid, rxbuff.data(), samples_per_slot, rx_data_time);
      }
      if (rx_data_status < 0) {
        MLPD_ERROR(
            "Rx status reporting error %d during frame %zu , slot %zu, "
            "exiting\n",
            rx_data_status, frame_id, slot_id);
        config_->running(false);
        break;
      } else if (rx_data_status != static_cast<int>(samples_per_slot)) {
        MLPD_WARN("BAD Receive(%d/%zu) at Time %lld, frame count %zu\n",
                  rx_data_status, samples_per_slot, rx_data_time, frame_id);
      }
    }  // end for
    if (loop_profile_every > 0) {
      const auto prof_t4 = profile_clock::now();
      auto us = [](profile_clock::time_point a, profile_clock::time_point b) {
        return std::chrono::duration<double, std::micro>(b - a).count();
      };
      prof_rx += us(prof_t0, prof_t1);
      prof_sync += us(prof_t1, prof_t2);
      prof_tx += us(prof_t2, prof_t3);
      prof_slot += us(prof_t3, prof_t4);
      prof_all += us(prof_t0, prof_t4);
      if (++prof_n >= loop_profile_every) {
        MLPD_INFO(
            "LOOP PROFILE [%d] over %zu iters: total %.0f us/iter = rx %.0f + "
            "sync %.0f + tx %.0f + slots %.0f (searched %zu, %.1f%%); "
            "%.1f iter/s vs 1000 frames/s\n",
            tid, prof_n, prof_all / prof_n, prof_rx / prof_n,
            prof_sync / prof_n, prof_tx / prof_n, prof_slot / prof_n,
            prof_sync_searched, 100.0 * prof_sync_searched / prof_n,
            1e6 / (prof_all / prof_n));
        prof_rx = prof_sync = prof_tx = prof_slot = prof_all = 0;
        prof_n = prof_sync_searched = 0;
      }
    }
    frame_id++;
  }  // end while
}

//Blocking function for beacon detected or exit()
// Full acquisition [user 2026-08-30]: hunt in the wide sliding window, take
// the first detection's stamped absolute time, then require two further
// detections to land on that lock's frame grid before trusting it. A false
// first lock (the artifact class, or a scatter outlier) gives
// grid-inconsistent residuals and restarts the hunt; false-anchor odds with
// two confirms are ~1e-4 even before the SNR floor. Used at startup and by
// the AP-18 resync escalation (beacon-lost fallback). Returns false only if
// running() went false or kMaxHunts detections never produced a confirmed
// lock; it BLOCKS while no detection at all is available (searching forever
// is the wanted behavior when the beacon is gone -- pilots pause).
bool Receiver::houdiniAcquireAnchor(int tid, size_t detect_window,
                                    const houdini::sync::SyncGeometry& geom,
                                    long long& anchor_out,
                                    double* period_out) {
  // Detector scatter (ledger 4.18) plus path, expressed in TIME so it scales
  // with the rate the way the tracking gate does. TAKEN FROM THE CALLER'S
  // geometry, not re-derived: the acquisition gate is clamped by the tracking
  // one (confirm <= scatter), so a second derivation that did not see the same
  // HOUDINI_SCATTER_TOL_US inverted them, which is a lock that escalates
  // immediately and forever. sync_geometry.h is the one derivation (AP-56).
  const long long kConfirmTol = geom.confirm_tol;
  // The refine stage wants a LONG baseline, because the rate error is the
  // detection-pair noise divided by the span and that noise is sub-sample
  // (measured 0.15-0.94 across four acquisitions). k ~ 20 gives ~4% rate
  // error; k >= 200 gives ~0.4%. The stage exists because a full-frame
  // detect window would otherwise SHORTEN the span: today's k of 17-37 is an
  // accident of the hunt needing ~13.6 windowed reads to find the beacon at
  // all, and a guaranteed first hit removes exactly that accident.
  const long long kRefineSpan =
      static_cast<long long>(config_->sync().resync.acq_refine_span);
  // Budget must scale with the span it now has to reach. With a full-frame
  // window every hunt is a near-certain hit, so k advances only by the wall
  // time of one read (a few frames) per hunt -- the old flat 200 could expire
  // before the baseline was met, and exceeding it THROWS at the caller
  // ("beacon acquisition: no confirmed lock") rather than retrying.
  const int kMaxHunts =
      static_cast<int>(std::max<long long>(200, 4 * kRefineSpan));
  const long long fr = static_cast<long long>(config_->samps_per_frame());
  long long first_abs = 0;
  bool have_first = false;
  int confirms = 0;
  int hunts = 0;
  int consecutive_fails = 0;
  const double kAcqMaxPpm = config_->sync().resync.acq_max_ppm;
  double period = static_cast<double>(fr);  // coarse after stage 2, fine after 3
  while (config_->running()) {
    if (++hunts > kMaxHunts) {
      MLPD_WARN(
          "houdiniAcquireAnchor [%d]: no confirmed lock after %d "
          "detections\n",
          tid, kMaxHunts);
      return false;
    }
    long long wstamp = 0;
    const ssize_t idx = clientSyncBeacon(tid, detect_window, &wstamp);
    if (idx < 0) continue;  // running() went false inside
    const long long abs_end = wstamp + idx;  // beacon END, UE ticks
    if (!have_first) {
      have_first = true;
      first_abs = abs_end;
      confirms = 0;
      period = static_cast<double>(fr);
      MLPD_INFO("houdiniAcquireAnchor [%d]: hunt lock at abs %lld (idx %ld)\n",
                tid, abs_end, idx);
      continue;
    }
    // Predict with the best period we have: nominal before stage 2, the coarse
    // rate after it. Without that the residual grows as drift*k and the
    // tolerance would have to be widened until it discriminated nothing.
    const long long k =
        llround(static_cast<double>(abs_end - first_abs) / period);
    const long long resid =
        abs_end - (first_abs + llround(static_cast<double>(k) * period));
    if (k != 0 && std::llabs(resid) <= kConfirmTol) {
      ++confirms;
      // Every accepted detection improves the rate: resid/k is the period error
      // over a span of k REAL frames -- but the gain is 1/k and k is SMALL
      // early, which the full-frame window made the common case rather than the
      // rare one (each hunt is about one read, so k advances a few frames at a
      // time). A scatter outlier accepted at k=4 with resid at the 640
      // tolerance moves the period by 160 samples/frame, about 1300 ppm and 150
      // times the real offset. Bound it to a physically plausible band, the
      // same guard the tracker applies to its own updates.
      const double cand =
          period + static_cast<double>(resid) / static_cast<double>(k);
      const double lo = static_cast<double>(fr) * (1.0 - kAcqMaxPpm * 1e-6);
      const double hi = static_cast<double>(fr) * (1.0 + kAcqMaxPpm * 1e-6);
      period = std::min(hi, std::max(lo, cand));
      consecutive_fails = 0;
      if (confirms >= 2 && k >= kRefineSpan) {
        anchor_out = first_abs - config_->shape().expectedEndOffset();
        if (period_out != nullptr) *period_out = period;
        MLPD_INFO(
            "houdiniAcquireAnchor [%d]: lock CONFIRMED (resid %lld over "
            "%lld frames, confirm %d) -> frame anchor %lld, bootstrap period "
            "%.4f (%+.4f samp/frame)\n",
            tid, resid, k, confirms, anchor_out, period,
            period - static_cast<double>(fr));
        return true;
      }
      MLPD_INFO(
          "houdiniAcquireAnchor [%d]: confirm %d at k=%lld (resid %lld), "
          "period %.4f -- %s\n",
          tid, confirms, k, resid, period,
          (k < kRefineSpan) ? "extending the baseline" : "need 2 confirms");
    } else {
      MLPD_INFO(
          "houdiniAcquireAnchor [%d]: confirm failed (resid %lld, k %lld) "
          "-> hunt restart\n",
          tid, resid, k);
      first_abs = abs_end;
      confirms = 0;
      // Keep the refined period across a failed confirm ONLY while it is still
      // plausible. The original reasoning holds -- the failure says the ANCHOR
      // was wrong and the oscillators did not change -- but it assumed the
      // period could not itself be the cause. It can: a bad refinement makes
      // every later prediction wrong, so every confirm fails, and keeping it
      // unconditionally made that state PERMANENT with no way back short of a
      // restart. Two consecutive failures means the rate is the suspect.
      if (++consecutive_fails >= 2) {
        MLPD_WARN(
            "houdiniAcquireAnchor [%d]: %d consecutive confirm failures -- "
            "resetting the period estimate %.4f back to nominal, it is the "
            "likely culprit\n",
            tid, consecutive_fails, period);
        period = static_cast<double>(fr);
        consecutive_fails = 0;
      }
    }
  }
  return false;
}

ssize_t Receiver::clientSyncBeacon(size_t radio_id, size_t sample_window,
                                   long long* window_time) {
  ssize_t sync_index = -1;
  long long rx_time = 0;
  assert(sample_window <= config_->samps_per_frame());
  const size_t num_rx_buffs = config_->cl_sdr_ch();
  std::vector<std::vector<std::complex<int16_t>>> syncbuffmem(
      num_rx_buffs, std::vector<std::complex<int16_t>>(
                        sample_window, std::complex<int16_t>(0, 0)));

  std::vector<void*> syncrxbuffs;
  for (size_t ch = 0; ch < num_rx_buffs; ch++) {
    syncrxbuffs.push_back(syncbuffmem.at(ch).data());
  }

  while (config_->running() && (sync_index < 0)) {
    const int rx_status = client_radio_set_->radioRx(
        radio_id, syncrxbuffs.data(), sample_window, rx_time);

    if (rx_status < 0) {
      MLPD_ERROR("clientSyncBeacon [%zu]: BAD SYNC Received (%d/%zu) %lld\n",
                 radio_id, rx_status, sample_window, rx_time);
    } else {
      const size_t new_samples = static_cast<size_t>(rx_status);
      if (new_samples == sample_window) {
        MLPD_TRACE(
            "clientSyncBeacon - Samples %zu - Window %zu\n",
            new_samples, sample_window);

        // Acquisition. Everything downstream is measured from this index, and
        // unlike a resync it is anchored ONCE and never revisited -- so it is
        // the one place a false lock is unrecoverable, and it was the one place
        // the 2026-09-02 fix had not been applied.
        //
        // The multi-copy argument for kFirstClusterRefined no longer holds. It
        // dates from the loops=forever era that filled a symbol with ~15 copies
        // 4096 apart; the strobe now plays loops=1 once per TDD frame, so copies
        // are 122880 samples apart while the acquisition window is at most
        // samps_per_frame (122880) and the escalation window 9543. Both hold at
        // most ONE copy, so there is no copy ambiguity to be repeatable about
        // and the targeted rule's precondition holds here too.
        //
        // Iris/UHD keeps the old rule: different hardware, different framer, and
        // this bench cannot exercise it.
        // NOTE the earlier corr_scale_init change did NOT resolve the run-to-run
        // constellation split it was investigated for -- see config.cc.
        sync_index = syncSearch(syncbuffmem.at(kSyncDetectChannel).data(),
                                sample_window,
                                config_->num_cl_sdrs() > 1
                                    ? config_->corr_scale_init(radio_id)
                                    : static_cast<float>(
                                          config_->sync().detector.bar.corr_scale_init),
                                sync_detector_->pick());
        // SNR floor: a correlation crossing at noise level is the artifact
        // class, not the beacon (measured: real ~45 dB, artifacts ~0 dB).
        // Reject and keep hunting rather than anchor on it.
        if (config_->is_houdini() && sync_index >= 0) {
          const double snr = sync_guard_->snrDb(
              syncbuffmem.at(kSyncDetectChannel).data(), sample_window,
              sync_index);
          if (!sync_guard_->accept(snr)) {
            static std::atomic<int> rej{0};
            const int nrej = rej.fetch_add(1);
            if ((nrej % 16) == 0) {
              MLPD_INFO(
                  "clientSyncBeacon [%zu]: rejected low-SNR detection "
                  "(idx %ld, %.1f dB < %.1f dB floor), count %d\n",
                  radio_id, sync_index, snr, sync_guard_->floorDb(), nrej + 1);
            }
            sync_index = -1;
          } else if (window_time != nullptr) {
            MLPD_INFO("clientSyncBeacon [%zu]: idx %ld snr %.1f dB\n",
                      radio_id, sync_index, snr);
          }
        }
        if (sync_index >= 0 && window_time != nullptr) *window_time = rx_time;
      } else {
        MLPD_ERROR(
            "clientSyncBeacon [%zu]: BAD SYNC - Rx samples not requested size "
            "(%zu/%zu) %lld\n",
            radio_id, new_samples, sample_window, rx_time);
      }
    }
  }  // end while sync_index < 0
  return sync_index;
}

/// This function blocks untill all the discard_samples are received for a given radio
void Receiver::clientAdjustRx(size_t radio_id, size_t discard_samples) {
  const size_t num_rx_buffs = config_->cl_sdr_ch();
  long long rx_time = 0;

  //This can be fixed and combined with other scratch memory
  std::vector<std::vector<std::complex<int16_t>>> temp_mem(
      num_rx_buffs, std::vector<std::complex<int16_t>>(discard_samples));

  std::vector<void*> trash_memory;
  for (size_t ch = 0; ch < num_rx_buffs; ch++) {
    trash_memory.push_back(temp_mem.at(ch).data());
  }

  while (config_->running() && (discard_samples > 0)) {
    const int rx_status = client_radio_set_->radioRx(
        radio_id, trash_memory.data(), discard_samples, rx_time);

    if (rx_status < 0) {
      MLPD_ERROR(
          "clientAdjustRx [%zu]: BAD Rx Adjust Status Received (%d/%zu) %lld\n",
          radio_id, rx_status, discard_samples, rx_time);
    } else {
      size_t new_samples = static_cast<size_t>(rx_status);
      if (new_samples <= discard_samples) {
        discard_samples -= new_samples;
        MLPD_TRACE("clientAdjustRx [%zu]: Discarded Samples (%zu/%zu)\n",
                   radio_id, new_samples, discard_samples);
      } else {
        MLPD_ERROR(
            "clientAdjustRx [%zu]: BAD radioRx more samples then requested "
            "(%zu/%zu) %lld\n",
            radio_id, new_samples, discard_samples, rx_time);
      }
    }
  }  // request_samples > 0
}
