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
#include <algorithm>
#include <cmath>
#include <cstring>
#include <cstdlib>
#include <random>

#include "SoapySDR/Time.hpp"
#if defined(USE_UHD)
#include "include/ClientRadioSetUHD.h"
#else
#include "include/ClientRadioSet.h"
#endif
#include "include/comms-lib.h"
#include "include/logger.h"
#include "include/macros.h"
#include "include/node_version.h"
#include "include/utils.h"

//Default to detect the beacon on first channel
static constexpr size_t kSyncDetectChannel = 0;
static constexpr float kBeaconDetectWindowScaler = 2.33f;
static constexpr bool kEnableCfo = false;
// Beacon core geometry, mirrored from Config::genBeacon (config.cc): 15 reps of
// STS(16) then 2 reps of gold(128). estimateCFO() correlates BOTH structures
// and guards on the total at runtime.
static constexpr int kStsLen = 16;
static constexpr int kStsReps = 15;
static constexpr int kGoldLen = 128;
static constexpr int kGoldReps = 2;
static constexpr int kBeaconCoreLen = kStsLen * kStsReps + kGoldLen * kGoldReps;
// Beacon CFO logs at ~9/s; print 1 in N so a long run does not add millions of
// lines. The panel gets every sample regardless. HOUDINI_CFO_LOG_EVERY=1 makes
// it dense, which is what a calibration run wants.
static size_t cfoLogEvery(void) {
  static const size_t n = [] {
    const char* e = std::getenv("HOUDINI_CFO_LOG_EVERY");
    const long v = (e != nullptr) ? std::strtol(e, nullptr, 10) : 10;
    return static_cast<size_t>(v > 0 ? v : 1);
  }();
  return n;
}

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
static constexpr ssize_t kHoudiniStrobeOffsTicks = 384;
static ssize_t houdiniBeaconEnd(Config* cfg) {
  if (cfg->is_houdini()) {
    return kHoudiniStrobeOffsTicks + static_cast<ssize_t>(cfg->beacon_size());
  }
  return static_cast<ssize_t>(cfg->beacon_size() + cfg->prefix());
}

// In-window SNR of a claimed beacon detection: energy of the presumed core
// [end_idx - core_len, end_idx) against the rest of the window. On this bench
// a real beacon measures ~45 dB and the noise-window artifact class that
// crosses the correlation threshold (DEMO_VERIFICATION.md 4.25) measures
// ~0 dB, so the floor separates them by orders of magnitude. [user 2026-08-30:
// "keep the sync snr about 30 dB or so" -- default 20 leaves margin both
// ways; HOUDINI_SYNC_SNR_DB overrides for bench tuning.]
static double beaconSnrDb(const std::complex<int16_t>* w, size_t n,
                          ssize_t end_idx, size_t core_len) {
  const ssize_t lo = end_idx - static_cast<ssize_t>(core_len);
  if (lo < 0 || end_idx > static_cast<ssize_t>(n) || core_len == 0) return -99.0;
  // Guard band around the core: the detector index jitters 1-2 samples about
  // the true core end (measured -1/-2 on 6/6 dumped windows, ledger 4.42),
  // and with a slot-length window each core sample leaking into the rest-mean
  // costs ~8 dB (sweep on a ~46 dB wire: 0/-1/-2 -> 45.8/37.5/31.6 dB), so
  // the reported SNR tracked detector jitter, not the link. Excluding a few
  // samples on each side of the core from BOTH sums makes the number read
  // the link: a bias up to +-8 samples now costs <0.1 dB instead of ~14.
  constexpr ssize_t kGuard = 8;
  double core = 0, rest = 0;
  size_t nrest = 0;
  for (size_t i = 0; i < n; ++i) {
    const double re = w[i].real(), im = w[i].imag();
    const double e = re * re + im * im;
    const ssize_t si = static_cast<ssize_t>(i);
    if (si >= lo && si < end_idx) {
      core += e;
    } else if (si < lo - kGuard || si >= end_idx + kGuard) {
      rest += e;
      ++nrest;
    }
  }
  if (nrest == 0 || rest <= 0.0) return 99.0;
  return 10.0 * std::log10((core / core_len) / (rest / nrest) + 1e-30);
}

// Env override for a tunable double, with the compiled default when unset.
// The AP-31 tracker gains are the first users: they need to be sweepable on a
// live bench without a rebuild, since the right damping depends on the arrival
// jitter and the detection rate, both of which are bench properties.
static double envDouble(const char* name, double dflt) {
  const char* e = getenv(name);
  if (e == nullptr) return dflt;
  char* end = nullptr;
  const double v = std::strtod(e, &end);
  return (end != e) ? v : dflt;
}

static double syncSnrFloorDb() {
  // [user 2026-08-30]: "keep the sync snr about 30 dB or so". The earlier
  // default of 20 compensated for the pre-guard-band metric under-reading by
  // ~14 dB (ledger 4.42); with the metric now reading true link dB (47.6
  // measured live on the bench wire), 30 is the intended floor with margin.
  static const double v = [] {
    const char* e = getenv("HOUDINI_SYNC_SNR_DB");
    return e ? atof(e) : 30.0;
  }();
  return v;
}

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
  houdini_pilot_cursor_reset_.reserve(config_->num_cl_sdrs());
  for (size_t i = 0; i < config_->num_cl_sdrs(); ++i)
    houdini_pilot_cursor_reset_.emplace_back(
        std::make_unique<std::atomic<bool>>(false));
  /* initialize random seed: */
  srand(time(NULL));

  MLPD_TRACE("Receiver Construction - CL present: %d, BS Present: %d\n",
             config_->client_present(), config_->bs_present());
  try {
#if defined(USE_UHD)
    this->client_radio_set_ =
        config_->client_present() ? new ClientRadioSetUHD(config_) : nullptr;
    this->base_radio_set_ =
        config_->bs_present() ? new BaseRadioSetUHD(config_) : nullptr;
#else
    this->client_radio_set_ =
        config_->client_present() ? new ClientRadioSet(config_) : nullptr;
    this->base_radio_set_ =
        config_->bs_present() ? new BaseRadioSet(config_, false) : nullptr;
#endif
  } catch (std::exception& e) {
    throw ReceiverException(e.what());
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
      delete this->base_radio_set_;
    }
    if (this->client_radio_set_ != nullptr) {
      MLPD_WARN("Invalid Client Radio Setup: %d\n",
                this->client_radio_set_ == nullptr);
      this->client_radio_set_->radioStop();
      delete this->client_radio_set_;
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
    delete this->base_radio_set_;
  }
  if (this->client_radio_set_ != nullptr) {
    this->client_radio_set_->radioStop();
    delete this->client_radio_set_;
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

void Receiver::clientTxPilots(size_t user_id, long long base_time) {
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
    const long long frame = static_cast<long long>(config_->samps_per_frame());
    // The driver only ACCEPTS burst anchors on the 384-tick / 3125 ns grid
    // (the finest ns-exact grid, TxTickAnchor SH-248), but a burst's INTERIOR
    // advances tick-exactly. So compose ONE burst per frame -- [front-pad
    // zeros | pilot slot | gap zeros | data slot] -- anchored at the grid
    // point floored below the desired start: the pad places the pilot to the
    // sample and the data rides at EXACTLY ul_off from it. Both snap draws
    // measured in ledger 4.44 (the +-192 per-run seat window and the bimodal
    // -128/+256 P->U differential) die here. [user 2026-08-30: "work around
    // the TX burst seam by zero padding".] with the shipped 1 ms frame
    // (= 320*384 exactly) the pad is constant within a run and the burst
    // composes once; a frame that is not a grid multiple would merely
    // recompose per iteration, still correctly.
    constexpr long long kTddGridTicks = 384;
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
    const long long end = txTime + static_cast<long long>(horizon) * frame;
    long long cur = std::max(pilot_cursor + frame, txTime);
    if (user_id < houdini_pilot_cursor_reset_.size() &&
        houdini_pilot_cursor_reset_.at(user_id)->exchange(false) &&
        pilot_cursor + frame > txTime) {
      const long long k =
          (pilot_cursor + frame - txTime + frame - 1) / frame;  // ceil
      cur = txTime + k * frame;
    }
    int nsched = 0;
    const bool ul_fits = ul_present && ul_off >= num_samps;
    for (; cur <= end; cur += frame) {
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
    if (std::getenv("HOUDINI_UE_TX_DEBUG") != nullptr && nsched > 0) {
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
                             bool refine_first_cluster) {
  ssize_t sync_index(-1);
  assert(search_window <= config_->samps_per_frame());
#if defined(USE_CUDA)
  const char* kPath = "cuda";
  sync_index = CommsLib::find_beacon_cuda(check_data, config_->gold_cf32(),
                                          search_window, corr_scale);
#else
  // portable find_beacon_avx works on x86 and aarch64 (see comms-lib-portable.cc)
  const char* kPath = "avx";
  sync_index = CommsLib::find_beacon_avx(check_data, config_->gold_cf32(),
                                         search_window, corr_scale,
                                         refine_first_cluster);
#endif
  if (std::getenv("HOUDINI_SYNC_DEBUG") != nullptr) {
    static std::atomic<int> c{0};
    if ((c.fetch_add(1) % 20) == 0) {  // braces load-bearing (macro)
      MLPD_INFO("syncSearch[%s]: window=%zu corr_scale=%.3f gold=%zu -> idx=%ld\n",
                kPath, search_window, corr_scale, config_->gold_cf32().size(),
                sync_index);
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
// [shift i32][samps_per_frame u32][carrier_hz f32][scatter_tol u32] -- 44 bytes.
// The trailing three let the panel convert to ppm and draw the accept/reject
// band without hardcoding config values into the page, which is exactly what
// breaks when a config changes (AP-31 proposes retuning the gate).
//
// `shift` is the schedule step actually applied, NOT fresh-minus-previous: both
// anchors are ABSOLUTE sample times taken k frames apart, so their difference is
// dominated by k*samps_per_frame elapsed time and overflows the int32 wire field
// after ~17.5 s of run. It is reduced modulo the frame period and centred, and
// is nonzero only on an escalation re-anchor -- the only place the UE moves its
// schedule today (resync is a liveness detector, not a micro-corrector).
static void sendSyncTelemetry(size_t frame, int tid, uint32_t state,
                              long long resid, double cfo_hz, double snr,
                              long long shift, uint32_t sfr, float carrier,
                              uint32_t scatter_tol) {
  const int fd = syncTelemetrySock();
  if (fd < 0) return;
  uint8_t buf[44];
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
  (void)::send(fd, buf, sizeof(buf), 0);
}

// Two-stage beacon CFO estimate, normalized (cycles/sample); multiply by the
// sample rate for Hz.
//
// The beacon core is 15 x STS(16) followed by 2 x gold(128) = 496 samples
// (Config::genBeacon, config.cc), so it carries TWO independent repetition
// structures and therefore two estimators:
//
//   coarse  consecutive STS blocks, lag 16   -> unambiguous to +-rate/32
//   fine    gold rep2 against rep1, lag 128  -> unambiguous to +-rate/256,
//                                               8x finer resolution
//
// The coarse stage resolves the fine stage's 1/128 ambiguity, so the result
// keeps the fine resolution across the coarse range. Both stages are plain
// repetition correlations: for x[n] = s[n]*exp(j2*pi*f*n) with s[n+N] = s[n],
// sum conj(x[n])*x[n+N] has argument 2*pi*f*N.
//
// The PREVIOUS implementation split the core in half and correlated
// half against half. That split falls INSIDE the structure (240 STS + 8 gold
// against 120 gold + 128 gold), correlating two uncorrelated sequences, so it
// returned noise. That is why kEnableCfo was false and why no CFO line has
// ever appeared in a run log.
//
// `sync_index` is the beacon END (syncSearch convention), so the core occupies
// [sync_index - 496, sync_index).
float Receiver::estimateCFO(const std::complex<int16_t>* buf, size_t buf_len,
                            int sync_index) const {
  if (buf == nullptr) return 0.0f;
  // Geometry guard: this estimator is tied to the STS+gold layout above. If the
  // beacon is ever rebuilt to another shape, fail to 0 rather than silently
  // return a wrong frequency that a correction loop would then act on.
  if (static_cast<int>(config_->beacon_size()) != kBeaconCoreLen) {
    static std::atomic<bool> warned{false};
    if (warned.exchange(true) == false) {
      MLPD_WARN(
          "estimateCFO: beacon is %d samples, expected %d (%d x STS(%d) + "
          "%d x gold(%d)) -- CFO estimation disabled\n",
          static_cast<int>(config_->beacon_size()), kBeaconCoreLen, kStsReps,
          kStsLen, kGoldReps, kGoldLen);
    }
    return 0.0f;
  }
  const int start = sync_index - kBeaconCoreLen;
  if (start < 0 || sync_index < 0 ||
      static_cast<size_t>(sync_index) > buf_len) {
    return 0.0f;
  }

  auto at = [buf](int i) {
    return std::complex<double>(static_cast<double>(buf[i].real()),
                                static_cast<double>(buf[i].imag()));
  };

  // Fine: gold rep2 against rep1 (lag 128).
  const int g1 = start + kStsLen * kStsReps;
  const int g2 = g1 + kGoldLen;
  std::complex<double> r_fine(0.0, 0.0);
  for (int i = 0; i < kGoldLen; ++i) r_fine += std::conj(at(g1 + i)) * at(g2 + i);

  // Coarse: every consecutive STS pair (lag 16), summed coherently.
  std::complex<double> r_coarse(0.0, 0.0);
  for (int k = 0; k + 1 < kStsReps; ++k) {
    for (int i = 0; i < kStsLen; ++i) {
      r_coarse += std::conj(at(start + k * kStsLen + i)) *
                  at(start + (k + 1) * kStsLen + i);
    }
  }
  if (std::abs(r_fine) == 0.0 || std::abs(r_coarse) == 0.0) return 0.0f;

  const double f_fine = std::arg(r_fine) / (2.0 * M_PI * kGoldLen);
  const double f_coarse = std::arg(r_coarse) / (2.0 * M_PI * kStsLen);
  // Unwrap the fine estimate into the coarse stage's range.
  const double ambiguity = 1.0 / kGoldLen;
  const double m = std::round((f_coarse - f_fine) / ambiguity);
  double f = f_fine + m * ambiguity;
  // The matched-NCO R2C RX mixer delivers baseband CONJUGATED (the same
  // inversion recorder_worker undoes via rx_conj_ for CSI). Sync runs on RAW
  // samples, so a +f carrier offset reads as -f here; undo it so the sign is
  // physical. Sign and scale VERIFIED against deliberate injection (AP-30).
  // Not yet checked against a truth the estimator cannot infer: on a link with
  // the sample clocks shared -- where the arrival ramp measures eps = 0 exactly
  // -- this reads +353 Hz, so it carries a zero-point offset of that order
  // (2026-09-01, clock_drift_probe.py leg A). AP-34(b) is the fix: precision
  // scales with lag, and the bias is a fixed phase divided by a short one.
  if (config_->is_houdini()) f = -f;
  return static_cast<float>(f);
}

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
  const size_t beacon_detect_window =
      static_cast<size_t>(static_cast<float>(config_->samps_per_slot()) *
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
        tid, beacon_detect_window, houdini_anchor, &houdini_boot_period);
    if (!houdini_anchored && config_->running()) {
      throw std::runtime_error("beacon acquisition: no confirmed lock");
    }
  } else {
    while ((sync_count < kTargetSyncCount) && config_->running()) {
      const ssize_t sync_index = clientSyncBeacon(tid, beacon_detect_window);
      if (sync_index >= 0) {
        const ssize_t adjust = sync_index - houdiniBeaconEnd(config_);
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
  bool resync = true;
  bool resync_enable = (config_->frame_mode() == "continuous_resync");
  size_t resync_retry_cnt(0);
  size_t resync_retry_max(100);
  size_t resync_success(0);
  size_t cfo_log_cnt = 0;  // throttles the beacon-CFO line (kCfoLogEvery)
  // Liveness accept/reject half-width. Shipped ON THE WIRE so the panel draws
  // the band it actually illustrates rather than a hardcoded copy (AP-31
  // proposes retuning this, after which a page-side constant would silently lie).
  constexpr long long kScatterTol = 1024;
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
                      long long shift) {
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
                      static_cast<uint32_t>(kScatterTol));
  };
  // TODO: measure CFO from the first beacon and apply here
  const double kGridAlpha = envDouble("HOUDINI_GRID_ALPHA", 0.5);
  const double kGridBeta = envDouble("HOUDINI_GRID_BETA", 0.1);
  const size_t max_cfo = 100;  // in ppb, For Iris
  const size_t resync_period = static_cast<size_t>(
      std::floor(1e9 / (max_cfo * config_->samps_per_frame())));
  size_t last_resync = frame_id;
  if (config_->running() == true) {
    MLPD_INFO(
        "Start main client txrx loop... tid=%d with resync period of %zu, "
        "grid tracker alpha %.3f beta %.3f (0/0 = fixed-period grid), "
        "bootstrap period %.4f (%+.4f samp/frame vs nominal)\n",
        tid, resync_period, kGridAlpha, kGridBeta, houdini_boot_period,
        houdini_boot_period - static_cast<double>(config_->samps_per_frame()));
  }
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
  double houdini_frame_period = houdini_boot_period;
  // Count tracker updates so an escalation knows whether its own learned rate
  // is better than a fresh confirm-derived one (below).
  size_t houdini_grid_updates = 0;
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
  bool resync_hold_pending = false;  // one off-grid detection seen (4.18
                                     // scatter means singles are noise)
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
  constexpr size_t kEscalateExhaustedEpisodes = 2;
  size_t resync_exhausted_streak = 0;
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
    if (houdiniAcquireAnchor(tid, beacon_detect_window_esc, fresh,
                             &fresh_period)) {
      // The ONLY place the UE moves its schedule. Report the applied step so the
      // panel can mark it against the resid trace.
      // Both anchors are ABSOLUTE sample times k frames apart, so their raw
      // difference is dominated by elapsed time and overflows int32 in ~17.5 s.
      // The schedule step is that difference modulo the frame period, centred.
      long long step = 0;
      if (houdini_pilot_ref_valid) {
        const long long fr = static_cast<long long>(config_->samps_per_frame());
        step = ((fresh - prev_ref) % fr + fr) % fr;
        if (step > fr / 2) step -= fr;
      }
      emitSync(kSyncEscalating, 0, 0.0, 0.0, step);
      // Escalation means the OFFSET was lost; the two oscillators did not
      // change when that happened. So keep a rate the tracker actually learned
      // (many observations, ~0.1%) and take the confirm's fresh one (1-5%)
      // only while the tracker has learned nothing at all -- which is exactly
      // the case an escalation-first run lands in.
      if (houdini_grid_updates == 0) {
        houdini_frame_period = fresh_period;
      }
      houdini_pilot_ref = fresh;
      houdini_pilot_ref_valid = true;
      if (static_cast<size_t>(tid) < houdini_pilot_cursor_reset_.size())
        houdini_pilot_cursor_reset_.at(tid)->store(true);
      MLPD_INFO("Re-sync ESCALATION: re-anchored at %lld, tid %d\n", fresh,
                tid);
    } else if (config_->running()) {
      MLPD_WARN(
          "Re-sync ESCALATION: re-acquisition did not confirm; keeping the "
          "previous anchor, tid %d\n",
          tid);
      emitSync(kSyncReanchorFailed, 0, 0.0, 0.0, 0);
    }
    resync_exhausted_streak = 0;
    resync_hold_pending = false;
    resync = false;
    resync_retry_cnt = 0;
    last_resync = frame_id;
  };

  while (config_->running() == true) {
    if (config_->max_frame() > 0 && frame_id >= config_->max_frame()) {
      config_->running(false);
      break;
    }
    //Slot 0 / Beacon...
    const int request_samples = samples_per_slot - beacon_adjust;
    const int rx_status = client_radio_set_->radioRx(
        tid, rxbuff.data(), request_samples, rx_beacon_time);
    beacon_adjust = 0;
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

    if ((frame_id - last_resync) >= resync_period) {
      resync = resync_enable;
      last_resync = frame_id;
      MLPD_TRACE("Enable resyncing at frame %zu\n", frame_id);
    }
    if (resync == true) {
      ssize_t sync_index = -1;
      bool resync_attempted = true;
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
            static_cast<long long>(houdiniBeaconEnd(config_));
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
        constexpr long long kCorrContext = 256;  // gold correlator run-up
        constexpr long long kLead = kScatterTol + kCorrContext;
        constexpr long long kTail = kScatterTol + 64;
        if (off >= kLead && off + kTail <= request_samples) {
          auto* base = reinterpret_cast<std::complex<int16_t>*>(
              rxbuff.at(kSyncDetectChannel));
          const ssize_t s0 = static_cast<ssize_t>(off - kLead);
          const ssize_t slice_len = kLead + kTail;
          const ssize_t idx = this->syncSearch(
              base + s0, static_cast<size_t>(slice_len),
              config_->corr_scale(tid) + resync_retry_cnt);
          if (idx >= 0) sync_index = s0 + idx;
        } else {
          resync_attempted = false;  // beacon not due in this window
        }
      } else {
        sync_index = this->syncSearch(
            reinterpret_cast<std::complex<int16_t>*>(
                rxbuff.at(kSyncDetectChannel)),
            request_samples, config_->corr_scale(tid) + resync_retry_cnt);
      }
      if (sync_index >= 0 && config_->is_houdini() &&
          houdini_pilot_ref_valid) {
        // Liveness verdict on the targeted detection: SNR floor first, then
        // the grid residual (alive within scatter / moved beyond it).
        const double snr = beaconSnrDb(
            reinterpret_cast<std::complex<int16_t>*>(
                rxbuff.at(kSyncDetectChannel)),
            static_cast<size_t>(request_samples), sync_index,
            config_->beacon_size());
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
              fprintf(fg,
                      "n %d\nsync_index %zd\nsnr %.2f\nframe %zu\n"
                      "rx_beacon_time %lld\npilot_ref %lld\nbeacon_end %lld\n",
                      request_samples, sync_index, snr, frame_id,
                      rx_beacon_time, houdini_pilot_ref,
                      static_cast<long long>(houdiniBeaconEnd(config_)));
              fclose(fg);
            }
          }
        }
        if (snr < syncSnrFloorDb()) {
          // Report it: without this a WEAK beacon is indistinguishable from no
          // beacon on the panel, and they call for different operator actions.
          emitSync(kSyncWeak, 0, 0.0, snr, 0);
          sync_index = -1;  // fall through to the miss path below
        } else {
          const long long abs_end = rx_beacon_time + sync_index;
          const long long beacon_end =
              static_cast<long long>(houdiniBeaconEnd(config_));
          const long long kf = houdiniGridIndex(abs_end - beacon_end);
          const long long resid =
              abs_end - (houdiniGridStart(kf) + beacon_end);
          // Beacon CFO on the SAME validated detection (~600 flops at ~9/s).
          // Reported beside resid because they are one oscillator error seen
          // two ways: the SLOPE of resid is the fractional rate error, and
          // cfo/carrier is that same fraction read off the carrier. They must
          // agree -- a disagreement means one instrument is wrong, which is the
          // whole reason both are measured (BACKLOG AP-30/AP-31).
          const float cfo_norm =
              estimateCFO(reinterpret_cast<std::complex<int16_t>*>(
                              rxbuff.at(kSyncDetectChannel)),
                          static_cast<size_t>(request_samples),
                          static_cast<int>(sync_index));
          const double cfo_hz =
              static_cast<double>(cfo_norm) * config_->rate();
          const double cfo_ppm = (config_->freq() > 0.0)
                                     ? (cfo_hz / config_->freq()) * 1e6
                                     : 0.0;
          if ((cfo_log_cnt++ % cfoLogEvery()) == 0) {
            MLPD_INFO(
                "Beacon CFO frame %zu: %+.1f Hz (%+.3f ppm), resid %+lld, "
                "snr %.1f dB, tid %d\n",
                frame_id, cfo_hz, cfo_ppm, resid, snr, tid);
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
            // Accepted observation: advance the tracked grid. The gate keeps
            // its old role as the alive/moved verdict AND becomes the tracker's
            // outlier reject -- a rejected detection updates nothing rather
            // than levering the rate estimate (AP-31).
            if (kf > 0) {
              const double dk = static_cast<double>(kf);
              const double r = static_cast<double>(resid);
              houdini_pilot_ref = houdiniGridStart(kf) +
                                  llround(kGridAlpha * r);
              houdini_frame_period += kGridBeta * r / dk;
              houdini_grid_updates++;
            }
            resync_hold_pending = false;
            resync_exhausted_streak = 0;
            resync = false;
            resync_retry_cnt = 0;
            resync_success++;
            MLPD_INFO(
                "Re-sync frame %zu: beacon alive on the anchored grid "
                "(resid %+lld within scatter, snr %.1f dB), tid %d\n",
                frame_id, resid, snr, tid);
            emitSync(kSyncLocked, resid, cfo_hz, snr, 0);
          } else {
            MLPD_WARN(
                "Re-sync frame %zu: off-grid detection %+lld (snr %.1f dB, "
                "pending %d) -- beacon possibly moved, tid %d\n",
                frame_id, resid, snr, resync_hold_pending ? 1 : 0, tid);
            emitSync(kSyncHold, resid, cfo_hz, snr, 0);
            if (resync_hold_pending) {
              houdiniEscalate("beacon moved");
              continue;  // rx_beacon_time is pre-hunt; restart the frame loop
            }
            resync_hold_pending = true;
            // stay in resync; a moved beacon repeats off-grid, noise does not
          }
        }
      } else if (sync_index >= 0) {
        const int new_rx_offset =
            static_cast<int>(sync_index - houdiniBeaconEnd(config_));
        //Adjust tx time
        rx_beacon_time += new_rx_offset;
        if (config_->is_houdini() && !houdini_pilot_ref_valid) {
          houdini_pilot_ref = rx_beacon_time;
          houdini_pilot_ref_valid = true;
        }
        resync = false;
        resync_retry_cnt = 0;
        resync_success++;
        MLPD_INFO(
            "Re-syncing success at frame %zu with offset: %d, after %zu tries, "
            "index: %ld, tid %d\n",
            frame_id, new_rx_offset, resync_retry_cnt + 1, sync_index, tid);

        if (kEnableCfo && (sync_index >= 0)) {
          const auto cfo_phase_est =
              estimateCFO(samplemem.at(kSyncDetectChannel).data(),
                          samplemem.at(kSyncDetectChannel).size(), sync_index);
          MLPD_INFO("Client %d Estimated CFO (Hz): %f\n", tid,
                    cfo_phase_est * config_->rate());
        }

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
        resync_retry_cnt++;

        if (resync_retry_cnt > resync_retry_max) {
          if (config_->is_houdini() && houdini_pilot_ref_valid) {
            // Under recvHoudini's drain the per-frame slot-0 window carries
            // the beacon only a few percent of the time, so long miss runs
            // are NORMAL. The anchored grid keeps the pilots seated (drift
            // measured ~0), so log and retry next period instead of killing
            // the run.
            MLPD_WARN(
                "Re-sync: %zu misses this period for client %d (successes "
                "%zu); anchored grid keeps flying, retrying next period "
                "(exhausted streak %zu)\n",
                resync_retry_max, tid, resync_success,
                resync_exhausted_streak + 1);
            resync = false;
            resync_retry_cnt = 0;
            if (++resync_exhausted_streak >= kEscalateExhaustedEpisodes) {
              houdiniEscalate("episodes exhausted");
              continue;  // rx_beacon_time is pre-hunt; restart the frame loop
            }
          } else {
            // Iris/UHD path (on Houdini the anchor is always valid here:
            // acquisition either confirms or throws).
            MLPD_WARN(
                "Exceeded resync retry limit (%zu) for client %d reached "
                "after %zu resync successes at frame: %zu.  Stopping!\n",
                resync_retry_max, tid, resync_success, frame_id);
            resync = false;
            resync_retry_cnt = 0;
            config_->running(false);
            break;
          }
        }
      }
    }
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
        this->clientTxPilots(tid, pilot_base + txTimeDelta_);
      }
    }  // end if config_->ul_data_slot_present()

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
                                    long long& anchor_out,
                                    double* period_out) {
  constexpr long long kConfirmTol = 640;  // detector scatter (4.18) + path
  constexpr int kMaxHunts = 200;
  const long long fr = static_cast<long long>(config_->samps_per_frame());
  long long first_abs = 0;
  bool have_first = false;
  int confirms = 0;
  int hunts = 0;
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
      MLPD_INFO("houdiniAcquireAnchor [%d]: hunt lock at abs %lld (idx %ld)\n",
                tid, abs_end, idx);
      continue;
    }
    const long long k = llround(static_cast<double>(abs_end - first_abs) /
                                static_cast<double>(fr));
    const long long resid = abs_end - (first_abs + k * fr);
    if (k != 0 && std::llabs(resid) <= kConfirmTol) {
      if (++confirms >= 2) {
        anchor_out = first_abs - houdiniBeaconEnd(config_);
        // RATE BOOTSTRAP (AP-31b). resid/k IS the frame-period error: both
        // stamps are absolute UE sample times, so k counts REAL frames, and
        // resid is how far the beacon slipped across them. The confirm has
        // always computed it and always thrown it away.
        //
        // Why it decides whether the loop can close at all. The UE loop
        // iterates at ~205 frame_id/s against 1000 real frames/s (recvHoudini
        // drains), so resync_period 81 is ~395 ms of real time and the beacon
        // lands in the targeted window on ~1.4% of iterations -- about 2.8
        // attempts per second. On free-running clocks (-8.52 ppm, 1.047
        // samples per real frame) the +-kScatterTol budget is spent in 978 ms,
        // so the tracker gets ~2.8 chances to collect the >= 2 observations a
        // rate estimate needs, and normally collects zero. Seeded here to
        // within 1-5% (measured 1.056 / 1.027 / 1.000 against a true 1.047),
        // the residual rate error is <= 0.05 samples/frame and the same budget
        // lasts ~20 s instead of ~1 s, which is ~60 chances to refine.
        if (period_out != nullptr && k != 0) {
          *period_out = static_cast<double>(fr) +
                        static_cast<double>(resid) / static_cast<double>(k);
        }
        MLPD_INFO(
            "houdiniAcquireAnchor [%d]: lock CONFIRMED (resid %lld over "
            "%lld frames, confirm %d) -> frame anchor %lld, bootstrap period "
            "%.4f (%+.4f samp/frame)\n",
            tid, resid, k, confirms, anchor_out,
            static_cast<double>(fr) +
                static_cast<double>(resid) / static_cast<double>(k),
            static_cast<double>(resid) / static_cast<double>(k));
        return true;
      }
    } else {
      MLPD_INFO(
          "houdiniAcquireAnchor [%d]: confirm failed (resid %lld, k %lld) "
          "-> hunt restart\n",
          tid, resid, k);
      first_abs = abs_end;
      confirms = 0;
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

        // Acquisition: the strict threshold, and the earliest beacon copy refined
        // to the best index within it. Everything downstream is measured from this
        // index. NOTE this did NOT resolve the run-to-run constellation split it was
        // investigated for -- see the corr_scale_init note in config.cc.
        sync_index = syncSearch(syncbuffmem.at(kSyncDetectChannel).data(),
                                sample_window,
                                config_->corr_scale_init(radio_id),
                                /*refine_first_cluster=*/true);
        // SNR floor: a correlation crossing at noise level is the artifact
        // class, not the beacon (measured: real ~45 dB, artifacts ~0 dB).
        // Reject and keep hunting rather than anchor on it.
        if (config_->is_houdini() && sync_index >= 0) {
          const double snr =
              beaconSnrDb(syncbuffmem.at(kSyncDetectChannel).data(),
                          sample_window, sync_index, config_->beacon_size());
          if (snr < syncSnrFloorDb()) {
            static std::atomic<int> rej{0};
            const int nrej = rej.fetch_add(1);
            if ((nrej % 16) == 0) {
              MLPD_INFO(
                  "clientSyncBeacon [%zu]: rejected low-SNR detection "
                  "(idx %ld, %.1f dB < %.1f dB floor), count %d\n",
                  radio_id, sync_index, snr, syncSnrFloorDb(), nrej + 1);
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
