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
#if defined(USE_UHD)
#include "include/ClientRadioSetUHD.h"
#else
#include "include/ClientRadioSet.h"
#endif
#include "include/comms-lib.h"
#include "include/logger.h"
#include "include/macros.h"
#include "include/node_version.h"
#include "include/sync_geometry.h"
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
  // strtod ACCEPTS "nan" and "inf" with end != e, so a typo or a stale export
  // reaches consumers that feed llround / static_cast<long long> (undefined
  // behaviour) or that use the value in a clamp, where a NaN makes every
  // comparison false and disables the guard with no symptom. Reject non-finite
  // once here rather than at each of the ten call sites.
  if (end == e || !std::isfinite(v)) {
    MLPD_WARN("%s=\"%s\" is not a finite number -- using the default %g\n",
              name, e, dflt);
    return dflt;
  }
  return v;
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
// EVERY FAILURE PATH RETURNS NaN, NOT ZERO. A failed estimate is not a
// measurement of zero offset, and the caller cannot tell the two apart from a
// float. That mattered only for a log line while this value was print-only; it
// now rides SYN1 as the independent beacon reading, where a fabricated 0 Hz
// would be averaged into the panel's beacon figure and pull it toward zero with
// no symptom. NaN propagates: the log prints "nan", the wire field is dropped
// by the page rather than plotted.
float Receiver::estimateCFO(const std::complex<int16_t>* buf, size_t buf_len,
                            int sync_index) const {
  const float kNoEstimate = std::numeric_limits<float>::quiet_NaN();
  if (buf == nullptr) return kNoEstimate;
  // Geometry guard: this estimator is tied to the STS+gold layout above. If the
  // beacon is ever rebuilt to another shape, fail rather than silently return a
  // wrong frequency that a correction loop would then act on.
  if (static_cast<int>(config_->beacon_size()) != kBeaconCoreLen) {
    static std::atomic<bool> warned{false};
    if (warned.exchange(true) == false) {
      MLPD_WARN(
          "estimateCFO: beacon is %d samples, expected %d (%d x STS(%d) + "
          "%d x gold(%d)) -- CFO estimation disabled\n",
          static_cast<int>(config_->beacon_size()), kBeaconCoreLen, kStsReps,
          kStsLen, kGoldReps, kGoldLen);
    }
    return kNoEstimate;
  }
  const int start = sync_index - kBeaconCoreLen;
  if (start < 0 || sync_index < 0 ||
      static_cast<size_t>(sync_index) > buf_len) {
    return kNoEstimate;
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
  if (std::abs(r_fine) == 0.0 || std::abs(r_coarse) == 0.0) return kNoEstimate;

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
  // ACQUISITION window = one whole frame. The beacon repeats every
  // samps_per_frame, so a full-frame read contains one with probability 1
  // (bar the ~0.4% that straddle the boundary, which simply retries at a new
  // phase) instead of the 7.4% a 2.33-slot window gets. Measured costs put the
  // optimum exactly here: expected time = F*a_read/W + F*(b_read+b_corr), so it
  // falls with W until the hit probability saturates at W = F, and reading
  // beyond a frame buys nothing. 15.5 ms -> 4.5 ms, and deterministic.
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
  const double kScatterTolUs = envDouble("HOUDINI_SCATTER_TOL_US", 8.3333);
  // The same argument that made kScatterTol a TIME (AP-40): what the
  // acquisition gate admits is detector scatter plus path, both properties of
  // the correlator and the cable measured in microseconds, so a fixed sample
  // count silently retunes it at every rate while the tracking gate scales.
  // 5.2083 us reproduces the old 640 samples exactly at 122.88 MSPS.
  const double kConfirmTolUs = envDouble("HOUDINI_CONFIRM_TOL_US", 5.2083);
  double sync_tol_samples =
      envDouble("HOUDINI_SYNC_TOL_SAMPLES",
                static_cast<double>(config_->prefix()) / 4.0);
  double sync_residual_ppm = envDouble("HOUDINI_SYNC_RESIDUAL_PPM", 1.0);
  // Both inputs are validated: a zero or negative ppm makes the cadence
  // quotient infinite, and a config without `ofdm_tx_zero_prefix` gives a zero
  // tolerance and a resync attempt every single frame. Neither should degrade
  // quietly. (envDouble already rejects NaN and inf, AP-54.)
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
  const Sounder::SyncGeometry geom = Sounder::computeSyncGeometry(
      config_->rate(), static_cast<long long>(config_->samps_per_slot()),
      static_cast<long long>(config_->samps_per_frame()), kScatterTolUs,
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
  if (!geom.usable) {
    MLPD_ERROR(
        "beacon accept window is %lld samples: the UE will never attempt a "
        "resync and will fly open loop with no telemetry. Lower "
        "HOUDINI_SCATTER_TOL_US.\n",
        geom.accept_window);
  } else {
    MLPD_INFO(
        "Beacon accept window %lld samples of a %zu-sample slot (%.1f%%), "
        "scatter tol %lld samples = %.2f us, confirm tol %lld samples\n",
        geom.accept_window, config_->samps_per_slot(),
        100.0 * geom.accept_window_frac, kScatterTol,
        kScatterTol / config_->rate() * 1e6, geom.confirm_tol);
  }
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
  const double kGridAlpha = envDouble("HOUDINI_GRID_ALPHA", 0.5);
  const double kGridBeta = envDouble("HOUDINI_GRID_BETA", 0.1);
  // Per-update slew limit on the rate estimate, distinct from the absolute
  // plausibility band further down: see the note at the update site for why
  // that band alone cannot reject an outlier.
  const double kGridStepPpm = envDouble("HOUDINI_GRID_STEP_PPM", 0.5);
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
  size_t last_resync = frame_id;
  auto last_resync_tp = std::chrono::steady_clock::now();
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
  const double kGridMaxPpm = envDouble("HOUDINI_GRID_MAX_PPM", 100.0);
  const double kGridTrustPpm = envDouble("HOUDINI_GRID_TRUST_PPM", 1.0);
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
  size_t houdini_grid_starved = 0;  // in-gate accepts with kf == 0 (no update)
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
      const double disagree_ppm =
          std::fabs(fresh_period - houdini_frame_period) /
          static_cast<double>(config_->samps_per_frame()) * 1e6;
      // Clamp the confirm's value the same way the incremental update is
      // clamped: houdiniAcquireAnchor has no band of its own, so without this
      // the two wholesale writes bypass the only guard in the system.
      fresh_period = std::min(kGridPeriodHi, std::max(kGridPeriodLo, fresh_period));
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
    last_resync_tp = std::chrono::steady_clock::now();
  };

  while (config_->running() == true) {
    if (config_->max_frame() > 0 && frame_id >= config_->max_frame()) {
      config_->running(false);
      break;
    }
    //Slot 0 / Beacon...
    const auto prof_t0 = profile_clock::now();
    const int request_samples = samples_per_slot - beacon_adjust;
    const int rx_status = client_radio_set_->radioRx(
        tid, rxbuff.data(), request_samples, rx_beacon_time);
    beacon_adjust = 0;
    const auto prof_t1 = profile_clock::now();
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

    if (config_->is_houdini()) {
      const auto resync_now = std::chrono::steady_clock::now();
      if (std::chrono::duration<double>(resync_now - last_resync_tp).count() >=
          resync_interval_s) {
        resync = resync_enable;
        last_resync_tp = resync_now;
        MLPD_TRACE("Enable resyncing at frame %zu\n", frame_id);
      }
    } else if ((frame_id - last_resync) >= resync_period) {
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
              static_cast<long long>(envDouble("HOUDINI_CFO_INDEX_GUARD", 8.0));
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
              cfo_index < static_cast<long long>(kBeaconCoreLen)) {
            cfo_index = sync_index;  // fall back to the detector's own index
          }
          const float cfo_norm =
              estimateCFO(reinterpret_cast<std::complex<int16_t>*>(
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
          //     hardware-clock ratio on every leg to <= 0.05 ppm, and the
          //     tracked residual is 0.036 ppm = ~18 Hz at 500 MHz.
          // Both are logged so the disagreement stays visible rather than
          // becoming folklore; AP-34(b) is the fix for the estimator itself.
          const double eps_tracked =
              (houdini_frame_period > 0.0)
                  ? (static_cast<double>(config_->samps_per_frame()) /
                         houdini_frame_period - 1.0)
                  : 0.0;
          const double cfo_tracked_hz = eps_tracked * config_->freq();
          if ((cfo_log_cnt++ % cfoLogEvery()) == 0) {
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
            if (kf > 0) {
              const double dk = static_cast<double>(kf);
              const double r = static_cast<double>(resid);
              // This IS a schedule move, up to kGridAlpha * kScatterTol = 512
              // samples in one step, and it used to be reported as shift = 0
              // while only the escalation's move was shown. Carry it out to
              // emitSync so the panel sees every move the UE makes.
              applied_shift = llround(kGridAlpha * r);
              houdini_pilot_ref = houdiniGridStart(kf) + applied_shift;
              // The absolute band below is a PLAUSIBILITY bound and cannot
              // serve as the outlier reject: it admits a single edge-of-gate
              // detection kicking the rate by kGridBeta * kScatterTol /
              // (the cadence in real frames) ~ 3.2 ppm at the shipped
              // settings, a third of the real 8.5 ppm offset
              // and ~30x looser than the kick its own note describes. Bound
              // how far ONE observation may move the estimate. The default is
              // 14x the measured 0.036 ppm tracked residual, so a normal
              // update (~0.003 ppm) is untouched and only an outlier is
              // trimmed; convergence from the acquisition confirm, itself good
              // to ~0.04 ppm, is unaffected.
              const double step_lim =
                  kGridStepPpm * 1e-6 *
                  static_cast<double>(config_->samps_per_frame());
              const double dp = std::min(
                  step_lim, std::max(-step_lim, kGridBeta * r / dk));
              houdini_frame_period += dp;
              // Clamp to a plausible oscillator band. The gate that feeds this
              // accepts |resid| <= kScatterTol = 1024, and the soonest an
              // update follows the previous is one resync interval, so ONE
              // edge-of-gate detection can kick the rate by
              // beta*1024/260 = 0.39 samp/frame ~ 3.2 ppm -- a third again the
              // real 8.5 ppm offset, and enough to walk the grid back out of
              // the gate. kScatterTol is far too wide to serve as the outlier
              // reject on its own at these spacings.
              houdini_frame_period =
                  std::min(kGridPeriodHi, std::max(kGridPeriodLo,
                                                   houdini_frame_period));
              houdini_grid_updates++;
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
            resync_hold_pending = false;
            resync_exhausted_streak = 0;
            resync = false;
            resync_retry_cnt = 0;
            resync_success++;
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
                frame_id, resid, snr, resync_hold_pending ? 1 : 0, tid);
            emitSync(kSyncHold, resid, cfo_tracked_hz, snr, 0, cfo_hz);
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
    const auto prof_t2 = profile_clock::now();
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
    const auto prof_t3 = profile_clock::now();

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
        rx_data_status = (whole == run) ? static_cast<int>(samples_per_slot)
                                        : got;
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
        if (rem != 0) {
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
                                    long long& anchor_out,
                                    double* period_out) {
  // Detector scatter (ledger 4.18) plus path, expressed in TIME so it scales
  // with the rate the way the tracking gate does. Derived in sync_geometry.h,
  // which is where the whole rate ladder is checked (AP-56).
  const long long kConfirmTol =
      Sounder::computeSyncGeometry(
          config_->rate(), static_cast<long long>(config_->samps_per_slot()),
          static_cast<long long>(config_->samps_per_frame()), 8.3333,
          envDouble("HOUDINI_CONFIRM_TOL_US", 5.2083), 32.0, 1.0)
          .confirm_tol;
  // The refine stage wants a LONG baseline, because the rate error is the
  // detection-pair noise divided by the span and that noise is sub-sample
  // (measured 0.15-0.94 across four acquisitions). k ~ 20 gives ~4% rate
  // error; k >= 200 gives ~0.4%. The stage exists because a full-frame
  // detect window would otherwise SHORTEN the span: today's k of 17-37 is an
  // accident of the hunt needing ~13.6 windowed reads to find the beacon at
  // all, and a guaranteed first hit removes exactly that accident.
  const long long kRefineSpan = static_cast<long long>(
      envDouble("HOUDINI_ACQ_REFINE_SPAN", 200.0));
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
  const double kAcqMaxPpm = envDouble("HOUDINI_ACQ_MAX_PPM", 100.0);
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
        anchor_out = first_abs - houdiniBeaconEnd(config_);
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
