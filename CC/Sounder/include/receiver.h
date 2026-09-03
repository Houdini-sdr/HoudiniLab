/** @file receiver.h
  * @brief Declaration file for the Receiver class.
  * Copyright (c) 2018-2022, Rice University
  * RENEW OPEN SOURCE LICENSE: http://renew-wireless.org/license
  * ----------------------------------------------------------
  * Handles received samples from massive-mimo base station 
  *----------------------------------------------------------
*/
#ifndef DATARECEIVER_H_
#define DATARECEIVER_H_

#include <atomic>
#include <pthread.h>

#include <complex>
#include <stdexcept>
#include <string>
#include <memory>
#include <vector>


#if defined(USE_UHD)
#include "BaseRadioSetUHD.h"
#include "ClientRadioSetUHD.h"
#else
#include "BaseRadioSet.h"
#include "ClientRadioSet.h"
#endif
#include "concurrentqueue.h"
#include "config.h"
#include "sync/beacon_shape.h"
#include "sync/cfo_estimator.h"
#include "sync/confirm.h"
#include "sync/detector.h"
#include "sync_geometry.h"
#include "macros.h"

class ReceiverException : public std::runtime_error {
 public:
  ReceiverException()
      : std::runtime_error("Receiver could not be setup correctly!") {}
  explicit ReceiverException(const std::string& message)
      : std::runtime_error(message) {}
};

class Receiver {
 public:
  // use for create pthread
  struct ReceiverContext {
    Receiver* ptr;
    SampleBuffer* buffer;
    size_t core_id;
    size_t tid;
  };

 public:
  Receiver(Config* config, moodycamel::ConcurrentQueue<Event_data>* in_queue,
           std::vector<moodycamel::ConcurrentQueue<Event_data>*> tx_queue,
           std::vector<moodycamel::ProducerToken*> tx_ptoks,
           std::vector<moodycamel::ConcurrentQueue<Event_data>*> cl_tx_queue,
           std::vector<moodycamel::ProducerToken*> cl_tx_ptoks);
  ~Receiver();

  std::vector<pthread_t> startRecvThreads(SampleBuffer* rx_buffer,
                                          size_t n_rx_threads,
                                          SampleBuffer* tx_buffer,
                                          unsigned in_core_id = 0);
  void completeRecvThreads(const std::vector<pthread_t>& recv_thread);
  std::vector<pthread_t> startClientThreads(SampleBuffer* rx_buffer,
                                            SampleBuffer* tx_buffer,
                                            unsigned in_core_id = 0);
  void go();
  static void* loopRecv_launch(void* in_context);
  void loopRecv(int tid, int core_id, SampleBuffer* rx_buffer);
  void baseTxBeacon(int radio_id, int cell, int frame_id, long long base_time);
  int baseTxData(int radio_id, int cell, int frame_id, long long base_time);
  void notifyPacket(NodeType node_type, int frame_id, int slot_id, int ant_id,
                    int buff_size, int offset = 0);
  static void* clientTxRx_launch(void* in_context);
  void clientTxRx(int tid);
  void clientSyncTxRx(int tid, int core_id, SampleBuffer* rx_buffer);
  // `pick` is the crossing-selection rule, and the two callers need DIFFERENT
  // ones. Acquisition searches a wide window that can hold several beacon copies
  // 4096 samples apart, so it takes the earliest copy refined to its own peak --
  // repeatable across restarts, which the once-only pilot anchor depends on.
  // Re-sync searches a targeted lead+tail slice that cannot hold two copies, so
  // it takes the strongest crossing: there the earliest one is the beacon's own
  // lag-128 self-coherent STS preamble, hundreds of samples early, and whether
  // it wins depends on received level. See houdini::sync::PickRule.
  ssize_t syncSearch(const std::complex<int16_t>* check_data,
                     size_t search_window, float corr_scale,
                     houdini::sync::PickRule pick,
                     houdini::sync::Detection* detection = nullptr);

  // Two-stage beacon CFO estimate, normalized (cycles/sample); multiply by
  // the sample rate for Hz. Pointer form so both the vector-backed legacy
  // path and the targeted-resync path (raw rxbuff) can call it.
  float estimateCFO(const std::complex<int16_t>* buf, size_t buf_len,
                    int sync_index) const;
  void initBuffers();
  // frame_period: the TRACKED BS frame period in UE samples (AP-31c). The
  // horizon ladder steps by it, not by samps_per_frame; <= 0 means nominal.
  void clientTxPilots(size_t user_id, long long base_time,
                      double frame_period = 0.0);
  int clientTxData(int tid, int frame_id, long long base_time);
  ssize_t clientSyncBeacon(size_t radio_id, size_t sample_window,
                           long long* window_time = nullptr);
  // period_out, when non-null, receives the BS frame period MEASURED by the
  // confirm (in UE samples). The confirm already spans k real frames and knows
  // how far the beacon slipped over them, so the rate costs nothing extra --
  // see the bootstrap note at the definition.
  bool houdiniAcquireAnchor(int tid, size_t detect_window,
                            const Sounder::SyncGeometry& geom,
                            long long& anchor_out,
                            double* period_out = nullptr);
  void clientAdjustRx(size_t radio_id, size_t discard_samples);

 private:
  Config* config_;
  // The sync library objects, built once in the constructor from the
  // configured beacon shape and the sync block. Every beacon search, every
  // SNR confirm and every beacon carrier read in this class goes through them,
  // so a run has one detector, one guard and one estimator, all describable.
  std::unique_ptr<houdini::sync::Detector> sync_detector_;
  std::unique_ptr<houdini::sync::SnrWindowGuard> sync_guard_;
  std::unique_ptr<houdini::sync::RepetitionPhaseEstimator> cfo_estimator_;

#if defined(USE_UHD)
  ClientRadioSetUHD* client_radio_set_;
  BaseRadioSetUHD* base_radio_set_;
#else
  ClientRadioSet* client_radio_set_;
  BaseRadioSet* base_radio_set_;
#endif

  size_t thread_num_;
  // pointer of message_queue_
  moodycamel::ConcurrentQueue<Event_data>* message_queue_;
  std::vector<moodycamel::ConcurrentQueue<Event_data>*> tx_queue_;
  std::vector<moodycamel::ProducerToken*> tx_ptoks_;
  std::vector<moodycamel::ConcurrentQueue<Event_data>*> cl_tx_queue_;
  std::vector<moodycamel::ProducerToken*> cl_tx_ptoks_;

  // Data buffers
  SampleBuffer* cl_tx_buffer_;
  SampleBuffer* bs_tx_buffer_;
  std::vector<void*> pilotbuffA_;
  std::vector<void*> pilotbuffB_;
  std::vector<void*> ue_databuffA_;  // viewing-mode UE uplink-data slot (ch A)
  std::vector<void*> zeros_;
  size_t txTimeDelta_;
  size_t txFrameDelta_;
};

#endif  // DATARECEIVER_H_
