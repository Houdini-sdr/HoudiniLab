/*
 Copyright (c) 2018-2026, Rice University 
 RENEW OPEN SOURCE LICENSE: http://renew-wireless.org/license

---------------------------------------------------------------------
 Event based message queue thread class for the recorder worker
---------------------------------------------------------------------
*/

#include "include/recorder_thread.h"

#include "include/logger.h"
#include "include/macros.h"
#include "include/utils.h"

namespace Sounder {
RecorderThread::RecorderThread(std::unique_ptr<RecorderWorkerInterface> worker,
                               size_t packet_data_length, size_t thread_id,
                               int core, size_t queue_size, bool wait_signal)
    : event_queue_(queue_size),
      producer_token_(event_queue_),
      worker_(std::move(worker)),
      thread_(),
      id_(thread_id),
      packet_data_length_(packet_data_length),
      core_alloc_(core),
      wait_signal_(wait_signal) {
  worker_->init();
  running_ = false;
}

RecorderThread::~RecorderThread() { Finalize(); }

//Launching thread in seperate function to guarantee that the object is fully constructed
//before calling member function
void RecorderThread::Start(void) {
  MLPD_INFO("Launching recorder task thread with id: %zu and core %d\n",
            this->id_, this->core_alloc_);
  {
    std::lock_guard<std::mutex> thread_lock(this->sync_);
    this->thread_ = std::thread(&RecorderThread::DoRecording, this);
    this->running_ = true;
  }
  this->condition_.notify_all();
}

/* Cleanly allows the thread to exit */
void RecorderThread::Stop(void) {
  Event_data event;
  event.event_type = kThreadTermination;
  this->DispatchWork(event);
}

void RecorderThread::Finalize(void) {
  //Wait for thread to cleanly finish the messages in the queue
  if (this->thread_.joinable() == true) {
    MLPD_TRACE("Joining Recorder Thread on CPU %d \n", sched_getcpu());
    this->Stop();
    this->thread_.join();
  }
}

/* TODO:  handle producer token better */
//Returns true for success, false otherwise
bool RecorderThread::DispatchWork(Event_data event) {
  //MLPD_TRACE("Dispatching work\n");
  bool ret = true;
  if (this->event_queue_.try_enqueue(this->producer_token_, event) == 0) {
    MLPD_WARN("Queue limit has reached! try to increase queue size.\n");
    if (this->event_queue_.enqueue(this->producer_token_, event) == 0) {
      // Report failure instead of throwing: callers own the policy (the
      // scheduler throws; rx-recorder aborts gracefully), and a throw here
      // would terminate() when Stop() dispatches during stack unwinding.
      MLPD_ERROR("Record task enqueue failed\n");
      ret = false;
    }
  }

  if (this->wait_signal_ == true) {
    if (ret == true) {
      std::lock_guard<std::mutex> thread_lock(this->sync_);
    }
    this->condition_.notify_all();
  }
  return ret;
}

void RecorderThread::DoRecording(void) {
  //Sync the start
  {
    std::unique_lock<std::mutex> thread_wait(this->sync_);
    this->condition_.wait(thread_wait, [this] { return this->running_; });
  }

  if (this->core_alloc_ >= 0) {
    MLPD_INFO("Pinning recording thread %zu to core %d\n", this->id_,
              this->core_alloc_);
    pthread_t this_thread = this->thread_.native_handle();
    if (pin_thread_to_core(this->core_alloc_, this_thread) != 0) {
      MLPD_ERROR("Pin recording thread %zu to core %d failed\n", this->id_,
                 this->core_alloc_);
      throw std::runtime_error("Pin recording thread to core failed");
    }
  }

  moodycamel::ConsumerToken ctok(this->event_queue_);
  MLPD_INFO("Recording thread %zu has %zu antennas starting at %zu\n",
            this->id_, this->worker_->num_antennas(),
            this->worker_->antenna_offset());

  Event_data event;
  bool ret = false;
  while (this->running_ == true) {
    ret = this->event_queue_.try_dequeue(ctok, event);

    if (ret == false) /* Queue empty */
    {
      if (this->wait_signal_ == true) {
        std::unique_lock<std::mutex> thread_wait(this->sync_);
        /* Wait until a new message exists, should eliminate the CPU polling */
        this->condition_.wait(thread_wait, [this, &ctok, &event] {
          return this->event_queue_.try_dequeue(ctok, event);
        });
        /* return from here with a valid event to process */
        ret = true;
      }
    }

    if (ret == true) {
      this->HandleEvent(event);
    }
  }
  this->worker_->finalize();
}

void RecorderThread::HandleEvent(Event_data event) {
  if (event.event_type == kThreadTermination) {
    this->running_ = false;
  } else {
    size_t buffer_id = (event.offset / event.buff_size);
    size_t buffer_offset = event.offset - (buffer_id * event.buff_size);
    if (event.event_type == kTaskRecord) {
      // read info
      size_t packet_length = sizeof(Packet) + this->packet_data_length_;
      char* cur_ptr_buffer = event.buffer[buffer_id].buffer.data() +
                             (buffer_offset * packet_length);

      this->worker_->record(this->id_,
                            reinterpret_cast<Packet*>(cur_ptr_buffer),
                            event.node_type);
      /* Free up the buffer memory */
      sample_buf_release(event.buffer[buffer_id].pkt_buf_inuse, buffer_offset);
    }
  }
}
};  //End namespace Sounder
