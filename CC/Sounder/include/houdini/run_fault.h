/**
 * @file houdini/run_fault.h
 * @brief The fault that ended a run, so the sounder exits with a failure.
 *
 * A loop that meets an error it cannot continue past (a stream the host plugin
 * ended, a thread stopped by an exception) stops the run through
 * Config::stopOnFault, which records the reason here. main() prints the reason
 * and exits with a failure status, where it used to exit with success because
 * the run had stopped on its own.
 */
#pragma once

#include <mutex>
#include <string>

namespace houdini {

/// The first fault reported, kept for the exit. Later reports are not kept: the
/// first fault is the cause and the rest are usually its consequences (both
/// nodes' threads stop once the run does). Safe to call from any thread.
class RunFault {
 public:
  /// Records `why` unless a fault is already recorded; true if this call did.
  bool record(const std::string& why) {
    std::lock_guard<std::mutex> lock(mu_);
    if (recorded_) return false;
    recorded_ = true;
    why_ = why;
    return true;
  }
  bool recorded() const {
    std::lock_guard<std::mutex> lock(mu_);
    return recorded_;
  }
  std::string reason() const {
    std::lock_guard<std::mutex> lock(mu_);
    return why_;
  }

 private:
  mutable std::mutex mu_;
  bool recorded_ = false;
  std::string why_;
};

}  // namespace houdini
