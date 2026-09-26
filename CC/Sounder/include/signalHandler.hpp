// http://www.yolinux.com/TUTORIALS/C++Signals.html

#ifndef __SIGNALHANDLER_H__
#define __SIGNALHANDLER_H__
#include <atomic>
#include <stdexcept>
using std::runtime_error;

class SignalException : public runtime_error {
 public:
  SignalException(const std::string& _message) : std::runtime_error(_message) {}
};

class SignalHandler {
 protected:
  // Written by the SIGINT handler and read by every thread: a lock-free atomic
  // is safe in a handler and visible across threads (a plain bool is neither).
  static std::atomic<bool> mbGotExitSignal;

 public:
  SignalHandler();
  ~SignalHandler();

  static bool gotExitSignal();
  static void setExitSignal(bool _bExitSignal);

  void setupSignalHandlers();
  static void exitSignalHandler(int _ignored);
};
#endif
