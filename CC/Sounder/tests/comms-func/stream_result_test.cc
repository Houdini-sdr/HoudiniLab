// The Houdini stream-result rules (houdini/stream_result.h) and the run's fault
// record (houdini/run_fault.h). Each assertion names the mutation that breaks it.
#include <SoapySDR/Errors.h>

#include <atomic>
#include <stdexcept>
#include <cstdio>
#include <string>
#include <thread>
#include <vector>

#include "houdini/run_fault.h"
#include "houdini/stream_result.h"

static int failures = 0;
static void check(bool ok, const char* what) {
  std::printf("%s  %s\n", ok ? "PASS" : "FAIL", what);
  if (!ok) ++failures;
}

int main() {
  using houdini::stream::ended;
  using houdini::stream::mergeWrite;

  check(ended(SOAPY_SDR_STREAM_ERROR), "STREAM_ERROR is an ended stream");
  check(!ended(SOAPY_SDR_TIMEOUT), "TIMEOUT is not (an empty ring on a healthy stream; end on it and every idle read ends the run)");
  check(!ended(SOAPY_SDR_OVERFLOW) && !ended(SOAPY_SDR_TIME_ERROR) && !ended(SOAPY_SDR_CORRUPTION),
        "the other error codes are not (ended() taking any negative breaks this)");
  check(!ended(0) && !ended(2032), "a count is not");

  // RadioHoudini's conversion: an ended stream throws, anything else returns.
  {
    using houdini::stream::Ended;
    using houdini::stream::throwIfEnded;
    std::string what;
    try {
      throwIfEnded(SOAPY_SDR_STREAM_ERROR, "UE x: the RX stream ended");
    } catch (const Ended& e) {
      what = e.what();
    }
    check(what == "UE x: the RX stream ended", "STREAM_ERROR throws Ended with the radio's text");
    bool threw = false;
    try {
      throwIfEnded(SOAPY_SDR_TIMEOUT, "x");
      throwIfEnded(SOAPY_SDR_OVERFLOW, "x");
      throwIfEnded(0, "x");
      throwIfEnded(4096, "x");
    } catch (...) {
      threw = true;
    }
    check(!threw, "TIMEOUT, OVERFLOW, 0 and a count return (throwing on any negative breaks this)");
    bool is_runtime = false;
    try {
      throwIfEnded(SOAPY_SDR_STREAM_ERROR, "x");
    } catch (const std::runtime_error&) {
      is_runtime = true;
    }
    check(is_runtime, "Ended is a std::runtime_error, so the receiver threads' catch takes it");
  }

  // Per-channel TX writes: S whole, then each channel's result in order.
  const int S = 4096;
  auto merge = [S](std::vector<int> rs) {
    int ret = S;
    for (int r : rs) ret = mergeWrite(ret, r, S);
    return ret;
  };
  check(merge({S, S}) == S, "two whole writes merge to whole");
  check(merge({S, 100}) == 100, "a short write is reported (the caller's BAD Write check needs it)");
  check(merge({100, SOAPY_SDR_TIMEOUT}) == 100, "the FIRST failure is kept, as before (keeping the last breaks this)");
  check(merge({SOAPY_SDR_TIMEOUT, SOAPY_SDR_STREAM_ERROR}) == SOAPY_SDR_STREAM_ERROR,
        "an ended stream on a later channel wins over an earlier failure (first-failure-only masks it)");
  check(merge({SOAPY_SDR_STREAM_ERROR, 100}) == SOAPY_SDR_STREAM_ERROR,
        "an ended stream is not replaced by a later failure");
  check(merge({S, SOAPY_SDR_STREAM_ERROR, S}) == SOAPY_SDR_STREAM_ERROR,
        "a whole write after an ended stream does not clear it");

  // The fault record: the first reason is kept, from any thread.
  {
    houdini::RunFault f;
    check(!f.recorded() && f.reason().empty(), "a new record holds no fault");
    check(f.record("first"), "the first report is recorded");
    check(!f.record("second") && f.reason() == "first",
          "a later report does not replace it (the first fault is the cause; keeping the last breaks this)");
    check(f.recorded(), "it stays recorded");
  }
  {
    houdini::RunFault f;
    std::atomic<int> won{0};
    std::vector<std::thread> ts;
    for (int i = 0; i < 8; ++i)
      ts.emplace_back([&f, &won, i] {
        if (f.record("thread " + std::to_string(i))) ++won;
      });
    for (auto& t : ts) t.join();
    check(won == 1 && f.reason().rfind("thread ", 0) == 0,
          "eight threads reporting at once: exactly one is recorded (an unlocked check-then-set lets two through)");
  }

  std::printf("%s\n", failures == 0 ? "ALL PASS" : "FAILURES");
  return failures == 0 ? 0 : 1;
}
