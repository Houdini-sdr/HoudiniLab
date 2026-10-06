/**
 * @file receiver_fault_test.cc
 * @brief How the receiver's threads end a run on an ended stream, with NO
 *        hardware: the real Receiver over fake radio sets.
 *
 * The Houdini host plugin ends a faulted stream, and RadioHoudini throws
 * houdini::stream::Ended from the read or write instead of returning
 * STREAM_ERROR (houdini/stream_result.h), so no loop can retry it. These checks
 * hold the receiver's side: the exception leaves every loop at its first call
 * (nothing swallows it), the thread's catch records the fault (so main exits
 * with a failure), and a negative CODE stays what it was (a TIMEOUT, or another
 * platform's STREAM_ERROR such as native UHD's late command, drops the round).
 *
 * This file replaces RadioSetFactory.cc in the build, so Receiver's
 * makeClientRadioSet / makeBaseRadioSet return the fakes below. A fake returns
 * the scripted code, or throws Ended for kEnded, and counts the calls; at kCap
 * calls it stops the run itself (the run's timer), so a loop that retries ends
 * the test with a count of kCap instead of hanging it. Each check names the
 * mutation that breaks it.
 *
 * Run from CC/Sounder (the config's relative path): ctest sets the working
 * directory.
 */
#include <SoapySDR/Errors.h>

#include <atomic>
#include <cstdio>
#include <memory>
#include <string>
#include <vector>

#include "include/RadioSetInterfaces.h"
#include "include/config.h"
#include "include/houdini/stream_result.h"
#include "include/macros.h"
#include "include/receiver.h"

namespace {
int g_fail = 0;
void check(bool ok, const std::string& what) {
  std::printf("%s  %s\n", ok ? "PASS" : "FAIL", what.c_str());
  if (!ok) ++g_fail;
}

constexpr int kCap = 200;
constexpr int kEnded = 1;  // a script value no stream call returns: the fake throws Ended, as RadioHoudini does

// What the fakes return, and what they saw. One test at a time.
struct Script {
  Config* cfg = nullptr;
  int rx = SOAPY_SDR_TIMEOUT;
  int tx = SOAPY_SDR_TIMEOUT;
  std::atomic<int> rx_calls{0}, tx_calls{0};
  void reset(Config* c, int rx_script, int tx_script) {
    cfg = c;
    rx = rx_script;
    tx = tx_script;
    rx_calls = 0;
    tx_calls = 0;
  }
  int onCall(std::atomic<int>& calls, int ret, const char* dir) {
    if (++calls >= kCap) cfg->running(false);  // the run's timer
    if (ret == kEnded) throw houdini::stream::Ended(std::string("fake radio: the ") + dir + " stream ended");
    return ret;
  }
} g;

class FakeClientSet : public IClientRadioSet {
 public:
  int triggers(int) override { return 0; }
  int radioRx(size_t, void* const*, int, long long& t) override {
    t = 0;
    return g.onCall(g.rx_calls, g.rx, "RX");
  }
  int radioTx(size_t, const void* const*, int, int, long long&) override { return g.onCall(g.tx_calls, g.tx, "TX"); }
  int drainTxStatus(size_t) override { return 0; }
  void radioStop() override {}
  bool getRadioNotFound() override { return false; }
};

class FakeBaseSet : public IBaseRadioSet {
 public:
  int radioTx(size_t, size_t, const void* const*, int, long long&) override { return g.onCall(g.tx_calls, g.tx, "TX"); }
  int radioRx(size_t, size_t, void* const*, long long& t) override {
    t = 0;
    return g.onCall(g.rx_calls, g.rx, "RX");
  }
  int radioRx(size_t, size_t, void* const*, int, long long& t) override {
    t = 0;
    return g.onCall(g.rx_calls, g.rx, "RX");
  }
  size_t lastRxPadSamples(size_t, size_t) const override { return 0; }
  void radioStart() override {}
  void radioStop() override {}
  bool getRadioNotFound() override { return false; }
};
}  // namespace

// The factory the Receiver calls (RadioSetFactory.cc's, replaced in this build).
std::unique_ptr<IBaseRadioSet> makeBaseRadioSet(Config*, bool) { return std::make_unique<FakeBaseSet>(); }
std::unique_ptr<IClientRadioSet> makeClientRadioSet(Config*) { return std::make_unique<FakeClientSet>(); }

namespace {
const char* kConf = "files/houdini-dualband-xw-steer-slots.json";

struct Rig {
  moodycamel::ConcurrentQueue<Event_data> q;
  std::unique_ptr<Receiver> rx;
  explicit Rig(Config* cfg) {
    rx = std::make_unique<Receiver>(cfg, &q, std::vector<moodycamel::ConcurrentQueue<Event_data>*>{},
                                    std::vector<moodycamel::ProducerToken*>{},
                                    std::vector<moodycamel::ConcurrentQueue<Event_data>*>{},
                                    std::vector<moodycamel::ProducerToken*>{});
  }
};

bool reasonHas(const Config& cfg, const std::string& s) { return cfg.faultReason().find(s) != std::string::npos; }

// Whether `call` lets Ended out, unswallowed.
template <class F>
bool throwsEnded(F call) {
  try {
    call();
  } catch (const houdini::stream::Ended&) {
    return true;
  }
  return false;
}

// A packet buffer as the scheduler allocates one: room for every slot of
// kSampleBufferFrameNum frames on `channels` channels, all free.
struct PacketBuffer {
  SampleBuffer buf;
  std::vector<std::atomic_int> inuse;
  PacketBuffer(const Config& cfg, size_t channels)
      : inuse(kSampleBufferFrameNum * cfg.slot_per_frame() * channels / sizeof(std::atomic_int) + 1) {
    const size_t packets = kSampleBufferFrameNum * cfg.slot_per_frame() * channels;
    buf.buffer.resize(packets * (sizeof(Packet) + cfg.getPacketDataLength()));
    for (auto& a : inuse) a = 0;
    buf.pkt_buf_inuse = inuse.data();
  }
};

// The BS receive loop (loopRecv, the hardware-framer branch the demo runs), on
// its own thread as the scheduler starts it.
void bsThread(int rx_script, bool expect_fault, const std::string& what) {
  Config cfg(kConf, "/tmp", true, false, false);
  Rig r(&cfg);
  g.reset(&cfg, rx_script, SOAPY_SDR_TIMEOUT);
  PacketBuffer rx(cfg, cfg.bs_rx_ch());
  r.rx->completeRecvThreads(r.rx->startRecvThreads(&rx.buf, 1, nullptr, 0));
  if (expect_fault) {
    check(g.rx_calls == 1 && cfg.faulted() && reasonHas(cfg, "BS receive thread 0") && reasonHas(cfg, "RX stream ended"),
          what + " (" + std::to_string(g.rx_calls) + " reads, fault '" + cfg.faultReason() + "')");
  } else {
    check(g.rx_calls == kCap && !cfg.faulted(), what + " (" + std::to_string(g.rx_calls) + " reads, fault '" +
                                                    cfg.faultReason() + "')");
  }
}

// The UE thread (clientSyncTxRx: acquisition first), as the scheduler starts it.
void ueThread() {
  Config cfg(kConf, "/tmp", false, true, false);
  Rig r(&cfg);
  g.reset(&cfg, kEnded, SOAPY_SDR_TIMEOUT);
  PacketBuffer rx(cfg, cfg.cl_sdr_ch()), tx(cfg, cfg.cl_tx_ch());  // one UE radio: thread 0's
  r.rx->completeRecvThreads(r.rx->startClientThreads(&rx.buf, &tx.buf, 0));
  check(g.rx_calls == 1 && cfg.faulted() && reasonHas(cfg, "UE thread 0") && reasonHas(cfg, "RX stream ended"),
        "UE thread: an ended stream stops the run at its first read with the fault (" + std::to_string(g.rx_calls) +
            " reads, fault '" + cfg.faultReason() + "'; catch it with running(false) and main exits clean)");
}

// The UE's loops, called directly: Ended leaves each at its first call.
void ueLoops() {
  {
    Config cfg(kConf, "/tmp", false, true, false);
    Rig r(&cfg);
    g.reset(&cfg, kEnded, SOAPY_SDR_TIMEOUT);
    const bool out = throwsEnded([&] { r.rx->clientSyncBeacon(0, cfg.samps_per_slot()); });
    check(out && g.rx_calls == 1, "UE beacon search: Ended leaves it at the first read (" +
                                      std::to_string(g.rx_calls) + " reads; a catch in the loop swallows it)");
  }
  {
    Config cfg(kConf, "/tmp", false, true, false);
    Rig r(&cfg);
    g.reset(&cfg, kEnded, SOAPY_SDR_TIMEOUT);
    const bool out = throwsEnded([&] { r.rx->clientAdjustRx(0, 1000); });
    check(out && g.rx_calls == 1, "UE realign discard: Ended leaves it at the first read (" +
                                      std::to_string(g.rx_calls) + " reads)");
  }
  {
    Config cfg(kConf, "/tmp", false, true, false);
    Rig r(&cfg);
    g.reset(&cfg, SOAPY_SDR_TIMEOUT, kEnded);
    const bool out = throwsEnded([&] { r.rx->clientTxPilots(0, 1000000000LL, 0.0); });
    check(out && g.tx_calls == 1, "UE pilots: Ended leaves the horizon at the first write (" +
                                      std::to_string(g.tx_calls) + " writes)");
  }
  {
    Config cfg(kConf, "/tmp", false, true, false);
    Rig r(&cfg);
    g.reset(&cfg, SOAPY_SDR_TIMEOUT, SOAPY_SDR_TIMEOUT);
    r.rx->clientSyncBeacon(0, cfg.samps_per_slot());
    check(g.rx_calls == kCap && !cfg.faulted(), "UE beacon search: a TIMEOUT is retried, not a fault (" +
                                                    std::to_string(g.rx_calls) + " reads)");
  }
}
}  // namespace

int main() {
  try {
    bsThread(kEnded, true,
             "BS receive thread: an ended stream stops the run at its first read with the fault (catch it with "
             "running(false) and main exits clean)");
    bsThread(SOAPY_SDR_TIMEOUT, false, "BS receive loop: a TIMEOUT drops the round and reads on, no fault");
    bsThread(SOAPY_SDR_STREAM_ERROR, false,
             "BS receive loop: a STREAM_ERROR code (another platform's, e.g. UHD's late command) drops the round, "
             "no fault (treat the code as ended in the loop and it stops at 1)");
    ueThread();
    ueLoops();
  } catch (const std::exception& e) {
    check(false, std::string("unexpected exception: ") + e.what());
  }
  std::printf("%s\n", g_fail == 0 ? "ALL PASS" : "FAILURES");
  return g_fail == 0 ? 0 : 1;
}
