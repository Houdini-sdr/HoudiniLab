// Radio-free unit test of the fillSlot slot-assembly state machine
// (include/rx_recorder_capture.h), driven by a scripted SampleSource.
// Covers the shapes of both device read paths: large chunked spans with a
// leading stamp (readStream) and small per-packet spans with per-packet
// stamps (the SH-254 direct-buffer path), plus the failure modes: forward
// gaps (exact and read-widened extents), backward stamps, time-base
// resyncs, timeout/overflow abort thresholds, and mid-slot interruption.
#include <atomic>
#include <cstdio>
#include <cstdlib>
#include <vector>

#include "include/rx_recorder_capture.h"

#define CHECK(cond)                                                        \
  do {                                                                     \
    if (!(cond)) {                                                         \
      std::fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
      return 1;                                                            \
    }                                                                      \
  } while (0)

namespace {

constexpr double kRate = 1e6;  // 1 sample = 1000 ns: easy stamp math
constexpr long long kT0 = 5'000'000'000LL;

inline long long stampAt(int64_t sample) {
  return kT0 + Sounder::sampleToNs(sample, kRate);
}

struct Ev {
  Sounder::FetchStatus st;
  size_t n = 0;  // span samples (kData only)
  bool has_time = false;
  long long time_ns = 0;
  short seed = 0;  // payload: interleaved short k of this span = seed + k
};

Ev dataEv(size_t n, long long time_ns, short seed) {
  return {Sounder::FetchStatus::kData, n, true, time_ns, seed};
}

Ev unstampedEv(size_t n, short seed) {
  return {Sounder::FetchStatus::kData, n, false, 0, seed};
}

// Replays a fixed fetch script; starves with kTimeout when it runs out.
// consume() past the span end aborts the test immediately.
class ScriptSource : public Sounder::SampleSource {
 public:
  explicit ScriptSource(std::vector<Ev> evs) : evs_(std::move(evs)) {}

  Sounder::FetchStatus fetch(void) override {
    if (pending() != 0) {
      std::fprintf(stderr, "FATAL: fetch() with %zu samples pending\n",
                   pending());
      std::exit(1);
    }
    if (idx_ >= evs_.size()) {
      return Sounder::FetchStatus::kTimeout;
    }
    const Ev& e = evs_[idx_++];
    if (e.st != Sounder::FetchStatus::kData) {
      return e.st;
    }
    buf_.resize(2 * e.n);
    for (size_t k = 0; k < 2 * e.n; k++) {
      buf_[k] = static_cast<short>(e.seed + static_cast<short>(k));
    }
    len_ = e.n;
    off_ = 0;
    has_time_ = e.has_time;
    time_ns_ = e.time_ns;
    return Sounder::FetchStatus::kData;
  }

  size_t pending(void) const override { return len_ - off_; }
  const short* data(void) const override { return buf_.data() + (2 * off_); }
  void consume(size_t samples) override {
    if (samples > pending()) {
      std::fprintf(stderr, "FATAL: consume(%zu) with %zu pending\n", samples,
                   pending());
      std::exit(1);
    }
    off_ += samples;
  }
  bool has_time(void) const override { return has_time_; }
  long long time_ns(void) const override { return time_ns_; }

 private:
  std::vector<Ev> evs_;
  size_t idx_ = 0;
  std::vector<short> buf_;
  size_t len_ = 0;
  size_t off_ = 0;
  bool has_time_ = false;
  long long time_ns_ = 0;
};

bool neverInterrupted(void) { return false; }

// Emits `slots` slots through fillSlot; returns how many filled completely.
size_t runSlots(Sounder::SampleSource& src, Sounder::CaptureState& state,
                std::vector<short>& out, size_t slot_samps, size_t slots,
                std::atomic<bool>& running,
                const std::function<bool(void)>& interrupt =
                    neverInterrupted) {
  out.assign(2 * slot_samps * slots, static_cast<short>(-1));
  for (size_t s = 0; s < slots; s++) {
    if (Sounder::fillSlot(src, out.data() + (2 * slot_samps * s), slot_samps,
                          state, running, interrupt) == false) {
      return s;
    }
  }
  return slots;
}

// The expected interleaved-short stream: spans in fetch order with pads
// (zeros) inserted where the grid detected gaps.
void appendSpan(std::vector<short>& exp, size_t n, short seed) {
  for (size_t k = 0; k < 2 * n; k++) {
    exp.push_back(static_cast<short>(seed + static_cast<short>(k)));
  }
}

void appendZeros(std::vector<short>& exp, size_t n) {
  exp.insert(exp.end(), 2 * n, 0);
}

int testCleanSpansAcrossSlots(void) {
  // Three contiguous stamped spans (40+40+48) -> exactly two 64-sample
  // slots; span 2 splits 24/16 across the slot boundary.
  ScriptSource src({dataEv(40, stampAt(0), 100), dataEv(40, stampAt(40), 200),
                    dataEv(48, stampAt(80), 300)});
  Sounder::CaptureState state(kRate);
  state.exact_gaps = true;
  std::atomic<bool> running(true);
  std::vector<short> out;
  CHECK(runSlots(src, state, out, 64, 2, running) == 2);

  std::vector<short> exp;
  appendSpan(exp, 40, 100);
  appendSpan(exp, 40, 200);
  appendSpan(exp, 48, 300);
  CHECK(out == exp);
  CHECK(state.grid_pos == 128);
  CHECK(state.extents.empty());
  CHECK(state.stats.gap_events == 0);
  CHECK(state.stats.has_first_sample_time == true);
  CHECK(state.stats.first_sample_time_ns == kT0);
  CHECK(running.load() == true);
  std::printf("PASS clean spans across slots\n");
  return 0;
}

int testForwardGapExactExtents(void) {
  // 64 samples, then a 10-sample drop, then 54: slot 2 opens with 10
  // placeholder zeros and the extent lands exactly at the gap.
  ScriptSource src({dataEv(64, stampAt(0), 100), dataEv(54, stampAt(74), 200)});
  Sounder::CaptureState state(kRate);
  state.exact_gaps = true;
  std::atomic<bool> running(true);
  std::vector<short> out;
  CHECK(runSlots(src, state, out, 64, 2, running) == 2);

  std::vector<short> exp;
  appendSpan(exp, 64, 100);
  appendZeros(exp, 10);
  appendSpan(exp, 54, 200);
  CHECK(out == exp);
  CHECK(state.stats.gap_events == 1);
  CHECK(state.stats.est_lost_samples == 10.0);
  CHECK(state.extents.size() == 1);
  CHECK(state.extents[0].start_sample == 64);
  CHECK(state.extents[0].n_samples == 10);
  CHECK(state.extents[0].cause == Sounder::kGapTimeJump);
  std::printf("PASS forward gap, exact extents\n");
  return 0;
}

int testForwardGapWidenedExtents(void) {
  // Same stream without the contiguity guarantee: the extent widens
  // backward over the previous 64-sample fetch (start 0, length 74).
  ScriptSource src({dataEv(64, stampAt(0), 100), dataEv(54, stampAt(74), 200)});
  Sounder::CaptureState state(kRate);
  state.exact_gaps = false;
  std::atomic<bool> running(true);
  std::vector<short> out;
  CHECK(runSlots(src, state, out, 64, 2, running) == 2);

  CHECK(state.extents.size() == 1);
  CHECK(state.extents[0].start_sample == 0);
  CHECK(state.extents[0].n_samples == 74);
  CHECK(state.extents[0].cause == Sounder::kGapTimeJump);
  std::printf("PASS forward gap, read-widened extents\n");
  return 0;
}

int testGapSpanningSlots(void) {
  // A 100-sample drop with 64-sample slots: the pads cover the rest of
  // slot 2 and open slot 3. 64 + 100 + 28 = 3 slots exactly.
  ScriptSource src({dataEv(64, stampAt(0), 100),
                    dataEv(28, stampAt(164), 200)});
  Sounder::CaptureState state(kRate);
  state.exact_gaps = true;
  std::atomic<bool> running(true);
  std::vector<short> out;
  CHECK(runSlots(src, state, out, 64, 3, running) == 3);

  std::vector<short> exp;
  appendSpan(exp, 64, 100);
  appendZeros(exp, 100);
  appendSpan(exp, 28, 200);
  CHECK(out == exp);
  CHECK(state.extents.size() == 1);
  CHECK(state.extents[0].start_sample == 64);
  CHECK(state.extents[0].n_samples == 100);
  CHECK(state.pad_remaining == 0);
  std::printf("PASS gap spanning slot boundaries\n");
  return 0;
}

int testDirectStylePerPacketSpans(void) {
  // Direct-path shape: 8-sample packets, each with its own stamp; packet
  // 4 of 9 is lost. The pad lands exactly between packets with no
  // widening -- 8 packets * 8 + 8 pads = 72... use slot 36 -> 2 slots.
  std::vector<Ev> evs;
  int64_t pos = 0;
  short seed = 0;
  for (int pkt = 0; pkt < 9; pkt++) {
    if (pkt == 4) {
      pos += 8;  // dropped on the wire: no event, stamps jump 8
      continue;
    }
    evs.push_back(dataEv(8, stampAt(pos), seed));
    pos += 8;
    seed = static_cast<short>(seed + 1000);
  }
  ScriptSource src(evs);
  Sounder::CaptureState state(kRate);
  state.exact_gaps = true;
  std::atomic<bool> running(true);
  std::vector<short> out;
  CHECK(runSlots(src, state, out, 36, 2, running) == 2);

  std::vector<short> exp;
  seed = 0;
  for (int pkt = 0; pkt < 9; pkt++) {
    if (pkt == 4) {
      appendZeros(exp, 8);
      continue;
    }
    appendSpan(exp, 8, seed);
    seed = static_cast<short>(seed + 1000);
  }
  CHECK(out == exp);
  CHECK(state.stats.gap_events == 1);
  CHECK(state.extents.size() == 1);
  CHECK(state.extents[0].start_sample == 32);
  CHECK(state.extents[0].n_samples == 8);
  std::printf("PASS direct-style per-packet spans\n");
  return 0;
}

int testBackwardStamp(void) {
  // A stamp 5 samples early (beyond the 1-sample tolerance at 1 MSPS):
  // flagged, never padded or subtracted -- samples emit where they fall.
  ScriptSource src({dataEv(64, stampAt(0), 100),
                    dataEv(64, stampAt(64 - 5), 200)});
  Sounder::CaptureState state(kRate);
  state.exact_gaps = true;
  std::atomic<bool> running(true);
  std::vector<short> out;
  CHECK(runSlots(src, state, out, 64, 2, running) == 2);

  std::vector<short> exp;
  appendSpan(exp, 64, 100);
  appendSpan(exp, 64, 200);
  CHECK(out == exp);
  CHECK(state.stats.backward_time_jumps == 1);
  CHECK(state.extents.size() == 1);
  CHECK(state.extents[0].start_sample == 64);
  CHECK(state.extents[0].n_samples == 0);
  CHECK(state.extents[0].cause == Sounder::kGapBackward);
  std::printf("PASS backward stamp flagged, not padded\n");
  return 0;
}

int testTimebaseResync(void) {
  // A stamp > kMaxGapSeconds off the grid re-anchors instead of
  // zero-filling; the marker records where, and later gaps are measured
  // against the NEW anchor.
  const long long jump = stampAt(64) + 20'000'000'000LL;
  ScriptSource src({dataEv(64, stampAt(0), 100), dataEv(32, jump, 200),
                    dataEv(32, jump + Sounder::sampleToNs(32 + 6, kRate), 300)});
  Sounder::CaptureState state(kRate);
  state.exact_gaps = true;
  std::atomic<bool> running(true);
  std::vector<short> out;
  CHECK(runSlots(src, state, out, 64, 2, running) == 2);

  CHECK(state.stats.time_resyncs == 1);
  CHECK(state.stats.gap_events == 1);  // the 6-sample gap after re-anchor
  CHECK(state.extents.size() == 2);
  CHECK(state.extents[0].start_sample == 64);
  CHECK(state.extents[0].n_samples == 0);
  CHECK(state.extents[0].cause == Sounder::kGapResync);
  CHECK(state.extents[1].start_sample == 96);
  CHECK(state.extents[1].n_samples == 6);
  CHECK(state.extents[1].cause == Sounder::kGapTimeJump);
  std::printf("PASS time-base resync re-anchors the grid\n");
  return 0;
}

int testUnstampedFirstSpan(void) {
  // t0 must be back-projected when unstamped reads precede the first
  // stamp: 40 unstamped samples, then a stamp at grid position 40.
  ScriptSource src({unstampedEv(40, 100), dataEv(24, stampAt(40), 200)});
  Sounder::CaptureState state(kRate);
  state.exact_gaps = true;
  std::atomic<bool> running(true);
  std::vector<short> out;
  CHECK(runSlots(src, state, out, 64, 1, running) == 1);

  CHECK(state.stats.has_first_sample_time == true);
  CHECK(state.stats.first_sample_time_ns == kT0);  // projected to sample 0
  CHECK(state.extents.empty());
  std::printf("PASS unstamped first span back-projects t0\n");
  return 0;
}

int testTimeoutAbort(void) {
  // A starved source: exactly kMaxConsecutiveTimeouts polls, then abort
  // with running latched false.
  ScriptSource src({});
  Sounder::CaptureState state(kRate);
  std::atomic<bool> running(true);
  std::vector<short> out;
  CHECK(runSlots(src, state, out, 64, 1, running) == 0);
  CHECK(running.load() == false);
  CHECK(state.stats.read_timeouts == Sounder::kMaxConsecutiveTimeouts);
  std::printf("PASS timeout abort threshold\n");
  return 0;
}

int testOverflowCountedNotFatal(void) {
  // Overflows reset the timeout counter (the stream is alive): 9 timeouts
  // + overflow + 9 timeouts must not abort, and the slot still completes.
  std::vector<Ev> evs(9, {Sounder::FetchStatus::kTimeout});
  evs.push_back({Sounder::FetchStatus::kOverflow});
  evs.insert(evs.end(), 9, {Sounder::FetchStatus::kTimeout});
  evs.push_back(dataEv(64, stampAt(0), 100));
  ScriptSource src(evs);
  Sounder::CaptureState state(kRate);
  std::atomic<bool> running(true);
  std::vector<short> out;
  CHECK(runSlots(src, state, out, 64, 1, running) == 1);
  CHECK(running.load() == true);
  CHECK(state.stats.read_timeouts == 18);
  CHECK(state.stats.overflow_returns == 1);
  std::printf("PASS overflow counted, resets timeout streak\n");
  return 0;
}

int testFatalAbort(void) {
  ScriptSource src({dataEv(32, stampAt(0), 100),
                    {Sounder::FetchStatus::kFatal}});
  Sounder::CaptureState state(kRate);
  std::atomic<bool> running(true);
  std::vector<short> out;
  CHECK(runSlots(src, state, out, 64, 1, running) == 0);
  CHECK(running.load() == false);
  std::printf("PASS fatal fetch aborts the capture\n");
  return 0;
}

int testInterruptMidSlot(void) {
  // The interrupt poll stops an incomplete slot without latching running
  // false (signal shutdown is not an error).
  ScriptSource src({dataEv(32, stampAt(0), 100), dataEv(32, stampAt(32), 200)});
  Sounder::CaptureState state(kRate);
  std::atomic<bool> running(true);
  std::vector<short> out;
  size_t polls = 0;
  const std::function<bool(void)> interrupt = [&polls](void) {
    return (++polls > 2);
  };
  CHECK(runSlots(src, state, out, 64, 1, running, interrupt) == 0);
  CHECK(running.load() == true);
  std::printf("PASS interrupt stops an incomplete slot\n");
  return 0;
}

}  // namespace

int main(void) {
  int rc = 0;
  rc |= testCleanSpansAcrossSlots();
  rc |= testForwardGapExactExtents();
  rc |= testForwardGapWidenedExtents();
  rc |= testGapSpanningSlots();
  rc |= testDirectStylePerPacketSpans();
  rc |= testBackwardStamp();
  rc |= testTimebaseResync();
  rc |= testUnstampedFirstSpan();
  rc |= testTimeoutAbort();
  rc |= testOverflowCountedNotFatal();
  rc |= testFatalAbort();
  rc |= testInterruptMidSlot();
  if (rc == 0) {
    std::printf("PASS all capture-path scenarios\n");
  }
  return rc;
}
