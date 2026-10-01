/** @file RadioHoudini.h
  * @brief The Houdini RFSoC backend on top of the Soapy plumbing: the stream
  *        arguments and their order, the rates set before the streams open,
  *        the UDP receive drain with its gap ledger, the TDD transmit grid.
  *
  * RENEW OPEN SOURCE LICENSE: http://renew-wireless.org/license
*/
#ifndef RADIO_HOUDINI_H_
#define RADIO_HOUDINI_H_

#include <string>
#include <utility>
#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include "RadioSoapy.h"
#include "houdini/mode_v_bringup.h"
#include "houdini/link_health.h"
#include "houdini/tx_rx_boundary.h"

class RadioHoudini : public RadioSoapy {
 public:
  explicit RadioHoudini(const RadioParams& params);
  ~RadioHoudini() override;

  /// AP-79 mode V: the per-channel plan the bring-up derived and applied,
  /// null on the one-rate path. rxChannelFilter(ch) says whether RX `ch`
  /// needs the +-24 MHz channel filter (its mirror lands in the output).
  const houdini::modev::Result* modeV() const { return mode_v_.get(); }
  bool rxChannelFilter(size_t ch) const { return mode_v_ != nullptr && mode_v_->rxFilter(ch); }
  /// The BS framer captures CONTINUOUSLY and uses only its P/U slots, so it
  /// turns the whole-read filter off and filters the slices it extracts
  /// (filterRxSlice). Default on (the UE's windows are used whole).
  void setRecvFilter(bool on) { recv_filter_ = on; }
  /// AP-80: the next recv() places its window (IClientRadioSet::placeNextRx),
  /// in this radio's own time base (ns). One-shot.
  void placeNextWindow(std::function<long long(long long)> start_ns_for_head_ns) {
    placer_ = std::move(start_ns_for_head_ns);
  }
  bool rxLaneFiltered(size_t lane) const { return rx_filters_ != nullptr && rx_filters_->laneOn(lane); }
  void filterRxSlice(const void* capture, size_t cap_len, size_t start, size_t n, void* dst) {
    rx_filters_->filterSlice(static_cast<const houdini::boundary::cs16*>(capture), cap_len, start, n,
                             static_cast<houdini::boundary::cs16*>(dst));
  }

  Type type() const override { return Type::kSoapyHoudini; }
  houdini::sync::Platform platform() const override { return houdini::sync::Platform::kHoudini; }
  void printSettings() const override;
  bool hasHardwareTrigger() const override { return false; }
  bool hasAgc() const override { return false; }
  /// The driver's TxTickAnchor accepts HAS_TIME starts on the 3.125 us TDD
  /// window grid (SH-248/SH-301) when the stream was opened with tdd=1, so
  /// the beacon-referenced time is snapped to it (the whole-ms fallback
  /// otherwise); `advance_ticks` is the fine calibration added before the
  /// snap (ue_tx_advance_ticks).
  long long txTimeNs(long long frame_ticks, double rate_hz, bool tdd_pilot,
                     long long advance_ticks) const override;

  /// Rate, NCO and (mode V) the gains were applied before the streams opened,
  /// so this only reports; the Iris-style per-channel gains passed in are not
  /// used (the Houdini gains are houdini_rx_gain_db / houdini_tx_gain_db,
  /// written by the mode-V bring-up).
  void setup(int ch, double rxgain, double txgain) override;
  /// SoapyHoudiniSDR delivers ~1 MTU per readStream and buffers a backlog:
  /// drain it, then accumulate a contiguous window, zero-padding any
  /// dropped-packet gap the timestamps reveal (AP-10).
  int recv(void* const* buffs, int samples, long long& frameTime) override;
  /// AP-79: at TX = 2 x sample_rate the burst (built in ticks) is x2
  /// interpolated and beat-padded here; otherwise RadioSoapy::xmit unchanged.
  int xmit(const void* const* buffs, int samples, int flags, long long& frameTime) override;
  size_t lastPadSamples() const override { return last_pad_samples_; }
  /// AP-87: the armed schedule's rx slots (one '0'/'1' per slot, slots of n
  /// ticks, frames of fr from `epoch`). With the device's slots mode the guards
  /// and the beacon slot are cut from the stream, so a read's timestamp gap
  /// there is the schedule, not a loss: recv zero-pads it as always but counts
  /// (and reports to the gap sink) only the part inside an rx slot.
  void setRxSlotMap(long long epoch, long long n, long long fr, std::string rx) {
    slot_epoch_ = epoch;
    slot_n_ = n;
    slot_fr_ = fr;
    slot_rx_ = std::move(rx);
  }
  int64_t rxSamplePos() const override { return rx_sample_pos_; }

  /// The device and stream arguments for a Houdini node.
  static SoapySDR::Kwargs deviceArgs(const RadioParams& p);
  static SoapySDR::Kwargs rxStreamArgs(const RadioParams& p);
  static SoapySDR::Kwargs txStreamArgs(const RadioParams& p);

 private:
  RadioHoudini(const RadioParams& params, std::shared_ptr<houdini::modev::Result> mv);
  /// The mode-V plan for this node, from its params.
  static houdini::modev::Plan modeVPlan(const RadioParams& p);
  static void logModeV(const std::string& label, const std::vector<std::string>& lines);
  /// The converter state as the device reports it (RFDC_SNAPSHOT, the full
  /// RFDC_PREFLIGHT, getChannelInfo per channel in use), written to
  /// rfdc_<label>_<stage>_<time>.txt under --dump_dir. Every Houdini
  /// run writes one before activate and one at the end, so a run's settings
  /// can be checked against its plan and drift across it is visible. Best
  /// effort: never throws.
  static void writeStateRecord(const std::string& label, SoapySDR::Device& dev, const std::string& stage,
                               const std::vector<size_t>& rx_channels, const std::vector<size_t>& tx_channels);
  static void writeModeVRecord(const std::string& label, SoapySDR::Device& dev,
                               const houdini::modev::Result& r, const houdini::modev::PostSetup& ps,
                               const std::string& failure);

  std::shared_ptr<houdini::modev::Result> mode_v_;  // null unless mode V
  // TX = 2 x rate: one interpolator per TX lane (lane c = tx_channels[c]), AP-85.
  std::vector<std::unique_ptr<houdini::boundary::TxBurstInterpolator>> tx_interp_;
  std::unique_ptr<houdini::boundary::RxLaneFilters> rx_filters_;       // any lane filtered
  bool recv_filter_ = true;  // off when the BS framer filters its slices itself
  std::function<long long(long long)> placer_;  // set for one recv() by placeNextWindow

  // AP-79 link health: the software lane's checks on this handle, on a thread
  // started at the first successful read; plus what only the app can count.
  void maybeStartHealth();
  void healthLoop(double period_s);
  std::atomic<bool> health_started_{false};
  bool health_stop_ = false;
  std::mutex health_mtx_;
  std::condition_variable health_cv_;
  std::thread health_thread_;
  std::atomic<unsigned long long> app_rx_err_{0}, app_rx_short_{0}, app_rx_pad_{0}, app_tx_short_{0}, app_tx_sat_{0};
  double rx_rate_ = 0.0;         // cached RX sample rate for the grid tracker
  int64_t rx_sample_pos_ = 0;    // absolute samples emitted across recv calls
  size_t last_pad_samples_ = 0;  // zeros inserted into the last window
  long long slot_epoch_ = 0, slot_n_ = 0, slot_fr_ = 0;  // AP-87 rx slot map (setRxSlotMap)
  std::string slot_rx_;
  // AP-87 check: every read's stamped samples must lie inside an rx slot.
  long long slot_reads_ = 0, slot_samples_ = 0, slot_stray_ = 0, slot_stray_reads_ = 0;
  // Every stamped read against the stream's sample count, so the data read is
  // where it was expected (the RX packets carry no sequence number, so the
  // stamp is the count): on it, after a gap (samples lost vs the schedule's
  // own gaps), out of order (earlier than the count), a time jump.
  long long rd_reads_ = 0, rd_on_count_ = 0, rd_gap_reads_ = 0, rd_gap_lost_ = 0, rd_gap_sched_ = 0,
            rd_backward_ = 0, rd_resync_ = 0;
};

#endif  // RADIO_HOUDINI_H_
