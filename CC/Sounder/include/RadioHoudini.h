/** @file RadioHoudini.h
  * @brief The Houdini RFSoC backend on top of the Soapy plumbing: the stream
  *        arguments and their order, the rates set before the streams open,
  *        the UDP receive drain with its gap ledger, the TDD transmit grid.
  *
  * RENEW OPEN SOURCE LICENSE: http://renew-wireless.org/license
*/
#ifndef RADIO_HOUDINI_H_
#define RADIO_HOUDINI_H_

#include <atomic>
#include <condition_variable>
#include <cstdint>
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
  /// needs the +-25 MHz channel filter (its mirror lands in the output).
  const houdini::modev::Result* modeV() const { return mode_v_.get(); }
  bool rxChannelFilter(size_t ch) const { return mode_v_ != nullptr && mode_v_->rxFilter(ch); }
  /// The BS framer captures CONTINUOUSLY and uses only its P/U slots, so it
  /// turns the whole-read filter off and filters the slices it extracts
  /// (filterRxSlice). Default on (the UE's windows are used whole).
  void setRecvFilter(bool on) { recv_filter_ = on; }
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

  /// The mixer NCO is the only tuning knob and there are no gain stages: the
  /// rate and NCO were applied before the streams opened, so this reports.
  void setup(int ch, double rxgain, double txgain) override;
  /// SoapyHoudiniSDR delivers ~1 MTU per readStream and buffers a backlog:
  /// drain it, then accumulate a contiguous window, zero-padding any
  /// dropped-packet gap the timestamps reveal (AP-10).
  int recv(void* const* buffs, int samples, long long& frameTime) override;
  /// AP-79: at TX = 2 x sample_rate the burst (built in ticks) is x2
  /// interpolated and beat-padded here; otherwise RadioSoapy::xmit unchanged.
  int xmit(const void* const* buffs, int samples, int flags, long long& frameTime) override;
  size_t lastPadSamples() const override { return last_pad_samples_; }
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
  static void writeModeVRecord(const std::string& label, SoapySDR::Device& dev,
                               const houdini::modev::Result& r, const houdini::modev::PostSetup& ps);

  std::shared_ptr<houdini::modev::Result> mode_v_;  // null unless mode V
  std::unique_ptr<houdini::boundary::TxBurstInterpolator> tx_interp_;  // TX = 2 x rate
  std::unique_ptr<houdini::boundary::RxLaneFilters> rx_filters_;       // any lane filtered
  bool recv_filter_ = true;  // off when the BS framer filters its slices itself

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
};

#endif  // RADIO_HOUDINI_H_
