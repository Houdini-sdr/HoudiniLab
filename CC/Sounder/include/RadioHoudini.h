/** @file RadioHoudini.h
  * @brief The Houdini RFSoC backend on top of the Soapy plumbing: the stream
  *        arguments and their order, the rates set before the streams open,
  *        the UDP receive drain with its gap ledger, the TDD transmit grid.
  *
  * RENEW OPEN SOURCE LICENSE: http://renew-wireless.org/license
*/
#ifndef RADIO_HOUDINI_H_
#define RADIO_HOUDINI_H_

#include <cstdint>
#include <vector>

#include "RadioSoapy.h"

class RadioHoudini : public RadioSoapy {
 public:
  explicit RadioHoudini(const RadioParams& params);

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
  size_t lastPadSamples() const override { return last_pad_samples_; }
  int64_t rxSamplePos() const override { return rx_sample_pos_; }

  /// The device and stream arguments for a Houdini node.
  static SoapySDR::Kwargs deviceArgs(const RadioParams& p);
  static SoapySDR::Kwargs rxStreamArgs(const RadioParams& p);
  static SoapySDR::Kwargs txStreamArgs(const RadioParams& p);

 private:
  double rx_rate_ = 0.0;         // cached RX sample rate for the grid tracker
  int64_t rx_sample_pos_ = 0;    // absolute samples emitted across recv calls
  size_t last_pad_samples_ = 0;  // zeros inserted into the last window
};

#endif  // RADIO_HOUDINI_H_
