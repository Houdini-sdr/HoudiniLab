/**
 * @file mode_v_bringup_test.cc
 * @brief houdini/mode_v_bringup.h driven against a recording fake device,
 *        NO hardware: the call ORDER the device requires and the VALUES the
 *        HS-202 plan prescribes, for the BS (.22) and UE (.21) of the
 *        dual-band config.
 *
 * The fake subclasses SoapySDR::Device, records every call as a line, and
 * reads back what was written (optionally corrupted, to prove the readback
 * checks fire). The order and value rules are predicates over the recorded
 * lines, so the mutation matrix can build a mutated call sequence (a step
 * moved, a channel's rate dropped) and require the predicate to reject it.
 *
 * Build: CMake target mode_v_bringup_test. Run: ./mode_v_bringup_test (or ctest).
 */
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <map>
#include <string>
#include <vector>

#include "houdini/mode_v_bringup.h"

namespace {

using houdini::modev::Plan;
using houdini::modev::Result;
using Lines = std::vector<std::string>;

int g_fail = 0;
void check(bool ok, const std::string& what) {
  std::printf("%s  %s\n", ok ? "PASS" : "FAIL", what.c_str());
  if (!ok) ++g_fail;
}

std::string num(double v) {
  char b[48];
  std::snprintf(b, sizeof b, "%.0f", v);
  return b;
}

class FakeDevice : public SoapySDR::Device {
 public:
  Lines calls;
  double nco_error_hz = 0.0;   ///< added to every NCO readback
  bool drop_gain = false;      ///< gain readback stays at the make() default

  size_t getNumChannels(const int dir) const override { return dir == SOAPY_SDR_TX ? 2 : 4; }
  void setSampleRate(const int dir, const size_t ch, const double rate) override {
    calls.push_back(std::string("rate ") + d(dir) + " " + std::to_string(ch));
    rate_[{dir, ch}] = rate;
  }
  double getSampleRate(const int dir, const size_t ch) const override { return at(rate_, dir, ch); }
  void setFrequency(const int dir, const size_t ch, const double f, const SoapySDR::Kwargs&) override {
    calls.push_back(std::string("freq ") + d(dir) + " " + std::to_string(ch) + " " + num(f));
    freq_[{dir, ch}] = f;
  }
  double getFrequency(const int dir, const size_t ch) const override { return at(freq_, dir, ch) + nco_error_hz; }
  void setGain(const int dir, const size_t ch, const double v) override {
    calls.push_back(std::string("gain ") + d(dir) + " " + std::to_string(ch) + " " + num(v));
    if (!drop_gain) gain_[{dir, ch}] = v;
  }
  double getGain(const int dir, const size_t ch) const override {
    const auto it = gain_.find({dir, ch});
    return it == gain_.end() ? (dir == SOAPY_SDR_TX ? -6.13 : 0.0) : it->second;
  }
  void writeSetting(const std::string& key, const std::string& value) override {
    calls.push_back(value.empty() ? key : key + "=" + value);
    set_[key] = value;
  }
  std::string readSetting(const std::string& key) const override {
    const auto it = set_.find(key);
    return it == set_.end() ? std::string("ok") : it->second;
  }
  SoapySDR::Stream* setupStream(const int, const std::string&, const std::vector<size_t>&,
                                const SoapySDR::Kwargs&) override {
    calls.push_back("setupStream");
    return nullptr;
  }

 private:
  using Key = std::pair<int, size_t>;
  static const char* d(int dir) { return dir == SOAPY_SDR_TX ? "TX" : "RX"; }
  static double at(const std::map<Key, double>& m, int dir, size_t ch) {
    const auto it = m.find({dir, ch});
    return it == m.end() ? 0.0 : it->second;
  }
  std::map<Key, double> rate_, freq_, gain_;
  std::map<std::string, std::string> set_;
};

Plan basePlan() {
  Plan p;
  p.tx_rate_hz = 245.76e6;
  p.rx_rate_hz = 122.88e6;
  p.adc_fs_hz = 4915.2e6;
  p.dac_fs_hz = 5898.24e6;
  p.default_nco_hz = 2425e6;
  p.nco_by_channel = {{1, 4380e6}, {2, 4380e6}};  // "B" and "C" in the config
  p.half_bw_hz = 1596 * 30e3 / 2.0;
  p.tx_gain_db = 0.0;
  p.rx_gain_db = 0.0;
  return p;
}
Plan uePlan() { auto p = basePlan(); p.tx_channels = {0, 1}; p.rx_channels = {0}; return p; }
Plan bsPlan() { auto p = basePlan(); p.tx_channels = {0}; p.rx_channels = {0, 2}; return p; }

long idx(const Lines& c, const std::string& prefix, bool last = false) {
  long found = -1;
  for (size_t i = 0; i < c.size(); ++i)
    if (c[i].rfind(prefix, 0) == 0) {
      found = static_cast<long>(i);
      if (!last) return found;
    }
  return found;
}
size_t count(const Lines& c, const std::string& prefix) {
  return static_cast<size_t>(std::count_if(c.begin(), c.end(), [&](const std::string& s) { return s.rfind(prefix, 0) == 0; }));
}

// ---- the order rules (the device's), as one predicate ----------------------
bool orderOk(const Lines& c, std::string* why = nullptr) {
  auto fail = [why](const char* w) { if (why) *why = w; return false; };
  if (c.empty() || c.front() != "FORCE_IDLE") return fail("FORCE_IDLE first");
  if (count(c, "rate TX ") != 2 || count(c, "rate RX ") != 4) return fail("a rate on EVERY channel of each direction");
  const long dac = idx(c, "RFDC_DAC_FS="), adc = idx(c, "RFDC_ADC_FS=");
  if (!(idx(c, "rate TX ", true) < dac)) return fail("every TX rate before RFDC_DAC_FS");
  if (!(dac < adc)) return fail("RFDC_DAC_FS before RFDC_ADC_FS");
  if (!(adc < idx(c, "rate RX "))) return fail("RFDC_ADC_FS before any RX rate");
  const long last_rx_rate = idx(c, "rate RX ", true);
  const long tz = idx(c, "RFDC_TX_NYQUIST_ZONE="), is = idx(c, "RFDC_TX_INVSINC="),
             rz = idx(c, "RFDC_RX_NYQUIST_ZONE="), cal = idx(c, "RFDC_ADC_CAL=");
  if (tz >= 0 && !(last_rx_rate < tz)) return fail("zones after every rate and Fs");
  if (tz >= 0 && !(tz < is)) return fail("TX zone before the inverse sinc");
  if (rz >= 0 && !(last_rx_rate < rz)) return fail("RX zone after every rate and Fs");
  if (cal >= 0 && !(rz < cal)) return fail("ADC_CAL after the RX zone");
  const long first_freq = idx(c, "freq ");
  if (!(std::max({tz, is, rz, cal}) < first_freq)) return fail("every NCO after every zone, inverse sinc and cal write");
  if (idx(c, "gain ") >= 0 && !(last_rx_rate < idx(c, "gain "))) return fail("gains after the rates");
  if (idx(c, "setupStream") >= 0) return fail("no stream opened inside the bring-up");
  return true;
}

bool has(const Lines& c, const std::string& line) { return std::find(c.begin(), c.end(), line) != c.end(); }

// ---- the plan's values, per node ------------------------------------------
bool ueValuesOk(const Lines& c, const Result& r) {
  return has(c, "RFDC_DAC_FS=5898.2400") && has(c, "RFDC_ADC_FS=4915.2000") &&
         has(c, "RFDC_TX_NYQUIST_ZONE=ch0:1;ch1:2") && has(c, "RFDC_TX_INVSINC=ch0:zone;ch1:zone") &&
         has(c, "RFDC_RX_NYQUIST_ZONE=ch0:1") && has(c, "RFDC_ADC_CAL=ch0:cal=mode1") &&
         has(c, "freq TX 0 2425000000") && has(c, "freq TX 1 4380000000") && has(c, "freq RX 0 2425000000") &&
         has(c, "gain TX 0 0") && has(c, "gain TX 1 0") && has(c, "gain RX 0 0") && r.rxFilter(0);
}
bool bsValuesOk(const Lines& c, const Result& r) {
  return has(c, "RFDC_TX_NYQUIST_ZONE=ch0:1") && has(c, "RFDC_TX_INVSINC=ch0:zone") &&
         has(c, "RFDC_RX_NYQUIST_ZONE=ch0:1;ch2:2") && has(c, "RFDC_ADC_CAL=ch0:cal=mode1;ch2:cal=mode2") &&
         has(c, "freq TX 0 2425000000") && has(c, "freq RX 0 2425000000") && has(c, "freq RX 2 4380000000") &&
         has(c, "gain TX 0 0") && has(c, "gain RX 0 0") && has(c, "gain RX 2 0") && r.rxFilter(0) && !r.rxFilter(2);
}

Lines moved(Lines c, const std::string& prefix, const std::string& before_prefix) {
  const long from = idx(c, prefix), to = idx(c, before_prefix);
  const std::string s = c[static_cast<size_t>(from)];
  c.erase(c.begin() + from);
  c.insert(c.begin() + (to < from ? to : to - 1), s);
  return c;
}

}  // namespace

int main() {
  FakeDevice ue, bs;
  const Result ur = houdini::modev::bringUp(ue, uePlan());
  const Result br = houdini::modev::bringUp(bs, bsPlan());
  std::printf("UE calls:\n");
  for (const auto& s : ue.calls) std::printf("  %s\n", s.c_str());
  std::printf("BS log:\n");
  for (const auto& s : br.log) std::printf("  %s\n", s.c_str());

  std::string why;
  check(orderOk(ue.calls, &why), "UE call order is the device's [mutations: DAC_FS before the TX rates; inverse sinc before the zone; an NCO before the zones; one RX rate dropped]" + (why.empty() ? "" : " (" + why + ")"));
  check(orderOk(bs.calls), "BS call order is the device's");
  check(ueValuesOk(ue.calls, ur), "UE (.21) values: TX ch0 zone 1 / ch1 zone 2, inverse sinc on both, RX ch0 zone 1 Mode 1, NCOs 2425/4380, gains 0 dB, sub-6 RX filtered [mutation: the per-channel NCO override ignored]");
  check(bsValuesOk(bs.calls, br), "BS (.22) values: TX ch0 zone 1, RX ch0 zone 1 Mode 1 + ch2 zone 2 Mode 2, NCOs, gains, filter on ch0 only [mutation: the per-channel NCO override ignored]");

  {  // a refused plan writes NOTHING
    FakeDevice f;
    auto p = bsPlan();
    p.default_nco_hz = 2440e6;  // straddles the 2457.6 MHz zone edge
    bool threw = false;
    try { houdini::modev::bringUp(f, p); } catch (const std::invalid_argument&) { threw = true; }
    check(threw && f.calls.empty(), "a plan the rules refuse throws before any write (device untouched)");
  }
  {  // readback checks fire
    FakeDevice f;
    f.nco_error_hz = 1000.0;
    bool threw = false;
    try { houdini::modev::bringUp(f, uePlan()); } catch (const std::runtime_error&) { threw = true; }
    check(threw, "an NCO readback 1 kHz off throws [mutation: a readback check that never compares]");
    FakeDevice g;
    g.drop_gain = true;
    threw = false;
    try { houdini::modev::bringUp(g, uePlan()); } catch (const std::runtime_error&) { threw = true; }
    check(threw, "a gain that does not land (make() default -6.13 dB kept) throws");
  }

  std::printf("-- mutation matrix (each line must read PASS: the mutant was caught) --\n");
  check(!orderOk(moved(ue.calls, "RFDC_DAC_FS=", "rate TX ")), "mutant RFDC_DAC_FS before the TX rates is rejected");
  check(!orderOk(moved(ue.calls, "RFDC_TX_INVSINC=", "RFDC_TX_NYQUIST_ZONE=")), "mutant inverse sinc before the zone is rejected");
  check(!orderOk(moved(ue.calls, "freq ", "RFDC_RX_NYQUIST_ZONE=")), "mutant NCO before the RX zone is rejected");
  check(!orderOk(moved(ue.calls, "RFDC_ADC_FS=", "RFDC_DAC_FS=")), "mutant ADC Fs before DAC Fs is rejected");
  {
    Lines c = ue.calls;
    c.erase(c.begin() + idx(c, "rate RX 3"));
    check(!orderOk(c), "mutant with one RX channel's rate dropped is rejected");
  }
  {
    FakeDevice f;
    auto p = uePlan();
    p.nco_by_channel.clear();  // the X-band override ignored -> ch1 lands at 2425
    const Result r = houdini::modev::bringUp(f, p);
    check(!ueValuesOk(f.calls, r), "mutant ignoring the per-channel NCO fails the UE values");
  }

  std::printf("%s: %d failure(s)\n", g_fail == 0 ? "ALL PASS" : "FAILED", g_fail);
  return g_fail == 0 ? 0 : 1;
}
