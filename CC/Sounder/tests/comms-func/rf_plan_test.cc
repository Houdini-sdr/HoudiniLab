/**
 * @file rf_plan_test.cc
 * @brief houdini/rf_plan.h against the HS-202 plan's own table, NO hardware.
 *
 * The plan (Houdini-Streaming docs/DEMO_FREQUENCY_PLAN.md sections 2 and 3,
 * reviewed and bench-validated on W2/W3) states for mode V: sub-6 at NCO 2425
 * is ADC zone 1 with calibration Mode 1 and DAC zone 1 (NRZ); the X-band IF at
 * 4380 is ADC zone 2 with Mode 2 and DAC zone 2 (mix-mode); the sub-6 channel's
 * mirror lands 40.2 MHz and up from the NCO, inside the 122.88 output, so the
 * application filters it, while the X-band mirror is over 1 GHz away. The
 * derivation must reproduce exactly that, for every rung of the config ladder
 * (R1 fft 256 / 96 subcarriers, R3 fft 4096 / 1596, the 40 MHz rollback), and
 * must refuse the placements the rules forbid.
 *
 * Each plan assertion names the Rules mutation that breaks it, and the mutation
 * matrix builds that mutation and requires the assertion to fail.
 *
 * Build: CMake target rf_plan_test. Run: ./rf_plan_test (or ctest).
 */
#include <cmath>
#include <cstdio>
#include <functional>
#include <string>

#include "houdini/rf_plan.h"

namespace {

using houdini::rfplan::ConverterPoint;
using houdini::rfplan::planRx;
using houdini::rfplan::planTx;
using houdini::rfplan::Rules;

int g_fail = 0;
void check(bool ok, const std::string& what) {
  std::printf("%s  %s\n", ok ? "PASS" : "FAIL", what.c_str());
  if (!ok) ++g_fail;
}

const ConverterPoint kModeV{4915.2e6, 5898.24e6, 122.88e6, 245.76e6};
constexpr double kScs30 = 30e3;
constexpr double kHb50 = 1596 * kScs30 / 2.0;     // R3, 133 RB: +-23.94 MHz
constexpr double kHb40 = 1272 * kScs30 / 2.0;     // rollback, 106 RB: +-19.08 MHz
constexpr double kHbR1 = 96 * 480e3 / 2.0;        // R1, fft 256: +-23.04 MHz

// Refused, and (when `says` is given) for the reason named: two rules can
// refuse the same input, so the message tells which one did.
bool throws(const std::function<void()>& f, const char* says = "") {
  try { f(); } catch (const std::invalid_argument& e) { return std::string(e.what()).find(says) != std::string::npos; }
  return false;
}

// The plan's assertions as predicates over a Rules, so the real rules and each
// mutant are judged by the same code. An exception is a failed assertion.
bool sub6Rx(const Rules& r, double nco = 2425e6, double hb = kHb50, double mirror_mhz = 41.26) {
  try {
    const auto p = planRx(nco, hb, kModeV, r);
    return p.zone == 1 && p.cal_mode == 1 && p.channel_filter &&
           std::fabs(p.mirror_offset_hz / 1e6 - mirror_mhz) < 0.005;
  } catch (const std::invalid_argument&) { return false; }
}
bool xifRx(const Rules& r) {
  try {
    const auto p = planRx(4380e6, kHb50, kModeV, r);
    return p.zone == 2 && p.cal_mode == 2 && !p.channel_filter && p.mirror_offset_hz > 1.0e9;
  } catch (const std::invalid_argument&) { return false; }
}
bool sub6Tx(const Rules& r) {
  try { return planTx(2425e6, kHb50, kModeV, r).zone == 1; } catch (const std::invalid_argument&) { return false; }
}
bool xifTx(const Rules& r) {
  try { return planTx(4380e6, kHb50, kModeV, r).zone == 2; } catch (const std::invalid_argument&) { return false; }
}

}  // namespace

int main() {
  const Rules real;

  // ---- the plan's table ----------------------------------------------------
  {
    const auto p = planRx(2425e6, kHb50, kModeV);
    std::printf("sub-6 RX: zone %d, cal mode %d, filter %d, mirror from %.3f MHz\n", p.zone, p.cal_mode,
                p.channel_filter ? 1 : 0, p.mirror_offset_hz / 1e6);
    const auto x = planRx(4380e6, kHb50, kModeV);
    std::printf("X-IF RX: zone %d, cal mode %d, filter %d, mirror from %.1f MHz\n", x.zone, x.cal_mode,
                x.channel_filter ? 1 : 0, x.mirror_offset_hz / 1e6);
  }
  check(sub6Rx(real), "sub-6 RX (2425, R3): zone 1, cal Mode 1, filter on, mirror from 41.26 MHz "
                      "[mutations: cal split 0.5; filter stop 45 MHz; filter pass 20 MHz]");
  check(xifRx(real), "X-IF RX (4380): zone 2, cal Mode 2, no filter, mirror over 1 GHz away "
                     "[mutations: cal split 0.05; max zone 1]");
  check(sub6Tx(real), "sub-6 TX (2425): DAC zone 1, NRZ [mutation: NRZ edge 0.40 Fs]");
  check(xifTx(real), "X-IF TX (4380): DAC zone 2, mix-mode [mutation: mix-mode from 0.75 Fs]");
  check(sub6Rx(real, 2420e6, kHb40, 56.12), "rollback RX (2420, 106 RB): zone 1, Mode 1, filter on, mirror from 56.12 MHz");
  check(sub6Rx(real, 2425e6, kHbR1, 42.16), "R1 RX (2425, fft 256 / 96 sc): zone 1, Mode 1, filter on, mirror from 42.16 MHz");

  // ---- refusals --------------------------------------------------------------
  // Fails under: the straddle rule disabled (the mirror rule then refuses the
  // same channel, for another reason).
  check(throws([] { planRx(2440e6, kHb50, kModeV); }, "straddles"), "refuses a channel straddling the 2457.6 MHz zone edge");
  check(throws([] { planRx(2435e6, 10e6, kModeV); }, "stop edge"), "refuses a mirror inside the filter's 40.2 MHz stop edge (NCO 2435)");
  check(throws([] { planRx(2420e6, 30e6, kModeV); }, "exceeds the channel filter passband"), "refuses a filtered channel wider than the +-24 MHz passband");
  check(throws([] { planRx(5500e6, 10e6, kModeV); }, "ADC zone"), "refuses ADC zone 3 (the device accepts 1 and 2)");
  check(throws([] { planRx(1966.08e6, 10e6, kModeV); }, "aliases across"), "refuses a channel aliasing across 0.4 Fs (the cal-mode split)");
  check(throws([] { planTx(2700e6, kHb50, kModeV); }), "refuses TX above the NRZ band (0.45 Fs = 2654 MHz)");
  check(throws([] { planTx(3000e6, kHb50, kModeV); }), "refuses TX in the gap between NRZ and mix-mode");
  check(throws([] { planRx(2425e6, kHb50, ConverterPoint{}); }), "refuses an unset converter point");

  // ---- mutation matrix: each mutant must break the assertion that names it ---
  std::printf("-- mutation matrix (each line must read PASS: the mutant was caught) --\n");
  auto mut = [](auto set) { Rules r; set(r); return r; };
  check(!sub6Rx(mut([](Rules& r) { r.cal_split_frac = 0.5; })), "mutant cal split 0.5 breaks the sub-6 RX assertion");
  check(!xifRx(mut([](Rules& r) { r.cal_split_frac = 0.05; })), "mutant cal split 0.05 breaks the X-IF RX assertion");
  check(!sub6Rx(mut([](Rules& r) { r.filter_stop_hz = 45e6; })), "mutant filter stop 45 MHz breaks the sub-6 RX assertion");
  check(!sub6Rx(mut([](Rules& r) { r.filter_pass_hz = 20e6; })), "mutant filter pass 20 MHz breaks the sub-6 RX assertion");
  check(!xifRx(mut([](Rules& r) { r.max_rx_zone = 1; })), "mutant max zone 1 breaks the X-IF RX assertion");
  check(!sub6Tx(mut([](Rules& r) { r.nrz_max_frac = 0.40; })), "mutant NRZ edge 0.40 breaks the sub-6 TX assertion");
  check(!xifTx(mut([](Rules& r) { r.mix_min_frac = 0.75; })), "mutant mix-mode from 0.75 breaks the X-IF TX assertion");

  std::printf("%s: %d failure(s)\n", g_fail == 0 ? "ALL PASS" : "FAILED", g_fail);
  return g_fail == 0 ? 0 : 1;
}
