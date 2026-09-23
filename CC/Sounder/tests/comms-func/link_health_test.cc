/**
 * @file link_health_test.cc
 * @brief houdini/link_health.h, the C++ port of the software lane's link-health
 *        monitor, pinned case for case to their T0 tests
 *        (SoapyHoudiniSDR host/tests/test_link_health.py @ d3ade5a) and to real
 *        captures from a streaming mode-V node
 *        (fixtures/link_health/mode_v_streaming_21.txt). NO hardware.
 *
 * Build: CMake target link_health_test. Run: ./link_health_test <fixture> (or ctest).
 */
#include <cstdio>
#include <fstream>
#include <map>
#include <set>
#include <string>
#include <vector>

#include "houdini/link_health.h"

using namespace houdini::health;

namespace {
int g_fail = 0;
void check(bool ok, const std::string& what) {
  std::printf("%s  %s\n", ok ? "PASS" : "FAIL", what.c_str());
  if (!ok) ++g_fail;
}

const std::string kSnap =
    "build: device_build=6dd9d402 fpga_commit=0a32114f fpga_version=1.30\n"
    "fab_clk: rx=30720000 Hz (code 2) tx=122880000 Hz (code 0)\n"
    "blocks:\n"
    "ADC0.0:en=1 fs=4915.200MHz nco=2425.000000MHz scale=1.0\n"
    "DAC0.0:en=1 fs=5898.240MHz nco=2425.000000MHz invsinc=1 scale=0.7\n"
    "rx_intr: ADC0.0=0[]\n"
    "tx_banks: ch0:late=0\n"
    "rx_banks: ch0:gated=0";

std::string replaceAll(std::string s, const std::string& a, const std::string& b) {
  for (size_t p = s.find(a); p != std::string::npos; p = s.find(a, p + b.size())) s.replace(p, a.size(), b);
  return s;
}

struct FakeNode {
  std::map<std::string, std::string> keys = {
      {"TX_BANK_STATUS", "ch0:acked=9,late=0,under=0,seqerr=0,zerofill=0,drops=0,efault=0,smiss=0,clkerr=0,aclose=0,fill=3"},
      {"RX_BANK_STATUS", "ch0:gated=0,aborts=0,hwm=12"},
      {"TX_HOST_STATUS", "eob_recloses=0 eob_recloses_ch0=0"},
      {"RX_HOST_STATUS", "rxq_ovfl=0 rxq_ovfl_ch0=0 ring_ovfl=0 ring_ovfl_ch0=0"},
      {"EGRESS_STATUS", "drop=p0:0,p1:0,p2:0,p3:0;stall_seen=0,stall_evt=0;marked=p0:0,p1:0,p2:0,p3:0"},
      {"RFDC_INTR_FIRE_COUNT", "1000"},
      {"RFDC_PREFLIGHT", "ok known DAC0.0:FIFO_OVR(HS-207)\ndetail"},
      {"RFDC_SNAPSHOT", kSnap}};
  double t = 0.0;
  Read read() { return [this](const std::string& k) { return keys.at(k); }; }
  std::function<double()> clock() { return [this] { return t += 5.0; }; }
};

bool eq(const std::vector<std::string>& a, const std::vector<std::string>& b) { return a == b; }
}  // namespace

int main(int argc, char** argv) {
  {  // test_egress_status_splits_per_port_groups_from_flat_ones
    const auto g = parseEgressStatus("drop=p0:3,p1:0,p2:0,p3:255;stall_seen=1,stall_evt=2;marked=p0:0,p1:1,p2:0,p3:0");
    check(g.at("drop_p0") == 3 && g.at("drop_p3") == 255 && g.at("marked_p1") == 1 && g.at("stall_seen") == 1 &&
              g.at("stall_evt") == 2 && parseEgressStatus("").empty() && parseEgressStatus("junk;x").empty(),
          "egress: per-port groups split from flat ones; junk yields nothing");
  }
  {  // test_flat_counts_skip_malformed_tokens
    const auto g = parseFlatCounts("rxq_ovfl=2 rxq_ovfl_ch0=2 junk x=y z=-1");
    check(g.size() == 2 && g.at("rxq_ovfl") == 2 && g.at("rxq_ovfl_ch0") == 2, "flat counts skip malformed tokens");
  }
  {  // test_snapshot_config_keeps_block_lines_and_drops_counter_sections
    const auto c = snapshotConfig(kSnap);
    std::set<std::string> keys;
    for (const auto& kv : c) keys.insert(kv.first);
    check(keys == std::set<std::string>{"build", "fab_clk", "blocks"} &&
              c.at("blocks").substr(c.at("blocks").find('\n') + 1).rfind("DAC0.0:", 0) == 0,
          "snapshot keeps block lines and drops the counter sections");
  }
  {  // test_config_drift_names_the_changed_line_only
    const auto a = snapshotConfig(kSnap);
    const auto b = snapshotConfig(replaceAll(kSnap, "invsinc=1", "invsinc=0"));
    const auto d = configDrift(a, b);
    auto nb = a;
    nb.erase("build");
    check(d.size() == 1 && d[0].rfind("blocks: DAC0.0:", 0) == 0 && d[0].find("invsinc=1") != std::string::npos &&
              d[0].find("invsinc=0") != std::string::npos && configDrift(a, a).empty() &&
              eq(configDrift(a, nb), {"build: " + a.at("build") + " -> "}),
          "config drift names the changed line only; a missing section reads empty");
  }
  {  // test_counters_rise_is_flagged_and_a_clear_is_not
    const auto inc = counterIncreases({{"a", 1}, {"b", 5}}, {{"a", 3}, {"b", 0}, {"c", 2}});
    check(inc == Counters{{"a", 2}, {"c", 2}}, "a rise is flagged, a clear (fall) is not, a new counter counts from zero");
  }
  {  // test_saturated_or_sticky_egress_is_flagged_even_at_baseline
    const auto b = blindCounters({{"egress.drop_p0", 255}, {"egress.drop_p1", 254}, {"egress.marked_p2", 255},
                                  {"egress.stall_seen", 1}, {"egress.stall_evt", 255}, {"tx0.late", 255}});
    check(eq(b, {"egress.drop_p0=255 (saturated: further drops cannot be counted)",
                 "egress.marked_p2=255 (saturated: further drops cannot be counted)",
                 "egress.stall_seen=1 (sticky: a stall happened; stall_evt no longer proves a new one)"}),
          "saturated and sticky egress counters are flagged");
    FakeNode n;
    n.keys["EGRESS_STATUS"] = "drop=p0:255,p1:0,p2:0,p3:0;stall_seen=0,stall_evt=0;marked=p0:0,p1:0,p2:0,p3:0";
    LinkHealth h(n.read(), "bs", n.clock());
    check(eq(h.check().alarms(), {"egress.drop_p0=255 (saturated: further drops cannot be counted)"}),
          "a counter saturated before the session began is flagged on the first check");
  }
  {  // test_preflight_items_split_failures_from_known
    check(preflightItems("ok known DAC0.0:FIFO_OVR(HS-207)").empty() &&
              preflightItems("FAIL ADC0.1:FIFOUSRDAT_OF,FIFOUSRDAT_UF;ADC2.1:X known DAC0.0:FIFO_OVR(HS-207)") ==
                  std::set<std::string>{"ADC0.1:FIFOUSRDAT_OF,FIFOUSRDAT_UF", "ADC2.1:X"},
          "preflight items: failures split from the known part");
  }
  {  // test_a_standing_preflight_failure_is_baseline_and_a_new_one_alarms
    FakeNode n;
    n.keys["RFDC_PREFLIGHT"] = "FAIL ADC0.1:FIFOUSRDAT_OF,FIFOUSRDAT_UF known X\n";
    LinkHealth h(n.read(), "bs", n.clock());
    bool ok = h.baselineFailures() == std::set<std::string>{"ADC0.1:FIFOUSRDAT_OF,FIFOUSRDAT_UF"} && h.check().alarms().empty();
    n.keys["RFDC_PREFLIGHT"] = "FAIL ADC0.0:OVR_RANGE;ADC0.1:FIFOUSRDAT_OF,FIFOUSRDAT_UF\n";
    ok = ok && eq(h.check().alarms(), {"preflight new FAIL ADC0.0:OVR_RANGE"}) && h.check().alarms().empty();
    n.keys["RFDC_PREFLIGHT"] = "FAIL ADC0.1:FIFOUSRDAT_OF,FIFOUSRDAT_UF\n";
    ok = ok && h.check().alarms().empty();
    n.keys["RFDC_PREFLIGHT"] = "FAIL ADC0.0:OVR_RANGE;ADC0.1:FIFOUSRDAT_OF,FIFOUSRDAT_UF\n";
    ok = ok && eq(h.check().alarms(), {"preflight new FAIL ADC0.0:OVR_RANGE"});
    check(ok, "a standing preflight FAIL is baseline (once); a new one alarms, is not repeated, and alarms again when it comes back");
  }
  {  // test_collect_counters_takes_only_the_alarm_fields
    FakeNode n;
    const auto g = collectCounters(n.read());
    check(g.count("tx0.late") && g.count("tx0.aclose") && !g.count("tx0.acked") && !g.count("tx0.fill") &&
              !g.count("rx0.hwm") && g.at("rx0.gated") == 0 && g.at("host.ring_ovfl") == 0 &&
              !g.count("host.rxq_ovfl_ch0") && g.at("egress.stall_evt") == 0,
          "collect takes only the alarm fields");
  }
  {  // test_link_health_clean_then_each_alarm_class
    FakeNode n;
    LinkHealth h(n.read(), "bs", n.clock());
    auto r = h.check();
    bool ok = r.alarms().empty() && r.line().find("clean") != std::string::npos;
    n.keys["TX_BANK_STATUS"] = replaceAll(n.keys["TX_BANK_STATUS"], "late=0", "late=4");
    n.keys["RX_HOST_STATUS"] = "rxq_ovfl=7 rxq_ovfl_ch0=7 ring_ovfl=0";
    n.keys["RFDC_INTR_FIRE_COUNT"] = "46000";
    r = h.check();
    ok = ok && eq(r.alarms(), {"host.rxq_ovfl +7", "tx0.late +4"}) && r.irq_per_s > 0;
    ok = ok && h.check().alarms().empty();
    n.keys["RFDC_PREFLIGHT"] = "FAIL ADC0.0:OVR_RANGE\n";
    n.keys["RFDC_SNAPSHOT"] = replaceAll(kSnap, "nco=2425.000000MHz scale=1.0", "nco=2430.000000MHz scale=1.0");
    r = h.check();
    const auto a = r.alarms();
    ok = ok && a.size() == 2 && a[0] == "preflight new FAIL ADC0.0:OVR_RANGE" && a[1].rfind("config blocks: ADC0.0:", 0) == 0;
    h.rebaseline();
    n.keys["RFDC_PREFLIGHT"] = "ok";
    ok = ok && h.check().alarms().empty();
    check(ok, "clean, then each alarm class (counter rise, per-period judgement, new FAIL, drift), then rebaseline");
  }
  {  // the real captures from a streaming mode-V node
    std::map<std::string, std::string> keys;
    if (argc > 1) {
      std::ifstream f(argv[1]);
      std::string line;
      while (std::getline(f, line)) {
        if (line.empty() || line[0] == '#') continue;
        const auto eq = line.find('=');
        keys[line.substr(0, eq)] = line.substr(eq + 1);
      }
    }
    check(keys.size() == 7, "the streaming fixture has its 7 keys");
    if (keys.size() == 7) {
      keys["RFDC_SNAPSHOT"] = kSnap;  // not captured; drift is covered above
      Read rd = [&keys](const std::string& k) { return keys.at(k); };
      const auto g = collectCounters(rd);
      check(g.at("host.ring_ovfl") == 22718 && g.at("egress.marked_p0") == 7 && g.at("tx0.late") == 0 &&
                g.at("tx1.aclose") == 0 && g.at("rx3.aborts") == 0 && g.count("host.eob_recloses") == 1,
            "real capture: every alarm field parses (ring_ovfl 22718, marked_p0 7, four RX banks, two TX)");
      double t = 0.0;
      LinkHealth h(rd, "bs", [&t] { return t += 5.0; });
      check(h.baselineFailures() == std::set<std::string>{"ADC0.1:FIFOUSRDAT_OF,FIFOUSRDAT_UF"},
            "real capture: SH-421's standing FAIL is the baseline; the HS-207 known item is not a failure");
      check(h.check().alarms().empty(), "real capture: an unchanged node checks clean (ring_ovfl judged by its increase)");
      keys["RFDC_INTR_FIRE_COUNT"] = std::to_string(14032361 + 5 * 46600);
      keys["RX_HOST_STATUS"] = "rxq_ovfl=0 rxq_ovfl_ch0=0 ring_ovfl=22800 ring_ovfl_ch0=22800";
      const auto r = h.check();
      std::printf("  %s\n", r.line().c_str());
      check(eq(r.alarms(), {"host.ring_ovfl +82"}) && r.irq_per_s > 46000 && r.irq_per_s < 47000,
            "real capture: a ring drop rise alarms, and the HS-207 IRQ rate reads 46.6k/s");
    }
  }
  std::printf("%s: %d failure(s)\n", g_fail == 0 ? "ALL PASS" : "FAILED", g_fail);
  return g_fail == 0 ? 0 : 1;
}
