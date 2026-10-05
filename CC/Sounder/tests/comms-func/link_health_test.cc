/**
 * @file link_health_test.cc
 * @brief houdini/link_health.h against the software lane's link-alarm contract
 *        (SoapyHoudiniSDR shared/HOUDINI_PROTOCOL.md section 2.7), clause by
 *        clause, and against real captures: a pre-HS-220 streaming mode-V node
 *        (fixtures/link_health/mode_v_streaming_21.txt) and the current stack
 *        (fixtures/link_health/demo_stack_fi1.txt). NO hardware. Each check
 *        names the mutation that breaks it.
 *
 * Build: CMake target link_health_test. Run: ./link_health_test <old fixture> <current fixture> (or ctest).
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

std::string replaceAll(std::string s, const std::string& a, const std::string& b) {
  for (size_t p = s.find(a); p != std::string::npos; p = s.find(a, p + b.size())) s.replace(p, a.size(), b);
  return s;
}

struct FakeNode {
  std::map<std::string, std::string> keys = {
      {"TX_BANK_STATUS",
       "ch0:acked=9,late=0,under=0,seqerr=0,zerofill=0,drops=0,efault=0,smiss=0,clkerr=0,aclose=0,fill=3,epoch=1:1"},
      {"RX_BANK_STATUS", "ch0:gated=0,aborts=0,hwm=12"},
      {"TX_HOST_STATUS", "eob_recloses=0 eob_recloses_ch0=0"},
      {"RX_HOST_STATUS", "rxq_ovfl=0 rxq_ovfl_ch0=0 ring_ovfl=0 ring_ovfl_ch0=0"},
      {"EGRESS_STATUS", "drop=p0:0,p1:0,p2:0,p3:0;stall_seen=0,stall_evt=0;marked=0"},
      {"RFDC_INTR_FIRE_COUNT", "1000"},
      {"RFDC_PREFLIGHT", "ok known DAC0.0:FIFO_OVR(HS-207)\ndetail"}};
  double t = 0.0;
  Read read() { return [this](const std::string& k) { return keys.at(k); }; }
  std::function<double()> clock() { return [this] { return t += 5.0; }; }
};

bool eq(const std::vector<std::string>& a, const std::vector<std::string>& b) { return a == b; }

std::map<std::string, std::string> loadFixture(const char* path) {
  std::map<std::string, std::string> keys;
  std::ifstream f(path);
  std::string line;
  while (std::getline(f, line)) {
    if (line.empty() || line[0] == '#') continue;
    const auto e = line.find('=');
    keys[line.substr(0, e)] = line.substr(e + 1);
  }
  return keys;
}
}  // namespace

int main(int argc, char** argv) {
  {  // the parsers
    const auto g = parseEgressStatus("drop=p0:3,p1:0,p2:0,p3:255;stall_seen=1,stall_evt=2;marked=5");
    check(g.at("drop_p0") == 3 && g.at("drop_p3") == 255 && g.at("marked") == 5 && g.at("stall_seen") == 1 &&
              g.at("stall_evt") == 2 && g.size() == 7 && parseEgressStatus("").empty() &&
              parseEgressStatus("junk;x").empty(),
          "egress: per-port drops beside flat counters, seven in all; junk yields nothing (mutation: marked split "
          "as per-port, or the per-port test dropped)");
    const auto f = parseFlatCounts("rxq_ovfl=2 rxq_ovfl_ch0=2 junk x=y z=-1");
    check(f.size() == 2 && f.at("rxq_ovfl") == 2 && f.at("rxq_ovfl_ch0") == 2,
          "flat counts skip malformed tokens (mutation: a non-digit value accepted)");
  }
  {  // 2.7 "A fall": a counter without an epoch that fell was cleared; its current value is the rise
    check(counterIncreases({{"rx0.gated", 1}, {"host.rxq_ovfl_ch0", 5}, {"egress.drop_p0", 9}},
                           {{"rx0.gated", 3}, {"host.rxq_ovfl_ch0", 2}, {"egress.drop_p0", 0}, {"rx1.aborts", 2}}) ==
              Counters{{"rx0.gated", 2}, {"host.rxq_ovfl_ch0", 2}, {"rx1.aborts", 2}},
          "a rise alarms, a fall reports the current value (5 -> 2 is +2, 9 -> 0 is nothing), a new key counts from "
          "zero (mutation: a fall re-based silently, the old plain rule)");
  }
  {  // an empty or partial read between two full ones must not erase the baseline
    const Counters full{{"a.x", 10}, {"b.y", 7}};
    const Counters kept = carryCounters(carryCounters(Counters{}, full), Counters{});
    check(counterIncreases(kept, full).empty(),
          "an empty read does not erase the baseline: the next full read reports nothing new (mutation: prev = cur, "
          "every counter's running total reads as new)");
    const Counters later{{"a.x", 12}, {"b.y", 7}};
    check(counterIncreases(carryCounters(kept, Counters{{"b.y", 7}}), later) == Counters{{"a.x", 2}},
          "a partial read keeps the missing counter's last value (mutation: prev = cur, a +12 false alarm)");
  }
  {  // 2.7 "Egress counters" and "No other alarm": standing at a ceiling or a sticky bit is not an alarm
    FakeNode n;
    n.keys["EGRESS_STATUS"] = "drop=p0:255,p1:0,p2:0,p3:0;stall_seen=1,stall_evt=4;marked=255";
    LinkHealth h(n.read(), "bs", n.clock());
    check(h.check().alarms().empty(),
          "a counter saturated, and a sticky stall_seen set, before the session began are not alarms (mutation: the "
          "blind-counter alarm restored)");
    n.keys["EGRESS_STATUS"] = "drop=p0:3,p1:0,p2:0,p3:0;stall_seen=0,stall_evt=4;marked=255";
    check(eq(h.check().alarms(), {"egress.drop_p0 +3"}),
          "an eth reset then 3 new drops (255 -> 3) reports +3 (mutation: a fall re-based)");
    n.keys["EGRESS_STATUS"] = "drop=p0:3,p1:0,p2:0,p3:0;stall_seen=1,stall_evt=5;marked=255";
    check(eq(h.check().alarms(), {"egress.stall_evt +1", "egress.stall_seen +1"}),
          "a new stall alarms once, stall_seen and stall_evt both rising (mutation: stall_seen not collected)");
  }
  {  // 2.7 "Host fields come from the per-stream keys, never the totals"
    FakeNode c;
    c.keys["TX_HOST_STATUS"] = "eob_recloses=4 eob_recloses_ch0=1 eob_recloses_ch1=3 cold_releases_ch0=0 "
                               "cold_window_ms_ch0=inf rate_ppm_ch0=-31.34";
    c.keys["RX_HOST_STATUS"] = "rxq_ovfl=0 rxq_ovfl_ch0=0 tdd_drop=9 tdd_drop_ch0=9 tdd_straddle_ch2=0";
    const auto g = collectCounters(c.read());
    check(g.at("host.eob_recloses_ch0") == 1 && g.at("host.eob_recloses_ch1") == 3 && g.count("host.eob_recloses") == 0 &&
              g.count("host.rxq_ovfl") == 0,
          "host fields are the per-stream keys, never the totals (mutation: the totals read)");
    check(g.count("host.tdd_straddle_ch2") == 1 && g.count("host.tdd_drop_ch0") == 0 &&
              g.count("host.cold_releases_ch0") == 0,
          "tdd_straddle alarms, tdd_drop (the slot cut) and the pacer's other keys do not (mutation: the host field "
          "list changed)");
    FakeNode n;
    n.keys["RX_HOST_STATUS"] = "rxq_ovfl=5 rxq_ovfl_ch0=2 rxq_ovfl_ch2=3";
    LinkHealth h(n.read(), "bs", n.clock());
    const bool quiet = h.check().alarms().empty();
    n.keys["RX_HOST_STATUS"] = "rxq_ovfl=2 rxq_ovfl_ch0=2";  // ch2's stream closed: its counts leave the total
    const bool closed = h.check().alarms().empty();
    n.keys["RX_HOST_STATUS"] = "rxq_ovfl=9 rxq_ovfl_ch0=9";
    check(quiet && closed && eq(h.check().alarms(), {"host.rxq_ovfl_ch0 +7"}),
          "a stream that closes raises nothing (the total's 5 -> 2 is not a clear), and the open stream's drops "
          "alarm by its key (mutation: the totals read, the close then reports +2)");
    // ch0 closes while ch2 streams on, then a new stream opens on ch0: its
    // key appears again and counts from zero (2.7), not from the old 2.
    FakeNode r;
    r.keys["RX_HOST_STATUS"] = "rxq_ovfl=5 rxq_ovfl_ch0=2 rxq_ovfl_ch2=3";
    LinkHealth hr(r.read(), "bs", r.clock());
    const bool open0 = hr.check().alarms().empty();
    r.keys["RX_HOST_STATUS"] = "rxq_ovfl=3 rxq_ovfl_ch2=3";
    const bool closed0 = hr.check().alarms().empty();
    r.keys["RX_HOST_STATUS"] = "rxq_ovfl=63 rxq_ovfl_ch0=60 rxq_ovfl_ch2=3";
    check(open0 && closed0 && eq(hr.check().alarms(), {"host.rxq_ovfl_ch0 +60"}),
          "a stream reopened on a channel counts from zero (mutation: the closed stream's key carried, +58)");
    // The LAST TX stream closes: the status answers with its total alone, and
    // a stream reopened on ch0 still counts from zero.
    FakeNode t;
    t.keys["TX_HOST_STATUS"] = "eob_recloses=5 eob_recloses_ch0=5";
    LinkHealth ht(t.read(), "ue", t.clock());
    const bool open_t = ht.check().alarms().empty();
    t.keys["TX_HOST_STATUS"] = "eob_recloses=0";
    const bool closed_t = ht.check().alarms().empty();
    t.keys["TX_HOST_STATUS"] = "eob_recloses=7 eob_recloses_ch0=7";
    check(open_t && closed_t && eq(ht.check().alarms(), {"host.eob_recloses_ch0 +7"}),
          "after the last stream's close a reopened stream counts from zero (mutation: a status judged answered "
          "only by its per-stream keys, the old 5 carried, +2)");
    FakeNode e;
    LinkHealth he(e.read(), "bs", e.clock());
    e.keys["RX_HOST_STATUS"] = "";  // a failed RX read: nothing of it returned
    const bool blank = he.check().alarms().empty();
    e.keys["RX_HOST_STATUS"] = "rxq_ovfl=0 rxq_ovfl_ch0=0 ring_ovfl=0 ring_ovfl_ch0=0";
    check(blank && he.check().alarms().empty(),
          "an empty host read keeps its baseline, so the next full read reports nothing new (mutation: every "
          "absent host key dropped)");
  }
  {  // 2.7 "Preflight"
    check(preflightItems("ok known DAC0.0:FIFO_OVR(HS-207)").empty() &&
              preflightItems("FAIL ADC0.1:FIFOUSRDAT_OF,FIFOUSRDAT_UF;ADC2.1:X known DAC0.0:FIFO_OVR(HS-207)") ==
                  std::set<std::string>{"ADC0.1:FIFOUSRDAT_OF,FIFOUSRDAT_UF", "ADC2.1:X"},
          "preflight items: failures split from the known part (mutation: the known tail kept)");
    check(preflightItems("FAIL A suppressed DAC0.0:FIFO_OVR(46600/s,backoff=8s)") == std::set<std::string>{"A"} &&
              preflightItems("FAIL A;B known X suppressed Y") == std::set<std::string>{"A", "B"},
          "a suppression is never a failure, with or without a known part (mutation: cut at ' known ' alone)");
    check(preflightKey("TX0:late=3") == "TX0:late" && preflightKey("TX0:late=4") == preflightKey("TX0:late=3") &&
              preflightKey("ADC2.1:X") == "ADC2.1:X",
          "an item's key is its text before the first '=' (mutation: the whole text keyed)");
    FakeNode n;
    n.keys["RFDC_PREFLIGHT"] = "FAIL ADC0.1:FIFOUSRDAT_OF,FIFOUSRDAT_UF known X\n";
    LinkHealth h(n.read(), "bs", n.clock());
    check(h.startFailures() == std::set<std::string>{"ADC0.1:FIFOUSRDAT_OF,FIFOUSRDAT_UF"} &&
              eq(h.check().alarms(), {"preflight new FAIL ADC0.1:FIFOUSRDAT_OF,FIFOUSRDAT_UF"}) &&
              h.check().alarms().empty(),
          "a FAIL standing at the session's start alarms on the first check, then stands (mutation: the start's "
          "items taken as a silent baseline, SH-428 item 2)");
    n.keys["RFDC_PREFLIGHT"] = "FAIL TX0:late=3;ADC0.1:FIFOUSRDAT_OF,FIFOUSRDAT_UF\n";
    const auto a1 = h.check().alarms();
    n.keys["RFDC_PREFLIGHT"] = "FAIL TX0:late=4;ADC0.1:FIFOUSRDAT_OF,FIFOUSRDAT_UF\n";
    const auto a2 = h.check().alarms();
    n.keys["RFDC_PREFLIGHT"] = "ok\n";
    const auto a3 = h.check().alarms();
    n.keys["RFDC_PREFLIGHT"] = "FAIL TX0:late=5\n";
    check(eq(a1, {"preflight new FAIL TX0:late=3"}) && a2.empty() && a3.empty() &&
              eq(h.check().alarms(), {"preflight new FAIL TX0:late=5"}),
          "TX0:late=3 then =4 is one standing item; gone and back, it alarms again (mutation: the whole text keyed, "
          "=4 then reads as new)");
    FakeNode g;
    g.keys["RFDC_PREFLIGHT"] = "FAIL ADC0.1:SUBADC_DCDR\n";
    LinkHealth hg(g.read(), "ue", g.clock());
    g.keys["RFDC_PREFLIGHT"] = "ok\n";
    check(eq(hg.check().alarms(), {"preflight new FAIL ADC0.1:SUBADC_DCDR"}),
          "an item in the start read alarms on the first check even when it is gone by then (mutation: only the "
          "first check's own read judged)");
  }
  {  // every TX alarm field is collected under its own name
    std::map<std::string, std::string> k;
    Read rd = [&k](const std::string& key) { return k.count(key) != 0u ? k.at(key) : std::string(); };
    k["TX_BANK_STATUS"] =
        "ch0:drops=1,late=2,under=3,seqerr=4,zerofill=5,efault=6,smiss=7,clkerr=8,aclose=9,malformed=10,acked=11,"
        "epoch=0:0";
    const auto g = collectCounters(rd);
    const std::vector<std::pair<std::string, long long>> want = {
        {"drops", 1}, {"late", 2},   {"under", 3},  {"seqerr", 4}, {"zerofill", 5},
        {"efault", 6}, {"smiss", 7}, {"clkerr", 8}, {"aclose", 9}, {"malformed", 10}};
    for (const auto& w : want) {
      const auto it = g.find("tx0." + w.first);
      check(it != g.end() && it->second == w.second,
            "TX alarm field " + w.first + " is collected (mutation: " + w.first + " dropped from txAlarmFields)");
    }
    check(g.count("tx0.acked") == 0, "acked is not an alarm field (mutation: every bank field collected)");
  }
  {  // 2.7 "A fall": the TX bank re-baselines on an epoch change (HS-220)
    for (const char* f : {"drops", "late", "under", "seqerr", "aclose", "malformed", "gated", "acked", "played"}) {
      const std::string key = std::string("tx0.") + f;
      check(counterIncreases({{"tx0.epoch", 3}, {key, 65530}}, {{"tx0.epoch", 3}, {key, 4}}) == Counters{{key, 10}},
            std::string("HS-220: ") + f + " wraps mod 2^16 within an epoch, 65530 -> 4 is +10 (mutation: " + f +
                " dropped from txWrapsMod16)");
    }
    for (const char* f : {"zerofill", "efault", "smiss", "clkerr"}) {
      const std::string key = std::string("tx0.") + f;
      check(counterIncreases({{"tx0.epoch", 3}, {key, 65530}}, {{"tx0.epoch", 3}, {key, 4}}) == Counters{{key, 4}},
            std::string("HS-220: ") + f + " does not wrap: a fall within an epoch reports its value, 4 (mutation: " +
                f + " added to txWrapsMod16, +10)");
    }
    check(counterIncreases({{"tx0.epoch", 3}, {"tx0.late", 500}}, {{"tx0.epoch", 4}, {"tx0.late", 7}}) ==
              Counters{{"tx0.late", 7}},
          "HS-220: after a clear (a new epoch) the values ARE the counts since it (mutation: differenced across the "
          "epoch change, 65043)");
    check(counterIncreases({{"tx0.epoch", 3}, {"tx0.late", 0}}, {{"tx0.epoch", 4}, {"tx0.late", 0}}).empty(),
          "HS-220: the epoch itself is never an alarm (mutation: the epoch skip dropped)");
    check(counterIncreases({{"tx0.late", 9}}, {{"tx0.epoch", 2}, {"tx0.late", 12}}).empty(),
          "HS-220: the first usable poll after none is a baseline (mutation: differenced against no epoch)");
    std::map<std::string, std::string> k;
    Read rd = [&k](const std::string& key) { return k.count(key) != 0u ? k.at(key) : std::string(); };
    k["TX_BANK_STATUS"] = "ch0:late=3,zerofill=5,epoch=7:8";
    const auto torn = collectCounters(rd);
    k["TX_BANK_STATUS"] = "ch0:late=3,zerofill=5,epoch=65543:65543";
    const auto busy = collectCounters(rd);
    k["TX_BANK_STATUS"] = "ch0:late=3,zerofill=5";
    const auto noepoch = collectCounters(rd);
    check(torn.empty() && busy.empty(),
          "HS-220: a poll caught in a clear (torn, or clear_busy set) is not used, zerofill included (mutation: only "
          "the wrapping counters dropped)");
    check(noepoch.empty(),
          "a TX bank without the epoch field is not used (mutation: the pre-HS-220 plain rule restored)");
    double t = 0.0;
    k["TX_BANK_STATUS"] = "ch0:late=65530,zerofill=0,epoch=7:7";
    LinkHealth h(rd, "ue", [&t] { return t += 5.0; });
    k["TX_BANK_STATUS"] = "ch0:late=2,zerofill=0,epoch=7:8";  // torn: skipped
    const bool quiet = h.check().alarms().empty();
    k["TX_BANK_STATUS"] = "ch0:late=4,zerofill=0,epoch=7:7";
    const auto a = h.check().alarms();
    k["TX_BANK_STATUS"] = "ch0:late=2,zerofill=0,epoch=8:8";  // a clear, 2 since
    const auto b = h.check().alarms();
    check(quiet && a == std::vector<std::string>{"tx0.late +10"} && b == std::vector<std::string>{"tx0.late +2"},
          "HS-220 end to end: a torn poll is skipped, a wrap counts +10, a clear then counts from zero (mutation: "
          "the baseline reset on the torn poll)");
  }
  {  // clean, then each alarm class
    FakeNode n;
    LinkHealth h(n.read(), "bs", n.clock());
    auto r = h.check();
    bool ok = r.alarms().empty() && r.line().find("clean") != std::string::npos;
    n.keys["TX_BANK_STATUS"] = replaceAll(n.keys["TX_BANK_STATUS"], "late=0", "late=4");
    n.keys["RX_HOST_STATUS"] = "rxq_ovfl=7 rxq_ovfl_ch0=7 ring_ovfl=0 ring_ovfl_ch0=0";
    n.keys["RFDC_INTR_FIRE_COUNT"] = "46000";
    r = h.check();
    ok = ok && eq(r.alarms(), {"host.rxq_ovfl_ch0 +7", "tx0.late +4"}) && r.irq_per_s > 0;
    ok = ok && h.check().alarms().empty();
    n.keys["RFDC_PREFLIGHT"] = "FAIL ADC0.0:OVR_RANGE\n";
    ok = ok && eq(h.check().alarms(), {"preflight new FAIL ADC0.0:OVR_RANGE"});
    check(ok, "clean, then a counter rise, a quiet period, a new FAIL (mutation: an alarm class dropped)");
  }
  {  // the real captures
    if (argc > 1) {  // pre-HS-220, device d72ee358
      auto keys = loadFixture(argv[1]);
      check(keys.size() == 7, "the pre-HS-220 fixture has its 7 keys");
      if (keys.size() == 7) {
        Read rd = [&keys](const std::string& k) { return keys.at(k); };
        const auto g = collectCounters(rd);
        check(g.count("tx0.late") == 0 && g.count("tx1.aclose") == 0,
              "pre-HS-220 capture: its TX banks carry no epoch and are not used (mutation: the plain rule restored)");
        check(txBanksWithoutEpoch(keys.at("TX_BANK_STATUS")) == std::vector<int>{0, 1},
              "pre-HS-220 capture: both TX banks are named as unjudged (mutation: the epoch-less banks not listed)");
        check(g.at("host.ring_ovfl_ch0") == 22718 && g.at("egress.marked_p0") == 7 && g.at("rx3.aborts") == 0 &&
                  g.count("host.ring_ovfl") == 0,
              "pre-HS-220 capture: the per-stream host keys, four RX banks and the per-port groups parse");
        double t = 0.0;
        LinkHealth h(rd, "bs", [&t] { return t += 5.0; });
        check(eq(h.check().alarms(), {"preflight new FAIL ADC0.1:FIFOUSRDAT_OF,FIFOUSRDAT_UF"}),
              "pre-HS-220 capture: SH-421's standing FAIL alarms on the first check; the HS-207 known item does not");
        keys["RFDC_INTR_FIRE_COUNT"] = std::to_string(14032361 + 5 * 46600);
        keys["RX_HOST_STATUS"] = "rxq_ovfl=0 rxq_ovfl_ch0=0 ring_ovfl=22800 ring_ovfl_ch0=22800";
        const auto r = h.check();
        std::printf("  %s\n", r.line().c_str());
        check(eq(r.alarms(), {"host.ring_ovfl_ch0 +82"}) && r.irq_per_s > 46000 && r.irq_per_s < 47000,
              "pre-HS-220 capture: a ring drop rise alarms by its stream, and the HS-207 IRQ rate reads 46.6k/s");
      }
    }
    if (argc > 2) {  // the current stack, DEMO_VERIFICATION 9.83 FI1
      auto keys = loadFixture(argv[2]);
      check(keys.size() == 5, "the current-stack fixture has its 5 keys");
      Read rd = [&keys](const std::string& k) { return keys.count(k) != 0u ? keys.at(k) : std::string(); };
      const auto g = collectCounters(rd);
      check(txBanksWithoutEpoch(keys["TX_BANK_STATUS"]).empty(),
            "current capture: no TX bank is unjudged (mutation: every bank listed)");
      check(g.at("tx0.epoch") == 587 && g.at("tx1.epoch") == 521 && g.at("tx0.late") == 0 && g.count("tx0.acked") == 0,
            "current capture: the TX banks carry their epochs (587, 521) and are used (mutation: the epoch parse "
            "broken)");
      check(g.count("host.tdd_straddle_ch0") == 1 && g.count("host.eob_recloses_ch1") == 1 &&
                g.count("host.tdd_drop_ch0") == 0 && g.count("host.rxq_ovfl") == 0 &&
                g.count("host.anchor_rejects_ch0") == 0,
            "current capture: the host alarm fields per stream, not tdd_drop, the totals or the pacer's state");
      check(g.count("host.tdd_refused_ch2") == 0,
            "current capture: its tdd_refused, a key HOUDINI_PROTOCOL 6 retired, is not judged (mutation: the "
            "retired key left in the host field list)");
      double t = 0.0;
      LinkHealth h(rd, "ue", [&t] { return t += 5.0; });
      const auto first = h.check().alarms();
      const bool settled = h.check().alarms().empty();
      keys["TX_BANK_STATUS"] = replaceAll(keys["TX_BANK_STATUS"], "acked=784,late=0", "acked=1284,late=2");
      check(eq(first, {"preflight new FAIL ADC0.1:SUBADC_DCDR"}) && settled &&
                eq(h.check().alarms(), {"tx0.late +2"}),
            "current capture: the bring-up's ADC latch alarms once if not cleared, the beacon's acked rise is not "
            "an alarm, a late rise within epoch 587 is");
    }
  }
  std::printf("%s: %d failure(s)\n", g_fail == 0 ? "ALL PASS" : "FAILED", g_fail);
  return g_fail == 0 ? 0 : 1;
}
