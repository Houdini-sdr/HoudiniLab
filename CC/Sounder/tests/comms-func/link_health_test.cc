/**
 * @file link_health_test.cc
 * @brief houdini/link_health.h against the software lane's link-alarm contract
 *        (SoapyHoudiniSDR shared/HOUDINI_PROTOCOL.md section 2.7), clause by
 *        clause, and against the driver's own golden renders of the keys
 *        (fixtures/link_health/driver_golden.txt). NO hardware. Each check
 *        names the mutation that breaks it.
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

std::string replaceAll(std::string s, const std::string& a, const std::string& b) {
  for (size_t p = s.find(a); p != std::string::npos; p = s.find(a, p + b.size())) s.replace(p, a.size(), b);
  return s;
}

struct FakeNode {
  std::map<std::string, std::string> keys = {
      {"TX_BANK_STATUS",
       "ch0:acked=9,late=0,under=0,seqerr=0,zerofill=0,fill=3,hwm=3,drops=0,last_seq=8,active=1,cap=64,gated=0,"
       "smiss=0,aclose=0,parked=0,late_seen=0,efault=0,played=9,clear_drops=0,clear_busy=0,tick=100:130,"
       "malformed=0,cancels=0,flushed=0,clock_aborts=0"},
      {"RX_BANK_STATUS", "ch0:gated=0,aborts=0,hwm=12,halted=0,stop=none"},
      {"TX_HOST_STATUS", "eob_recloses=0 eob_recloses_ch0=0"},
      {"RX_HOST_STATUS", "rxq_ovfl=0 rxq_ovfl_ch0=0 ring_ovfl=0 ring_ovfl_ch0=0"},
      {"EGRESS_STATUS", "drop=p0:0,p1:0,p2:0,p3:0;stalls=0;marked=0;link_down=0"},
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
    const auto g = parseEgressStatus("drop=p0:3,p1:0,p2:0,p3:4294967296;stalls=2;marked=18446744073709551615;link_down=7");
    check(g.at("drop_p0") == 3 && g.at("drop_p3") == 4294967296ULL && g.at("marked") == 18446744073709551615ULL &&
              g.at("stalls") == 2 && g.at("link_down") == 7 && g.size() == 7 && parseEgressStatus("").empty() &&
              parseEgressStatus("junk;x").empty(),
          "egress: per-port drops beside flat counters, seven in all, a 64-bit count read whole; junk yields nothing "
          "(mutations: marked split as per-port; a signed 64-bit parse, which throws or drops the count at its "
          "ceiling)");
    const auto f = parseFlatCounts("rxq_ovfl=2 rxq_ovfl_ch0=2 junk x=y z=-1 big=18446744073709551616");
    check(f.size() == 2 && f.at("rxq_ovfl") == 2 && f.at("rxq_ovfl_ch0") == 2,
          "flat counts skip malformed tokens and a value past 64 bits (mutations: a non-digit value accepted; the "
          "overflow guard dropped, which wraps 2^64 to 0)");
  }
  {  // 2.7 "A fall": a host counter that fell was cleared; a device counter never clears
    check(counterIncreases({{"rx0.gated", 1}, {"host.rxq_ovfl_ch0", 5}, {"egress.drop_p0", 9}},
                           {{"rx0.gated", 3}, {"host.rxq_ovfl_ch0", 2}, {"egress.drop_p0", 9}, {"host.rxq_ovfl_ch2", 4}}) ==
              Counters{{"rx0.gated", 2}, {"host.rxq_ovfl_ch0", 2}, {"host.rxq_ovfl_ch2", 4}},
          "a rise alarms, a host counter's fall reports its current value (5 -> 2 is +2), a new host key counts "
          "from zero (mutation: a host fall re-based silently)");
    check(counterIncreases({{"rx0.gated", 1}}, {{"rx0.gated", 1}, {"rx1.aborts", 2}, {"tx0.late", 7}, {"egress.marked", 3}})
              .empty(),
          "a device counter with no previous read is a baseline: it counts from the PL load, not from the session "
          "(mutation: a new device key counted from zero, the whole standing total a false alarm)");
    const Counters before{{"tx0.late", 500}, {"tx0.drops", 2}, {"rx0.gated", 40}, {"egress.drop_p0", 9},
                          {"egress.stalls", 3}, {"host.rxq_ovfl_ch0", 1}};
    const Counters after{{"tx0.late", 7}, {"tx0.drops", 5}, {"rx0.gated", 41}, {"egress.drop_p0", 12},
                         {"egress.stalls", 0}, {"host.rxq_ovfl_ch0", 4}};
    check(plReload(before, after) == "tx0.late 500 -> 7" && plReload(before, before).empty() &&
              plReload({{"host.rxq_ovfl_ch0", 5}, {"tx0.smiss", 1}, {"egress.stalls", 9}, {"tx0.clock_aborts", 9}},
                       {{"host.rxq_ovfl_ch0", 2}, {"tx0.smiss", 0}, {"egress.stalls", 1}, {"tx0.clock_aborts", 1}})
                  .empty(),
          "a 64-bit device counter below its previous read is a PL reload, named with both reads; a host counter, a "
          "sticky flag or a 32-bit count that fell is not (mutation: every fall read as a reload)");
    check(counterIncreases(before, after) == Counters{{"host.rxq_ovfl_ch0", 3}},
          "after a PL reload no device counter is differenced, the ones that rose included, and the host's still "
          "are (mutations: a fall's value reported as the rise, tx0.late +7; only the fallen counter skipped, "
          "tx0.drops +3 across two PL loads)");
    check(counterIncreases({{"egress.marked", 18446744073709551000ULL}}, {{"egress.marked", 18446744073709551615ULL}}) ==
              Counters{{"egress.marked", 615}} &&
              counterIncreases({{"tx0.late", 65530}}, {{"tx0.late", 65540}}) == Counters{{"tx0.late", 10}} &&
              counterIncreases({{"rx0.aborts", 4294967290ULL}}, {{"rx0.aborts", 4294967300ULL}}) == Counters{{"rx0.aborts", 10}},
          "a 64-bit count differences plainly across 2^16, 2^32 and up to its ceiling (mutations: the retired mod "
          "2^16 or a mod 2^32 on a bank counter; a signed difference)");
    check(counterIncreases({{"egress.stalls", 4294967294ULL}, {"tx0.clock_aborts", 4294967295ULL}},
                           {{"egress.stalls", 1}, {"tx0.clock_aborts", 2}}) ==
              Counters{{"egress.stalls", 3}, {"tx0.clock_aborts", 3}},
          "the 32-bit counts wrap: their rise is the change mod 2^32 (mutation: differenced as 64-bit, which reads "
          "the wrap as a fall or as 2^64 - 4294967293)");
    for (const char* f : {"efault", "smiss"}) {
      const std::string key = std::string("tx0.") + f;
      check(counterIncreases({{key, 1}}, {{key, 0}}).empty() && plReload({{key, 1}}, {{key, 0}}).empty() &&
                counterIncreases({{key, 0}}, {{key, 1}}) == Counters{{key, 1}} &&
                counterIncreases({}, {{key, 1}}) == Counters{{key, 1}},
            std::string("the sticky flag ") + f + " alarms when it sets, and its clearing is neither a rise nor a PL "
            "reload (mutation: " + f + " differenced as a 64-bit counter)");
    }
  }
  {  // an empty or partial read between two full ones must not erase the baseline
    const Counters full{{"a.x", 10}, {"b.y", 7}};
    const Counters kept = carryCounters(carryCounters(Counters{}, full, HostRead{}), Counters{}, HostRead{});
    check(counterIncreases(kept, full).empty(),
          "an empty read does not erase the baseline: the next full read reports nothing new (mutation: prev = cur, "
          "every counter's running total reads as new)");
    const Counters later{{"a.x", 12}, {"b.y", 7}};
    check(counterIncreases(carryCounters(kept, Counters{{"b.y", 7}}, HostRead{}), later) == Counters{{"a.x", 2}},
          "a partial read keeps the missing counter's last value (mutation: prev = cur, a +12 false alarm)");
  }
  {  // 2.7 "Egress counters" and "No other alarm": a count standing nonzero is not an alarm
    FakeNode n;
    n.keys["EGRESS_STATUS"] = "drop=p0:1148909,p1:0,p2:0,p3:0;stalls=4;marked=13;link_down=2";
    LinkHealth h(n.read(), "bs", n.clock());
    check(h.check().alarms().empty(),
          "drops and stalls counted since the PL load, before the session began, are not alarms (mutation: a "
          "nonzero stalls read as stalled now)");
    n.keys["EGRESS_STATUS"] = "drop=p0:1148912,p1:0,p2:0,p3:0;stalls=4;marked=13;link_down=5";
    check(eq(h.check().alarms(), {"egress.drop_p0 +3", "egress.link_down +3"}),
          "new drops and frames drained with the link down each alarm by their rise (mutation: link_down not "
          "collected)");
    n.keys["EGRESS_STATUS"] = "drop=p0:1148912,p1:0,p2:0,p3:0;stalls=5;marked=13;link_down=5";
    check(eq(h.check().alarms(), {"egress.stalls +1"}) && h.check().alarms().empty(),
          "a new stall alarms once, by the stalls count's rise, and then stands (mutation: stalls not collected, "
          "the retired stall_seen read instead)");
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
        "ch0:drops=1,late=2,under=3,seqerr=4,zerofill=5,efault=1,smiss=1,aclose=9,malformed=10,acked=11,"
        "clock_aborts=12,flushed=13,tick=5:6";
    const auto g = collectCounters(rd);
    const std::vector<std::pair<std::string, unsigned long long>> want = {
        {"drops", 1},  {"late", 2},  {"under", 3},  {"seqerr", 4},     {"zerofill", 5},
        {"efault", 1}, {"smiss", 1}, {"aclose", 9}, {"malformed", 10}, {"clock_aborts", 12}, {"acked", 11}};
    for (const auto& w : want) {
      const auto it = g.find("tx0." + w.first);
      check(it != g.end() && it->second == w.second,
            "TX field " + w.first + " is collected (mutation: " + w.first + " dropped from the TX fields)");
    }
    check(g.count("tx0.flushed") == 0 && g.count("tx0.tick") == 0 && g.size() == want.size(),
          "flushed and the tick pair are not read (mutation: every bank field collected)");
    check(counterIncreases({{"tx0.acked", 11}, {"tx0.played", 4}}, {{"tx0.acked", 900}, {"tx0.played", 800}}).empty(),
          "acked and played are read as reload witnesses and never reported: a beacon's every burst moves them "
          "(mutation: the witnesses differenced as alarm counters)");
    const Counters clean{{"tx0.acked", 784}, {"tx0.late", 0}, {"tx0.clock_aborts", 13}, {"egress.stalls", 5},
                         {"egress.drop_p0", 0}};
    const Counters reloaded{{"tx0.acked", 3}, {"tx0.late", 0}, {"tx0.clock_aborts", 0}, {"egress.stalls", 0},
                            {"egress.drop_p0", 0}};
    check(plReload(clean, reloaded) == "tx0.acked 784 -> 3" && counterIncreases(clean, reloaded).empty(),
          "a PL reload on a clean link, every alarm counter 0 before and after, shows on a witness, and the 32-bit "
          "counts standing above 0 are not differenced across it (mutation: no witness collected, which reports "
          "egress.stalls +4294967291 and tx0.clock_aborts +4294967283 and no reload)");
  }
  {  // 2.7 "A fall", the TX bank: used as read, with no clear count to gate it
    std::map<std::string, std::string> k;
    Read rd = [&k](const std::string& key) { return k.count(key) != 0u ? k.at(key) : std::string(); };
    double t = 0.0;
    FakeNode proto;
    k["TX_BANK_STATUS"] = proto.keys["TX_BANK_STATUS"];
    check(collectCounters(rd).count("tx0.late") == 1,
          "a TX bank's counters are used as the device renders them, which carries no epoch field (mutation: the "
          "retired epoch gate, which skips every bank and reads the session clean whatever its TX does)");
    LinkHealth h(rd, "ue", [&t] { return t += 5.0; });
    const bool quiet = h.check().alarms().empty();
    k["TX_BANK_STATUS"] = replaceAll(replaceAll(k["TX_BANK_STATUS"], "late=0", "late=70000"), "zerofill=0", "zerofill=4294967297");
    const auto a = h.check().alarms();
    k["TX_BANK_STATUS"] = replaceAll(k["TX_BANK_STATUS"], "clock_aborts=0", "clock_aborts=1");
    const auto b = h.check().alarms();
    k["TX_BANK_STATUS"] = replaceAll(replaceAll(k["TX_BANK_STATUS"], "late=70000", "late=2"), "clock_aborts=1", "clock_aborts=0");
    const auto c = h.check();
    k["TX_BANK_STATUS"] = replaceAll(k["TX_BANK_STATUS"], "late=2", "late=3");
    const auto d = h.check().alarms();
    check(quiet && eq(a, {"tx0.late +70000", "tx0.zerofill +4294967297"}),
          "end to end: a rise past 2^16 and one past 2^32 report whole (mutation: a modulus on the bank counters)");
    check(eq(b, {"tx0.clock_aborts +1"}),
          "a clock event that aborted the bank alarms by clock_aborts, the frames it took being in no other "
          "counter (mutation: clock_aborts not collected)");
    check(eq(c.alarms(), {"PL reloaded (tx0.late 70000 -> 2)"}) && c.line().find("PL reloaded (tx0.late 70000 -> 2)") != std::string::npos,
          "a bank counter that went back is one PL-reload alarm, not a rise of its value and not a 2^32 - 1 rise "
          "of clock_aborts (mutations: the fall's value as the rise; the 32-bit counts differenced across the "
          "reload)");
    check(eq(d, {"tx0.late +1"}),
          "the read after a PL reload is the new baseline (mutation: the baseline kept from before the reload, a "
          "second reload alarm)");
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
  {  // the driver's own renders
    if (argc > 1) {
      auto keys = loadFixture(argv[1]);
      check(keys.size() == 7, "the fixture has its 7 keys");
      Read rd = [&keys](const std::string& k) { return keys.count(k) != 0u ? keys.at(k) : std::string(); };
      const auto g = collectCounters(rd);
      check(g.at("tx0.late") == 1 && g.at("tx1.late") == 101 && g.at("tx1.malformed") == 110 &&
                g.at("tx0.clock_aborts") == 13 && g.at("tx0.acked") == 4294967299ULL && g.count("tx0.epoch") == 0,
            "the golden render: both TX banks are used, each field under its own name (mutation: a substring "
            "search for a field, which reads zerofill's value as fill's or the tick pair as a count)");
      check(g.at("rx0.aborts") == 3 && g.at("rx1.gated") == 7 && g.count("rx1.hwm") == 0 && g.count("rx1.halted") == 0,
            "the golden render: the RX banks' gated and aborts, not the high-water mark or the fail-stop state");
      check(g.at("egress.drop_p3") == 4294967296ULL && g.at("egress.marked") == 18446744073709551615ULL &&
                g.at("egress.stalls") == 5 && g.at("egress.link_down") == 4294967294ULL,
            "the golden render: the egress counts whole, the 64-bit ceiling included (mutation: a signed parse)");
      check(g.count("host.tdd_straddle_ch0") == 1 && g.count("host.eob_recloses_ch1") == 1 &&
                g.count("host.tdd_drop_ch0") == 0 && g.count("host.rxq_ovfl") == 0 &&
                g.count("host.anchor_rejects_ch1") == 0 && g.count("host.ring_bytes_ch0") == 0,
            "the golden render: the host alarm fields per stream, not tdd_drop, the totals, the pacer's state or "
            "the ring's size");
      double t = 0.0;
      LinkHealth h(rd, "ue", [&t] { return t += 5.0; });
      const bool settled = h.check().alarms().empty() && h.check().alarms().empty();
      keys["TX_BANK_STATUS"] = replaceAll(keys["TX_BANK_STATUS"], "acked=4294967299,late=1", "acked=4294967799,late=3");
      check(settled && eq(h.check().alarms(), {"tx0.late +2"}),
            "the golden render: counts standing since the PL load and a suppressed preflight item are no alarm, a "
            "replay's acked rise is not one, a late rise is");
    }
  }
  std::printf("%s: %d failure(s)\n", g_fail == 0 ? "ALL PASS" : "FAILED", g_fail);
  return g_fail == 0 ? 0 : 1;
}
