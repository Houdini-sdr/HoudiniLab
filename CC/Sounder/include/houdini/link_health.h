/**
 * @file houdini/link_health.h
 * @brief The software lane's link-health monitor (SoapyHoudiniSDR
 *        host/examples/link_health.py + setting_reports.py, feat/dual-band-freqplan
 *        d3ade5a), ported so the sounder can run it on its OWN device handle
 *        during the dual-band demo (AP-79). A second make() on a node resets
 *        it, so the monitor must never open its own connection.
 *
 * Every read is passive. Counters are judged by their INCREASE since the
 * previous check: a counter that goes down was cleared (a stream setup clears
 * its channel's bank) and is re-based, not flagged. The egress per-port counts
 * saturate at 255 and stall_seen is sticky (only an eth reset clears either),
 * so those are flagged whenever they stand there. The preflight is judged
 * against its own baseline: the FAIL items standing at the start (SH-421's
 * idle ADC sibling, until its fix lands) are reported once, and a check alarms
 * only on an item that is new or comes back. Any change of a configuration
 * section of RFDC_SNAPSHOT since the baseline is drift.
 *
 * The parsers mirror setting_reports.py function for function (the T0-pinned
 * reference; unknown bank fields are ignorable, the bank strings are
 * append-only) and link_health_test pins them against real captures from a
 * streaming mode-V node. What this cannot see, the application counts itself:
 * readStream flags, short reads, timestamp jumps, writeStream returns.
 *
 * RENEW OPEN SOURCE LICENSE: http://renew-wireless.org/license
 */
#pragma once

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cstdio>
#include <functional>
#include <map>
#include <set>
#include <sstream>
#include <string>
#include <vector>

namespace houdini {
namespace health {

using Counters = std::map<std::string, long long>;
using Read = std::function<std::string(const std::string&)>;

namespace detail {
inline std::vector<std::string> split(const std::string& s, char d) {
  std::vector<std::string> out;
  std::string cur;
  for (char c : s) {
    if (c == d) { out.push_back(cur); cur.clear(); } else { cur += c; }
  }
  out.push_back(cur);
  return out;
}
inline bool isDigits(const std::string& v) {
  return !v.empty() && std::all_of(v.begin(), v.end(), [](unsigned char c) { return std::isdigit(c) != 0; });
}
inline bool isInt(const std::string& v) {
  if (v.empty()) return false;
  size_t i = (v[0] == '-' || v[0] == '+') ? 1 : 0;
  return i < v.size() && isDigits(v.substr(i));
}
}  // namespace detail

/// TX_BANK_STATUS / RX_BANK_STATUS 'ch0:acked=..,late=..;ch1:..' -> {chan: {field: value}}.
/// A chunk without 'ch<N>:' is skipped.
inline std::map<int, std::map<std::string, std::string>> parseBankStatus(const std::string& raw) {
  std::map<int, std::map<std::string, std::string>> out;
  for (const auto& chunk : detail::split(raw, ';')) {
    const auto colon = chunk.find(':');
    if (colon == std::string::npos) continue;
    std::string ch_s = chunk.substr(0, colon);
    if (ch_s.rfind("ch", 0) == 0) ch_s = ch_s.substr(2);
    if (!detail::isInt(ch_s)) continue;
    auto& d = out[std::stoi(ch_s)];
    for (const auto& kv : detail::split(chunk.substr(colon + 1), ',')) {
      const auto eq = kv.find('=');
      if (eq != std::string::npos) d[kv.substr(0, eq)] = kv.substr(eq + 1);
    }
  }
  return out;
}

/// TX_HOST_STATUS / RX_HOST_STATUS 'k=v k_ch0=v ..' -> {k: int}; a token without
/// '=' or with a non-digit value is skipped.
inline Counters parseFlatCounts(const std::string& raw) {
  Counters out;
  std::istringstream is(raw);
  std::string tok;
  while (is >> tok) {
    const auto eq = tok.find('=');
    if (eq == std::string::npos) continue;
    const std::string v = tok.substr(eq + 1);
    if (detail::isDigits(v)) out[tok.substr(0, eq)] = std::stoll(v);
  }
  return out;
}

/// EGRESS_STATUS 'drop=p0:a,p1:b;stall_seen=s,stall_evt=e;marked=p0:..' ->
/// {'drop_p0': a, .., 'stall_seen': s, 'stall_evt': e, 'marked_p0': ..}.
inline Counters parseEgressStatus(const std::string& raw) {
  Counters out;
  for (const auto& group : detail::split(raw, ';')) {
    const auto eq = group.find('=');
    if (eq == std::string::npos) continue;
    const std::string key = group.substr(0, eq), rest = group.substr(eq + 1);
    const bool per_port = rest.size() > 2 && rest[0] == 'p' && std::isdigit(static_cast<unsigned char>(rest[1])) != 0 &&
                          rest.find(':') != std::string::npos;
    if (per_port) {
      for (const auto& item : detail::split(rest, ',')) {
        const auto c = item.find(':');
        if (c == std::string::npos) continue;
        const std::string v = item.substr(c + 1);
        if (detail::isDigits(v)) out[key + "_" + item.substr(0, c)] = std::stoll(v);
      }
    } else {
      std::string g = group;
      std::replace(g.begin(), g.end(), ',', ' ');
      for (const auto& kv : parseFlatCounts(g)) out[kv.first] = kv.second;
    }
  }
  return out;
}

/// The FAIL part of an RFDC_PREFLIGHT verdict line. The grammar is
/// 'ok' | 'FAIL <item>;..', then an optional ' known <item>;..', then an
/// optional ' suppressed <item>;..' (SH-423's IRQ-storm backoff). Either tail
/// may appear without the other, so the FAIL part ends at the FIRST marker:
/// cutting at ' known ' alone glues 'suppressed ..' onto the last FAIL item.
/// A suppression is never a failure (the masked bit keeps latching, so a real
/// fault still reads FAIL). Returns "" for an 'ok' line.
inline std::string preflightFailBody(const std::string& line) {
  if (line.rfind("FAIL ", 0) != 0) return "";
  const std::string body = line.substr(5);
  return body.substr(0, std::min(body.find(" known "), body.find(" suppressed ")));
}

/// The FAIL items of an RFDC_PREFLIGHT verdict line ('ok ..' -> none):
/// 'FAIL A;B known C suppressed D' -> {A, B}.
inline std::set<std::string> preflightItems(const std::string& line) {
  std::set<std::string> out;
  const std::string body = preflightFailBody(line);
  for (const auto& i : detail::split(body, ';'))
    if (!i.empty()) out.insert(i);
  return out;
}

/// RFDC_SNAPSHOT -> {section: text} for its configuration sections. A section
/// opens at a line '<lowercase_keyword>:'; any other line continues the open
/// one. The counter sections (rx_intr, tx_banks, rx_banks) are dropped.
inline std::map<std::string, std::string> snapshotConfig(const std::string& raw) {
  std::map<std::string, std::string> out;
  std::string cur;
  bool open = false;
  for (const auto& line : detail::split(raw, '\n')) {
    size_t i = 0;
    while (i < line.size() && (std::islower(static_cast<unsigned char>(line[i])) != 0 || line[i] == '_')) ++i;
    if (i > 0 && i < line.size() && line[i] == ':') {
      cur = line.substr(0, i);
      std::string rest = line.substr(i + 1);
      if (!rest.empty() && std::isspace(static_cast<unsigned char>(rest[0])) != 0) rest = rest.substr(1);
      out[cur] = rest;
      open = true;
    } else if (open) {
      std::string& s = out[cur];
      s = s.empty() ? line : s + "\n" + line;
      while (!s.empty() && s.back() == '\n') s.pop_back();
    }
  }
  for (const char* c : {"rx_intr", "tx_banks", "rx_banks"}) out.erase(c);
  return out;
}

/// The snapshot lines that differ from the baseline: 'blocks: <old> -> <new>'.
inline std::vector<std::string> configDrift(const std::map<std::string, std::string>& base,
                                            const std::map<std::string, std::string>& now) {
  std::vector<std::string> out;
  std::set<std::string> secs;
  for (const auto& kv : base) secs.insert(kv.first);
  for (const auto& kv : now) secs.insert(kv.first);
  for (const auto& sec : secs) {
    const auto ia = base.find(sec), ib = now.find(sec);
    const auto a = ia == base.end() ? std::vector<std::string>{} : detail::split(ia->second, '\n');
    const auto b = ib == now.end() ? std::vector<std::string>{} : detail::split(ib->second, '\n');
    if (a == b) continue;
    for (size_t i = 0; i < std::max(a.size(), b.size()); ++i) {
      const std::string la = i < a.size() ? a[i] : "", lb = i < b.size() ? b[i] : "";
      if (la != lb) out.push_back(sec + ": " + la + " -> " + lb);
    }
  }
  return out;
}

// The alarm fields (link_health.py): the TX clean-burst set plus the TDD/stream
// set; RX gated windows and timed-start aborts; the host drop and re-close totals.
inline const std::vector<std::string>& txAlarmFields() {
  static const std::vector<std::string> f = {"drops", "late", "under", "seqerr", "zerofill",
                                             "efault", "smiss", "clkerr", "aclose"};
  return f;
}
inline const std::vector<std::string>& rxAlarmFields() {
  static const std::vector<std::string> f = {"gated", "aborts"};
  return f;
}
inline const std::vector<std::string>& hostAlarmFields() {
  static const std::vector<std::string> f = {"rxq_ovfl", "ring_ovfl", "eob_recloses"};
  return f;
}
constexpr long long kEgressSaturated = 0xFF;

/// The alarm counters from one pass over the status keys, flattened to
/// {'tx0.late', 'rx0.gated', 'host.rxq_ovfl', 'egress.drop_p0', ..}.
inline Counters collectCounters(const Read& read) {
  Counters out;
  auto bank = [&out](const std::string& raw, const std::string& pre, const std::vector<std::string>& fields) {
    for (const auto& ch : parseBankStatus(raw))
      for (const auto& f : fields) {
        const auto it = ch.second.find(f);
        if (it != ch.second.end() && detail::isInt(it->second))
          out[pre + std::to_string(ch.first) + "." + f] = std::stoll(it->second);
      }
  };
  bank(read("TX_BANK_STATUS"), "tx", txAlarmFields());
  bank(read("RX_BANK_STATUS"), "rx", rxAlarmFields());
  Counters host = parseFlatCounts(read("TX_HOST_STATUS"));
  for (const auto& kv : parseFlatCounts(read("RX_HOST_STATUS"))) host[kv.first] = kv.second;
  for (const auto& f : hostAlarmFields()) {
    const auto it = host.find(f);
    if (it != host.end()) out["host." + f] = it->second;
  }
  for (const auto& kv : parseEgressStatus(read("EGRESS_STATUS"))) out["egress." + kv.first] = kv.second;
  return out;
}

/// Egress counters that can no longer show a rise (saturated, or sticky stall).
inline std::vector<std::string> blindCounters(const Counters& cur) {
  std::vector<std::string> out;
  for (const auto& kv : cur) {
    const std::string& k = kv.first;
    if (k.rfind("egress.", 0) != 0) continue;
    if (k == "egress.stall_seen" && kv.second != 0) {
      out.push_back(k + "=1 (sticky: a stall happened; stall_evt no longer proves a new one)");
    } else if ((k.rfind("egress.drop_", 0) == 0 || k.rfind("egress.marked_", 0) == 0) && kv.second >= kEgressSaturated) {
      out.push_back(k + "=" + std::to_string(kv.second) + " (saturated: further drops cannot be counted)");
    }
  }
  return out;
}

/// {name: increase} for every counter that rose; a fall is a clear, and a
/// counter new since `prev` counts from zero.
inline Counters counterIncreases(const Counters& prev, const Counters& cur) {
  Counters out;
  for (const auto& kv : cur) {
    const auto it = prev.find(kv.first);
    const long long p = it == prev.end() ? 0 : it->second;
    if (kv.second > p) out[kv.first] = kv.second - p;
  }
  return out;
}

struct Report {
  std::string label;
  double seconds = 0.0;
  double irq_per_s = 0.0;
  std::string preflight;
  Counters increases;
  std::vector<std::string> new_failures, blind, drift;

  std::vector<std::string> alarms() const {
    std::vector<std::string> out;
    for (const auto& kv : increases) out.push_back(kv.first + " +" + std::to_string(kv.second));
    out.insert(out.end(), blind.begin(), blind.end());
    for (const auto& i : new_failures) out.push_back("preflight new FAIL " + i);
    for (const auto& d : drift) out.push_back("config " + d);
    return out;
  }
  std::string line() const {
    const auto a = alarms();
    std::string state;
    for (const auto& s : a) state += (state.empty() ? "" : "; ") + s;
    char b[96];
    std::snprintf(b, sizeof b, "] %.1f s: irq %.0f/s, preflight ", seconds, irq_per_s);
    return "[" + label + b + preflight + ": " + (a.empty() ? std::string("clean") : state);
  }
};

class LinkHealth {
 public:
  /// Baseline at construction: call it once the session is configured AND
  /// streaming. `now_s` is injectable for the test; the default is a steady clock.
  LinkHealth(Read read, std::string label, std::function<double()> now_s = {})
      : read_(std::move(read)), label_(std::move(label)), now_(std::move(now_s)) {
    if (!now_) {
      now_ = [] {
        return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
      };
    }
    rebaseline();
  }

  void rebaseline() {
    baseline_ = snapshotConfig(read_("RFDC_SNAPSHOT"));
    baseline_failures_ = preflightItems(preflight());
    standing_ = baseline_failures_;
    prev_ = collectCounters(read_);
    prev_irq_ = irq();
    prev_t_ = now_();
  }

  const std::set<std::string>& baselineFailures() const { return baseline_failures_; }

  Report check() {
    const Counters cur = collectCounters(read_);
    const long long ir = irq();
    const double t = now_();
    const double dt = std::max(t - prev_t_, 1e-9);
    Report r;
    r.label = label_;
    r.seconds = dt;
    r.irq_per_s = static_cast<double>(ir - prev_irq_) / dt;
    r.preflight = preflight();
    const auto items = preflightItems(r.preflight);
    for (const auto& i : items)
      if (standing_.count(i) == 0) r.new_failures.push_back(i);
    standing_ = items;
    standing_.insert(baseline_failures_.begin(), baseline_failures_.end());
    r.increases = counterIncreases(prev_, cur);
    r.blind = blindCounters(cur);
    r.drift = configDrift(baseline_, snapshotConfig(read_("RFDC_SNAPSHOT")));
    prev_ = cur;
    prev_irq_ = ir;
    prev_t_ = t;
    return r;
  }

 private:
  std::string preflight() const {
    const std::string s = read_("RFDC_PREFLIGHT");
    return s.substr(0, s.find('\n'));
  }
  long long irq() const {
    const std::string s = read_("RFDC_INTR_FIRE_COUNT");
    try { return std::stoll(s.substr(0, s.find(' '))); } catch (...) { return prev_irq_; }
  }

  Read read_;
  std::string label_;
  std::function<double()> now_;
  std::map<std::string, std::string> baseline_;
  std::set<std::string> baseline_failures_, standing_;
  Counters prev_;
  long long prev_irq_ = 0;
  double prev_t_ = 0.0;
};

}  // namespace health
}  // namespace houdini
