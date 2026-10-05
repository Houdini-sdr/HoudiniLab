/**
 * @file houdini/link_health.h
 * @brief The link-alarm monitor of a long session, on the sounder's OWN device
 *        handle (a second make() on a node resets it). Its rules are the
 *        software lane's contract, SoapyHoudiniSDR shared/HOUDINI_PROTOCOL.md
 *        section 2.7 "Link alarms on a long session"; their Python twin is gone.
 *
 * Every read is passive, and each counter is judged by its change since its
 * previous read. A rise alarms. The RX bank, host and egress counters carry no
 * epoch, so a fall means the counter was cleared and its current value is the
 * rise; the TX bank re-baselines on its clear epoch instead (HS-220), and a poll
 * caught in a clear is not used. Host fields come from the per-stream
 * '<field>_ch<N>' keys, never the totals, which fall when a stream closes. A
 * preflight item is keyed on its text before the first '=' and alarms on the
 * first read it appears in, a standing item at the session's start included.
 * Nothing else alarms: no configuration drift, and no counter merely standing
 * at its ceiling or a sticky bit merely standing set.
 *
 * What this cannot see, the application counts itself: readStream flags, short
 * reads, timestamp jumps, writeStream returns.
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

/// EGRESS_STATUS 'drop=p0:a,p1:b;stall_seen=s,stall_evt=e;marked=m' ->
/// {'drop_p0': a, .., 'stall_seen': s, 'stall_evt': e, 'marked': m}. A group
/// of 'p<k>:<n>' items is per port ('<group>_p<k>'), any other is flat.
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

// The alarm fields (HOUDINI_PROTOCOL 2.7): the TX bank's event counters, the RX
// bank's gated windows and timed-start aborts, the host's per-stream drops,
// re-closes and TDD refusals. tdd_drop is the slot cut itself, not a fault.
inline const std::vector<std::string>& txAlarmFields() {
  static const std::vector<std::string> f = {"drops", "late",  "under",  "seqerr", "zerofill",
                                             "efault", "smiss", "clkerr", "aclose", "malformed"};
  return f;
}

/// HS-220: the TX event counters WRAP mod 2^16, and the CLEAR_EPOCH register,
/// read before and after them (TX_BANK_STATUS "epoch=<before>:<after>", [16]
/// clear_busy, [15:0] clears since the PL load), says whether a clear came
/// between two polls. The TX_STREAM_CONTRACT section 2 rule, as the host
/// plugin's shared/houdini_tx_counter_delta.h applies it: a poll is usable only
/// if both reads match with bit 16 clear; within one epoch a wrapping counter's
/// delta is taken mod 2^16; after an epoch change the values ARE the counts
/// since the clear.
inline bool txWrapsMod16(const std::string& field) {
  // All nine HS-220 wrapping counters, so a field added to txAlarmFields later
  // is read right; zerofill is saturating (48-bit) and efault/smiss/clkerr are
  // sticky flags, so those difference plainly within an epoch.
  return field == "drops" || field == "late" || field == "under" || field == "seqerr" || field == "aclose" ||
         field == "malformed" || field == "gated" || field == "acked" || field == "played";
}
/// The usable epoch of an "epoch=<before>:<after>" value, or -1 when the poll
/// fails the read protocol (reads differ, clear_busy set, malformed).
inline long long txUsableEpoch(const std::string& v) {
  const auto c = v.find(':');
  if (c == std::string::npos) return -1;
  const std::string b = v.substr(0, c), a = v.substr(c + 1);
  if (!detail::isDigits(b) || !detail::isDigits(a) || b.size() > 10 || a.size() > 10) return -1;
  const long long eb = std::stoll(b), ea = std::stoll(a);
  return (eb == ea && eb >= 0 && eb <= 0xFFFF) ? eb : -1;
}
inline const std::vector<std::string>& rxAlarmFields() {
  static const std::vector<std::string> f = {"gated", "aborts"};
  return f;
}
inline const std::vector<std::string>& hostAlarmFields() {
  // tdd_refused left RX_HOST_STATUS with HOUDINI_PROTOCOL 6: a map the packets do
  // not tile now ends the stream (STREAM_ERROR) instead of counting.
  static const std::vector<std::string> f = {"rxq_ovfl", "ring_ovfl", "eob_recloses", "tdd_straddle"};
  return f;
}

/// Which host statuses answered a pass. An answer always carries its totals
/// (`eob_recloses=`, `rxq_ovfl=`, ..), also with no stream of its direction
/// open, so a status with no counts at all returned nothing.
struct HostRead {
  bool rx = false;
  bool tx = false;
};

/// The alarm counters from one pass over the status keys, flattened to
/// {'tx0.late', 'rx0.gated', 'host.rxq_ovfl_ch0', 'egress.drop_p0', ..}.
inline Counters collectCounters(const Read& read, HostRead* host_read = nullptr) {
  Counters out;
  auto bank = [&out](const std::string& raw, const std::string& pre, const std::vector<std::string>& fields) {
    for (const auto& ch : parseBankStatus(raw)) {
      const std::string key = pre + std::to_string(ch.first) + ".";
      // HS-220: a TX bank counts only from a usable poll, whose epoch rides
      // along as "<key>epoch"; an unusable one (torn, mid-clear, or no epoch
      // field) is not used, so the previous baseline stands until a usable
      // one (the interval just widens).
      if (pre == "tx") {
        const auto ep = ch.second.find("epoch");
        const long long epoch = ep == ch.second.end() ? -1 : txUsableEpoch(ep->second);
        if (epoch < 0) continue;
        out[key + "epoch"] = epoch;
      }
      for (const auto& f : fields) {
        const auto it = ch.second.find(f);
        if (it != ch.second.end() && detail::isInt(it->second)) out[key + f] = std::stoll(it->second);
      }
    }
  };
  bank(read("TX_BANK_STATUS"), "tx", txAlarmFields());
  bank(read("RX_BANK_STATUS"), "rx", rxAlarmFields());
  // The per-stream keys only: a closing stream takes its counts out of the total.
  for (const char* key : {"TX_HOST_STATUS", "RX_HOST_STATUS"}) {
    const Counters flat = parseFlatCounts(read(key));
    if (host_read != nullptr && !flat.empty()) (key[0] == 'T' ? host_read->tx : host_read->rx) = true;
    for (const auto& kv : flat) {
      const auto at = kv.first.rfind("_ch");
      if (at == std::string::npos || !detail::isDigits(kv.first.substr(at + 3))) continue;
      const std::string field = kv.first.substr(0, at);
      if (std::find(hostAlarmFields().begin(), hostAlarmFields().end(), field) != hostAlarmFields().end())
        out["host." + kv.first] = kv.second;
    }
  }
  for (const auto& kv : parseEgressStatus(read("EGRESS_STATUS"))) out["egress." + kv.first] = kv.second;
  return out;
}

/// {name: increase} for every counter that changed. A counter new since `prev`
/// counts from zero. A TX bank counter is differenced only within one epoch
/// (mod 2^16 if it wraps); after an epoch change its value IS the count since
/// the clear, and with no usable baseline epoch the poll is the baseline. Any
/// other counter that fell was cleared, so its current value is the rise.
inline Counters counterIncreases(const Counters& prev, const Counters& cur) {
  Counters out;
  for (const auto& kv : cur) {
    const std::string& k = kv.first;
    const auto dot = k.rfind('.');
    const std::string field = dot == std::string::npos ? k : k.substr(dot + 1);
    if (field == "epoch") continue;  // the clear count, not an event
    const auto it = prev.find(k);
    const long long p = it == prev.end() ? 0 : it->second;
    if (k.rfind("tx", 0) == 0) {
      const auto ec = cur.find(k.substr(0, dot + 1) + "epoch");
      const auto ep = prev.find(k.substr(0, dot + 1) + "epoch");
      if (ec == cur.end() || ep == prev.end()) continue;  // no usable baseline yet: this poll becomes it
      long long d = kv.second;                            // after a clear: the count since it
      if (ep->second == ec->second) d = txWrapsMod16(field) ? ((kv.second - p) % 65536 + 65536) % 65536
                                                           : (kv.second >= p ? kv.second - p : kv.second);
      if (d > 0) out[k] = d;
      continue;
    }
    const long long d = kv.second >= p ? kv.second - p : kv.second;  // a fall is a clear
    if (d > 0) out[k] = d;
  }
  return out;
}

/// The TX channels whose bank reports no HS-220 clear epoch: their counters are
/// never used (collectCounters), so the caller says so once.
inline std::vector<int> txBanksWithoutEpoch(const std::string& raw) {
  std::vector<int> out;
  for (const auto& ch : parseBankStatus(raw))
    if (ch.second.count("epoch") == 0) out.push_back(ch.first);
  return out;
}

/// Whether a host per-stream key belongs to TX_HOST_STATUS (else RX_HOST_STATUS).
inline bool txHostKey(const std::string& k) { return k.rfind("host.eob_recloses_", 0) == 0; }

/// The previous-check state after this check: every counter read now, plus
/// the last known value of any counter this read did not return. Do not
/// replace the state with the read: an empty or partial read would erase the
/// baseline, and the next full read would then report every counter's whole
/// running total as new. The exception is a per-stream host key missing from
/// a host status that answered (`host_read`, also when its last stream closed):
/// its stream closed, and a key that appears again counts from zero
/// (HOUDINI_PROTOCOL 2.7), so it is not carried.
inline Counters carryCounters(const Counters& prev, const Counters& cur, const HostRead& host_read) {
  Counters out;
  for (const auto& kv : prev) {
    const bool closed = kv.first.rfind("host.", 0) == 0 && (txHostKey(kv.first) ? host_read.tx : host_read.rx) &&
                        cur.count(kv.first) == 0;
    if (!closed) out.insert(kv);
  }
  for (const auto& kv : cur) out[kv.first] = kv.second;
  return out;
}

/// A preflight item's key: its text before the first '=', so TX0:late=3 and
/// TX0:late=4 are one standing item.
inline std::string preflightKey(const std::string& item) { return item.substr(0, item.find('=')); }

struct Report {
  std::string label;
  double seconds = 0.0;
  double irq_per_s = 0.0;
  std::string preflight;
  Counters increases;
  std::vector<std::string> new_failures;

  std::vector<std::string> alarms() const {
    std::vector<std::string> out;
    for (const auto& kv : increases) out.push_back(kv.first + " +" + std::to_string(kv.second));
    for (const auto& i : new_failures) out.push_back("preflight new FAIL " + i);
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
  /// The baseline is read at construction: call it once the session is
  /// configured, streaming, and its caller has cleared RFDC_PREFLIGHT after its
  /// activations. `now_s` is injectable for the test; the default is a steady
  /// clock.
  LinkHealth(Read read, std::string label, std::function<double()> now_s = {})
      : read_(std::move(read)), label_(std::move(label)), now_(std::move(now_s)) {
    if (!now_) {
      now_ = [] {
        return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
      };
    }
    // A FAIL item standing at the start alarms on the first check, even if it
    // is gone by then: it appeared in this read.
    start_failures_ = preflightItems(preflight());
    tx_unjudged_ = txBanksWithoutEpoch(read_("TX_BANK_STATUS"));
    prev_ = collectCounters(read_);
    prev_irq_ = irq();
    prev_t_ = now_();
  }

  /// The FAIL items the construction read found (all alarmed on the first check).
  const std::set<std::string>& startFailures() const { return start_failures_; }
  /// The TX channels whose bank has no clear epoch at the start (never judged).
  const std::vector<int>& txUnjudged() const { return tx_unjudged_; }

  Report check() {
    HostRead host_read;
    const Counters cur = collectCounters(read_, &host_read);
    const long long ir = irq();
    const double t = now_();
    const double dt = std::max(t - prev_t_, 1e-9);
    Report r;
    r.label = label_;
    r.seconds = dt;
    r.irq_per_s = static_cast<double>(ir - prev_irq_) / dt;
    r.preflight = preflight();
    // The current read's items first, so an item standing since the start is
    // reported by its current text; one alarm per key.
    const std::set<std::string> now_items = preflightItems(r.preflight);
    std::vector<std::string> cands(now_items.begin(), now_items.end());
    if (first_) cands.insert(cands.end(), start_failures_.begin(), start_failures_.end());
    std::set<std::string> keys;
    for (const auto& i : cands)
      if (standing_.count(preflightKey(i)) == 0 && keys.insert(preflightKey(i)).second) r.new_failures.push_back(i);
    standing_.clear();
    for (const auto& i : now_items) standing_.insert(preflightKey(i));
    first_ = false;
    r.increases = counterIncreases(prev_, cur);
    prev_ = carryCounters(prev_, cur, host_read);
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
  std::set<std::string> start_failures_, standing_;
  std::vector<int> tx_unjudged_;
  bool first_ = true;
  Counters prev_;
  long long prev_irq_ = 0;
  double prev_t_ = 0.0;
};

}  // namespace health
}  // namespace houdini
