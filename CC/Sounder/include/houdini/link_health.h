/**
 * @file houdini/link_health.h
 * @brief The link-alarm monitor of a long session, on the sounder's OWN device
 *        handle (a second make() on a node resets it). Its rules are the
 *        software lane's contract, SoapyHoudiniSDR shared/HOUDINI_PROTOCOL.md
 *        section 2.7 "Link alarms on a long session"; their Python twin is gone.
 *
 * Every read is passive, and each counter is judged by its change since its
 * previous read. A rise alarms. The device's counters (the TX and RX banks',
 * the egress drop, marked and link_down counts) are 64-bit, counted from the PL
 * load and never cleared, each bank's read at one instant: their rise is the
 * plain difference, and one that reads below its previous read means the PL
 * was reloaded, which alarms once as that and not as a rise. The egress stalls
 * and a TX bank's clock_aborts are 32-bit counts whose rise is taken mod 2^32.
 * A host counter that fell was cleared and its current value is the rise, and
 * so is a TX bank's sticky flag (efault, smiss). Host fields come from the
 * per-stream '<field>_ch<N>' keys, never the totals, which fall when a stream
 * closes. A preflight item is keyed on its text before the first '=' and
 * alarms on the first read it appears in, a standing item at the session's
 * start included. Nothing else alarms: no configuration drift, and no counter
 * merely standing nonzero or a sticky bit merely standing set.
 *
 * What this cannot see, the application counts itself: readStream flags, short
 * reads, timestamp jumps, writeStream returns. Nor can it see a sticky flag
 * that was cleared and set again between two reads (a TX clear in a re-arm,
 * then a new miss): it reads set both times.
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

using Counters = std::map<std::string, unsigned long long>;
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
/// A decimal count of up to 64 bits into `out`; false, `out` untouched, for
/// anything else (a sign, a non-digit, a value past 2^64 - 1).
inline bool parseCount(const std::string& v, unsigned long long& out) {
  if (!isDigits(v) || v.size() > 20) return false;
  unsigned long long n = 0;
  for (char c : v) {
    const unsigned long long d = static_cast<unsigned long long>(c - '0');
    if (n > (~0ULL - d) / 10) return false;
    n = n * 10 + d;
  }
  out = n;
  return true;
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

/// TX_HOST_STATUS / RX_HOST_STATUS 'k=v k_ch0=v ..' -> {k: count}; a token
/// without '=' or whose value is not a count of up to 64 bits is skipped.
inline Counters parseFlatCounts(const std::string& raw) {
  Counters out;
  std::istringstream is(raw);
  std::string tok;
  while (is >> tok) {
    const auto eq = tok.find('=');
    if (eq == std::string::npos) continue;
    unsigned long long n = 0;
    if (detail::parseCount(tok.substr(eq + 1), n)) out[tok.substr(0, eq)] = n;
  }
  return out;
}

/// EGRESS_STATUS 'drop=p0:a,p1:b,p2:c,p3:d;stalls=s;marked=m;link_down=l' ->
/// {'drop_p0': a, .., 'stalls': s, 'marked': m, 'link_down': l}. A group of
/// 'p<k>:<n>' items is per port ('<group>_p<k>'), any other is flat.
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
        unsigned long long n = 0;
        if (detail::parseCount(item.substr(c + 1), n)) out[key + "_" + item.substr(0, c)] = n;
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

// The alarm fields (HOUDINI_PROTOCOL 2.7): the TX bank's event counters and
// sticky flags, the RX bank's gated windows and timed-start aborts, the host's
// per-stream drops, re-closes and slot straddles. tdd_drop is the slot cut
// itself, not a fault.
inline const std::vector<std::string>& txAlarmFields() {
  static const std::vector<std::string> f = {"drops", "late",  "under", "seqerr", "zerofill",
                                             "efault", "smiss", "aclose", "malformed"};
  return f;
}
/// A TX bank's clock_aborts (TX_CLOCK_ABORTS): the clock events that aborted
/// the bank whole. Collected beside the alarm fields: its flush is not counted
/// frame by frame in any of them, so its move is the only trace of the frames
/// such an event took.
inline const char* txClockAborts() { return "clock_aborts"; }
/// A TX bank's counters that move in every healthy run and are never an
/// alarm: collected only so that a PL reload shows on a clean link, where
/// every alarm counter reads 0 before and after it (plReload).
inline const std::vector<std::string>& txReloadWitnesses() {
  static const std::vector<std::string> f = {"acked", "played"};
  return f;
}
/// Whether '<group>.<field>' is a reload witness: read, never reported.
inline bool isReloadWitness(const std::string& name) {
  if (name.rfind("tx", 0) != 0) return false;
  const auto dot = name.rfind('.');
  const std::string field = dot == std::string::npos ? name : name.substr(dot + 1);
  return std::find(txReloadWitnesses().begin(), txReloadWitnesses().end(), field) != txReloadWitnesses().end();
}

/// How a counter named '<group>.<field>' is differenced.
enum class CounterKind {
  kDevice64,  ///< 64-bit from the PL load, never cleared: a plain rise; a fall is a PL reload
  kDevice32,  ///< 32-bit from the PL load, never cleared, wraps: the rise mod 2^32
  kClearable, ///< a host counter or a sticky flag: a fall is a clear, its value the rise
};
inline CounterKind counterKind(const std::string& name) {
  const auto dot = name.rfind('.');
  const std::string field = dot == std::string::npos ? name : name.substr(dot + 1);
  if (name.rfind("host.", 0) == 0) return CounterKind::kClearable;
  if (name.rfind("tx", 0) == 0) {
    if (field == "efault" || field == "smiss") return CounterKind::kClearable;
    if (field == txClockAborts()) return CounterKind::kDevice32;
    return CounterKind::kDevice64;
  }
  if (name == "egress.stalls") return CounterKind::kDevice32;
  return CounterKind::kDevice64;  // the RX bank's and the egress drop, marked and link_down counts
}
inline const std::vector<std::string>& rxAlarmFields() {
  static const std::vector<std::string> f = {"gated", "aborts"};
  return f;
}
inline const std::vector<std::string>& hostAlarmFields() {
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
/// {'tx0.late', 'tx0.clock_aborts', 'rx0.gated', 'host.rxq_ovfl_ch0',
/// 'egress.drop_p0', ..}, with each TX bank's reload witnesses beside them.
inline Counters collectCounters(const Read& read, HostRead* host_read = nullptr) {
  Counters out;
  auto bank = [&out](const std::string& raw, const std::string& pre, std::vector<std::string> fields) {
    if (pre == "tx") {
      fields.push_back(txClockAborts());
      fields.insert(fields.end(), txReloadWitnesses().begin(), txReloadWitnesses().end());
    }
    for (const auto& ch : parseBankStatus(raw)) {
      const std::string key = pre + std::to_string(ch.first) + ".";
      for (const auto& f : fields) {
        const auto it = ch.second.find(f);
        unsigned long long n = 0;
        if (it != ch.second.end() && detail::parseCount(it->second, n)) out[key + f] = n;
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

/// The first 64-bit device counter (in name order) that reads below its
/// previous read, as '<name> <before> -> <after>', or "" when none does. Such a
/// counter never clears and cannot wrap, so the PL was reloaded between the
/// two reads. The reload witnesses count here: on a clean link they are the
/// only 64-bit counts above 0. A reload of a radio that has sent nothing at
/// all since its load shows in none of them, and a 32-bit count standing
/// above 0 then reads as a rise of nearly 2^32: an alarm either way.
inline std::string plReload(const Counters& prev, const Counters& cur) {
  for (const auto& kv : cur) {
    if (counterKind(kv.first) != CounterKind::kDevice64) continue;
    const auto it = prev.find(kv.first);
    if (it != prev.end() && kv.second < it->second)
      return kv.first + " " + std::to_string(it->second) + " -> " + std::to_string(kv.second);
  }
  return "";
}

/// {name: increase} for every counter that rose. A device counter (64- or
/// 32-bit) counts from the PL load, so one with no previous read is a baseline
/// and reports nothing; a 64-bit one differences plainly and a 32-bit one mod
/// 2^32. After a PL reload (plReload) no device counter is differenced: the
/// reload is the alarm, and this read is their baseline. A host counter new
/// since `prev` counts from zero, and one that fell (or a sticky flag that
/// did) was cleared, so its current value is the rise.
inline Counters counterIncreases(const Counters& prev, const Counters& cur) {
  Counters out;
  const bool reloaded = !plReload(prev, cur).empty();
  for (const auto& kv : cur) {
    if (isReloadWitness(kv.first)) continue;  // read for plReload only
    const auto it = prev.find(kv.first);
    const CounterKind kind = counterKind(kv.first);
    unsigned long long d = 0;
    if (kind == CounterKind::kClearable) {
      const unsigned long long p = it == prev.end() ? 0 : it->second;
      d = kv.second >= p ? kv.second - p : kv.second;  // a fall is a clear
    } else if (it == prev.end() || reloaded) {
      continue;  // no baseline from this PL load yet: this read becomes it
    } else if (kind == CounterKind::kDevice32) {
      d = (kv.second - it->second) & 0xFFFFFFFFULL;
    } else {
      d = kv.second - it->second;  // never below: a fall is a reload
    }
    if (d > 0) out[kv.first] = d;
  }
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
  std::string pl_reload;  ///< plReload's '<name> <before> -> <after>', "" when the PL was not reloaded
  std::vector<std::string> new_failures;

  std::vector<std::string> alarms() const {
    std::vector<std::string> out;
    if (!pl_reload.empty()) out.push_back("PL reloaded (" + pl_reload + ")");
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
    prev_ = collectCounters(read_);
    prev_irq_ = irq();
    prev_t_ = now_();
  }

  /// The FAIL items the construction read found (all alarmed on the first check).
  const std::set<std::string>& startFailures() const { return start_failures_; }

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
    r.pl_reload = plReload(prev_, cur);
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
  bool first_ = true;
  Counters prev_;
  long long prev_irq_ = 0;
  double prev_t_ = 0.0;
};

}  // namespace health
}  // namespace houdini
