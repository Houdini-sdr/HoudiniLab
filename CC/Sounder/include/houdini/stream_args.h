/**
 * @file houdini/stream_args.h
 * @brief Extra TX stream arguments from the environment (HOUDINI_TX_STREAM_ARGS),
 *        for the host plugin's diagnostic and tuning knobs (for example
 *        tx_target_frac, SH-427) without a rebuild of either side.
 *
 * "key=value,key=value", applied to every live (tx_mode=stream) TX stream. The
 * keys the sounder sets itself (tx_mode, tdd, mts) are refused: overriding them
 * would change what the sounder believes it opened.
 *
 * RENEW OPEN SOURCE LICENSE: http://renew-wireless.org/license
 */
#pragma once

#include <map>
#include <string>

namespace houdini {

/// The parsed pairs, or an empty map with `err` set when any item is malformed
/// or names a key the sounder owns. An empty or null spec is no pairs, no error.
inline std::map<std::string, std::string> extraStreamArgs(const char* spec, std::string* err) {
  std::map<std::string, std::string> out;
  err->clear();
  if (spec == nullptr) return out;
  const std::string s(spec);
  size_t p = 0;
  while (p <= s.size()) {
    const size_t e = std::min(s.find(',', p), s.size());
    const std::string item = s.substr(p, e - p);
    p = e + 1;
    if (item.empty()) continue;
    const size_t eq = item.find('=');
    if (eq == std::string::npos || eq == 0 || eq + 1 == item.size()) {
      *err = "'" + item + "' is not key=value";
      return {};
    }
    const std::string key = item.substr(0, eq);
    if (key == "tx_mode" || key == "tdd" || key == "mts") {
      *err = "'" + key + "' is set by the sounder itself";
      return {};
    }
    out[key] = item.substr(eq + 1);
  }
  return out;
}

}  // namespace houdini
