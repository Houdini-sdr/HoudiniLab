/**
 * @file houdini/core_map.h
 * @brief Where the sounder pins its own threads (HOUDINI_CORE_MAP), for the
 *        CPU isolation experiments: "main=15,recorder=18,bsrx=5,ue=6".
 *
 * Each named role gets that base core, and its i-th thread base + i; a role
 * not named keeps the default layout (main on the scheduler's start core, the
 * recorders after it, then the BS receive threads, then the UE threads). The
 * host plugin's TX pacer workers have their own knob, HOUDINI_TX_CPU_AFFINITY.
 *
 * RENEW OPEN SOURCE LICENSE: http://renew-wireless.org/license
 */
#pragma once

#include <algorithm>
#include <string>

namespace houdini {

struct CoreMap {
  int main = -1;      ///< the scheduler's dispatch thread
  int recorder = -1;  ///< recorder thread i on recorder + i
  int bsrx = -1;      ///< BS receive thread i on bsrx + i
  int ue = -1;        ///< UE thread i on ue + i
};

/// The parsed map, or all -1 with `err` set when an item is malformed, names
/// an unknown role, or repeats one. A null or empty spec is the default map.
inline CoreMap parseCoreMap(const char* spec, std::string* err) {
  CoreMap m;
  err->clear();
  if (spec == nullptr) return m;
  const std::string s(spec);
  size_t p = 0;
  while (p <= s.size()) {
    const size_t e = std::min(s.find(',', p), s.size());
    const std::string item = s.substr(p, e - p);
    p = e + 1;
    if (item.empty()) continue;
    const size_t eq = item.find('=');
    const std::string key = eq == std::string::npos ? item : item.substr(0, eq);
    const std::string val = eq == std::string::npos ? "" : item.substr(eq + 1);
    const bool digits = !val.empty() && val.size() <= 4 && val.find_first_not_of("0123456789") == std::string::npos;
    if (!digits) {
      *err = "'" + item + "' is not role=core";
      return {};
    }
    int* slot = key == "main" ? &m.main : key == "recorder" ? &m.recorder : key == "bsrx" ? &m.bsrx
              : key == "ue" ? &m.ue : nullptr;
    if (slot == nullptr) {
      *err = "'" + key + "' is not a role (main, recorder, bsrx, ue)";
      return {};
    }
    if (*slot >= 0) {
      *err = "'" + key + "' is named twice";
      return {};
    }
    *slot = std::stoi(val);
  }
  return m;
}

}  // namespace houdini
