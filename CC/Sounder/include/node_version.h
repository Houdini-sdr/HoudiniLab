/** @file node_version.h
  * @brief Cross-node version skew check for a multi-node run.
  *
  * Every participating radio reports its whole stack through
  * getHardwareInfo(): gateware (fpga_*), device firmware (device_*), host
  * plugin (host_*) and the wire protocol (proto_version). A run with two nodes
  * on different builds can fail in ways that look like RF or timing problems,
  * so the versions are collected at bring-up, printed once, and any difference
  * is WARNed about before the run starts.
  *
  * Process-wide registry rather than a parameter threaded through the radio
  * sets: the BS and the UE are built by different classes (and there are UHD
  * variants that simply never register), so a shared sink is the least invasive
  * bridge -- the same reasoning as rx_gap_sink.h.
  */
#ifndef NODE_VERSION_H_
#define NODE_VERSION_H_

#include <algorithm>
#include <map>
#include <mutex>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "SoapySDR/Types.hpp"
#include "logger.h"

namespace Sounder {

class NodeVersions {
 public:
  static NodeVersions& instance(void) {
    static NodeVersions v;
    return v;
  }

  /// Record one participating node. @p who is how the operator identifies it
  /// (role + address), @p info is its getHardwareInfo() dict.
  void add(const std::string& who, const SoapySDR::Kwargs& info) {
    std::lock_guard<std::mutex> lock(mtx_);
    nodes_.emplace_back(who, info);
  }

  /// Print each node's stack and warn on any disagreement. Returns the number
  /// of keys that differ. Safe to call with 0 or 1 node (nothing to compare).
  size_t checkAndWarn(void) {
    std::lock_guard<std::mutex> lock(mtx_);
    if (nodes_.empty()) return 0;

    // Keys every node in one run must agree on. Deliberately excludes the ones
    // that SHOULD differ per node (serial, label, ip_address, data_iface,
    // hostname), so a normal two-node bench is quiet.
    static const std::vector<std::string> kMustMatch = {
        "fpga_version",   "fpga_commit",  "fpga_board", "device_version",
        "device_build",   "host_version", "host_build", "proto_version"};

    for (const auto& n : nodes_) {
      std::string line;
      for (const auto& k : kMustMatch) {
        const auto it = n.second.find(k);
        if (it != n.second.end()) line += " " + k + "=" + it->second;
      }
      MLPD_INFO("Node stack %s:%s\n", n.first.c_str(), line.c_str());
    }
    if (nodes_.size() < 2) return 0;

    size_t differing = 0;
    for (const auto& k : kMustMatch) {
      std::set<std::string> vals;
      std::string detail;
      for (const auto& n : nodes_) {
        const auto it = n.second.find(k);
        const std::string v = (it == n.second.end()) ? "<absent>" : it->second;
        vals.insert(v);
        detail += "  " + n.first + "=" + v;
      }
      if (vals.size() > 1) {
        ++differing;
        MLPD_WARN("VERSION SKEW: %s differs across nodes:%s\n", k.c_str(),
                  detail.c_str());
      }
    }
    if (differing > 0) {
      MLPD_WARN(
          "VERSION SKEW: %zu key(s) differ across the %zu participating "
          "node(s). Mismatched gateware/firmware/host builds can fail as "
          "timing or RF symptoms; re-flash or redeploy so every node matches "
          "before trusting this run.\n",
          differing, nodes_.size());
    } else {
      MLPD_INFO("Node versions: all %zu node(s) agree on the full stack.\n",
                nodes_.size());
    }
    return differing;
  }

 private:
  NodeVersions(void) = default;
  std::mutex mtx_;
  std::vector<std::pair<std::string, SoapySDR::Kwargs>> nodes_;
};

}  // namespace Sounder

#endif  // NODE_VERSION_H_
