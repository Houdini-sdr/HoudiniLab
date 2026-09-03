/**
 * @file sync/beacon_shape.cc
 */
#include "sync/beacon_shape.h"

#include <stdexcept>

#include "beacon_shapes.h"

namespace houdini {
namespace sync {

std::vector<std::string> BeaconShape::names() {
  std::vector<std::string> out;
  for (const char* n : beacon_shapes::kAllNames) out.emplace_back(n);
  return out;
}

BeaconShape BeaconShape::make(const std::string& name, Platform platform,
                              size_t prefix_samples) {
  beacon_shapes::Shape s;
  if (!beacon_shapes::parse(name, &s)) {
    std::string expected;
    for (const auto& n : names()) expected += (expected.empty() ? "" : ", ") + n;
    throw std::invalid_argument("unknown beacon_type \"" + name + "\" -- expected one of " +
                                expected);
  }
  return fromDesc(beacon_shapes::make(s), platform, prefix_samples);
}

BeaconShape BeaconShape::fromDesc(const beacon_shapes::Desc& d, Platform platform,
                                  size_t prefix_samples) {
  BeaconShape b;
  b.name_ = d.name;
  b.platform_ = platform;
  b.prefix_ = prefix_samples;
  b.core_ = d.core;
  b.replica_ = d.replica;
  b.replica_off_ = d.replica_off;
  b.replica_reps_ = d.replica_reps;
  b.guard_len_ = d.guard_len;
  b.papr_db_ = d.papr_db();
  // The tail is a Houdini convention; the Iris/UHD framer never had one.
  b.tail_ = platform == Platform::kHoudini ? d.replica_tail() : 0;
  b.geometry_.core_len = static_cast<int>(d.core.size());
  b.geometry_.fine_off = static_cast<int>(d.fine_off);
  b.geometry_.fine_len = static_cast<int>(d.fine_len);
  b.geometry_.fine_reps = static_cast<int>(d.fine_reps);
  b.geometry_.coarse_off = static_cast<int>(d.coarse_off);
  b.geometry_.coarse_len = static_cast<int>(d.coarse_len);
  b.geometry_.coarse_reps = static_cast<int>(d.coarse_reps);
  return b;
}

ssize_t BeaconShape::endFromCorrelatorIndex(ssize_t idx, size_t window_len) const {
  if (idx < 0) return -1;
  if (tail_ == 0) return idx;
  const ssize_t end = idx + static_cast<ssize_t>(tail_);
  return end < static_cast<ssize_t>(window_len) ? end : -1;
}

ssize_t BeaconShape::expectedEndOffset() const {
  if (platform_ == Platform::kHoudini) {
    return static_cast<ssize_t>(kHoudiniStrobeOffsetTicks) + static_cast<ssize_t>(core_.size());
  }
  return static_cast<ssize_t>(core_.size() + prefix_);
}

}  // namespace sync
}  // namespace houdini
