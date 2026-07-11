/*
 Copyright (c) 2018-2026, Rice University
 RENEW OPEN SOURCE LICENSE: http://renew-wireless.org/license

----------------------------------------------------------------------
 Implementation of RxRecorderConfig (rx-recorder JSON parsing)
---------------------------------------------------------------------
*/
#include "include/rx_recorder_config.h"

#include <sys/stat.h>

#include <ctime>
#include <fstream>
#include <stdexcept>

#include "include/logger.h"
#include "nlohmann/json.hpp"

using json = nlohmann::json;

namespace Sounder {

// Kwargs values may legitimately be JSON numbers or booleans
// (e.g. "ring_bytes": 268435456); SoapySDR wants them as strings.
static std::map<std::string, std::string> parseKwargs(const json& obj,
                                                      const std::string& key) {
  std::map<std::string, std::string> kwargs;
  if (obj.contains(key) == false) {
    return kwargs;
  }
  for (const auto& item : obj.at(key).items()) {
    const json& v = item.value();
    if (v.is_string()) {
      kwargs[item.key()] = v.get<std::string>();
    } else {
      kwargs[item.key()] = v.dump();
    }
  }
  return kwargs;
}

static std::string defaultOutputFile(const std::string& storepath) {
  char timebuf[32];
  std::time_t now = std::time(nullptr);
  std::tm tm_now;
  localtime_r(&now, &tm_now);
  std::strftime(timebuf, sizeof(timebuf), "%Y%m%d_%H%M%S", &tm_now);
  return storepath + "/rx_record_" + timebuf + ".h5";
}

RxRecorderConfig::RxRecorderConfig(const std::string& json_file,
                                   const std::string& storepath) {
  std::ifstream conf_stream(json_file);
  if (conf_stream.is_open() == false) {
    throw std::runtime_error("Could not open config file: " + json_file);
  }
  json conf;
  conf_stream >> conf;

  device_args_ = parseKwargs(conf, "device");
  if (device_args_.count("driver") == 0) {
    device_args_["driver"] = "houdinisdr";
  }
  stream_args_ = parseKwargs(conf, "stream");

  channels_ = conf.value("channels", std::vector<size_t>{0});
  // Single-channel RX is the validated Houdini path; the combined
  // multi-channel readStream data-merge is still WIP device-side.
  if (channels_.size() != 1) {
    throw std::runtime_error(
        "rx-recorder currently supports exactly one RX channel per capture, "
        "got " +
        std::to_string(channels_.size()));
  }

  rate_ = conf.value("rate", 0.0);
  has_freq_ = conf.contains("freq");
  freq_ = conf.value("freq", 0.0);
  has_gain_ = conf.contains("gain");
  gain_ = conf.value("gain", 0.0);
  antenna_ = conf.value("antenna", std::string());

  duration_sec_ = conf.value("duration_sec", 1.0);
  if (duration_sec_ <= 0.0) {
    throw std::runtime_error("duration_sec must be > 0");
  }
  samps_per_slot_ = conf.value("samps_per_slot", 65536u);
  if (samps_per_slot_ == 0) {
    throw std::runtime_error("samps_per_slot must be > 0");
  }
  read_chunk_samps_ = conf.value("read_chunk_samps", 16384u);
  if (read_chunk_samps_ == 0) {
    throw std::runtime_error("read_chunk_samps must be > 0");
  }
  buffer_slots_ = conf.value("buffer_slots", 512u);
  if (buffer_slots_ < 2) {
    throw std::runtime_error("buffer_slots must be >= 2");
  }
  rx_timeout_us_ = conf.value("rx_timeout_us", 1000000L);

  output_file_ = conf.value("output_file", std::string());
  if (output_file_.empty()) {
    // Best-effort create of the store directory (single level, like "logs").
    mkdir(storepath.c_str(), 0755);
    output_file_ = defaultOutputFile(storepath);
  }
  MLPD_INFO("RxRecorderConfig: %s -> %s\n", json_file.c_str(),
            output_file_.c_str());
}

}; /* End namespace Sounder */
