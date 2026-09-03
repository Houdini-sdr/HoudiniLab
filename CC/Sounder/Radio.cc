/** @file Radio.cc
  * @brief The radio factory: the only place that maps a type to a backend.
  *
  * RENEW OPEN SOURCE LICENSE: http://renew-wireless.org/license
*/
#include "include/Radio.h"

#include <stdexcept>

#include "include/RadioHoudini.h"
#include "include/RadioSoapy.h"
#include "include/config.h"
#include "include/macros.h"

const char* Radio::name(Type type) {
  switch (type) {
    case Type::kSoapyIris: return "iris";
    case Type::kSoapyUhd: return "uhd (SoapySDR)";
    case Type::kSoapyHoudini: return "houdini";
    case Type::kUhdNative: return "uhd (native)";
  }
  return "?";
}

std::unique_ptr<Radio> Radio::create(Type type, const RadioParams& params) {
  switch (type) {
    case Type::kSoapyIris:
    case Type::kSoapyUhd:
      return std::make_unique<RadioSoapy>(params, type);
    case Type::kSoapyHoudini:
      return std::make_unique<RadioHoudini>(params);
    case Type::kUhdNative:
      // Seam step S3 folds the native UHD radio behind this factory; until
      // then it is the separate compile-time path (RADIO_TYPE=PURE_UHD).
      throw std::invalid_argument("Radio::create: the native UHD backend is not behind the factory yet");
  }
  throw std::invalid_argument("Radio::create: unknown radio type");
}

Radio::Type radioTypeFor(const Config& cfg) {
  if (cfg.is_houdini()) return Radio::Type::kSoapyHoudini;
  return kUseSoapyUHD ? Radio::Type::kSoapyUhd : Radio::Type::kSoapyIris;
}
