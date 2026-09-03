/** @file BeaconFramer.cc
  * @brief The framer factory: the one place that maps a platform to a framer.
  *
  * RENEW OPEN SOURCE LICENSE: http://renew-wireless.org/license
*/
#include "include/BeaconFramer.h"

#include <stdexcept>

#include "include/HoudiniFramer.h"
#include "include/IrisFramer.h"

std::unique_ptr<BeaconFramer> BeaconFramer::create(houdini::sync::Platform platform, Config* cfg,
                                                   Radios& radios) {
  switch (platform) {
    case houdini::sync::Platform::kHoudini:
      return std::make_unique<HoudiniFramer>(cfg, radios);
    case houdini::sync::Platform::kIrisUhd:
      throw std::invalid_argument(
          "BeaconFramer::create: the Iris framer needs the set's hubs and trigger offsets; "
          "use IrisFramer directly");
  }
  throw std::invalid_argument("BeaconFramer::create: unknown platform");
}
