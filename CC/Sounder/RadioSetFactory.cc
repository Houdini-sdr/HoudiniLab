/** @file RadioSetFactory.cc
  * @brief The set factory: the only place that knows which set classes this
  *        build compiled in.
  *
  * RENEW OPEN SOURCE LICENSE: http://renew-wireless.org/license
*/
#include "include/RadioSetInterfaces.h"

#include <stdexcept>

#if defined(USE_UHD)
#include "include/BaseRadioSetUHD.h"
#include "include/ClientRadioSetUHD.h"
#else
#include "include/BaseRadioSet.h"
#include "include/ClientRadioSet.h"
#endif

std::unique_ptr<IBaseRadioSet> makeBaseRadioSet(Config* cfg, bool calibrate_proc) {
#if defined(USE_UHD)
  // The native-UHD set carries no wired calibration procedure (its
  // syncTimeOffsetUHD and dciqCalibrationProcUHD are unreferenced), so a
  // calibration run must not turn into a normal bring-up that looks like a
  // failed calibration (S3 review, item 1).
  if (calibrate_proc) {
    throw std::runtime_error(
        "this build (RADIO_TYPE=PURE_UHD) has no base-station calibration procedure");
  }
  return std::make_unique<BaseRadioSetUHD>(cfg);
#else
  return std::make_unique<BaseRadioSet>(cfg, calibrate_proc);
#endif
}

std::unique_ptr<IClientRadioSet> makeClientRadioSet(Config* cfg) {
#if defined(USE_UHD)
  return std::make_unique<ClientRadioSetUHD>(cfg);
#else
  return std::make_unique<ClientRadioSet>(cfg);
#endif
}
