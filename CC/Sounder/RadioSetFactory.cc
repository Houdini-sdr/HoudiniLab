/** @file RadioSetFactory.cc
  * @brief The set factory: the only place that knows which set classes this
  *        build compiled in.
  *
  * RENEW OPEN SOURCE LICENSE: http://renew-wireless.org/license
*/
#include "include/RadioSetInterfaces.h"

#if defined(USE_UHD)
#include "include/BaseRadioSetUHD.h"
#include "include/ClientRadioSetUHD.h"
#else
#include "include/BaseRadioSet.h"
#include "include/ClientRadioSet.h"
#endif

std::unique_ptr<IBaseRadioSet> makeBaseRadioSet(Config* cfg, bool calibrate_proc) {
#if defined(USE_UHD)
  (void)calibrate_proc;
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
