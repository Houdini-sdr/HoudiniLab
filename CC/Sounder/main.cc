/*
 Copyright (c) 2018-2022, Rice University 
 RENEW OPEN SOURCE LICENSE: http://renew-wireless.org/license
 
---------------------------------------------------------------------
 main function
 - initializes all Clients
 - Brings up Recorder and the BaseStation
---------------------------------------------------------------------
*/

#include <gflags/gflags.h>

#include <cstdlib>
#include <iostream>

#include "include/RadioSetInterfaces.h"
#include "include/data_generator.h"
#include "include/scheduler.h"
#include "include/signalHandler.hpp"
#include "include/version_config.h"

DEFINE_bool(
    gen_data_bits, false,
    "Generate random bits for uplink/downlink transmissions, otherwise read "
    "from file!");
DEFINE_string(conf_file, "files/conf.json", "JSON configuration file name");
DEFINE_string(storepath, "logs", "Dataset store path");
DEFINE_bool(bs_only, false, "Run BS only");
DEFINE_bool(client_only, false, "Run client only");
DEFINE_bool(calibrate, false, "Run radio set calibration");
DEFINE_bool(view, false,
            "Viewing mode: compute live CSI per antenna and stream it to the GUI "
            "over UDP (dest from HOUDINI_CSI_UDP, default 127.0.0.1:9999) instead "
            "of recording to HDF5");

int main(int argc, char* argv[]) {
  gflags::SetVersionString(GetSounderProjectVersion());
  gflags::SetUsageMessage(
      "sounder Options: -bs_only -client_only -conf "
      "-gen_data_bits -storepath -view");
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  // Viewing mode is switched on inside the recorder by the presence of
  // HOUDINI_CSI_UDP; --view just sets a sensible default destination.
  if (FLAGS_view && std::getenv("HOUDINI_CSI_UDP") == nullptr) {
    setenv("HOUDINI_CSI_UDP", "127.0.0.1:9999", 1);
  }
  // A BAD CONFIG SHOULD SAY SO, NOT ABORT. Config's constructor throws on
  // invalid input by design (an unknown `beacon_type` must not silently fall
  // back to the old beacon), but an exception escaping main is std::terminate --
  // the operator sees "Aborted" and a core, not the sentence explaining what to
  // fix. Catch here so the design choice reaches the person who has to act on it.
  std::unique_ptr<Config> config;
  try {
    config =
        std::make_unique<Config>(FLAGS_conf_file, FLAGS_storepath, FLAGS_bs_only,
                                 FLAGS_client_only, FLAGS_calibrate);
  } catch (const std::exception& e) {
    std::fprintf(stderr, "\nConfiguration error in %s:\n  %s\n\n",
                 FLAGS_conf_file.c_str(), e.what());
    return EXIT_FAILURE;
  }
  int ret = EXIT_FAILURE;
  if (FLAGS_gen_data_bits) {
    auto dg = std::make_unique<DataGenerator>(config.get());
    dg->GenerateData(FLAGS_storepath);
  } else if (FLAGS_calibrate) {
    // The calibration run: the set this build provides, constructed in its
    // calibration mode (the Iris sample-offset procedure), through the
    // factory. A build with no procedure refuses; the reason reaches the
    // operator here instead of escaping main (S4 review, item 3).
    try {
      auto base_radio_set_ = makeBaseRadioSet(config.get(), true);
    } catch (const std::exception& e) {
      std::fprintf(stderr, "Calibration not run: %s\n", e.what());
      return EXIT_FAILURE;
    }
  } else {
    int cnt = 0;
    int maxTry = 2;

    // Register signal handler to handle kill signal
    SignalHandler signalHandler;
    signalHandler.setupSignalHandlers();

    while (cnt++ < maxTry && ret == EXIT_FAILURE) {
      try {
        auto dr = std::make_unique<Sounder::Scheduler>(config.get());
        dr->do_it();
        ret = EXIT_SUCCESS;

      } catch (const SignalException& e) {
        std::cerr << "SignalException: " << e.what() << std::endl;
        ret = EXIT_FAILURE;
        break;

      } catch (ReceiverException& rex) {
        // Discovery usually fails on the first run, re-try
        std::cout << "Exception: " << rex.what() << " Re-Try Now!" << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(1));

      } catch (const std::exception& exc) {
        std::cerr << "Exception Encountered... Program terminated due to "
                  << exc.what() << std::endl;
        ret = EXIT_FAILURE;
        break;
      }
    }
  }
  gflags::ShutDownCommandLineFlags();
  return ret;
}
