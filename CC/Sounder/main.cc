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

#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <vector>

#include "include/RadioSetInterfaces.h"
#include "include/data_generator.h"
#include "include/run_options.h"
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
            "over UDP (to --csi_udp, default 127.0.0.1:9999) instead of recording "
            "to HDF5");

// The run options (include/run_options.h), one flag each; the defaults are the
// behaviour with the option unset.
DEFINE_string(topology, "", "Topology file overriding the config's serial_file (e.g. the demo venue's)");
DEFINE_string(core_map, "", "Thread placement by role, e.g. main=15 (houdini/core_map.h)");
DEFINE_string(tx_cpu_affinity, "", "Pacer cpu per live TX stream, c0,c1 (the plugin's cpu_affinity)");
DEFINE_string(tx_stream_args, "", "Extra live-TX stream kwargs k=v,k=v (houdini/stream_args.h)");
DEFINE_double(link_health_s, -1.0, "Link-health period in s: < 0 the default (5 s in mode V, off otherwise), 0 off");
DEFINE_bool(tx_host_status, false, "Log TX_HOST_STATUS and TX_BANK_STATUS every link-health period");
DEFINE_int64(max_frame, -1, "Frames to run: < 0 the config's max_frame");
DEFINE_int32(pilot_horizon, -1, "UE pilot horizon: < 0 the config's ue_pilot_horizon");
DEFINE_bool(coalesce_slots, true, "Read runs of discarded slots in one call (false: per-slot reads, for A/B)");
DEFINE_double(ue_rx_freq_offset_hz, 0.0, "Test injection: detune the UE receive path by this much (Hz)");
DEFINE_double(ue_tx_freq_offset_hz, 0.0, "Test injection: detune the UE transmit path by this much (Hz)");
DEFINE_string(csi_udp, "", "Stream live CSI to host:port instead of recording HDF5 (--view: 127.0.0.1:9999)");
DEFINE_double(csi_fps, 30.0, "CSI datagrams per second per antenna (at least 0.5)");
DEFINE_double(csi_spc_fps, 4.0, "Spectrum datagrams per second per antenna (0 off)");
DEFINE_string(csi_sym_start, "", "Symbol-0 FFT start: an integer, auto, or empty for prefix - CP/2");
DEFINE_bool(csi_timing_fix, true, "The per-frame pilot-vs-data timing re-align of the CSI view");
DEFINE_bool(csi_phase_fix, true, "The per-symbol pilot common-phase fix of the CSI view (AP-38)");
DEFINE_bool(sync_debug, false, "Log the UE sync state");
DEFINE_bool(ue_tx_debug, false, "Log the UE pilot bursts and its TX bank");
DEFINE_bool(bs_rx_debug, false, "Log the BS receive cut, one line per --bs_rx_every frames");
DEFINE_int32(bs_rx_every, 20, "With --bs_rx_debug, one line per this many frames");
DEFINE_bool(cl_rx_debug, false, "Log the UE's received level");
DEFINE_bool(csi_r_debug, false, "Log the CSI view's pilot correlation");
DEFINE_bool(find_beacon_debug, false, "Log the beacon detector's best ratio");
DEFINE_int64(rx_profile, 0, "Profile RadioHoudini::recv every this many calls (0 off)");
DEFINE_int64(loop_profile, 0, "Profile the UE loop every this many iterations (0 off)");
DEFINE_string(dump_dir, "/tmp", "Directory for the dumps below");
DEFINE_bool(dump_beacon, false, "Dump the BS beacon RAM (beacon_ram.bin)");
DEFINE_bool(dump_gold, false, "Dump the detector's matched field (gold.bin)");
DEFINE_bool(dump_win, false, "Dump the UE's first strong RX window (cl_win.bin)");
DEFINE_int32(csi_dump, -1, "Dump a constellation per antenna after this many frames (< 0 off; 0 or 1: 30)");
DEFINE_string(bs_dump_frame, "", "Directory: dump a few BS raw frames with their grid");
DEFINE_string(dump_resync_win, "", "Directory: dump the UE's re-sync windows");
DEFINE_string(cns_dump_low, "", "Directory: dump the first low-scoring constellations");

namespace {
// Fill the run options from the flags, and log every one not at its default so
// the run's own record says how it was steered.
void setRunOptions(void) {
  Sounder::RunOptions& o = Sounder::runOptions();
  o.topology = FLAGS_topology;
  o.core_map = FLAGS_core_map;
  o.tx_cpu_affinity = FLAGS_tx_cpu_affinity;
  o.tx_stream_args = FLAGS_tx_stream_args;
  o.link_health_s = FLAGS_link_health_s;
  o.tx_host_status = FLAGS_tx_host_status;
  o.max_frame = FLAGS_max_frame;
  o.pilot_horizon = FLAGS_pilot_horizon;
  o.coalesce_slots = FLAGS_coalesce_slots;
  o.ue_rx_freq_offset_hz = FLAGS_ue_rx_freq_offset_hz;
  o.ue_tx_freq_offset_hz = FLAGS_ue_tx_freq_offset_hz;
  o.csi_udp = (FLAGS_view && FLAGS_csi_udp.empty()) ? "127.0.0.1:9999" : FLAGS_csi_udp;
  o.csi_fps = FLAGS_csi_fps;
  o.csi_spc_fps = FLAGS_csi_spc_fps;
  o.csi_sym_start = FLAGS_csi_sym_start;
  o.csi_timing_fix = FLAGS_csi_timing_fix;
  o.csi_phase_fix = FLAGS_csi_phase_fix;
  o.sync_debug = FLAGS_sync_debug;
  o.ue_tx_debug = FLAGS_ue_tx_debug;
  o.bs_rx_debug = FLAGS_bs_rx_debug;
  o.bs_rx_every = FLAGS_bs_rx_every > 0 ? FLAGS_bs_rx_every : 20;
  o.cl_rx_debug = FLAGS_cl_rx_debug;
  o.csi_r_debug = FLAGS_csi_r_debug;
  o.find_beacon_debug = FLAGS_find_beacon_debug;
  o.rx_profile = static_cast<long>(FLAGS_rx_profile);
  o.loop_profile = static_cast<long>(FLAGS_loop_profile);
  o.dump_dir = FLAGS_dump_dir.empty() ? "/tmp" : FLAGS_dump_dir;
  o.dump_beacon = FLAGS_dump_beacon;
  o.dump_gold = FLAGS_dump_gold;
  o.dump_win = FLAGS_dump_win;
  o.csi_dump = FLAGS_csi_dump;
  o.bs_dump_frame = FLAGS_bs_dump_frame;
  o.dump_resync_win = FLAGS_dump_resync_win;
  o.cns_dump_low = FLAGS_cns_dump_low;
  std::vector<gflags::CommandLineFlagInfo> flags;
  gflags::GetAllFlags(&flags);
  std::string set;
  for (const auto& f : flags)
    if (!f.is_default && f.filename.find("main.cc") != std::string::npos) set += " --" + f.name + "=" + f.current_value;
  std::printf("Run options:%s\n", set.empty() ? " all at their defaults" : set.c_str());
}
}  // namespace

int main(int argc, char* argv[]) {
  gflags::SetVersionString(GetSounderProjectVersion());
  gflags::SetUsageMessage(
      "sounder Options: -bs_only -client_only -conf_file "
      "-gen_data_bits -storepath -view, and the run options (--helpon=main)");
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  setRunOptions();
  // A BAD CONFIG SHOULD SAY SO, NOT ABORT. Config's constructor throws on
  // invalid input by design (an unknown `beacon_type` must not silently fall
  // back to the default beacon), but an exception escaping main is
  // std::terminate: the operator sees "Aborted" and a core, not the sentence
  // explaining what to fix. Catch here so the reason reaches the operator.
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
    // operator here instead of escaping main.
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
        // A run a fault stopped is a failed run, and not one to re-try: the
        // re-try below is for discovery (ReceiverException) only.
        if (config->faulted()) {
          std::cerr << "The run stopped on a fault: " << config->faultReason() << std::endl;
          ret = EXIT_FAILURE;
          break;
        }
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
