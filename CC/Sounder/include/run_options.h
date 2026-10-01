/** @file run_options.h
  * @brief The sounder's run options: what an operator sets for one run on the
  *        sounder's command line (main.cc's flags), where the code used to read
  *        environment variables. A run steered by an exported variable cannot be
  *        reproduced from its own record; a flag is in the command, and main()
  *        logs every option that is not at its default before any radio opens
  *        (houdini-agents conventions/no-env-knobs.md, AP-111).
  *
  * One process-wide instance, filled once by main() after the flags parse and
  * read-only afterwards; a test sets the fields it needs. Each field's default
  * is the behaviour with the old variable unset.
  */
#ifndef RUN_OPTIONS_H_
#define RUN_OPTIONS_H_

#include <string>

namespace Sounder {

struct RunOptions {
  // The run and its host.
  std::string topology;         ///< the topology file, overriding the config's serial_file ("" = the config's)
  std::string core_map;         ///< thread placement by role, "main=15,recorder=1" (scheduler, houdini/core_map.h)
  std::string tx_cpu_affinity;  ///< "c0,c1": the i-th live TX stream's pacer cpu (the plugin's cpu_affinity)
  std::string tx_stream_args;   ///< extra live-TX stream kwargs "k=v,k=v" (houdini/stream_args.h)
  double link_health_s = -1.0;  ///< link-health period: < 0 the default (5 s in mode V, off otherwise), 0 off
  bool tx_host_status = false;  ///< log TX_HOST_STATUS and TX_BANK_STATUS every health period
  long long max_frame = -1;     ///< frames to run: < 0 the config's max_frame
  int pilot_horizon = -1;       ///< UE pilot horizon: < 0 the config's ue_pilot_horizon
  bool coalesce_slots = true;   ///< read runs of discarded slots in one call (off: per-slot reads, for A/B)
  int corr_threads = 0;         ///< beacon correlator threads: 0 the default (1)
  double ue_rx_freq_offset_hz = 0.0;  ///< a test injection: the UE receive path detuned by this much
  double ue_tx_freq_offset_hz = 0.0;  ///< a test injection: the UE transmit path detuned by this much

  // The live CSI view (the dashboard).
  std::string csi_udp;          ///< "host:port": stream CSI there instead of recording HDF5 ("" = record)
  double csi_fps = 30.0;        ///< CSI datagrams per second per antenna (at least 0.5)
  double csi_spc_fps = 4.0;     ///< spectrum datagrams per second per antenna (0 = off)
  std::string csi_sym_start;    ///< the symbol-0 FFT start: an integer, "auto", or "" for prefix - CP/2
  bool csi_timing_fix = true;   ///< the per-frame pilot-vs-data timing re-align
  bool csi_phase_fix = true;    ///< the per-symbol pilot common-phase fix (AP-38)

  // Diagnostics: logs.
  bool sync_debug = false;
  bool ue_tx_debug = false;
  bool bs_rx_debug = false;
  int bs_rx_every = 20;         ///< with bs_rx_debug, one line per this many frames
  bool cl_rx_debug = false;
  bool csi_r_debug = false;
  bool find_beacon_debug = false;
  long rx_profile = 0;          ///< RadioHoudini::recv profile every this many calls (0 = off)
  long loop_profile = 0;        ///< UE loop profile every this many iterations (0 = off)

  // Diagnostics: dumps, written under dump_dir.
  std::string dump_dir = "/tmp";
  bool dump_beacon = false;     ///< the BS beacon RAM, beacon_ram.bin
  bool dump_gold = false;       ///< the detector's matched field, gold.bin
  bool dump_win = false;        ///< the UE's first strong RX window, cl_win.bin
  int csi_dump = -1;            ///< a constellation dump per antenna after this many frames (< 0 off; 0 or 1: 30)
  std::string bs_dump_frame;    ///< a directory: BS raw frames with their grid (a few, then stop)
  std::string dump_resync_win;  ///< a directory: the UE's re-sync windows
  std::string cns_dump_low;     ///< a directory: the first low-scoring constellations
};

/// The process's run options.
inline RunOptions& runOptions(void) {
  static RunOptions o;
  return o;
}

}  // namespace Sounder

#endif  // RUN_OPTIONS_H_
