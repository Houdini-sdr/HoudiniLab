# Sounder and dashboard options

The option tables: the dashboard backend's flags, the sounder's run flags, the beacon waveforms and the `sync` block of the config. A reference to look things up in; the shipped demo configs carry the values that ran, and section 7 of `CSI_DEMO_WALKTHROUGH.md` says when to come here.

## 1. Dashboard backend options

| Option | Default | What it does |
|---|---|---|
| `--http-port` | 8080 | Web server port |
| `--udp-port` | 9999 | Port the CSI datagrams arrive on |
| `--fps` | 30 | How often the page is pushed new data |
| `--stale-ms` | 1500 | Dim an antenna's plots when its last update is older than this (`DASHBOARD_GUIDE.md` section 2) |
| `--mag-top` | the config's `dashboard_mag_top`, else 90 | Where each card's \|H\| axis starts, in dB; the axis then steps by 10 dB when the trace leaves it (`DASHBOARD_GUIDE.md` section 1) |
| `--mag-span` | 40 | Height of the \|H\| axis, in dB below its top |
| `--csi-fps` | sounder default (30) | Per antenna stream rate out of the sounder |
| `--launch` | off | Start the sounder in viewing mode on this host at once |
| `--control` | off | Start, Stop, Restart and Check buttons and a config list (the sounder's `files/houdini*.json`) in the page header; `CSI_DEMO_WALKTHROUGH.md` section 4.4 |
| `--conf` | `files/houdini-1u.json` | The config to run (with `--launch` or `--control`), and the one the page's \|H\| axis top comes from (`dashboard_mag_top`) |
| `--sounder-dir` | the checkout `csi_server.py` is in | Which checkout's `build/sounder` runs |
| `--venv` | the activated venv, else `~/houdini_test` | The SoapySDR runtime the sounder runs with, and its Houdini plugin unless `--soapy-root` names a release prefix |
| `--soapy-root` | none | The release's host-plugin prefix (`CSI_DEMO_WALKTHROUGH.md` section 2.4), for the sounder, the setup check and the teardown |
| `--examples` | the teardown's own (`~/repos/SoapyHoudiniSDR/host/examples`) | The SoapyHoudiniSDR host examples the teardown imports (`CSI_DEMO_WALKTHROUGH.md` section 2.4) |
| `--topology` | each config's `serial_file` | A topology file every run uses instead, for the sounder, the setup check and the teardown (`CSI_DEMO_WALKTHROUGH.md` section 2.6) |
| `--sounder-arg=<flag>` | none | One more flag for the sounder's command line (repeatable), for example `--sounder-arg=--core_map=main=15`; section lists them |
| `--log-dir` | off | Write each start's sounder output to `<dir>/sounder_<UTC>.log` |
| `--replay FILE=LABEL` | none | With `--control`, a recording offered in the config list under LABEL (repeatable); Start plays it in a loop into this dashboard with no radio, no setup check and no teardown |
| `--configs` | `all` | With `--control`, the configs the list offers: every `files/houdini*.json` (`all`), or only those carrying a short `_label` (`labelled`: the demo's four) |
| `--record` | off | Record every datagram to a new file for `replay_feed.py` (a name that exists is refused, never overwritten) |
| `--record-max-mb` | 2048 | Stop recording at this size |
| `--dest-host` | 127.0.0.1 | Where the sounder sends datagrams, when using `--launch` |
| `--http-host` | 0.0.0.0, or 127.0.0.1 with `--control` | Web server bind address |

`python3 csi_gui/csi_server.py --help` lists every option.

Run the backend on the same host as the sounder unless you have a reason not
to. If you split them, set `--dest-host` to the backend's address and make sure
UDP port 9999 is open between the two.

## 2. Sounder run flags

All of these are flags of the sounder, its run options
(`./build/sounder --helpon=main` lists every one). Put them on the sounder's
command line (mode B), or pass each through the dashboard as
`--sounder-arg=<flag>` (mode A). The sounder logs a `Run options:` line at
start naming every one not at its default. The sync knobs (the SNR floor, the
beacon carrier log rate, the tracker and the steering) are config keys
instead, in the config's `sync` block (section 4).

| Flag | Default | What it does |
|---|---|---|
| `--csi_sym_start` | the zero prefix minus half the cyclic prefix (`CSI_DEMO_WALKTHROUGH.md` section 6) | Where the FFT window starts inside a received slot. An integer, or `auto` for the energy edge detector. |
| `--csi_timing_fix` | on | `--csi_timing_fix=false` disables the per frame pilot re-alignment. |
| `--csi_fps` | 30 | Per antenna datagram rate out of the sounder. |
| `--max_frame` | from config `max_frame` | Frame count to run. Set large for continuous viewing. |
| `--csi_udp` | `127.0.0.1:9999` with `--view` | Where datagrams go, as `host:port`. |
| `--csi_dump=N` | off | One shot raw slot and H dump for offline analysis, per antenna after N frames (0 or 1: 30), under `--dump_dir`. |
| `--pilot_horizon` | from config `ue_pilot_horizon` (96 in the legacy configs and the dual-band `r1` and `r2`, 10 in the other dual-band ones) | How many frames of client bursts are queued ahead of real time. Larger survives slower host loops; every extra frame delays a timing correction reaching the wire. |
| `--bs_rx_debug` | off | Base station prints its rederivation of the client schedule: `pilot_grid_off` should sit within a few samples of zero and hold steady through a run, and `clamped` (slots placed past the capture's edge) should read 0. |
| `--ue_tx_debug` | off | Client prints its burst scheduling (frames queued, pad). |
| `--core_map` | unset | Where the sounder pins its own threads: `main=<core>,recorder=<core>,bsrx=<core>,ue=<core>`, each the base core of that role (thread i on base + i); a role not named keeps the default layout. Logged at start. For CPU isolation experiments. |
| `--tx_cpu_affinity` | unset | `c0,c1,...`: the i-th live client TX stream gets the host plugin's `cpu_affinity=ci`, pinning its pacer worker (keep the workers off the cores that take the data NIC's interrupts). Logged per stream at open. |
| `--tx_stream_args` | unset | Extra host-plugin arguments for the client's live TX streams, `key=value,key=value` (for example `tx_target_frac=0.75`), each logged at open; `tx_mode`, `tdd` and `mts` are refused. A diagnostic and tuning knob: set it only when the host plugin's owners ask. |
| `--csi_r_debug` | off | Recorder prints the per frame pilot re-alignment it chose (`r`, and the blind score behind it), one line per 30 corrections. |
| `--tx_host_status` | off | Logs the client host plugin's pacer state (`TX_HOST_STATUS`, `TX_BANK_STATUS`) every link-health period (`--link_health_s`). Cheap; the demo runs with it. |
| `--cns_dump_low` | unset | Directory for autopsy dumps of the first few low scoring constellations. The directory must already exist. |

## 3. Choosing the beacon waveform

The base station transmits a short burst at the top of every frame and the
client finds it by correlation. Which burst it sends is the config key
`sync.beacon.type` (the top-level `beacon_type` of older configs is still
read, and the two may not disagree). Six are available:

| value | what it is |
| --- | --- |
| `legacy` | The default. Fifteen repeats of a 16 sample training symbol, then two repeats of a 128 sample Gold sequence. |
| `legacy_guard` | The same, with a 32 sample cyclic guard inserted before the Gold field, in the style of an 802.11 long training field. |
| `dot11` | The 802.11a/g/n legacy preamble as the standard defines it: the short training field, then the guard and two long training symbols. |
| `nr` | The 5G NR primary synchronisation signal, then a guard and two repeats of a tracking symbol built from the NR reference sequence. The client finds it on the repeated tracking symbol. |
| `nr_pss` | The same burst as `nr`, sample for sample, but the client finds it the way an NR handset does: a plain matched filter on the primary synchronisation signal, with no repeat check. The log says `threshold form forced to coherence` when this is in effect. |
| `nr_pss_bl` | The band-limited beacon of the dual-band configs: the NR primary synchronisation signal, then a guarded pair of tracking symbols on the central 96 tones, all inside the sub-6 channel filter's plus or minus 24 MHz. |

The first four were measured on the bench, four rounds each with the order
rotated, about 8000 detections apiece; the first five have been run end to end
through the client (`nr_pss` in `DEMO_VERIFICATION.md` 8.154 to 8.162), and
`nr_pss_bl` is the beacon of every dual-band run (`DEMO_VERIFICATION.md` section 9).
The margin figures below were measured with an earlier comparison rule, so
read them as a ranking rather than as absolute numbers. **Timing is the same
for all of them**, within measurement error. What separates them is detection margin: the worst detection
of the run cleared the threshold by 12x for `legacy` and `legacy_guard`, 7x for
`dot11`, and only 2.3x for `nr`.

**On the legacy configs, leave it at `legacy` unless you have a reason.** The
margin is the best of the four, and it is the waveform every legacy
measurement in this repository was taken against. **The dual-band configs need
`nr_pss_bl`:** in mode V the sub-6 lanes are filtered to plus or minus 24 MHz,
so the sounder refuses a wider beacon at load. If you set a value that is not
in the table the client refuses to start rather than falling back, so a typo
cannot quietly leave you on a different beacon than you think.

The one thing the alternatives are better at is the beacon's own frequency
estimate, where `dot11` is about a third more stable. That number is a
secondary reading on the sync panel, not what the client actually tracks
frequency with, so it does not currently justify the margin it costs.

## 4. Sync knobs: the `sync` block of the config

These belong to the timing tracker, the beacon detector, its SNR confirm, the
beacon's own frequency estimate and the client's clock steering. They live in
ONE place: a `sync` object in the JSON config, loaded into one validated structure whose
every value is printed at startup with where it came from (`default`, `json`
or `derived`). Every default below is a measured value, not a guess, and the run
is expected to be correct with all of them left alone.

Three things to know:

1. The table is GENERATED from the code (`./build/sync_config_schema`) and
   checked by `sync_config_test`, which fails when the committed copy
   differs from the schema, so it cannot drift from what the client actually
   reads. If you edit a knob in the code, regenerate this table.
   `sync.detector.corr_scale` and `corr_scale_init` are the detection bars;
   the legacy top-level `corr_scale` arrays are still read when the block
   does not set them (the first client's value), so no old file needs a new
   key.
2. A key the loader does not know, or a value outside its range, stops the
   client with a message naming the key. A typo cannot quietly leave a knob at
   its default.
3. There is no environment override: the config's `sync` block is the only
   way to set these knobs, so a run's sync settings are all in its config.
   Bench sweeps go through the JSON: `run_shape_campaign.sh` merges
   `SYNC_OVERLAY` (a JSON object) into the `sync` block of every config it
   writes.

Example, the block of the demo config
`files/houdini-dualband-xw-steer-slots.json`:

```json
"sync": {
  "steer":    { "enable": true, "deadband_ppm": 0.12 },
  "beacon":   { "type": "nr_pss_bl", "tx_full_scale": 0.34 },
  "detector": { "pick": "argmax", "min_bar": 0.1 },
  "confirm":  { "snr_floor_db": 25.0 }
}
```

<!-- sync-knob-table:begin (generated by ./build/sync_config_schema; sync_config_test diffs it, do not edit by hand) -->
| key | default | range | what it does |
| --- | --- | --- | --- |
| `sync.beacon.type` | `legacy` |  | Which beacon waveform the base station transmits (legacy, legacy_guard, dot11, nr, nr_pss, nr_pss_bl; nr_pss_bl is the band-limited beacon of the dual-band configs). |
| `sync.beacon.tx_full_scale` | 0.6 | 0.001 to 1 | Transmit peak of the beacon as a fraction of DAC full scale. 0.6 shipped; lower it to stand in for path loss on a cable. |
| `sync.detector.threshold` | `auto` | auto, power, xcorr, coherence | Decision statistic: auto picks coherence for a single-copy replica and the normalised cross-correlation otherwise; power is the original level-dependent form (4th order in amplitude against 2nd) and the Iris/UHD default. |
| `sync.detector.pfa_per_window` | 0.001 | 1e-09 to 0.5 | The coherence form's bar when set: the false-alarm probability per search window, turned into a bar by the replica and window lengths (8.163). Unset, corr_scale applies; ignored for the repeated-field forms. |
| `sync.detector.pick` | `first_path` | first_crossing, cluster_refined, argmax, first_path | Which crossing is returned: first_path (the Houdini default), argmax, cluster_refined, or first_crossing (the Iris/UHD default; unsafe on a strong link). |
| `sync.detector.first_path_window` | derived | -1 to 4095 | Samples the first-path search looks back from the peak; -1 (default) means half the replica length. Must stay inside the preamble's self-coherent plateau. A correlator quantity: samples, not scaled with the rate. |
| `sync.detector.first_path_floor_db` | -9 | -30 to 0 | How much weaker, in dB of path power, an earlier arrival may be and still be taken as the first path. |
| `sync.detector.first_path_guard` | 0 | 0 to 1 | Samples immediately before the peak the first-path search skips. A beacon between samples splits its peak over two adjacent taps and the earlier one is the SAME arrival, not an earlier one; 1 skips it. Only 0 and 1: 2 loses a genuine two-sample-earlier arrival and 3 a three-sample one (measured). 0 is the default (DEMO_VERIFICATION.md 8ak). |
| `sync.detector.corr_scale` | 10 | 0.0001 to 1e+07 | Resync detection threshold: the bar is 1 / corr_scale, relaxed by one per retry. Read from the legacy per-client top-level array when absent. |
| `sync.detector.corr_scale_init` | 10 | 0.0001 to 1e+07 | Acquisition detection threshold (bar 1 / corr_scale_init); defaults to corr_scale. |
| `sync.detector.min_bar` | 0 | 0 to 1 | The lowest bar the resync retry relaxation (+1 on corr_scale per retry) may reach; 0 = no limit. Set it with a small corr_scale, where +1 per retry would walk the bar into the noise. |
| `sync.detector.corr_threads` | 1 | 1 to 256 | Threads for the correlator's matched filter. 1 shipped; measured a net loss below ~4 on the rig host. |
| `sync.confirm.snr_floor_db` | 30 | -10 to 80 | In-window SNR a detection must clear. A property of the link and the waveform: re-derive it when either changes. |
| `sync.cfo.index_guard` | 8 | 0 to 64 | Samples the carrier estimator's windows slide later than the detected end. A correlator quantity: samples, not scaled with the rate. |
| `sync.cfo.window_margin` | 0 | 0 to 32 | Samples shrunk from both ends of each estimator window so neither touches the burst's edge (8.164). A correlator quantity: samples, not scaled with the rate. |
| `sync.cfo.log_every` | 10 | 1 to 1e+06 | Print one beacon-CFO log line in this many. |
| `sync.tracker.type` | `alpha_beta` | alpha_beta, kalman | Which estimator tracks the base station frame grid: alpha_beta (shipped) or kalman. |
| `sync.tracker.alpha` | 0.5 | 0 to 1 | Fraction of each accepted residual applied to the schedule. |
| `sync.tracker.beta` | 0.1 | 0 to 1 | Fraction of the residual applied to the frame period estimate. |
| `sync.tracker.step_ppm` | 0.5 | 0 to 1000 | Most one detection may move the period estimate, ppm. 0 disables the limit. |
| `sync.tracker.max_ppm` | 100 | 0.1 to 10000 | Absolute band the period estimate may occupy either side of nominal, ppm. |
| `sync.tracker.trust_ppm` | 1 | 0 to 1000 | How far the tracked period and a fresh acquisition confirm may disagree before the confirm is preferred, ppm. |
| `sync.tracker.kalman.meas_var` | 0.5 | 1e-06 to 1e+06 | Kalman only: assumed detector scatter variance, samples squared. |
| `sync.tracker.kalman.rate_rw` | 1e-09 | 0 to 1 | Kalman only: how fast the frame period wanders, samples squared per frame cubed. |
| `sync.tracker.kalman.innov_gate` | 4 | 0 to 100 | Kalman only: sigmas an observation may sit from the prediction before it is ignored. 0 disables. |
| `sync.steer.enable` | false |  | Steer the UE's clock onto the beacon's with CLOCK_ADJ, from the tracked grid rate, inside the sounder. Needs the UE's clock_ref to be calibrated. Off by default; keep it off for A/B and regression runs of the TX path (SH-427): each CLOCK_ADJ RPC holds the device's stream lock about 200 ms, and each push is a rate step the host pacer re-learns. |
| `sync.steer.period_s` | 20 | 2 to 3600 | Seconds between steering decisions; the tracked rate is averaged over each. A held oscillator drifts slowly, so this need not be short. |
| `sync.steer.gain` | 0.7 | 0.05 to 1 | Fraction of the averaged offset removed at each push. |
| `sync.steer.deadband_ppm` | 0.06 | 0 to 10 | Offsets smaller than this are left alone: half the actuator quantum is the floor of what a push can fix. |
| `sync.steer.max_offset` | 30 | 0 to 400 | Bounded authority: never steer further than this many counts from the calibration point. |
| `sync.steer.max_push` | 2 | 1 to 4 | Most counts one push may move, so no single frequency step is large. At most 4: the step is fed forward when the push lands, about 0.2 s after the DAC moves (up to 0.4 s when a failed write is read back), so 4 counts (0.5 ppm) leave 12 to 25 samples of grid error, well inside the 246-sample re-sync gate; 50 would leave 150 to 300. |
| `sync.steer.ppm_per_count` | 0.1251 | 0.001 to 10 | Actuator gain, ppm per CLOCK_ADJ count (magnitude; +1 count raises the UE clock). Measured 0.1251 on the bench. |
| `sync.steer.keep` | false |  | Leave the steered code in place when the sounder exits instead of releasing to the calibrated hold. |
| `sync.resync.residual_ppm` | 0.1 | 0.0001 to 1000 | Assumed worst-case clock error after tracking; with sync_tol_samples it sets how often the beacon is looked at. |
| `sync.resync.scatter_tol_us` | 2 | 0.01 to 1000 | How far a detection may land from the tracked grid and still count as the same beacon, microseconds. |
| `sync.resync.confirm_tol_us` | 5.2083 | 0.01 to 1000 | The same tolerance during acquisition. Never applied looser than the tracking gate. |
| `sync.resync.sync_tol_samples` | derived | 0.5 to 1e+06 | Timing slack budgeted to drift between looks, samples. Default: a quarter of the OFDM zero prefix. |
| `sync.resync.retry_max` | 100 | 1 to 100000 | Misses in one resync period before the client logs an exhausted episode. |
| `sync.resync.escalate_episodes` | 2 | 1 to 1000 | Consecutive exhausted episodes before the client abandons tracking and re-acquires. |
| `sync.resync.hold_offgrid` | 2 | 1 to 1000 | Consecutive off-grid detections before the beacon counts as moved. |
| `sync.resync.acq_refine_span` | 200 | 2 to 100000 | Frames of baseline acquisition wants before it trusts its rate estimate. |
| `sync.resync.acq_max_ppm` | 100 | 0.1 to 10000 | Plausibility band applied to a rate that acquisition hands back, ppm. |
<!-- sync-knob-table:end -->

Diagnostics that dump files or print profiles (`--loop_profile`,
`--rx_profile`, `--coalesce_slots`, `--csi_phase_fix`, `--bs_rx_every`, the
dumps) are sounder flags like the rest of section, never `sync` keys: they
are not configuration, and a dump switch in a shipped JSON is a trap. Every
`--dump_beacon`, `--dump_gold`, `--dump_win` and `--csi_dump` file lands under
`--dump_dir` (default `/tmp`), so a bench can keep its dumps out of `/tmp` with
one flag. View mode (`--csi_udp`, set by `--view`) writes NO HDF5 file and says
so at startup, so a stray `--csi_udp` cannot silently disable recording.

`sync.resync.retry_max`, `sync.resync.escalate_episodes` and
`sync.resync.hold_offgrid` are the escalation net. Their defaults were tuned
for a client grid that drifts out of tolerance in about a second. With the
clock steered it holds for minutes, so the defaults are conservative by a
wide margin. Change them one at a time against a known good baseline run, and
keep the net rather than removing it: it is what stands between a lost beacon
and a client that flies on stale timing without saying so.

`--csi_sym_start` is the one worth understanding. The cyclic prefix guard
is one sided. A window placed early, still inside the prefix, is a valid
circular shift and produces a pure phase ramp that the timing fix recovers. A
window even one sample late pulls the next symbol into the FFT and produces
inter symbol interference that no correction recovers. A window placed
exactly on that cliff edge lets beacon re-lock jitter tip runs into
interference at random. Backing the window off by half the cyclic prefix
centers it in the guard and gives margin on both sides. Measured on a window
interference run, this moved blind error vector magnitude from 19.8 percent to
3.2 percent.

The energy edge auto detector is opt in for a reason. Its 15 percent threshold
can trigger on pre symbol leakage and misalign the windows, so prefer a fixed
integer once you know the right value for your bench.
