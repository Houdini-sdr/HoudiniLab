# Archived demo-verify tools

The probes and campaign scripts of finished investigations. None is part of
the current demo workflow (`DEMO_BENCH_RUNBOOK.md`); each is kept because a
`DEMO_VERIFICATION.md` row cites it, so the row can be re-derived. They are
not maintained against the current sounder: a probe that drives the radios
directly assumes the device plugin, configs and rig of its campaign.

Running one:

- Paths inside the files name their old home, `tests/demo-verify/<file>`.
  Run from `CC/Sounder` and give this directory instead.
- `ab_report.py`, `fstage_hratio.py` and `fstage_spur.py` import the kept
  `rig_dumps` and `fstage_report` modules: run them with
  `PYTHONPATH=tests/demo-verify`.
- `run_pad_campaign.sh` finds its checkout from its own location, two levels
  up, which is now `tests/`: give `SOUNDER_DIR` explicitly.
- The rest import only each other (all here) or SoapySDR and numpy.

The beacon-shape sweep (`run_shape_campaign.sh`, with `gate_summary.py` and
`shape_campaign_summary.py`) stays at the top level: the walkthrough's sync
knobs section names it as the way a bench sweep sets `sync` overrides.

Rows are `DEMO_VERIFICATION.md` row ids; "8l leg N" is leg N of the gate-run
table in section 8l, and a section name is given where the citation sits in
prose rather than in a row.

| File | What it did | Cited by |
| --- | --- | --- |
| `ab_report.py` | SH-427 host-pacer A/B: one row per leg from `fstage_run.sh` run dirs (stack, TX counters, trim, clock lines, re-syncs, CNS low, MER per antenna) | 9.26, 9.30 |
| `ap15_correlate.py` | AP-15 correlation campaign: restarted the demo N times and paired each run's CNS1 constellation score with its pilot timing draw | 4.33, 4.56 |
| `ap15_diff.py` | AP-15 offline diff of a clean run's dump against a ring run's (tone coherence, reference rotation, pilot phase, slot lag) | 4.36 |
| `beacon_phase_coherence.py` | AP-34(b): whether the beacon replay's carrier phase is coherent from frame to frame | 8.166, 8.171, 8.172 |
| `beacon_shape_compare.py` | The candidate beacon shapes on silicon, interleaved: timing residual, CFO and detection per shape | section 8w |
| `beacon_spectrum.py` | The received beacon's spectrum from recorded re-sync windows or a raw probe window (band powers, centroid, tilt) | 8.171, 8.172 |
| `bs_init_walk.py` | Replayed the sounder's BS init calls one at a time and dumped the device state between them (phase 2) | section 2, 2.17; `evidence/rfdc-state-bs-20260830.md` |
| `bs_rx_timestamp_verify.py` | BS RX timestamp and slot mapping from the stream alone (HAS_TIME, tick continuity, frame and slot mapping, no loss) | 3.5 |
| `burst_rx_cost_probe.py` | AP-43: whether a timed burst read costs the continuous read's fixed ~855 us | 8l leg 6, 8.78, 8.86 |
| `cfo_ladder_model.py` | AP-34(b): the three-stage beacon CFO ladder, offline and synthetic | 8.72 |
| `cfo_ladder_probe.py` | AP-34(b): the CFO ladder on silicon from one contiguous capture | 8l leg 7, 8.73, 8.78 |
| `cfo_model.py` | Offline validator of receiver.cc's two-stage beacon CFO estimator, on a synthetic beacon (`cfo_ladder_model.py` copies its geometry) | none directly |
| `clock_drift_probe.py` | AP-33: the BS to UE clock offset from the beacon arrival ramp and the beacon CFO, independent of the sounder's acquisition | 8l legs 5 and 7, 8.87, 8.89, 8.137; `evidence/20260902-rig/README.md` |
| `clock_stability.py` | Allan deviation of the two-node clock offset from a `clock_drift_probe.py` run (AP-31 cadence budget) | 8.30, 8.48, 8l leg 4 |
| `clock_steer_cal.py` | AP-48: the CLOCK_ADJ actuator gain, a hold-code sweep on the UE | 8.38 |
| `clock_steer_loop.py` | AP-47: the external clock-steering loop; the in-sounder loop (`include/sync/clock_steer.h`) replaced it | none directly |
| `evm_compare.py` | Pilot against UL-data constellation SNR from BS frame dumps (FFT 64 layout hard-coded) | 4.45, 4.56 |
| `fstage_hratio.py` | AP-79 filter staging: the in-channel H change between two stages per tone, both BS antennas at R2 (R2 NCOs hard-coded) | 9.19 to 9.24 |
| `fstage_spur.py` | AP-79 filter staging: the UE ADC's Fs/2 offset spur per bring-up from the re-sync windows (R2 NCO hard-coded) | 9.24 |
| `hwtime_rate_probe.py` | Board sample-clock rate from `getHardwareTime()` against the host clock, no RF | 8.89, 8.91, 8.100 |
| `landing_map.py` | Where the pilot and data bursts land on the absolute slot grid in BS frame dumps (FFT 64 layout hard-coded) | 4.42, 4.44, 4.48, 4.56 |
| `map_scan.py` | Which BS TX channel reaches which UE RX channel on the cabled pair | section 4, 4.16, 4.17, 4.19 |
| `phase_probe_null.py` | The null and calibration for the phase probe's image ratio and lobe phase step | 8.167, 8.169 |
| `rearm_drain_sweep.py` | Swept the host plugin's `rx_rearm_drain_ms`, including a value that must fail, repeated per value | none |
| `run_pad_campaign.sh` | Back-to-back capture runs with the BS RX and UE TX debug dumps on, for the zero-pad sizing rows | none directly (BACKLOG AP-57) |
| `rx_bits_probe.py` | Bottom-bits liveness of RX samples on a plain BS capture | 2.19 |
| `rx_gap_census.py` | AP-59: whether short reads are dropped packets, from each read's first-sample time | none (BACKLOG AP-59) |
| `tdd_arm_experiment.py` | Phase 3 and 4 experiments on the BS TDD framer: the arming contract and the beacon liveness counters | section 3, 4.10, 4.11, 4.12 |
| `two_node_beacon_arrival.py` | Two-node beacon arrival: hunt to lock, tracking residual, re-arm determinism, re-make variability | section 4, 4.21 |
| `two_way_transfer.py` | AP-51: the clock offset separated from the range rate, two-way | 8l leg 8, 8.75, 8.78, 8.93 |
| `ue_init_walk.py` | Replayed the sounder's UE init calls one at a time and dumped the device state between them (phase 5) | 4.50, 4.51 |
