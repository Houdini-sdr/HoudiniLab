# The sync library: architecture and migration plan

Status: P1 LANDED and gated (DEMO_VERIFICATION 8.170), then reworked to the architecture review of 2026-09-03 (8.175): `BeaconShape`, `Detection` with evidence, the schema/values split, `ResyncPolicy`, `sim::Channel` and `Numerology` exist; P3 (the pfa-derived bar) and P5 (`PhaseTracker`) remain. Written 2026-09-03. Written after the AP-66 campaign, to roll the beacon,
detector, confirm, carrier and grid-tracking code and everything the ledger has
measured about it into one classed, configured, tested library. Section 8 lists
the decisions that are the user's to make; everything else is a proposal with a
recommended default.

## 1. What exists today, and why it needs a home

The UE synchronisation path works and is measured (DEMO_VERIFICATION 8.x), but
it is spread across `receiver.cc` (3000 lines, about half of them sync),
`config.cc`, `comms-lib-portable.cc`, two header-only helpers
(`sync/sync_geometry.h`, `sync/grid_tracker.h`), `sync/beacon_shapes.h`, and Python mirrors of
the detector and the CFO estimator in `tests/demo-verify/`. Its configuration is
about thirty `HOUDINI_*` environment variables: 17 numeric knobs read through
`envDouble` in `receiver.cc`, five enumerations and switches read by `getenv`
there, two in `comms-lib-portable.cc`, one transmit-level knob in
`BaseRadioSet.cc`, plus the diagnostic dumps and profiles. The sample rate
122.88 MSPS and the 4096-sample slot appear as literals in the tests and probes.

Three costs, all paid this week:

- A knob accepted a nonsense value silently (AP-56) and a knob inherited across
  beacons became a cliff (8.157: a 30 dB floor derived for one waveform rejects
  a 1.1 dB quieter one wholesale).
- Every learning had to be placed in three places, C++ detector, C++ test, and
  Python probe, and once was placed in two of them (8.140).
- The behaviour that mattered was only knowable from a run's log, because the
  effective configuration is assembled at runtime from defaults, JSON and
  environment with no single record.

## 2. Goals and non-goals

Goals:

1. One library, `houdini::sync`, with one class per concern, each testable
   against a channel simulator without a radio.
2. One configuration object, loaded from a `sync` block in the JSON config,
   validated (ranges and cross-constraints), printed at startup with the
   provenance of every value.
3. Numerology as data: sample rate, slot and frame lengths, symbol sizes, so
   the same code runs at another rate or another subcarrier spacing without a
   literal changing.
4. The ledger's learnings encoded as code and tests, not comments: the
   threshold bar from a false-alarm probability (8.163), the CFO window margin
   and postfix (8.164), replica-aware detector form (8.154), first-path pick
   (8.143), the SNR guard covering the first-path window (8.151).
5. Byte-identical behaviour for the shipped path through the migration: the
   `legacy` waveform sample-for-sample, the shipped defaults, the same
   detections on the same recorded windows.

Non-goals, for this plan:

- No change to the driver, the gateware or the frame structure.
- No new dependency. The library is C++17, links what the sounder links.
- No change of default behaviour. `legacy` stays; new capability lands behind
  configuration.

## 3. Module map

Namespace `houdini::sync`, static library `houdini_sync`, sources under
`CC/Sounder/sync/`, public headers under `CC/Sounder/include/sync/`. The sounder
and the tests link it; `rx-recorder` may later.

| Class | Owns | Comes from | Measured basis |
|---|---|---|---|
| `Numerology` | rate, slot and frame sample counts, symbol and prefix sizes, tick conversions | literals in receiver, tests, probes; `sync_geometry.h` inputs | 8.65, AP-40 |
| `BeaconShape` and `ShapeRegistry` | the waveform, its replica, its field geometry (coarse, fine, replica, postfix), PAPR | `beacon_shapes.h` | 8.111 to 8.116, 8.154 |
| `MatchedFilter` | the cross-correlation engine (`correlate_mt`), energy sums, the statistic per form | `find_beacon_avx` body | 8.138, 8.140 |
| `ThresholdPolicy` | `PowerRatio`, `NormalizedXCorr`, `Coherence`; the bar, derived from `pfa_per_window` and L for coherence forms, from `corr_scale` for the legacy form | `BeaconThresh` and `corr_scale` | 8.138, 8.163 |
| `PickRule` | `FirstCrossing`, `ClusterRefined`, `Argmax`, `FirstPath{window, floor_db}` | `BeaconPick` | 8.139, 8.143 |
| `Detector` | composes the three above; selects the form from the replica; returns `Detection{end_index, statistic, form}` with the replica tail applied | `syncSearch` | 8.154 |
| `Confirm` | `SnrWindowGuard{floor_db, guard = max(8, first_path_window)}`; interface for `SequenceConfirm` (SSS) | `beaconSnrDb` | 8.151, 8.155 |
| `CfoEstimator` | `RepetitionPhase{margin}` with the coarse unwrap; NaN on failure; interface for `SymbolPairPhase` | `estimateCFO` | AP-39, 8.164 |
| `SyncGeometry` | `SliceGeometry` (targeted slice from tolerance and the replica length) and `ResyncSchedule` (the cadence) | `sync/sync_geometry.h` | 8.65 |
| `GridTracker` | `AlphaBeta`, `Kalman`, `TrackerConfig` (built from the JSON tracker block) | `sync/grid_tracker.h` | 8.81 |
| `ResyncPolicy` | cadence from residual ppm, retry ladder, escalation, hold-offgrid, acquisition refine span | constants and env in the client loop | 8.104 to 8.108 |
| `PhaseTracker` | two-state phase and frequency with a random-walk model, fed by the detection's complex peak | new | AP-67 |
| `SyncConfig` | the validated struct behind all of the above, with provenance | `envDouble` and `getenv` sites | AP-56 |
| `Telemetry` | the SYN1 datagram from library structs | `sendSyncTelemetry` | AP-32 |
| `sim::Channel` (test only) | taps, CFO, fractional delay with a chosen kernel, noise, level, int16 quantisation | `beacon_geometry_test` helpers | 8.143, 8.160, 8.164 |

What stays in `receiver.cc`: the threads, the radio I/O, the schedule, the
recorder hand-off. The client loop becomes a sequence of library calls whose
inputs and outputs are structs, which is what makes it testable.

## 4. Data flow

Acquisition: a wide window is read; `Detector` runs with `ClusterRefined`;
`Confirm` rejects noise-window crossings; two grid-consistent confirms fix the
anchor and bootstrap the period (AP-31). Tracking: `ResyncPolicy` says when the
beacon is due; `SyncGeometry` sizes the slice; `Detector` runs with `FirstPath`;
`Confirm` guards; the residual feeds `GridTracker`, whose state drives the
transmit schedule; `CfoEstimator` reads the same detection for telemetry;
`PhaseTracker`, when it exists, reads the detection's complex peak. Every stage
consumes and returns plain structs, so the same sequence runs in a test against
`sim::Channel` and in the client against the radio.

## 5. Configuration

A `sync` object in the existing JSON config, one struct in code, validated on
load. Diagnostics that dump files or print profiles stay as environment
variables: they are not configuration, and a dump switch in a shipped JSON is a
foot-gun. Numeric and behavioural knobs move.

```json
"sync": {
  "beacon":   { "type": "legacy", "tx_full_scale": 0.6, "postfix_len": 0 },
  "detector": { "threshold": "auto", "pfa_per_window": 1e-3, "corr_scale": 10,
                "pick": "first_path", "first_path_window": 64,
                "first_path_floor_db": -9 },
  "confirm":  { "snr_floor_db": 30 },
  "cfo":      { "estimator": "repetition", "window_margin": 12 },
  "tracker":  { "type": "alpha_beta", "alpha": 0.5, "beta": 0.1,
                "step_ppm": 0.5, "max_ppm": 100, "trust_ppm": 1.0,
                "kalman": { "meas_var": 0.5, "rate_rw": 1e-9, "innov_gate": 4.0 } },
  "resync":   { "residual_ppm": 0.1, "scatter_tol_us": 2.0,
                "confirm_tol_us": 5.2083, "retry_max": 100,
                "escalate_episodes": 2, "hold_offgrid": 2,
                "acq_refine_span": 200, "acq_max_ppm": 100 },
  "allow_env_overrides": false
}
```

Rules:

- Every key has a range and a default in ONE table in `SyncConfig`; the
  walkthrough's knob table is generated from that table, not maintained by
  hand.
- Cross-constraints are checked: `confirm_tol_us >= scatter_tol_us`, the
  geometry the tolerance implies fits the slot, `first_path_window` stays
  inside the preamble's self-coherent plateau for the chosen shape, a
  `snr_floor_db` above the shape's expected in-window SNR at `tx_full_scale`
  is a warning with the number (8.157).
- `threshold: auto` picks the form from the replica (single copy: coherence)
  and derives the bar from `pfa_per_window` and L; `corr_scale` is read only
  when a legacy form is named explicitly.
- Startup prints the effective struct with one of `default`, `json`, `env`
  beside every value. With `allow_env_overrides: false` an `HOUDINI_*` knob
  that would have changed a value is reported and ignored. AS SHIPPED IN P1
  the flag defaults to TRUE, because the bench scripts still sweep through
  the environment: every override is logged; a number outside a knob's range
  is clamped to the bound with a note for the knobs whose old readers clamped
  or had no bound at all (the AP-56 class), and ignored with a note for the
  three whose old readers ignored it (`beacon.tx_full_scale`,
  `detector.first_path_window`, `detector.first_path_floor_db`). The
  recommendation is to flip the default next release and remove the path
  after.
- `detector.pfa_per_window` is loaded and validated in P1 but NOT applied
  (reserved for P3); giving it a value produces a startup note saying so.
- `detector.first_path_window` defaults to -1, "half the replica length",
  which is what the pre-library correlator derived (64 at 128 taps, 32 at 64);
  the resolved value is what the SNR guard and the log carry.

## 6. Testing strategy

- Unit: every class against `sim::Channel`, with the tables the ledger already
  runs: 5 shapes x 7 levels x 8 draws x 3 forms x 4 picks, 6 multipath
  channels at 8.5 ppm, SNR 10 to 45 dB, fractional delay 0 to 1 with two
  kernels, noise-only windows for the false-alarm rate.
- Regression: golden windows. A handful of resync slices per shape captured on
  the rig (`HOUDINI_DUMP_RESYNC_WIN` exists) committed as small fixtures; the
  detector, confirm and CFO estimator must return the recorded answers. This is
  the byte-identity guard for the migration.
- Property: the index convention `end == strobe + beacon_size` for every shape,
  tau, level and form (today's `kEndConvention` check, generalised).
- Configuration: a table-driven test that loads every documented knob at its
  bounds and one step outside, and a rate-ladder test that derives the geometry
  at 61.44, 122.88 and 245.76 MSPS and asserts the invariants AP-56 names.
- Silicon: `run_shape_campaign.sh`, interleaved, pre-registered (the 8v and 8z
  pattern), PRE and POST on one binary via configuration rather than via two
  builds.

## 7. Migration, in gated phases

| Phase | Content | Gate |
|---|---|---|
| P0 freeze | capture golden windows; the sample-identity test; record today's silicon baseline (8.154 to 8.162) | fixtures committed |
| P1 extract | library skeleton; move the header-only pieces; extract `Detector`, `Confirm`, `CfoEstimator`, `ResyncPolicy` from `receiver.cc` and `comms-lib-portable.cc` with behaviour preserved; sounder links the library | all tests pass; golden windows identical; 3 interleaved PRE/POST silicon runs indistinguishable |
| P2 configure | `SyncConfig` from JSON with validation and provenance; env mapped; walkthrough table generated | config tests; a run's log shows the effective struct; PRE/POST unchanged |
| P3 encode | coherence bar from Pfa (8.163); CFO margin (8.164, AP-69a); `postfix_len` (AP-69b) as a variant; shape registry | offline sweeps as in 8.163/8.164; interleaved `bcfo sd` PRE/POST |
| P4 numerology | rate and spacing as data; remove literals from tests and probes; rate-ladder test | ladder test; one run at the shipped rate unchanged |
| P5 extend | `PhaseTracker` (AP-67), `SequenceConfirm` and `SymbolPairPhase` for SSB-lite (AP-68), behind configuration | their own pre-registered campaigns |

Each phase ends with an Opus code review of the diff, repeated until a pass
reports no new finding, before the silicon gate runs; a review finding that
changes behaviour re-runs the offline suite first.

## 8. Decisions that are the user's

1. **Environment overrides**: remove outright at P2, or keep for one release
   behind `allow_env_overrides` (recommended: keep one release, logged and
   off by default, then remove).
2. **Location**: `CC/Sounder/sync/` (recommended: keeps one CMake project and
   the existing CI) or a top-level `CC/libsync` shared with `rx-recorder`.
3. **Default beacon**: `legacy` throughout (recommended); an NR-shaped default
   waits for SSB-lite and an over-the-air result.
4. **Python mirrors**: keep the probes reading the dumper's files (recommended)
   or bind the library into Python; the first costs nothing now, the second
   removes a class of drift later.
