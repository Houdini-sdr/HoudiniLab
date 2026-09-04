# Packaging the sounder changes: fixes and features

Status: PROPOSAL, 2026-09-03, from the baseline assessment of everything
`feat/internal-clock-cfo` changed against `master` (`84cb4d3`, stock RENEWLab
Sounder) and the fixes applied the same day (DEMO_VERIFICATION 8.175). The
question it answers: have we ruined the original purpose of the sounder, and
how should the work be packaged so the legacy sounder benefits from the fixes
while the Houdini features stay switchable.

## 1. Is the original sounder intact?

Intact in intent and, for `RADIO_TYPE=SOAPY_IRIS`, very nearly intact in fact.
Verified by inspection of the diff against master (not on Iris hardware; none
appears anywhere in this branch's evidence, and that limit is stated in
section 5):

- The Iris TDD framer and beacon transmission are byte-identical: the TDD JSON
  block, the beacon RAM and weights, the trigger and delay settings.
- The HDF5 output is identical: dataset names, dimensions, types, chunking,
  attributes and file naming. `/Data/Gaps` is added only under `is_houdini()`.
- The beacon waveform is bit-identical (`beacon_type` defaults to `legacy`
  and the shape builder reproduces `genPilots` exactly).
- The resync cadence on Iris is master's 100 ppb rule; the timed cadence is
  Houdini-only.
- No legacy config needs a new key; the `sync` block is optional and every
  default reproduces master's constant.

What had moved, and is now restored or made a choice:

- The Iris/UHD beacon acquisition pick had silently become cluster-refined;
  master returned the first threshold crossing. It is the first crossing
  again, derived as that platform's default by `SyncConfig::resolve()`, and
  a JSON `sync.detector.pick` overrides it on any platform.
- `RADIO_TYPE=PURE_UHD` did not compile (two Houdini-era calls with no UHD
  counterpart). Both methods now exist on the UHD classes as no-ops. This is
  inspection-only: there is no UHD on the build host.
- A cell configured with no radios aborted; it proceeds again, and only a
  cell that lists radios and opens none aborts.

What is changed for every backend and is left changed, deliberately:

- The beacon detector implementation is the portable matched filter
  (`comms-lib-portable.cc`), algebraically equivalent to the AVX kernel for
  the power-ratio form but not bit-identical (double accumulation, an O(n)
  trailing sum), compiled `-fPIC` into `houdini_sync`. This is the correlator
  workstream's port ([user], `feat/correlator-rate-test`); the AVX kernels
  remain and are exercised by `tests/comms-func/test-main.cc`.
- `Packet` is 20 bytes instead of 16 (`rx_pad`); every stride derives from
  `sizeof(Packet)` and nothing serialises it.
- Float-to-int16 conversions saturate instead of wrapping; only inputs that
  were already past full scale change, and they were corrupt before.
- Configuration errors exit 1 instead of aborting.

## 2. The fixes: standalone, upstreamable, ordered by value

Each is one commit on this branch and applies to the legacy sounder on its
own. They are the candidate `fix/legacy-sounder` series.

| # | Fix | Where | Risk upstream |
|---|---|---|---|
| 1 | Dangling pointer handed to the radio as a DMA target (`std::vector(...).data()` temporary) | `receiver.cc` BS RX | none |
| 2 | Use-after-free in the C API (`getTraceFileName` returned by value, `.c_str()` on the temporary) | `scheduler.h/.cc` | none |
| 3 | Uninitialised `frameTime` and a zero-length read published as a packet | `receiver.cc` Iris hw-framer arm | low |
| 4 | Saturating float-to-int16 in every conversion (`Utils::saturateToInt16`) | `utils.cc` | low |
| 5 | Self-insert undefined behaviour in the cyclic-prefix copy | `config.cc` | none |
| 6 | `printf` conversion/argument mismatch at TRACE level | `receiver.cc` | none |
| 7 | Include-guard typo | `signalHandler.hpp` | none |
| 8 | Indeterminate stream pointers before `setupStream` | `Radio.h` | none |
| 9 | Destructor out-of-range on a multi-cell failure; a failed open no longer poisons the in-process retry; `err.what()` printed | `BaseRadioSet.cc` | low |
| 10 | Dead per-cell re-parse of the topology JSON | `config.cc` | none |
| 11 | `resync_period` could be 0 for very long frames | `sync_geometry.h` | none |
| 12 | Beacon search could return an index past the caller's buffer | `comms-lib-portable.cc` | low |
| 13 | `#pragma once` in `comms-lib.h` | | none |
| 14 | CMake: aarch64 never selected the ARM muFFT | `CMakeLists.txt` | none |
| 15 | `plot_lib.py`: `set_ylim` on the wrong subplot | `PYTHON/IrisUtils` | none |

## 3. The features and their switches

| Feature | Switch | Default | Note |
|---|---|---|---|
| Houdini radio backend | `radio_type` | `iris` | 23 `is_houdini()` sites |
| Free-running-clock tracker, targeted resync, escalation | `is_houdini()`; `sync.tracker.*`, `sync.resync.*` | Houdini only | |
| Sync library (`houdini_sync`) | always linked (in-tree, no external dependency) | | `comms-lib` and `utils` moved into it |
| Beacon shapes | `beacon_type` / `sync.beacon.type` | `legacy` | five shapes |
| Detector pick and threshold forms | `sync.detector.pick`, `sync.detector.threshold` | Houdini: first_path + xcorr; Iris/UHD: first_crossing + power | platform defaults derived, overridable |
| GPU correlator | `HOUDINI_USE_CUDA` (CMake) | OFF | |
| CSI dashboard / view mode | `HOUDINI_CSI_UDP` (`--view`) | off | warns loudly that no HDF5 is written |
| UE UL data slot, fine-grid TX | `ue_tdd_pilot`, `ue_tx_advance_ticks`, `ue_pilot_horizon` | off | |
| Gap ledger / rx-recorder | `is_houdini()` | | |
| Acquisition threshold | `corr_scale_init` / `sync.detector.corr_scale_init` | = corr_scale | |
| Environment overrides of the sync knobs | `sync.allow_env_overrides` | true this release | flip planned next release |
| Correlator threads | `sync.detector.corr_threads` | 1 | `SOUNDER_CORR_THREADS` as the logged alias |
| Radio-free tests and bench tools | `SOUNDER_BUILD_TESTS` | ON | |
| Diagnostic dumps | `HOUDINI_DUMP_*`, under `HOUDINI_DUMP_DIR` | off, `/tmp` | |

## 4. Recommended packaging

1. **A `fix/legacy-sounder` series**: the fifteen fixes above, cherry-picked
   in that order onto `master`, each with its one-paragraph rationale. They
   need no Houdini hardware and no new configuration.
2. **The feature branch as it is**: the Houdini backend, the tracker, the
   library and the shapes, gated as the table shows. Nothing in it changes a
   legacy run at the defaults except the four items listed under "left
   changed" in section 1, which the PR description should name.
3. **Tree hygiene as its own PR**, because it is a policy decision and not a
   code change (section 5).

## 5. Decisions that are the user's

DECIDED 2026-09-03 (recorded in `RADIO_PLATFORM_SEAM.md` section 1): the
environment-override default flips to off with the bench scripts moved to
the JSON overlay; the ledger and walkthrough stay and the raw captures and
superseded probes move out at branch landing; Iris and UHD remain
inspection-only with a build matrix; the packet width and the portable
correlator are accepted; Agora's radio abstraction is adopted in shape, not
in code, for now.

DECIDED 2026-09-03 (branch model, [user]): `master` is the last stable
release and `develop` the latest tested state, promoted to `master` rarely.
So the feature branch lands as ONE pull request into `develop`, carrying the
fifteen fixes with it; the separate fix series of section 4 becomes a
stable-line patch onto `master` only if a patch release is wanted, and would
need its own verification against `master`'s tree. Order of operations: push
`develop` (it is ahead of `origin/develop`), then the feature branch, then
the PR; the raw captures leave the tip before the PR (section 4, item 3,
folded into the landing).


- Whether to flip `sync.allow_env_overrides` to false now (the bench scripts
  still sweep through the environment) or next release as planned.
- Whether the evidence captures (`tests/demo-verify/evidence`, 3.9 MB), the
  ledgers (`DEMO_VERIFICATION.md`, `DEMO_BENCH_RUNBOOK.md`, `BACKLOG.md`,
  `tools/tracker_lint.py`) and the bench probes (57 scripts, several
  superseded) stay in the shipped tree or move to the notes repository. The
  ledger is the evidence home the lane's process names, so this is a process
  decision, not a code one.
- Whether an Iris or UHD run is worth scheduling: every "unchanged" claim in
  section 1 is inspection-only until one exists. A build matrix over the
  three `RADIO_TYPE` values and a config-load smoke test would make the
  claims measurable without hardware.
- Whether the 20-byte `Packet` and the portable correlator are acceptable
  for the legacy sounder (they are for Houdini; they cost 4 bytes per packet
  and a not-bit-identical detector elsewhere).
