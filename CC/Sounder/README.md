# The Houdini channel sounder: what is where

This directory holds the sounder, its configurations, the live dashboard and
the tools that run and judge it. The documents come first, then the map.

## The three documents

| Document | Kind | Read it when |
|---|---|---|
| `CSI_DEMO_WALKTHROUGH.md` | How-to, for any bench | You are setting the demo up or running it; its section map says what each section gives you and when to skip it |
| `DEMO_BENCH_RUNBOOK.md` | Operating guide, our lab bench | You are on our rig: the machines, the RF chains, the launch, demo day, recovery |
| `DEMO_VERIFICATION.md` | Record | You want the evidence behind a number or a claim; the newest row of its last section is the current state |

`PER_BAND_BW_DESIGN.md` is a design note (each band at its own width on one
numerology); `docs/README.md` at the repository root indexes it with the
others.

## The map

| Path | What it holds |
|---|---|
| `main.cc`, `config.cc`, `receiver.cc`, `scheduler.cc`, `recorder_*.cc`, `hdf5_lib.cc` | The sounder: options, the config, the receive and sync path, the frame schedule, the recorder and its HDF5 output |
| `Radio*.cc`, `RadioSetFactory.cc`, `BaseRadioSet*.cc`, `ClientRadioSet*.cc`, `*Framer.cc` | The radio seam: one `Radio` interface with a Houdini, a Soapy and a UHD backend, one set class per role, and the framers (the Houdini one arms the hardware TDD schedule and the beacon) |
| `comms-lib*.cc`, `sync/`, `include/` | Signal processing (portable and AVX), the sync library headers (beacon shapes, detection, the grid tracker, the resync policy), the headers |
| `files/` | The configurations (`houdini-*.json`, each with a `_description`) and the topology files that name the radios |
| `csi_gui/` | The dashboard backend and its tools, below |
| `tests/` | The tests and the run tools, below |
| `tools/` | `build_matrix.sh` builds the sounder for every radio backend this host can compile, so a change that breaks another platform's build is caught the day it lands; `rig_release_holders.py` stops every local process holding a board's receive stream |
| `img/` | The pictures the walkthrough embeds |
| `probes/` | Records of one-off probes as they ran on the rig host; nothing a build or a gate reads |
| `mufft/`, `third_party/` | The FFT library (a submodule) and vendored code |
| `build/`, `logs/` | Build output and the recording mode's default store path; both ignored by git |

## The dashboard and its tools (`csi_gui/`)

| Tool | What it does |
|---|---|
| `csi_server.py` | The backend: receives the sounder's datagrams, serves the page, and with `--control` starts, stops and checks the sounder from the page |
| `check_setup.py` | Checks that this host and the two radios are ready for a config, and says how to fix what is not; run it before the first start and after any failed one |
| `teardown_framer.py` | Releases the TDD framer a killed run left armed, so the next run starts clean |
| `replay_feed.py` | Plays a recording of the dashboard's input back into the backend, so every panel shows real data with no radios |
| `csi_record.py` | The recording format (every datagram with its arrival time), shared by the backend and the replay |
| `fake_feed.py` | A synthetic feed in the sounder's datagram kinds, for looking at the page or testing a change to it with no radios |
| `score_runs.py` | Restarts the demo N times and scores each run's constellation, for a fault that is decided per run |

The walkthrough's section 4 runs them and its section 5 reads the page.

## Tests and run tools (`tests/`)

| Path | What it holds |
|---|---|
| `comms-func/` | The C++ unit tests behind `ctest` (the sync library, the band filters, the frame geometry, the radio set-up, the receiver's fault paths, the dashboard's signal processing), plus a few standalone bench helpers |
| `demo-verify/` | The scripts that run the demo for evidence and the tools that judge a run from its log and its dumps; its `README.md` lists them |
| `hil/` | Python probes that drive a radio directly, without the sounder: the beacon over a TDD schedule, a two-board tone, an OFDM loop over one board's own cable (`README_loopback_ofdm.md`), a passband sweep |
| `config_keys_test.py` | Every config key the sounder reads, checked against the shipped configs |

## Build and test

The walkthrough's section 2.5 builds the sounder (the `mufft` submodule
first, then CMake in `build/`). From `build/`, `ctest` runs the C++ unit tests
and the Python tests of the dashboard and the run tools together, with no
radio; a change is not done until it passes on the host it will run on.
