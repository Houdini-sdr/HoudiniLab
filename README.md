# HoudiniLab: the Houdini channel sounder and dashboard

HoudiniLab is the RENEWLab software suite adapted for Houdini software-defined
radios. Its centre is the C++ channel sounder in `CC/Sounder`, which drives
two Houdini nodes, a base station and a client, through the SoapyHoudiniSDR
driver, and a live web dashboard that shows each band's channel as it is
measured. Its current use is the dual-band channel sounding demo: sub-6 at
2425 MHz beside an X-band IF at 4380 MHz, one 5G-like numerology on both
bands, the channel estimate and the equalized constellation of each band side
by side.

## Start here

| You want to | Read |
|---|---|
| Run the demo on your own bench, from an empty host to a live display | `CC/Sounder/CSI_DEMO_WALKTHROUGH.md` (its section map tells you what to skip) |
| Read the dashboard, panel by panel | `CC/Sounder/DASHBOARD_GUIDE.md` |
| Look up an option, a flag or a sync knob | `CC/Sounder/SOUNDER_REFERENCE.md` |
| Fix a run that fails | `CC/Sounder/TROUBLESHOOTING.md`, through the walkthrough's symptom index |
| Run it on our lab bench | `CC/Sounder/DEMO_BENCH_RUNBOOK.md` |
| See what was measured, and on which stack | `CC/Sounder/DEMO_VERIFICATION.md` (the newest row of its last section is the current state) |
| Pick a configuration | `CC/Sounder/files/houdini-*.json`; each says what it runs in its `_description` |
| Find your way around the sounder's code and tools | `CC/Sounder/README.md` |
| Understand a design decision | `docs/README.md` |

## What is in the repository

| Path | What it is | Houdini state |
|---|---|---|
| `CC/Sounder/` | The channel sounder (C++), its configs, the dashboard (`csi_gui/`), the run and report tools, the tests | The Houdini work lives here |
| `PYTHON/` | The RENEW Python development suite: offline processing and the Iris-era design flow | `IrisUtils/plot_hdf5.py` reads the sounder's recordings and knows the Houdini receive path (its spectral inversion, its timing jitter); the rest is upstream |
| `MATLAB/` | The RENEW MATLAB development suite | Upstream, unchanged |
| `WEBGUI/` | The RENEW Dashboard: a web application that monitors a RENEW base station's health | Upstream, unchanged; not the sounder's dashboard (that is `CC/Sounder/csi_gui/`) |
| `docs/` | Design notes and, under `archive/`, finished investigations | See `docs/README.md` |
| `tools/`, `probes/` | The lane's bench tools (shipping a branch to a rig host and testing it there) and the records of one-off probes | Houdini |
| `BACKLOG.md` | The work tracker | |
| `install_*.sh`, `config_ci.sh`, `Jenkinsfile` | RENEWLab's own installers and CI | Upstream; not used for Houdini, whose install is the walkthrough's section 2 |

## Collect and process channel datasets

The sounder has two modes. In **viewing mode** (`--view`) it streams each
frame's channel estimate to the dashboard and writes no file; that is the
demo, and the walkthrough covers it. In **recording mode**, without `--view`,
it writes the measurements to an HDF5 trace file, which is what RENEWLab's
sounder has always done and what dataset collection uses:

1. Build the sounder and point a config at your radios as the walkthrough's
   section 2 describes.
2. If the config carries uplink or downlink data slots, generate the data
   bits first: `./build/sounder --conf_file files/<config>.json --gen_data_bits`.
3. Record: `./build/sounder --conf_file files/<config>.json --storepath <dir>`.
   The trace file lands in `<dir>`, named with a time stamp. On Houdini
   radios the file also carries a table of the sample ranges the receive path
   marked as untrusted, so a recording is not evidence of a clean link until
   that table is read.
4. Inspect it with `PYTHON/IrisUtils/plot_hdf5.py`, whose options include the
   Houdini-specific corrections. The RENEW documentation linked below
   describes the file's layout and the rest of the processing tools.

## RENEWLab heritage

RENEWLab is the open-source software toolbox of the
[RENEW massive MIMO platform](https://renew-wireless.org): a Python
development suite, a MATLAB development suite, the C++ Sounder framework and
the RENEW Dashboard, written for the Faros base station and Iris radios of
Skylark Wireless, with partial USRP support through SoapyUHD. HoudiniLab keeps
that structure and that code: the Iris paths are intact, and the Houdini
radio is one more backend behind the sounder's radio seam (`docs/README.md`).

- Upstream repository: [renew-wireless/RENEWLab](https://github.com/renew-wireless/RENEWLab)
- Upstream documentation: the [RENEW wiki](https://wiki.renew-wireless.org/)
  and the design flows for the
  [Python](https://docs.renew-wireless.org/dev-suite/design-flows/python-design-flow/),
  [MATLAB](https://docs.renew-wireless.org/dev-suite/design-flows/matlab-design-flow/)
  and [C++](https://docs.renew-wireless.org/dev-suite/design-flows/cpp/) suites
- API documentation: `doxygen RENEWLab_doxygen.conf` from the repository root

## License

RENEWLab, and with it HoudiniLab, is used under the
[RENEW open source license](https://renew-wireless.org/license).

## Acknowledgement

RENEW was funded by the NSF PAWR project.
