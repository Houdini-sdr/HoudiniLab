# Run and report tools

The scripts that run the demo for evidence on a rig host, and the tools that
judge a run from its log, its dumps and the dashboard stream. The runbook
(`../../DEMO_BENCH_RUNBOOK.md`) says which run which; `DEMO_VERIFICATION.md`
cites their output. Every `*_test.py` here is a unit test that `ctest` runs.

## Running

| Script | What it does |
|---|---|
| `demo_dashboard.sh` | Launches the dashboard in control mode on the rig host with the labelled configs and the replays in its list |
| `demo_run.sh` | One evidence run of the demo with the samplers alongside (the MER, the spectra, the freeze watch), filed in a run directory, then `run_summary.py`'s verdict |
| `fstage_run.sh` | One run of a filter stage: the sounder with its dumps, filed in a run directory with the stage noted |
| `run_rung.sh` | One run of a config, a rung of the ladder |
| `run_shape_campaign.sh` | An interleaved beacon-shape campaign: one config, only the beacon type varied, the order rotated between rounds |

## Judging a run

| Tool | Reads | Reports |
|---|---|---|
| `run_summary.py` | a run directory or a sounder log | The run against the runbook's "what good looks like": a verdict and its reasons |
| `demo_report.py` | a demo-length run's log | The end report: transmit totals, the status timeline, the warnings |
| `rung_report.py` | one sounder log | Every pass-criterion number, throttled warnings read by their last count rather than by counting lines |
| `fstage_report.py` | stage directories | Per-run link numbers and the change from stage to stage |
| `gate_summary.py` | a sounder log | The sync gate metrics: escalations, off-grid detections, the low-score fraction, the residual spread |
| `shape_campaign_summary.py` | a campaign directory | The per-shape, per-round table, on `gate_summary`'s numbers |
| `late_release_report.py` | a sounder log | Every transmit late-release warning with the stamps around it, then the per-channel totals |
| `dl_level.py` | the re-sync window dumps | The downlink level at the client: the beacon's power and the floor beside it |
| `gate_runs.py` | the base station's raw dumps | The fabric packet gate as the raw reads show it |

## Sampling while a run is live

| Tool | What it samples |
|---|---|
| `mer_sampler.py` | The dashboard stream: each antenna's one-second MER at a period, with its frame and age |
| `spc_sample.py` | The dashboard's pilot-slot spectra: in-band mean and peak, the floor outside, the strongest out-of-band bin |
| `freeze_watch.py` | The run's log, on the rig host, for a transmit playout freeze |
| `irq_sampler.py` | The data NIC's interrupt rate per core |
| `pacer_core_check.py` | Whether a NIC receive flow landed on one of the sounder's pinned cores (run it in the first minute of a run, after every host boot) |
| `reg_snap.py` | The packet gate's fault registers on each node, with no stream open, before and after a run |

## Shared by the tools

`sounder_log.py` parses the log lines more than one tool reads, in one place,
each pattern beside the source that prints it; `rig_dumps.py` reads the
sounder's dumps (the constellation, the beacon memory, the re-sync windows).

## Records

`evidence/` holds captured evidence by date, `archive/` superseded tools,
and the two notes beside them are records: a code review's findings, and a
cross-check whose interpretation was written down before it ran.
