# AP-109 wired on 0.5.0 (DEMO_VERIFICATION 9.84)

The probes behind 9.84, as they ran on the rig host. Nothing a build or gate
reads.

| File | What it is |
|---|---|
| `topology-houdini-dualband-swap.json` | The topology with the roles exchanged (BS .21, UE .22), the bench as cabled (AP-121). |
| `make_swap_configs.py` | Writes the two config copies that name it: the demo config and the 48 MHz fallback. |
| `three_runs.sh` | W5R1 and W5S3, each a `demo_run.sh` with a `pacer_core_check.py` 70 s in. W5F1 ran from an earlier form with a W5F1 line first, fed over `ssh ... bash -s`; the run's inner `ssh` read the rest of the script from that stdin, so only W5F1 ran. Run it as a file, as here. |
| `load_run.sh` | W5L1: a 180 s demo run with busy loops pinned to cores 0-14 (one process at +60 s, fifteen at +120 s, 10 s each). |
| `abs_times.py` | The UTC time of matching sounder log lines: the log prefix is CLOCK_REALTIME seconds mod 100 (`CC/Sounder/include/logger.h`), walked from the run's start time. |
