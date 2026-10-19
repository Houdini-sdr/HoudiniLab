# The demo wired on 0.6.0 (DEMO_VERIFICATION 9.85)

The probes behind 9.85, as they ran on the rig host. Nothing a build or gate
reads.

| File | What it is |
|---|---|
| `w6_run.sh` | One `demo_run.sh` run (W6S1, W6S2) with a `pacer_core_check.py` 70 s in, on the swapped config copy of 9.84 (`probes/ap109-wired-050/`). Copied to the rig host and run as a file, detached with `setsid nohup`. |
| `w6_mon.sh` | The hourly-firing monitor of W6S2: `mpstat -P ALL 1` and `pidstat -u 1` from a set start, and a `ps` snapshot of the apt, PackageKit, gdbus and esm processes with their parents every second through hh:26:35 to hh:27:15. Its output is filed in W6S2's run directory as `hourly_monitor/`. |

W6T1 needed no probe: it is `tests/demo-verify/demo_run.sh --topology` on the
repo's demo config. The log times come from `probes/ap109-wired-050/abs_times.py`.
