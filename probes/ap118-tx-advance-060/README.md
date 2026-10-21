# tx_advance on 0.6.0 (DEMO_VERIFICATION 9.86)

The probes behind 9.86, as they ran on the rig host. Nothing a build or gate
reads. The shell scripts were copied to the rig host's `/tmp` and run as files,
detached with `setsid nohup`; `sweep.sh` calls `/tmp/edge_run.sh`, which calls
`/tmp/slot_edges.py`.

| File | What it is |
|---|---|
| `edge_run.sh` | One run of a config through `fstage_run.sh` with the BS frame dumps (`--bs_dump_frame`) and the MER every 15 s, on the bench as cabled (the swapped topology, main on 16, the pacers on 18,19); then the run's verdict, `slot_edges.py` over the dumps and the logged `pilot_grid_off` of each dumped frame. A169a, A198a-c, F198a and R198a. |
| `sweep.sh` | One `edge_run.sh` per `tx_advance` value, each on a copy of the demo config in `/tmp/txadv` with only `tx_advance` changed. S158 to S238. |
| `long_run.sh` | One `demo_run.sh --topology` run of the repo's demo config with a `pacer_core_check.py` 70 s in. L198. |
| `hourly_mon.sh` | The hourly-firing monitor: `mpstat -P ALL 1` and `pidstat -u 1` from a set start, and a `ps` snapshot of the apt, PackageKit, gdbus and esm processes with their parents every second for 40 s from a second set start. Its output for L198 is filed in L198's run directory as `hourly_monitor/`. |
| `slot_edges.py` | Where the UE burst sits in each BS rx slot, from the frame dumps: the leading and trailing gaps around the content of lane 0 (sub-6), beside the gaps the reference lane's seat implies. It recomputes `pilot_grid_off` from the dump's grid as a check on the arithmetic. `--prefix`/`--postfix` give the pads (32 at the demo config, 128 at R1). Its summary's "over N frames" counts slot occurrences: a dump spans 1.2 frames, so a frame mostly gives each slot twice. |
| `seat_bins.py` | The logged `pilot_grid_off` of a run in bins of a set length, binned by frame number so that the lines printed without a time prefix count too. |
| `hs316_check.sh` | 9.87's first three bring-ups (H198a-c) on fpga 1.45 and SH-566's host: three `edge_run.sh` runs of the demo config at 198, from a copy of `edge_run.sh` with its stack string replaced (`/tmp/edge_run_145.sh`). |
| `hs316_more.sh` | 9.87's matched-carrier runs: one 600 s `edge_run.sh` (W198a), then three more bring-ups on the warm nodes (H198d-f), through the same copy. |
| `seat_vs_cfo.py` | A run's logged `pilot_grid_off` in bins of frames beside the BS's pre-FFT carrier reading of the UE pilot and the clock-steering pushes in each bin. |
| `seat_at_zero_cfo.py` | Each pre-FFT carrier reading paired with the mean seat within 100 frames of it (on the 198 scale, so runs at other `tx_advance` values compare); a line per run and per group read at zero offset, and the paired seats in bands of carrier offset with no line, the direct comparison. Validated on 9.86's runs: its run means equal 9.86's (-0.39, -0.34, -0.09). |

The log times come from `probes/ap109-wired-050/abs_times.py`.

Centred, at the demo config, is a leading gap equal to the trailing one. The
filters in the path spread each content edge by about 3.5 samples (the two
gaps sum to about 57 against the pads' 64), so a centred burst reads about
28.5 on each side of lane 0.

A bring-up's seat follows the carrier offset between the nodes, about +2.2
samples per kHz of the BS's pre-FFT reading at 2425 MHz, while the clock
steering pulls the offset in (9.87). Compare seats across stacks at a matched
offset (`seat_at_zero_cfo.py`'s bands), not as run means: a fitted line over
one bring-up carries the steering's transient and reads further from the
settled seat the further the run started from zero offset.
