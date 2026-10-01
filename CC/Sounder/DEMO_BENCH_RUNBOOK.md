# Lab bench runbook: exactly what runs where

This file is the bench-specific companion to `CSI_DEMO_WALKTHROUGH.md`. The
walkthrough stays neutral (placeholders and discovery commands) so it travels
to any bench; this runbook fills in every placeholder for OUR lab setup so a
run can be reproduced or debugged without archaeology. If the bench changes,
change this file in the same commit.

Evidence for everything below lives in `DEMO_VERIFICATION.md`.

Two benches are described. **Part A** is the dual-band demo (AP-79: sub-6 plus
the X-band IF, HS-202 mode V) on the rig host `.26`, the current one, and the
part operators follow. **Part B** is the earlier single-band CSI demo on rig B
(`.64`), kept as a record of that bench; do not follow it for the dual-band
demo.

# Part A: the dual-band demo (AP-79)

## A1. The machines

| Machine | Address | Role | What runs on it |
|---|---|---|---|
| Build VM | this repo at `/space/vmshare/repos/HoudiniLab` | Development only | Editing, compile checks, offline analysis. Code ships to the rig as a git bundle over ssh. |
| Rig host "burton" | `168.6.244.26`, user `houdini` | The whole host side | The sounder and the dashboard backend. A DGX Spark: 20 aarch64 cores, fast ones 5-9 and 15-19. Data NICs `enp1s0f0np0` 192.168.5.11 (UE side) and `enp1s0f1np1` 192.168.6.13 (BS side), MTU 9000. The control network to the boards is `enx00e04c242668` 192.168.10.26. |
| Base station | `192.168.10.22`, user `houdini` | BS radio | `SoapySDRServer` (systemd) with the device plugin. |
| Client | `192.168.10.21`, user `houdini` | UE radio | The same stack as the BS. |
| Your workstation | anywhere | Viewer | A browser through an ssh tunnel (section A5). |

The boards sit on a control network only the rig host reaches: from any other
machine, go through it (`ssh -J houdini@168.6.244.26 houdini@192.168.10.22`).
Every SoapyRemote control call from the sounder rides that network.

The roles are the reverse of Part B's: here `.22` is the BS and `.21` the UE
(`files/topology-houdini-dualband.json`). Both boards run their reference in
**calibrated hold** (`clock_ref = calibrated`; `.21` DAC code 408, `.22` 404),
with no shared 10 MHz: the UE-BS offset stays under about half a ppm, but it
moves between sessions and ramps as the boards warm (`DEMO_VERIFICATION.md`
8.125, 8.137, 9.28). Both boards carry the SYZYGY DNA adapter on POD2 and an
ADTR1107 power board (the X-band front end); the XUD1A up/down converter sits
on `.22`. Each node's X-band role (`houdini-role`: `.21` xband guard=ch1, `.22`
xband guard=none) is needed only for the X-band RF chain (A2b); for the cabled
chain the boards run with no role, in highz.

## A2. The wired RF chain (the final wired state, "F3b")

Every link is a ONE-WAY cable. Channel letters in the configs: A = ch0,
B = ch1, C = ch2.

| Link | From | Filter(s) | To |
|---|---|---|---|
| Sub-6 downlink (the beacon) | `.22` TX ch0 (DAC_B) | VBF-2450+ at the DAC and (since 9.69) at the ADC | `.21` RX ch0 (ADC_D) |
| Sub-6 uplink | `.21` TX ch0 (DAC_B) | VBF-2450+ at the DAC and (since 9.69) at the ADC | `.22` RX ch0 (ADC_D) |
| X-band IF uplink | `.21` TX ch1 (DAC_A) | VBFZ-4000-S+ at the DAC, and a second VBFZ-4000-S+ at the ADC | `.22` RX ch2 (ADC_B) |
| HIL self-loop (unused by the demo) | `.22` TX ch1 (DAC_A) | none | `.22` RX ch1 (ADC_C) |

There is no X-band IF downlink and no attenuator in the chain (a 10 dB pad at
each sub-6 receive end was tried and removed, 9.23 and 9.24). What each filter
did to the levels, and why the in-channel "tilt" is cable ripple rather than
the filters, is `DEMO_VERIFICATION.md` 9.15 to 9.24. The second sub-6 filter at each ADC
costs about 2 dB of level against the recorded wired baseline; compare a new
cabled run's levels with 9.69 to 9.71, not with 9.60/9.61.

## A2b. The X-band RF chain (F2b: through the XUD1A and both ADTR1107 boards)

From device 0.3.1. The X-band IF leaves `.21` DAC_A, goes up to RF 9.5 GHz in
the XUD1A's channel A and out through `.21`'s ADTR1107, crosses a 20 dB pad
into `.22`'s ADTR1107, comes back down to IF in the XUD1A's channel B and
reaches `.22` ADC_B:

- `.21` DAC_A -> VBFZ-4000-S+ -> XUD1A J1 (channel A TX in); J1A -> `.21`
  ADTR1107 TX_IN; its ANT -> 20 dB pad -> `.22` ADTR1107 ANT.
- `.22` ADTR1107 RX_OUT -> XUD1A J1B (channel B RF in); J6 -> VBFZ-4000-S+ ->
  VAT-10A+ -> `.22` ADC_B.
- Both ADTR1107 CPLR_OUT ports terminated (unless `.21`'s is on an analyzer).

The XUD1A is the reworked board (RF 9.5 GHz, LO 13.88 GHz). It needs:

1. The roles applied: `sudo houdini-role status` exits 0 on both nodes. `.21`
   applies its role at boot. `.22` applies it BY HAND after the XUD1A's 12 V
   is on: boot `.22` with the 12 V off, switch the 12 V on, wait for clean SPI
   (10 s to 2.5 min), then `sudo houdini-role apply bs`. On `.22` the role
   also tunes the LO to 13.88 GHz on rf16 and loads the doubler tracking
   filter from the XUD1A datasheet's Table 7; no manual LO tune is needed.
   From the rig host (its key is on both nodes; password login is the
   backup): `ssh houdini@192.168.10.22 'sudo houdini-role apply bs'`.
2. The LO check: `ssh houdini@192.168.10.22 'sudo houdini-xud1a pll status'` shows
   `lock_detect=1`, `rf16 ON at 13880000000...` and `doubler tracking table7
   (REG0070 0x23, filter 1 bias 3, ...)`; on 0.3.1 `houdini-role status`
   fails on `.22` unless the doubler reads table7. After any `houdini-xud1a
   bist`, sweep or manual tune on `.22`, run `sudo houdini-role apply bs`
   again before a run (bist parks the lines and turns the LO off).

   Levels (the software lane's measurements): with Table 7 the XUD1A loop
   reads about 22 dB stronger at RF 9.5 GHz than at 10 GHz (-32 against
   -54.5 dBm; the earlier "about 9.5 dB weaker at 10 GHz" of
   `DEMO_VERIFICATION.md` 9.68 was the automatic doubler filter's low state,
   SH-451). Through the whole chain a -12 dBFS tone reads -28 dBm at ADC_B
   with about 40 dB SNR in 100 MHz at attenuation 0. The sounder's MER through
   this chain is not measured yet: the first `-fe` run is that measurement.
3. A config with `xband_frontend_static` (AP-86: the session holds the UE's
   board in static TX and the BS's in static RX; without it `.21`'s guarded
   channel plays silence): `files/houdini-dualband-xw-steer-slots-fe.json`
   (with slots mode) or `files/houdini-dualband-xw-steer-fe.json`. Within the
   first minute the log shows `TDD_EXTPIN_SRC src=static,state=tx ...
   applied=tx ... drive_allow_ch1=1` for the UE and `state=rx` for the BS.
   With no role these configs are refused at start; the cabled chain (A2)
   uses the configs without `-fe`.

## A3. What runs on the rig host

- **Checkouts.** `~/repos/HoudiniLab` belongs to its owner: do not change it.
  The demo runs from `~/repos/HoudiniLab-rxwin` on develop, checked out
  there as the branch `rig/develop` (the owner's checkout holds `develop`).
  `--sounder-dir` and the checkout the dashboard lives in decide which binary
  runs. Superseded run
  directories are filed under `~/app_archive` (its `INDEX.md` maps them).
- **Shipping a build.** From the lane's checkout, `tools/ship_to_rig.sh
  [options] <user>@<rig-host> <rig worktree> [<branch>]` does all of it and fails
  closed (it refuses while a sounder or a hardware-in-the-loop suite runs, or
  the host is busy; `--check-string <text>` makes it require the new binary to
  carry a string only the new code logs; the options are at its head). By
  hand: bundle, copy, and fetch INSIDE the target worktree (`FETCH_HEAD` is per
  worktree), then relink muFFT, which a checkout restores as an empty directory:

  ```sh
  cd ~/repos/HoudiniLab-rxwin && git fetch /tmp/<bundle> <branch> && git reset --hard FETCH_HEAD
  cd CC/Sounder && rmdir mufft && ln -s ~/repos/HoudiniLab/CC/Sounder/mufft mufft
  source ~/houdini_test/bin/activate
  cmake -B build -DCMAKE_BUILD_TYPE=Release -DSoapySDR_DIR=$VIRTUAL_ENV/share/cmake/SoapySDR
  cmake --build build -j10 && (cd build && ctest)
  strings build/sounder | grep <a-string-only-the-new-code-logs>
  ```

  `ln -sfn` onto the directory nests the link inside it; remove the directory
  first. Without `SoapySDR_DIR` the configure fails ("SoapySDR development files
  not found").
- **Host plugin.** Activate the venv `~/houdini_test` before anything: it is
  the SoapySDR runtime and the Python bindings, and it carries NO Houdini
  module. The Houdini host plugin is the release's own prefix, built with the
  radios' device build: `~/houdini_0.3.1`. `--soapy-root $HOME/houdini_0.3.1`
  selects it for every config: on `check_setup.py`, `run_rung.sh`,
  `fstage_run.sh`, `teardown_framer.py` and `reg_snap.py`, AND on the dashboard,
  which hands it to its Check, Start and teardown; `demo_run.sh` takes the
  prefix as its fourth argument. Without it no radio opens. The setup check's
  stack line shows which one loaded (`host_build`, equal to `device_build`; a
  mismatch is a WARN). Going back a release is a deploy of that release's host
  prefix and device modules: the software lane's, on the user's go.
- **Cores.** The sounder's `--core_map` places its threads by role and the
  main thread pins itself only after the radios start, so the plugin's BS
  receive workers run on the housekeeping cores 0-9 (AP-81, 9.44). The launch
  (A4) puts the main thread on isolated core 15 and the UE's two TX pacers on
  isolated 18 and 19.

## A4. The launch

On the rig:

```sh
source ~/houdini_test/bin/activate
cd ~/repos/HoudiniLab-rxwin/CC/Sounder
cat /sys/devices/system/cpu/isolated   # 15-19 when A9 stage 1 is in force
python3 csi_gui/check_setup.py --soapy-root $HOME/houdini_0.3.1 --conf files/houdini-dualband-xw-steer-slots.json   # must print Ready, egress PASS on both nodes.
# served on every address (A5); the four demo configs and the replays in the list (A8b); one log per Start (A8c step 4)
# --soapy-root: the host plugin of the radios' release, for every config (A3)
# --core_map, --tx_cpu_affinity: isolated (A9); 18,19 while a BS receive flow lands on 16 (pacer_core_check, A8c)
# --tx_host_status: logs the host pacer's state every health period (free)
python3 csi_gui/csi_server.py --control --http-host 0.0.0.0 --configs labelled \
  --soapy-root $HOME/houdini_0.3.1 \
  --sounder-arg=--core_map=main=15 --sounder-arg=--tx_cpu_affinity=18,19 --sounder-arg=--tx_host_status \
  --replay "$HOME/demo_rec/replays/xband_chain_bench_60s.rec=Replay, X-band bench" \
  --replay "$HOME/demo_rec/replays/xband_ota_60s.rec=Replay, X-band over the air" \
  --replay "$HOME/demo_rec/replays/xband_ota_blockage_30s.rec=Replay, over the air with blockage" \
  --replay "$HOME/demo_rec/replays/xband_ota_noisy_sub6_60s.rec=Replay, over the air, noisy sub-6" \
  --log-dir ~/demo_logs --conf files/houdini-dualband-xw-steer-slots.json
```

**Why the pinning.** Left to the kernel, the host plugin's two UE TX pacer
workers land on the cores that take the 100G data NIC's interrupts (about 61k
completion IRQs a second on one core, 1-3k on several others), where receive
softirq work preempts them for milliseconds and whole bursts go out late
(`DEMO_VERIFICATION.md` 9.36). The dashboard hands its `--sounder-arg` flags
to the sounder it launches. **The values in force:** the main thread on isolated core
15 and the pacers on isolated 18 and 19, all performance cores. The NIC's
receive hashing is re-drawn at every rig-host boot and can put a BS receive
flow on a pinned core (it once landed on 16, which is why the pacers left 16
and 17), so check it every power-up with
`tests/demo-verify/pacer_core_check.py --cores 15,18,19` (A8c step 5). On
isolated performance cores the pacer's worst wake over a 35 min run is about
0.28 ms, against 10.5 ms on the efficiency cores 10 and 11 (9.41, 9.44).

The check's full form reads both radios' stacks and FAILs if they differ.
Read the stack there, not from this file: it changes with every deploy. The
last validated stack is in the newest `DEMO_VERIFICATION.md` section 9 row.

The demo configs (A8c): `houdini-dualband-xw-steer-slots.json` (cabled) and
`-xw-steer-slots-fe.json` (through the XUD1A, A2b), both on the release's plugin.
The slots variants also turn on the pre-FFT carrier correction and the 0.12 ppm
steering deadband, so dropping back to `-xw-steer(-fe).json` changes all three
at once. `houdini-dualband-steer.json` is the same build with the X-band at 48
MHz. The older ladder (walkthrough section 3): `houdini-r0.json` (control),
`houdini-dualband-r1.json`, `-r2.json`, `-r3a.json`, `houdini-dualband.json`
(R3, the 48 MHz unsteered baseline of 9.1-9.54) and `houdini-dualband-40.json`;
on a freshly deployed stack a short run of the demo config (9.70) replaces the
climb. R0 (NCO 500 MHz) cannot run while the F3b chain is fitted: the VBF-2450+
bandpasses on the sub-6 DAC paths block its beacon, so the UE never acquires
(`DEMO_VERIFICATION.md` 9.40). It is the control only with the filters out.

For evidence runs without the dashboard, `tests/demo-verify/demo_run.sh` runs
the demo head for a set time with the samplers the records cite (the freeze
watch, the MER every 15 s, three spectra), files them into the run directory
and ends with `run_summary.py`'s verdict against A6 (its exit status):

```sh
tests/demo-verify/demo_run.sh <TAG> files/houdini-dualband-xw-steer-slots.json 2100 ~/houdini_0.3.1
```

It hands the sounder the A4 placement itself (`--core-map` and
`--tx-cpu-affinity` change it, default `main=15` and `18,19`) and
`--tx_host_status`; `--record <file>` records the dashboard stream (A8b) and
`--ue-ssh houdini@<the UE's address in A1>` counts the UE's RFDC interrupts
over the run (the options are at the head of the script).

The run lands in `ap79_runs/DEMO/<TAG>_<HHMMSS>/`. `run_summary.py <run dir>`
judges a finished run again, and `run_summary.py ~/demo_logs/sounder_<UTC>.log`
a dashboard session (A8c step 4). Underneath, `run_rung.sh` and `fstage_run.sh`
(each taking `--soapy-root`, and `--sounder-arg=<flag>` for any further sounder
flag) launch one sounder run with the logs and dumps that `rung_report.py` and
`fstage_report.py` read; `reg_snap.py` reads the fpga 1.34 gate's fault
registers before and after a run, and `gate_runs.py` the BS landing dumps
(9.70).

`demo_run.sh` starts the freeze watcher itself. HS-227 is fixed from fpga 1.32
(A7) and the watcher stays as a cheap guard: it reads only the local log (no
network traffic) and prints one line when any TX channel's `played` stops, or
when the device reports `PLAYOUT FROZEN`. For a run started another way:

```sh
python3 tests/demo-verify/freeze_watch.py --run-dir $PWD/ap79_runs --tag <TAG> > /tmp/<TAG>_freeze.txt 2>&1 &
echo $! > /tmp/<TAG>_freeze.pid
```

Start it in the same breath as `fstage_run.sh` (it waits up to 90 s for the
run's pid and record). `--replay <log>` applies the same rule to a finished log.

## A5. Viewing

From your workstation:

The demo serves the dashboard on all addresses (`--http-host 0.0.0.0` in A4
and A8c, the user's choice for demo day): open `http://168.6.244.26:8080/` on
the lab network or `http://192.168.10.26:8080/` on the rig network. Anyone who
reaches those addresses can Start and Stop the radios. Without `--http-host`,
`--control` listens on 127.0.0.1 only, and an ssh tunnel is the way in:

```sh
ssh -L 8080:localhost:8080 houdini@168.6.244.26
```

then open `http://localhost:8080/`. Pick the config in the header list and
press Start. With `--configs labelled` the list offers only the four demo
configs, by their short `_label`: "Demo, cabled", "Demo, X-band chain",
"Fallback 48 MHz, cabled" and "Fallback 48 MHz, X-band chain" (hover for the
full description).

## A6. What good looks like

From the validation runs of the demo head (`DEMO_VERIFICATION.md` 9.69 on
fpga 1.33, 9.70 to 9.73 on fpga 1.34: `houdini-dualband-xw-steer-slots.json`,
steered, cabled):

| Where | Healthy | Note |
|---|---|---|
| MER (15 s samples) | sub-6 35.4-37.5 dB, X-IF 30.7-32.8 dB | a cold start (just after a node power cycle) swings for the first few minutes while the steering converges |
| Constellation low | 0.0 % in every 300 s window | from the run report's CNS lines |
| Beacon SNR at the UE | about 39-47 dB wired | the sync card's `beacon SNR`; over the air the detector floor is 25 dB |
| Pilot seat at the BS (`pilot_grid_off`) | within a few samples, steady within a run | it moves by a few samples between sessions (VL1 0/+1, RV1 -3/-4), untraced: the converters' MTS latency lands differently each session but does not predict the move (9.73); the slot margin is +-32 |
| BS frames per second | about 50 (slots config), about 57 (all-rx config) | from the HOUDINI_BS_RX lines |
| End-of-run lines | `RX read check`: 0 lost in rx slots, 0 out of order, 0 time jumps; `AP-87 slot check`: 0 outside the rx slots; `RX_HOST_STATUS`: tdd_straddle 0, tdd_refused 0, and on fpga 1.34 tdd_drop 0 | anything nonzero is a finding, not noise |

## A7. Known limits

- **Do not open a node from a second program while a run is live** (a manual
  `check_setup.py`, `SoapySDRUtil --probe`, a Python session). The sounder opens
  each node with its own packet size, so a second open gets a separate driver
  instance on the node, and its setup and its close reset state the run depends
  on (gains, and the X-band front end's static state). The dashboard's Check
  already refuses while a run is live. Run the setup check before Start.
- **UE transmit timing on the host (SH-427) is solved** with the isolated layout
  above: the pacer's worst wake over a 35 min run is about 0.3 ms (9.43-9.46).
  Read the installed host plugin's build id in the setup check before judging a
  run.
- **A UE TX playout freeze (HS-227)** is fixed from bitstream 1.32: no freeze
  in any demo-length run on 1.32 or later (9.57 to 9.71). On an older
  bitstream the UE's FPGA can stop playing its TX bank at a random time and
  judge every later packet late (9.44, 9.45); the BS then loses the UE's
  pilots and the dashboard's cards go stale. It does not recover within the
  run; a Stop and Start clears it (no node reboot needed). The freeze watcher
  (A4) reports it.
- **The rig host's management NIC (r8127, `enP7s7`)** carries ssh to the host.
  Its driver once hung in its ESD checker during a long run: new ssh sessions
  timed out until the sounder exited and host timing suffered (9.39). The
  driver is unchanged; check
  `journalctl -k | grep -E "rtl8127|blocked for more than"` after a long run.
  The control calls to the radios run on their own network (A1: a USB
  RTL8153B, `enx00e04c242668`, driver r8152, linked at 100 Mb/s). Never copy
  large files off the host during a run.
- **Radio opens that time out (SH-442).** A launch sometimes cannot open the BS
  within the device timeout (`SoapyRPCUnpacker::recv() TIMEOUT` in the log).
  The sounder retries the open itself ("Radios Not Found. Will attempt a
  retry..."); if every try fails, press Start again.
- **Clock steering** is in the demo build and ON through the config
  (`sync.steer.enable` in its `sync` block, the only place it is set).
  Without it MER depends on the day: the two boards' offset reached 0.6-0.9 ppm
  in four of seven long runs and MER fell from about 30 to 12-18 dB (sub-6) and
  9-12 dB (X-IF) (9.33, 9.34, 9.44, 9.45). The slots configs also remove each
  lane's residual offset before the FFT (`bs_cfo_pre_fft`) and widen the
  steering deadband to 0.12 ppm (one DAC count moves about 0.19 ppm, SH-462).
- **Host timing.** One run in several shows a single UE TX late-release event of
  a frame or two mid-run with no visible effect (9.71). After a rig-host reboot
  with the pinning unchecked, the demo run had 25 such events, 63 frames in
  71 min (9.82): run `pacer_core_check.py` after every boot (A8c step 5). The
  durable guard is keeping every NIC receive queue off the pacer cores (AP-106).
- **Egress drop counters saturate (HS-212).** Each radio's per-port egress
  drop counters, and its one marked-frame counter (a single count after the
  ports merge; device builds before 0.3.1 print it per port), stop at 255; a
  throughput test fills them (9.73). Judge a run by each counter's change
  over the run, not its value: a marked frame at a stream's teardown is the
  designed cleanup. A
  run is not affected, but its link health can no longer see a new egress
  drop, and check_setup says so with a WARN. No software path clears them:
  a node boot does (a reboot is enough, and so is a power cycle, because the
  boot service loads the bitstream again), as does a PL reload of the same
  bitstream (a deploy, so the user's call). With an XUD1A attached, a boot
  follows A2b step 1: its 12 V off through the boot, on only after the PL has
  loaded, then the role. A restart of the device services does not. Do not run
  a throughput test after the nodes' last boot before a demo, and after any
  boot confirm with check_setup that the counters read 0 and the fpga version
  is the expected one (a re-flashed card boots the bitstream baked into its
  image).
- **The pilot seat moves between sessions** by a few samples (9.73: 0/+1 in
  VL1, -3/-4 in RV1), untraced; the converters' MTS latency lands differently
  each session (SH-468). Inside the +-32-sample slot margin; MER unchanged.

## A8. Stopping and recovery

- Stop from the dashboard, or Ctrl+C on the backend (it stops the sounder's
  whole process group).
- A TX playout freeze (A7): Stop, then Start. The new run's stream setup clears it.
- The BS's sub-6 receive degraded from the very start of a run, with the log line
  `ADC0 SUBADC_DCDR STILL set after 4 attempts, proceeding`: the ADC's bring-up
  retry ran out (SH-227; `.22` needed its fourth and last attempt once). Stop,
  then Start: a fresh open reruns the tile bring-up (every retry so far came up
  healthy within four attempts; the recovery itself is inferred, not tested).
- After a rig-host reboot, run the setup check before anything else: a bounce of
  the host's data ports can wedge a node's FPGA egress (HS-225), which fails the
  check's `egress` line; recover by reloading that node's PL or rebooting it.
- A radio still held by an old sounder: the setup check names its pid; stop that
  run or `kill -INT <pid>`, the sounder's own stop (a plain `kill` skips its
  end-of-run checks and leaves the clock steered). Only if it has not exited
  after 10 s, `kill -9 <pid>`, then run the setup check again (it names a
  steered clock and how to release it). `tools/rig_release_holders.py` also works but stops EVERY
  sounder and dashboard on the host, including a running `--control` backend.

## A8b. The canned-data fallback

From the dashboard: the demo launch (A4, A8c) offers four replays in the config list, recorded through
the X-band chain on 0.3.1 and kept under `~/demo_rec/replays/`:

| In the list | File | What it shows |
|---|---|---|
| Replay, X-band bench | `xband_chain_bench_60s.rec` | 60 s through the chain with a 20 dB pad: sub-6 about 37 dB, X-band about 31 dB |
| Replay, X-band over the air | `xband_ota_60s.rec` | 60 s over the air (antennas about 1 m apart, aligned): sub-6 about 37 dB, X-band about 27 dB |
| Replay, over the air with blockage | `xband_ota_blockage_30s.rec` | 30 s over the air with the path blocked from +6 to +15 s: X-band 27, then about 10, then 28 dB |
| Replay, over the air, noisy sub-6 | `xband_ota_noisy_sub6_60s.rec` | 60 s over the air with a degraded sub-6 cable path: sub-6 about 16 dB, X-band about 27 dB |

Stop, pick one, Start: it plays in a loop with no radio, no setup check and no teardown
(a `--record` recording pauses while it plays); Stop ends it, and a demo config's Start goes live
again. Add more with `--replay <file>=<label>`. Without the dashboard's control, or on a laptop with
Python only, replay by hand as below.

A recording of the dashboard's input from a good run, played back into the
dashboard with no radios: every panel shows real rig data (channel,
constellation, CIR, ADC, beacon sync).

A recording starts when the dashboard starts, not at Start, and on freshly booted nodes the
data can begin about 30 s after the page says running (the first radio opens are retried):
time an event (a blockage, a hand on an antenna) from the first datagram, watching the file
grow, not from the Start.

1. Record during a good run: start the dashboard with `--record` added to
   its A4 line (a scripted run takes the same `--record` on `demo_run.sh`,
   `fstage_run.sh` or `run_rung.sh`):

   ```sh
   mkdir -p ~/demo_rec
   python3 csi_gui/csi_server.py <the A4 arguments> --record ~/demo_rec/<name>.rec
   ```

   The dashboard prints `recording every datagram to ...` at start. A name
   that already exists is refused, never overwritten (the dashboard prints
   `NOT recording: ... exists` and runs on without recording), so give each run
   its own name. It flushes
   about once a second and stops at 2 GB (`--record-max-mb`).
2. Replay, with no sounder running:

   ```sh
   python3 csi_gui/csi_server.py --conf files/houdini-dualband-xw.json &   # VL1_134.rec, FINAL_XW.rec; files/houdini-dualband.json for FINAL.rec
   python3 csi_gui/replay_feed.py ~/demo_rec/<name>.rec --loop
   ```

   The `--conf` sets the page's |H| axis for the recorded
   configuration (without it the dashboard falls back to a single-band config
   and the dual-band |H| reads off scale). `--start` and `--duration` pick a window (seconds into the recording), for
   example a stretch with steering settled. Each loop restarts the frame
   counters, which the beacon-sync card shows as a new segment.
3. The same replay on a laptop, independent of the rig host (the demo-day
   fallback to carry): it needs only Python 3, no SoapySDR and no radios. Copy
   the dashboard, the configs and a recording, then run step 2 there and browse
   `http://localhost:8080`:

   ```sh
   mkdir -p ~/houdini_replay && cd ~/houdini_replay
   scp -r <user>@<rig-host>:<checkout>/CC/Sounder/csi_gui .
   scp -r <user>@<rig-host>:<checkout>/CC/Sounder/files .
   scp <user>@<rig-host>:demo_rec/<name>.rec .
   python3 csi_gui/csi_server.py --conf files/houdini-dualband-xw.json &
   python3 csi_gui/replay_feed.py <name>.rec --loop
   ```

   On the rig host itself, never replay while a sounder run is live: the replay
   and the run's dashboard use the same ports (UDP 9999, HTTP 8080).

## A8d. Over the air: antennas and the room (before the first run)

The first sub-6 over-the-air runs lost about 20 dB to antenna placement and a
quarter of the beacons to nearby emitters (`DEMO_VERIFICATION.md` 9.62 to
9.65); both are fixed at the bench, not in software.

1. Stand every stick vertical (same polarisation), each node's TX stick
   broadside to the other node's RX stick (sides facing, never end to end),
   in line of sight, off the bench surface and away from metal. A stick
   rotated about its own axis changes nothing; a few centimetres of position
   can move a reflection null by 10 dB, so try small moves.
2. Clear the UE's antennas of 2.4 GHz emitters: phones, smartwatches, BLE tags,
   wireless keyboard and mouse dongles, and Wi-Fi access points close by. The
   bench survey found a bursty emitter at 2426 MHz (BLE advertising channel
   38, on the sub-6 centre) near the UE, and 27% of the UE's beacon detections
   fell below 10 dB until it was dealt with.
3. Aim against the live dashboard: the beacon-sync card's `beacon SNR` (the
   demo's detector floor is 25 dB; wired reads about 42), the Sub-6 card's MER
   over time and |H|, and the X-band card's MER holding steady (a dipping
   X-band on a cabled IF means the UE's beacon is weak or hit).
4. Hold still for a minute and read the numbers again before starting the
   demo run.

## A8c. Demo day: bring-up at the venue (in order)

**The demo build:** develop in
`~/repos/HoudiniLab-rxwin` (the X-band at 270 RB beside the sub-6 at 133 RB,
steered, the BS receiving only its rx slots): cabled, config
`files/houdini-dualband-xw-steer-slots.json`; through the XUD1A (A2b),
`files/houdini-dualband-xw-steer-slots-fe.json`. Both run on the release's host
plugin (`--soapy-root`, A3). Canned fallback: `~/demo_rec/VL1_134.rec` (fpga
1.34, the demo head). **The fallbacks:** the same build and plugin at 48 MHz
X-band, `files/houdini-dualband-steer.json`, recording `~/demo_rec/FINAL.rec`;
and the canned recordings replayed with no radio (A8).

Moving the rig means rebooting the rig host, which re-draws the NIC's receive
hashing (a node's RX flow can land on a pinned core) and can wedge a node's data
egress (HS-225). Do every step, every power-up.

On the demo rig, steps 1 to 4 are one command after the power-up: `~/start_demo.sh`
(the software lane's demo-day card, `docs/DEMO_DAY_CARD.md` section 4 in SoapyHoudiniSDR).
It refuses while a demo is up, runs `~/demo_check.sh` (a go/no-go over the rig host and both
nodes: isolation, the 100G ports, roles, the XUD1A LO and its Table 7 filter, the power
boards, the builds, the FPGA and the egress, each radio opened with retries), then `check_setup.py --quick`
(its radios just proven by the go/no-go), then the dashboard with
the line of step 4 on the X-band chain's config, in the foreground of its terminal: Stop on
the page, then Ctrl-C there. The steps below are what it does, by hand. It hands the
host-plugin prefix and the core placement over as environment variables, which the setup
check and the dashboard do not read: until the software lane switches it to the
`--soapy-root` and `--sounder-arg` arguments of steps 3 and 4 (SH-486 item 10), bring the
demo up with the steps below.

1. On the rig host: `cat /sys/devices/system/cpu/isolated` reads `15-19`.
2. For the X-band RF chain only: the roles and the LO check (A2b steps 1 and 2:
   `houdini-role status` exits 0 on both nodes, and `.22`'s `houdini-xud1a pll
   status` reads `doubler tracking table7`).
3. The setup check, on the release's host plugin (A3):

   ```sh
   source ~/houdini_test/bin/activate
   cd ~/repos/HoudiniLab-rxwin/CC/Sounder
   python3 csi_gui/check_setup.py --soapy-root $HOME/houdini_0.3.1 --conf files/houdini-dualband-xw-steer-slots.json
   ```

   Ready, egress PASS on both nodes, the stacks match (the stack line's
   `host_build` equals `device_build`). A stack FAIL reading
   `SoapyRPCUnpacker::recv() TIMEOUT` is a slow first open after a server
   restart: run it again (twice in a row has happened, 9.70). An egress FAIL
   needs that node's PL reload or reboot.
4. The dashboard, from the same shell, with the A4 arguments:

   ```sh
   # optional: add --record ~/demo_rec/<name>.rec to record a fallback; a new name each start
   python3 csi_gui/csi_server.py --control --http-host 0.0.0.0 --configs labelled \
     --soapy-root $HOME/houdini_0.3.1 \
     --sounder-arg=--core_map=main=15 --sounder-arg=--tx_cpu_affinity=18,19 --sounder-arg=--tx_host_status \
     --replay "$HOME/demo_rec/replays/xband_chain_bench_60s.rec=Replay, X-band bench" \
     --replay "$HOME/demo_rec/replays/xband_ota_60s.rec=Replay, X-band over the air" \
     --replay "$HOME/demo_rec/replays/xband_ota_blockage_30s.rec=Replay, over the air with blockage" \
     --replay "$HOME/demo_rec/replays/xband_ota_noisy_sub6_60s.rec=Replay, over the air, noisy sub-6" \
     --log-dir ~/demo_logs --conf files/houdini-dualband-xw-steer-slots.json
   ```

   `--log-dir` keeps each Start's sounder output in
   `~/demo_logs/sounder_<UTC>.log` (the dashboard prints the name), with the
   teardown before it: the A6 end-of-run lines (`RX read check`,
   `AP-87 slot check`, `RX_HOST_STATUS`) land there at Stop.

5. Within the first minute of a Start:
   - the log's sync configuration block lists `steer.enable = true` marked
     `[json]`, and a `Clock steering [0]: ON` line follows;
   - the BS logs `receives only its rx slots (AP-87): TDD_RX_SLOTS active=1`;
   - with a `-fe` config, the `TDD_EXTPIN_SRC` lines of A2b step 3;
   - `python3 tests/demo-verify/pacer_core_check.py --cores 15,18,19` prints
     `ok`. If it names a core, Stop, pick two isolated cores it did not name
     (`--cores` again to confirm), change the dashboard's
     `--sounder-arg=--tx_cpu_affinity=`, restart the dashboard, Start.
6. During the demo: a stalled stream, or the BS sub-6 degraded from the start
   with the DCDR line (A8): Stop, then Start. Each card's |H| axis moves in
   10 dB steps when its trace leaves the axis (at most every 3 s); a card that
   keeps its old values while its age grows is stale (the badge shows).
7. Anything that cannot be fixed in a minute: the canned-data fallback (A8b).
8. After the demo: Stop on the page, Ctrl-C in `start_demo.sh`'s terminal, then
   `~/power_down.sh` (the card's section 6). It refuses while a sounder or dashboard is
   up, turns the XUD1A's LO off and the X-band power boards down, powers the nodes
   off, and pauses for each switch you flip (the XUD1A, the nodes, the boards' 12 V);
   last it offers the rig host's own poweroff.

## A9. The CPU isolation experiment (checklist)

Goal: put the timing-critical threads on isolated performance cores and find
which setting buys the most. Cores 0-4 and 10-14 of this host are efficiency
cores (Cortex-A725, down to 338 MHz); 5-9 and 15-19 are performance cores
(Cortex-X925). The plan isolates 15-19: the sounder's dispatch thread on 15,
the UE's two TX pacer workers on 16 and 17, and 18 and 19 kept free for moving
threads around. Steps marked (sudo) are the owner's.

Status: stage 1 is in force on this host (9.43) and the isolated layout runs
the demo (A4). The pacers have since moved to 18 and 19 (A4), so where this
checklist says 16,17, use the cores `pacer_core_check.py` confirms. No stage 2
arm has a record row yet; the steps stay here for when one is run.

**Stage 1: the kernel command line (sudo, then one reboot).**

1. Find where the command line is set (a file in `/etc/default/grub.d/` wins
   over `/etc/default/grub`):
   `grep -rn "GRUB_CMDLINE_LINUX" /etc/default/grub /etc/default/grub.d/ 2>/dev/null; cat /proc/cmdline`
2. Append to `GRUB_CMDLINE_LINUX_DEFAULT` in that file, keeping what is there:
   ```
   isolcpus=domain,managed_irq,15-19 irqaffinity=0-14 rcu_nocbs=15-19 rcu_nocb_poll nowatchdog skew_tick=1
   ```
3. `sudo update-grub`, then `grep -c "isolcpus=domain,managed_irq,15-19" /boot/grub/grub.cfg` (1 or more).
4. Reboot only when no comparison run still needs the old kernel.
5. Verify after the reboot: `cat /sys/devices/system/cpu/isolated` prints
   `15-19`; `cat /proc/cmdline` shows the arguments. Then run `check_setup.py`
   (A4) before anything else: a host reboot bounces the data ports, and a
   bounce can wedge a node's FPGA egress even with no run active (HS-225; the
   node's ARP replies and control ACKs share the egress path). A wedged node
   fails the `egress` line, and its run would get no samples at all. Recover by
   reloading that node's PL or rebooting it; bouncing the host's data port once
   more (`ip link set <port> down`, then `up`) cleared it once, not proven.
6. Before the first run: `grep CONFIG_NO_HZ_FULL /boot/config-$(uname -r)`
   (stage 2 needs it), and `sudo apt install rt-tests` for cyclictest.
7. Undo: remove the arguments, `sudo update-grub`, reboot.

**The measurement, the same for every arm.**

- Instrument check first: `sudo cyclictest -q -m --laptop -a 16,17 -t 2 --policy=other -i 200 -D 60`,
  the wake-up latency on the TX cores at the TX workers' own priority, before
  any sounder run; the same on 10,11 for comparison. Each thread's line must
  read `P: 0`: a `-p` in any form switches cyclictest to SCHED_FIFO (it then
  says "defaulting realtime priority"). `--laptop` stops it holding
  `/dev/cpu_dma_latency` at 0, which keeps the cores out of deep idle and would
  measure arm 2 instead of the baseline. Repeat it after each runtime setting
  that should change it (for arm 5, `-p 40` in place of `--policy=other`).
- One run per arm, all the same length (600 s is enough to show the late rate),
  R3 `files/houdini-dualband.json`, with the sounder's threads placed by these
  flags (on `run_rung.sh` or the dashboard):
  ```sh
  --sounder-arg=--core_map=main=15 --sounder-arg=--tx_cpu_affinity=16,17
  ```
  plus `tests/demo-verify/irq_sampler.py <out> 10 <secs> 15,16,17` and
  `mer_sampler.py` in the background (pids recorded; nothing copied off the host
  during a run). Compare the UE late / underflow totals, the late-release lines,
  and the pacer's wake-jitter close lines (`demo_report.py`).

**Stage 2: the runtime settings, one arm at a time (sudo; no reboot; each undoable).**

- Arm 0, the baseline: stage 1 only.
- Arm 1, frequency: `for c in 15 16 17 18 19; do echo performance | sudo tee /sys/devices/system/cpu/cpu$c/cpufreq/scaling_governor; done`
  (check `scaling_available_governors` first; if `performance` is missing, set
  `scaling_min_freq` to `cpuinfo_max_freq` on those cores). Undo: the old governor.
- Arm 2, idle states: `ls /sys/devices/system/cpu/cpu16/cpuidle/`, then
  `echo 1 | sudo tee /sys/devices/system/cpu/cpu1[5-9]/cpuidle/state[1-9]/disable`
  (state0, the shallow wait, stays enabled). Undo: `echo 0` to the same files.
- Arm 3, kernel housekeeping: `echo 7fff | sudo tee /sys/devices/virtual/workqueue/cpumask`
  (unbound kernel workers on 0-14), and
  `cat /sys/kernel/mm/transparent_hugepage/enabled` (set `madvise` if it reads
  `always`). Undo: the old values.
- Arm 4, NIC queues: keep the data NICs' receive flows and the pinned threads'
  transmit completions off 15-19, either by `sudo ethtool -X <data-iface> equal 15`
  plus `xps_cpus` maps that send CPUs 15-19 to queues 0-14, or by
  `sudo ethtool -L <data-iface> combined 15`. Change them ONLY with no stream open.
  The host plugin assumes no queue numbers except its zero-copy RX path (AF_XDP,
  `rx_xsk=auto`): check a run's log for `xsk` lines first. If it engaged, it binds
  the highest free queue at setup and steers its flow there with an ntuple rule;
  after `combined 15` that is queue 14, on housekeeping cpu 14. `rx_xsk=off`
  rules it out. Undo: the old channel count and `ethtool -X <data-iface> default`.
- Arm 5, real-time priority: `sudo sysctl kernel.sched_rt_runtime_us=-1` (without
  it a spinning FIFO thread is forced off its core about 50 ms a second), grant
  CAP_SYS_NICE to `build/sounder` (`sudo setcap cap_sys_nice+ep build/sounder`, lost
  on every rebuild), and the sounder's `--tx_stream_args=rt_priority=40` (SCHED_FIFO;
  40 stays under the kernel's threaded-IRQ and RCU priorities, which default to 50),
  always with the workers pinned. Without the capability the plugin warns and runs at
  normal priority: read the log for that warning.
- The plugin's receive workers take the same `cpu_affinity=<cpu>` argument per RX
  stream (the sounder has no RX pass-through knob yet; add one if an arm needs them
  on 18 and 19).
- Stage 2b, if the arms leave oversleeps: add `nohz_full=15-19` to the command line
  (one more reboot) and compare against the best arm: it removes the tick from a
  core running one thread, but makes every syscall dearer, and the TX workers make
  many.

Record each arm as a `DEMO_VERIFICATION.md` section 9 row with its settings.

# Part B: the single-band CSI demo on rig B

A record of that bench, not a procedure to follow. The boards have since moved
to the Part A roles and addresses, and two things below no longer hold on the
current code: the sync knobs are keys of the config's `sync` block now, with
no environment override (walkthrough section 7.1), and the dashboard's
datagram counter prints every five seconds. The debug switches B3 exported are
sounder flags now; B3 shows them in that form.

## B1. The machines

| Machine | Address | Role | What runs on it |
|---|---|---|---|
| Build VM | (this repo's checkout at `/space/vmshare/repos/HoudiniLab`) | Development only | Editing, compile checks, offline analysis. NOTHING of the demo executes here. Code ships to the rig as a git bundle over ssh, never as a copied tree. Note the vboxsf trap: `touch` changed sources before a local build or cmake may relink nothing. |
| Rig host ("rig B") | `168.6.244.64`, user `houdini` | Runs the entire host side | The sounder binary AND the dashboard backend (details in section B2). |
| Base-station board | `168.6.244.21`, user `houdini`, serial `575524` | BS radio | `SoapySDRServer --bind` (systemd unit) serving the device plugin `libHoudiniSDRDevice.so`; FPGA bitstream v1.30 `c88e0b5f`; device software 0.2.2. |
| Client board | `168.6.244.22`, user `houdini`, serial `6596d2` | UE radio | Identical stack to the BS board. |
| Your workstation | anywhere | Viewer | A browser, through an ssh tunnel to the rig (section B4). |

Both boards share one 10 MHz reference (frequency lock; there is no
cross-board phase lock, so the corrected-phase panel re-anchors per run).

## B2. What runs on the rig host

One sounder process drives BOTH radios. There is no per-board host process:
`sounder` opens the BS radio (remote to `.21`) and the UE radio (remote to
`.22`) from the same process over the SoapyRemote control plane, and runs its
own UDP data planes to each board.

- Checkout: `~/repos/HoudiniLab`. The demo ran from this, the main checkout,
  on `feat/csi-gui-tabler`. The `~/repos/HoudiniLab-rx`
  worktree also sits at the same commit and still works, but it is now detached
  (a branch cannot be checked out in two worktrees) so it does NOT advance on a
  pull. Whichever you use, `--sounder-dir` (section B3) is what selects the
  binary, and a wrong value fails silently by running the other tree's build.
- Binary: `~/repos/HoudiniLab/CC/Sounder/build/sounder`. **Always wipe the build
  directory** rather than building incrementally: an incremental build over a
  cache configured on another branch linked stale objects into a binary TWICE
  the correct size, and that binary still passed a runtime-string check. So the
  string check alone does NOT certify a build:

  ```sh
  rm -rf build                                   # not optional
  /usr/bin/cmake -B build -DCMAKE_BUILD_TYPE=Release
  /usr/bin/cmake --build build --target sounder -j
  ls -l build/sounder                            # STEP 1: size, the check that
                                                 # actually caught the failure
  strings build/sounder | grep <a-string-only-the-new-code-logs>   # STEP 2
  ```

  Step 1 is the one that catches stale objects; step 2 catches a stale binary
  that was never rebuilt. Both are required, in that order. A clean binary is
  around 0.9 MB on this rig, so a figure near 1.8 MB means stale objects got
  linked -- treat the size as an order-of-magnitude sanity check rather than an
  exact constant, since it moves with every code change.
- Dashboard backend: `~/repos/HoudiniLab/CC/Sounder/csi_gui/csi_server.py`
  (HTTP on 8080, CSI datagrams in on UDP 9999). Both bind `0.0.0.0`, not
  localhost (`--http-host` / `--udp-host` defaults), so the dashboard is also
  reachable directly at `http://168.6.244.64:8080/` if the lab firewall allows
  it. Prefer the section B4 tunnel anyway: it works regardless of firewall and
  does not publish the panel on the lab network.
- SoapySDR host stack: the validated houdini HOST plugin lives ONLY at
  `/home/houdini/houdini_test/lib/SoapySDR/modules0.8-3/`. The system
  SoapySDR at `/usr/local` does NOT have it, so every launch must carry
  `SOAPY_SDR_PLUGIN_PATH=/home/houdini/houdini_test/lib/SoapySDR/modules0.8-3`
  (the backend's `--venv` default `~/houdini_test` sets this when launching
  through it). Without it every radio open fails with
  `SoapySDR::Device::make() no match`.
- Teardown helper: `csi_gui/teardown_framer.py` runs on the rig (it opens the
  boards, so it is device-touching).
- Logs land under `~/repos/HoudiniLab/CC/Sounder/logs/`.

## B3. The exact launch used for the live demo

On the rig, in one shell:

```sh
cd ~/repos/HoudiniLab/CC/Sounder
python3 csi_gui/csi_server.py --launch --conf files/houdini-ul.json \
    --sounder-dir ~/repos/HoudiniLab/CC/Sounder \
    --mag-top 85 --mag-span 5 \
    --sounder-arg=--bs_rx_debug --sounder-arg=--ue_tx_debug --sounder-arg=--csi_r_debug \
    --sounder-arg=--cns_dump_low=logs/cnslow
```

The backend sets the plugin path from its `--venv` default, runs the framer
teardown, then starts `sounder --view` and retries the flaky cold start. The
debug flags are optional but cheap, and they are what every verification in
`DEMO_VERIFICATION.md` greps for.

Without those three debug flags the run is nearly silent: the teardown, the startup
banner, and a `[csi]` datagram counter about once a second. That is the walkthrough's
mode A default and it is normal, not a fault. Set `sync.cfo.log_every` to 1 in the
config's `sync` block when you want every beacon CFO estimate rather than the default
one in ten.

Two back to back runs on this bench measured 6,189 lines in 60 s
with the three debug switches on, 342 lines in 85 s without them, and the `[csi]`
datagram counter advancing by an identical 443 per reporting interval in both.
The debug switches change the printing only. Judge run health by the datagram counter
and by the `Re-sync ... beacon alive` lines, never by how much scrolls past.

## B4. Viewing

From your workstation:

```sh
ssh -L 8080:localhost:8080 houdini@168.6.244.64
```

then open `http://localhost:8080/`. After any backend restart, refresh the
page once: the magnitude axis and page structure are baked in at page load.

## B5. Stopping, and the two recoveries that actually happen

Stop by process group, from a `ps` listing, never by name pattern (wrapper
shells carry the same names and a pattern kill leaves orphans holding the
ports):

```sh
ps -eo pid,pgid,cmd | grep -E "[c]si_server|[s]ounder --view"
kill -TERM -- -<pgid-of-csi_server> -<pgid-of-the-sounder-wrapper>
```

- Radio open fails with `SoapyRPCUnpacker::recv() TIMEOUT`: the board server
  wedged. `ssh houdini@168.6.244.21 'sudo systemctl restart SoapySDRServer'`
  (same for `.22`), wait a few seconds, relaunch. Tracked as AP-20.
- Radio open fails with `make() no match`: the plugin path is missing from
  the environment (see section B2).

A bare `SoapySDRUtil --find` proves nothing about board health (the device
factory answers only when the filter carries `show=1`). Probe a board with:

```sh
SoapySDRUtil --find="remote=tcp://168.6.244.21:55132,show=1"
```

## B6. Stack identity this runbook was written against

fpga 1.30 `c88e0b5f`, device 0.2.2 `71bcbc6b`, host 0.2.2
`d2861dc1`, protocol 1.0, SoapySDR 0.8.1, SoapyRemote 0.6.0. Both boards
report `clock_ref: external`. Config: `files/houdini-ul.json`
(30 slots x 4096 samples = exactly 1 ms per frame, beacon slot 0, pilot slot
16, uplink data slot 18, `tx_advance` 247).

The host plugin was rebuilt from `d2861dc1`, replacing the
`c20d7975` build that the DEMO_VERIFICATION.md rows were taken against. The
two are identical as compiled code: every commit between them touches only
tracker files, `host/tests/bench/README.md`, and `host/tests/hil/test_tdd.py`,
which is interpreted rather than linked. So the earlier evidence still stands;
only the stamped build id moved. Read the id back from the installed module,
never from the build log, because it is stamped at cmake CONFIGURE time and a
plain rebuild keeps a stale stamp:

```sh
strings $VIRTUAL_ENV/lib/SoapySDR/modules0.8-3/libHoudiniSDRSupport.so \
    | grep -E '^[0-9a-f]{8}$'
```
