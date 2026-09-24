# Lab bench runbook: exactly what runs where

This file is the bench-specific companion to `CSI_DEMO_WALKTHROUGH.md`. The
walkthrough stays neutral (placeholders and discovery commands) so it travels
to any bench; this runbook fills in every placeholder for OUR lab setup so a
run can be reproduced or debugged without archaeology. If the bench changes,
change this file in the same commit.

Evidence for everything below lives in `DEMO_VERIFICATION.md`.

Two benches are described. **Part A** is the dual-band demo (AP-79: sub-6 plus
the X-band IF, HS-202 mode V) on the rig host `.26`, the current one. **Part B**
is the earlier single-band CSI demo on rig B (`.64`), kept as it was.

# Part A: the dual-band demo (AP-79)

## A1. The machines

| Machine | Address | Role | What runs on it |
|---|---|---|---|
| Build VM | this repo at `/space/vmshare/repos/HoudiniLab` | Development only | Editing, compile checks, offline analysis. Code ships to the rig as a git bundle over ssh. |
| Rig host "burton" | `168.6.244.26`, user `houdini` | The whole host side | The sounder and the dashboard backend. A DGX Spark: 20 aarch64 cores, fast ones 5-9 and 15-19. Data NICs `enp1s0f0np0` 192.168.5.11 (UE side) and `enp1s0f1np1` 192.168.6.13 (BS side), MTU 9000. |
| Base station | `168.6.244.22`, user `houdini` | BS radio | `SoapySDRServer` (systemd) with the device plugin. |
| Client | `168.6.244.21`, user `houdini` | UE radio | The same stack as the BS. |
| Your workstation | anywhere | Viewer | A browser through an ssh tunnel (section A5). |

The roles are the reverse of Part B's: here `.22` is the BS and `.21` the UE
(`files/topology-houdini-dualband.json`). Both boards run their reference in
**calibrated hold** (`clock_ref = calibrated`; `.21` DAC code 408, `.22` 404),
with no shared 10 MHz: the UE-BS offset stays under about half a ppm, but it
moves between sessions and ramps as the boards warm (`DEMO_VERIFICATION.md`
8.125, 8.137, 9.28). Both boards carry the SYZYGY DNA adapter on
POD2 and no X-band power board.

## A2. The wired RF chain (the final wired state, "F3b")

Every link is a ONE-WAY cable. Channel letters in the configs: A = ch0,
B = ch1, C = ch2.

| Link | From | Filter(s) | To |
|---|---|---|---|
| Sub-6 downlink (the beacon) | `.22` TX ch0 (DAC_B) | VBF-2450+ at the DAC | `.21` RX ch0 (ADC_D) |
| Sub-6 uplink | `.21` TX ch0 (DAC_B) | VBF-2450+ at the DAC | `.22` RX ch0 (ADC_D) |
| X-band IF uplink | `.21` TX ch1 (DAC_A) | VBFZ-4000-S+ at the DAC, and a second VBFZ-4000-S+ at the ADC | `.22` RX ch2 (ADC_B) |
| HIL self-loop (unused by the demo) | `.22` TX ch1 (DAC_A) | none | `.22` RX ch1 (ADC_C) |

There is no X-band IF downlink and no attenuator in the chain (F4a and F4b
tried a 10 dB pad at each sub-6 receive end; it came out again). What each filter did to the levels, and why
the in-channel "tilt" is cable ripple rather than the filters, is
`DEMO_VERIFICATION.md` 9.15 to 9.24.

## A3. What runs on the rig host

- **Checkouts.** `~/repos/HoudiniLab` belongs to its owner: do not change it.
  The demo runs from its own worktree of the same repository, for example
  `~/repos/HoudiniLab-ap80` (the build with the AP-80 fix). Other worktrees on the
  host hold measurement builds (`-ap79` at `ca37797` for the SH-427 A/B, `-diag` for
  the pacer diagnostics); `--sounder-dir` and the checkout the dashboard lives in
  decide which binary runs.
- **Shipping a build.** Bundle, copy, and fetch INSIDE the target worktree
  (`FETCH_HEAD` is per worktree), then relink muFFT, which a checkout restores as
  an empty directory:

  ```sh
  cd ~/repos/HoudiniLab-ap80 && git fetch /tmp/<bundle> <branch> && git reset --hard FETCH_HEAD
  cd CC/Sounder && rmdir mufft && ln -s ~/repos/HoudiniLab/CC/Sounder/mufft mufft
  source ~/houdini_test/bin/activate
  cmake -B build -DCMAKE_BUILD_TYPE=Release -DSoapySDR_DIR=$VIRTUAL_ENV/share/cmake/SoapySDR
  cmake --build build -j10 && (cd build && ctest)
  strings build/sounder | grep <a-string-only-the-new-code-logs>
  ```

  `ln -sfn` onto the directory nests the link inside it; remove the directory
  first. Without `SoapySDR_DIR` the configure fails ("SoapySDR development files
  not found").
- **Host plugin.** The validated host plugin lives only in the venv
  `~/houdini_test` (`lib/SoapySDR/modules0.8-3/`). Activate it before anything.
- **Cores.** The sounder pins its own threads to cores 0-4, which are the slow
  ones on this host; the plugin's BS receive workers inherit core 0 (AP-81).

## A4. The launch

On the rig:

```sh
source ~/houdini_test/bin/activate
cd ~/repos/HoudiniLab-ap80/CC/Sounder
export HOUDINI_TX_CPU_AFFINITY=10,11   # the UE's TX pacer workers, off the data NIC's IRQ cores
python3 csi_gui/check_setup.py --conf files/houdini-dualband.json   # must print Ready.
python3 csi_gui/csi_server.py --control --conf files/houdini-dualband.json
```

**Why the pinning.** Left to the kernel, the host plugin's two UE TX pacer
workers land on the cores that take the 100G data NIC's interrupts (about 61k
completion IRQs a second on one core, 1-3k on several others), where receive
softirq work preempts them for milliseconds and whole bursts go out late
(`DEMO_VERIFICATION.md` 9.36). Cores 10 and 11 took no NIC interrupts on this
host; pinned there, the late bursts all but stop (9.37-9.39). The dashboard
passes the variable to the sounder it launches. Before trusting the choice
on another day, check the cores are still quiet:
`grep mlx5 /proc/interrupts` (the per-CPU columns for 10 and 11 should not
move between two reads a few seconds apart).

The check's full form reads both radios' stacks and FAILs if they differ.
Read the stack there, not from this file: it changes with every deploy. The
last validated stack is in the newest `DEMO_VERIFICATION.md` section 9 row.

The configs, from the ladder (walkthrough section 3): `houdini-r0.json`
(control), `houdini-dualband-r1.json`, `-r2.json`, `-r3a.json`,
`houdini-dualband.json` (R3, the demo) and `houdini-dualband-40.json` (the
40 MHz fallback). On a freshly deployed stack, climb R1 to R3 before running the
demo. R0 (NCO 500 MHz) cannot run while the F3b chain is fitted: the VBF-2450+
bandpasses on the sub-6 DAC paths block its beacon, so the UE never acquires
(`DEMO_VERIFICATION.md` 9.40). It is the control only with the filters out.

For evidence runs without the dashboard, `tests/demo-verify/run_rung.sh` and
`fstage_run.sh` launch one sounder run with the logs and dumps that
`rung_report.py`, `fstage_report.py` and `ab_report.py` read.

## A5. Viewing

From your workstation:

```sh
ssh -L 8080:localhost:8080 houdini@168.6.244.26
```

then open `http://localhost:8080/`. With `--control` the backend listens on
127.0.0.1 only, so the tunnel is the way in. Pick the config in the header list
and press Start.

## A6. What good looks like at R3

From the validation runs (`DEMO_VERIFICATION.md` 9.14 to 9.26), before clock
steering:

| Where | Healthy | Note |
|---|---|---|
| Acquisition | coherence 0.95 to 0.99 | one low value is a beacon cut by a read boundary |
| Beacon re-syncs | about one per 2.6 s, continuous | a long gap is AP-80 (fixed in the AP-80 build) |
| Printed beacon SNR | 40 to 50 dB | swings about 7 dB between bring-ups with the ADC's own Fs/2 spur; it is NOT a link indicator |
| Pilot seat at the BS | within about +-15 samples | a steady walk means the UE stopped re-syncing |
| Constellation low count | 0 of about 6,600 in a clean 180 s run | read it from the periodic SUMMARY line |
| MER (single frame) | sub-6 about 29-39 dB, X-IF about 24-34 dB | limited by each run's carrier offset until steering |

## A7. Known limits today

- **UE transmit plugin (SH-427, the software lane's).** The demo plugin is the
  software lane's B build with an on-core spin and a 200 us sleep cap (host build
  `e2a004d3` in the demo-length run D7, `DEMO_VERIFICATION.md` 9.39), run with the
  pinning above. Earlier plugins send late bursts at R3 over a long run. Read the
  installed host plugin's build id in the setup check before judging a run.
- **The rig host's management NIC (r8127, `enP7s7`).** During D7 its driver hung
  in its ESD checker: new ssh sessions timed out until the sounder exited, the
  control link to the nodes stalled, and host timing suffered (9.39). This NIC
  carries ssh and every control call to the radios. Until the driver is fixed,
  expect that ssh may be unreachable during a long run; check
  `journalctl -k | grep -E "rtl8127|blocked for more than"` after one.
- **Radio opens that time out (SH-442).** A launch sometimes cannot open the BS
  within the device timeout (`SoapyRPCUnpacker::recv() TIMEOUT` in the log).
  The sounder retries the open itself ("Radios Not Found. Will attempt a
  retry..."); if every try fails, press Start again.
- **Clock steering** is off by default and lives on its own branch
  (`feat/clock-steer-rollin`), reviewed, not yet validated on the rig. Without it
  R3's MER depends on the day: the two boards' offset drifted to 0.6-0.9 ppm in
  two of three long runs and MER fell from about 30 to 14-18 dB (sub-6) and 9-12
  dB (X-IF) (9.33, 9.34); in the third it stayed near 0.1 ppm and MER held
  (9.39).

## A8. Stopping and recovery

- Stop from the dashboard, or Ctrl+C on the backend (it stops the sounder's
  whole process group).
- A radio still held by an old sounder: the setup check names its pid; stop that
  run or `kill <pid>`. `tools/rig_release_holders.py` also works but stops EVERY
  sounder and dashboard on the host, including a running `--control` backend.

## A9. The CPU isolation experiment (checklist)

Goal: put the timing-critical threads on isolated performance cores and find
which setting buys the most. Cores 0-4 and 10-14 of this host are efficiency
cores (Cortex-A725, down to 338 MHz); 5-9 and 15-19 are performance cores
(Cortex-X925). The plan isolates 15-19: the sounder's dispatch thread on 15,
the UE's two TX pacer workers on 16 and 17, and 18 and 19 kept free for moving
threads around. Steps marked (sudo) are the owner's.

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
  R3 `files/houdini-dualband.json`, with the sounder's threads placed by:
  ```sh
  export HOUDINI_CORE_MAP=main=15 HOUDINI_TX_CPU_AFFINITY=16,17
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
  on every rebuild), and `export HOUDINI_TX_STREAM_ARGS=rt_priority=40` (SCHED_FIFO;
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

- Checkout: `~/repos/HoudiniLab`. Since 2026-08-31 the demo runs from this,
  the main checkout, which is on `feat/csi-gui-tabler`. The `~/repos/HoudiniLab-rx`
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
export HOUDINI_BS_RX_DEBUG=1 HOUDINI_UE_TX_DEBUG=1 HOUDINI_CSI_R_DEBUG=1
export HOUDINI_CNS_DUMP_LOW=logs/cnslow
python3 csi_gui/csi_server.py --launch --conf files/houdini-ul.json \
    --sounder-dir ~/repos/HoudiniLab/CC/Sounder \
    --mag-top 85 --mag-span 5
```

The backend sets the plugin path from its `--venv` default, runs the framer
teardown, then starts `sounder --view` and retries the flaky cold start. The
debug exports are optional but cheap, and they are what every verification in
`DEMO_VERIFICATION.md` greps for.

Without those three exports the run is nearly silent: the teardown, the startup
banner, and a `[csi]` datagram counter about once a second. That is the walkthrough's
mode A default and it is normal, not a fault. Add `HOUDINI_CFO_LOG_EVERY=1`
when you want every beacon CFO estimate rather than the default one in ten.

Measured 2026-09-01, two back to back runs on this bench: 6,189 lines in 60 s
with the three exports set, 342 lines in 85 s without them, and the `[csi]`
datagram counter advancing by an identical 443 per reporting interval in both.
The exports change the printing only. Judge run health by the datagram counter
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

fpga 1.30 `c88e0b5f` (2026-08-28), device 0.2.2 `71bcbc6b`, host 0.2.2
`d2861dc1`, protocol 1.0, SoapySDR 0.8.1, SoapyRemote 0.6.0. Both boards
report `clock_ref: external`. Config: `files/houdini-ul.json`
(30 slots x 4096 samples = exactly 1 ms per frame, beacon slot 0, pilot slot
16, uplink data slot 18, `tx_advance` 247).

The host plugin was rebuilt from `d2861dc1` on 2026-08-31, replacing the
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
