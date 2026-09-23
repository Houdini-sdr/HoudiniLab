# AP-79 first dual-band run: launch, what to watch, pass criteria

The first run of the mode-V dual-band sounder on silicon, run jointly with the
software lane (they watch both nodes' journals and CPU with `node_watch.sh`).
Branch `feat/sub6-xband-demo`. Roles and wiring follow the HS-202 plan
section 6: `.22` is the BS, `.21` is the UE, and the rig host `.26` runs the
sounder, which drives both nodes over their topology file. Nothing here runs
until the software lane hands over the rig.

## 0. Before the run

1. Confirm the stack on both nodes, and write it down: `fpga_commit` (the
   identity; `fpga_version` is cosmetic), `device_build` (the SH-421/SH-422
   build, `a1c8ecb7` when this was written), `host_build`. The sounder prints
   them at bring-up and warns `VERSION SKEW:` if the nodes differ; each
   node's mode-V session record (section 3) keeps them.
2. Confirm the rig is quiet:
   `ps aux | grep -iE "soapy|pytest|sounder|dualband" | grep -v grep`.
3. Confirm the cabling is the plan's (the fpga lane's tone walk, unchanged
   since W2/W3). Every link is a ONE-WAY cable:
   - `.22` TX ch0 (DAC_B) to `.21` RX ch0 (ADC_D): sub-6 downlink.
   - `.21` TX ch0 (DAC_B) to `.22` RX ch0 (ADC_D): sub-6 uplink.
   - `.21` TX ch1 (DAC_A) to `.22` RX ch2 (ADC_B): X-IF, UE to BS ONLY. There
     is no X-IF downlink cable.
   - `.22` TX ch1 (DAC_A) loops into `.22`'s own RX ch1 (ADC_C), the HIL
     self-loop; the demo neither transmits on it nor streams it. `.21` has no
     self-loop.
   Every other combination read under 12 dB in the tone walk: treat crosstalk
   as absent, not zero.

## 1. Ship and build on the rig host

No git credentials exist anywhere, so the branch travels as a bundle, and it
builds in OUR linked worktree, never in the user's `~/repos/HoudiniLab`
checkout.

```sh
# on this VM
git -C /space/vmshare/repos/HoudiniLab bundle create /tmp/ap79.bundle develop..feat/sub6-xband-demo
scp /tmp/ap79.bundle houdini@168.6.244.26:/tmp/
# on the rig host
cd ~/repos/HoudiniLab
git fetch /tmp/ap79.bundle feat/sub6-xband-demo:ap79-run
git worktree add ~/repos/HoudiniLab-ap79 ap79-run     # once; later: git -C ~/repos/HoudiniLab-ap79 reset --hard ap79-run
ln -sfn ~/repos/HoudiniLab/CC/Sounder/mufft ~/repos/HoudiniLab-ap79/CC/Sounder/mufft   # the built submodule
source ~/houdini_test/bin/activate
cd ~/repos/HoudiniLab-ap79/CC/Sounder && cmake -B build && cmake --build build -j"$(nproc)"
( cd build && ctest )                                   # all pass (17 at this writing) before any run
strings build/sounder | grep -c "preflight before the post-activate clear"   # 1 = this build or later, not a stale one
```

## 2. Launch

One sounder process drives both nodes. Mode B (two terminals) is the one for a
first run, so the sounder's own log is in front of us.

Terminal 1, the dashboard backend:

```sh
cd ~/repos/HoudiniLab-ap79/CC/Sounder && python3 csi_gui/csi_server.py
```

Terminal 2, the sounder. R0 FIRST, the control, exactly as the last demo ran
(no link-health thread: unset it so the control changes nothing):

```sh
cd ~/repos/HoudiniLab-ap79/CC/Sounder
source ~/houdini_test/bin/activate
export LD_LIBRARY_PATH=$VIRTUAL_ENV/lib
export SOAPY_SDR_PLUGIN_PATH=$VIRTUAL_ENV/lib/SoapySDR/modules0.8-3
export HOUDINI_MAX_FRAME=2000000000 HOUDINI_UE_TX_DEBUG=1
unset HOUDINI_LINK_HEALTH_S
./build/sounder --view --conf_file files/houdini-r0.json 2>&1 | tee ap79_r0_$(date +%H%M).log
```

Then R1 (and later rungs), with the mode-V monitoring on:

```sh
cd ~/repos/HoudiniLab-ap79/CC/Sounder
source ~/houdini_test/bin/activate
export LD_LIBRARY_PATH=$VIRTUAL_ENV/lib
export SOAPY_SDR_PLUGIN_PATH=$VIRTUAL_ENV/lib/SoapySDR/modules0.8-3
export HOUDINI_MAX_FRAME=2000000000
export HOUDINI_UE_TX_DEBUG=1 HOUDINI_CFO_LOG_EVERY=100 HOUDINI_LINK_HEALTH_S=5
export HOUDINI_DUMP_DIR=$PWD/ap79_runs && mkdir -p "$HOUDINI_DUMP_DIR"   # session records land here
./build/sounder --view --conf_file files/houdini-dualband-r1.json 2>&1 | tee ap79_r1_$(date +%H%M).log
```

To split the nodes while debugging, run the same command twice with
`--bs_only` and `--client_only`.

The ladder, one change per rung; run each only after the one before passes:

| Rung | Config | Adds |
|---|---|---|
| R0 | `files/houdini-r0.json` | the CONTROL: the last known-good single-band demo (NCO 500, 122.88 both ways, fft 64, legacy beacon) on this build, this stack and the plan's cabling. If R0 fails, the problem is the stack or the rig, not mode V |
| R1 | `files/houdini-dualband-r1.json` | mode-V clocks, TX 245.76, sub-6 at 2425, nr_pss_bl, fft 256, 1 ms frame |
| R2 | `files/houdini-dualband-r2.json` | the X-band IF channel (UE TX B to BS RX C at 4380) |
| R3 | `files/houdini-dualband.json` | the 5G-like numerology (fft 4096, 0.5 ms slots, 10 ms frame) |
| R3-40 | `files/houdini-dualband-40.json` | the 40 MHz rollback of R3 |

R1 and R2 run on the calibrated hold (0.3 to 0.5 ppm), where the carrier
offset is harmless at their fft 256. R3 needs the clock steered and the CSI
pilot average de-rotated per symbol; both are planned after the first run: an
in-sounder steering thread on the UE (the separate `clock_steer_loop.py` needs
its own connection, which would reset a running session, and its `--ue-ip`
defaults to `.22`, which is the BS here) and the BS-side de-rotation. R3 is
also to be split into R3a (fft 4096, sub-6 only) and R3b (+ X-IF).

## 3. What the sounder logs

At bring-up, per node:

- `<label> mode V: ...`, one line per step of the converter bring-up, with the
  device's readbacks: `RFDC_DAC_FS -> 5898.24..`, `RFDC_ADC_FS -> 4915.2..`,
  the zone lists, `RFDC_ADC_CAL ... -> ...`, then per channel
  `RX ch0: NCO 2425.000 MHz, zone 1, cal Mode 1, +-25 MHz channel filter ON`.
  A readback that disagrees stops the bring-up with the channel and the value.
- The stack per node, and `VERSION SKEW:` if the nodes differ.
- `Beacon: type nr_pss_bl, core 1076 samples, matched field 1 x 512 at offset 36 ...`.
- `CSI view mode: streaming to 127.0.0.1:9999 (<N> subcarriers, ...)`:
  N is 256 at R1/R2 and 4096 at R3 (the DC-centred FFT size).
- After the setups and before activate, per node: `TX chN: DAC.. mts_synced=1
  T1=..`, `RX chN: ADC.. mts_synced=1 T1=..` (an unsynced channel refuses to
  activate), the `RFDC_ADC_CAL` readback (sub-6 cal=mode1, X-IF mode2, checked),
  the preflight as latched by the bring-up (informational: the bring-up
  latches benign ADC flags, SH-372 class), the IRQ count, and
  `session record <path>`: one file per node with the stack, every bring-up
  and post-setup line and the `RFDC_SNAPSHOT`.
- Per ADC block at each RX setup: `IntrStatus=0xF after rate apply; WATCH`
  from the driver. Known setup noise (the reconcile's transient FIFO bounce,
  cleared straight after), under the software lane's review; not a fault.
- `syncSearch: detection #k statistic S vs bar B (coherence)`: the first five
  detections, then one in 100. A beacon reads about 0.97; the bar is 0.2.

Per frame and periodically:

- UE: `Re-sync frame N: beacon alive on the anchored grid (resid .., snr .. dB)`
  on every targeted re-sync, and `Re-sync frame N: detection idx .. snr ..` at
  acquisition.
- UE, every 100 frames: `Beacon CFO frame N: tracked <Hz> (<ppm>) | beacon ...`,
  the carrier offset from the tracked clock.
- UE: `UE pilot burst: scheduled N frames up to <tick> (pad ..)` (the TX queue).
- BS: `BS: UE PILOT LOST for N consecutive frames` / `BS: UE pilot RETURNED`.
- Both nodes, about 2 s after the first read: the preflight is CLEARED (the
  bring-up's own latches go) and then `<label> link health: baseline taken;
  preflight FAILs standing at start: ...` once (expected `none`), then a line every 5 s that has an alarm and a clean line
  once a minute: `[..] 5.0 s: irq <n>/s, preflight <verdict>: clean | app:
  rx_err +0, rx_short +0, rx_pad +0, tx_short +0, tx_sat +0`.
- A TX saturation warning names the channel if the interpolated burst clips.
- The backend prints `[csi] N datagrams, antennas=[..]` every 5 s.

## 4. Pass criteria for the first run (R0, then R1)

Stated before the run. Each needs three runs that agree (plan rule 5); an
anomaly is explained before it is called a pass. R0 is judged by the last
demo's own bar (beacon acquired, CSI datagrams climbing, both as before).

1. **Bring-up**: both nodes complete the mode-V bring-up (every Fs, zone,
   inverse-sinc, rate, NCO and gain readback equal to what was written, which
   the bring-up now ENFORCES) and the post-setup check passes (every channel
   MTS-synced, the calibration modes as derived), with no SH-422 refusal.
2. **Preflight**: after the post-activate clear, the link-health baseline's
   standing FAIL list is EMPTY on both nodes; only the HS-207 items sit in the
   "known" part (DAC0.0, and DAC2.0 on a node transmitting on TX ch1). An
   ADC0.1 or ADC2.1 FAIL means SH-421's fix did not take: report it.
3. **Acquisition**: the UE acquires the beacon within 10 s; the first
   `syncSearch` lines read a statistic well above the 0.2 bar (about 0.97
   expected on cables).
4. **Tracking**, 5 minutes: `beacon alive` on the targeted re-syncs, no
   escalation, no `UE PILOT LOST`; the sync residual REPORTED as a statistic,
   and passing when at least 99 % of re-syncs sit within +-2 samples and none
   beyond the gate (8.51 saw a maximum of 12 to 14 on the legacy beacon,
   inside the gate); the beacon's in-window SNR (the `snr` of the re-sync
   lines, the SNR the sync layer gates on) at least 10 dB over its 25 dB floor
   (about 42 dB predicted on cables).
5. **Clock**: the tracked offset (`Beacon CFO frame N: tracked ... ppm`) reads
   a steady value near the calibrated hold (0.3 to 0.5 ppm on 2026-09-01).
   Recorded, not gated.
6. **CSI (sub-6 uplink, BS ch0)**: the `[csi]` counter climbs steadily for
   antenna 0 and no BS frame is rejected for want of a pilot. The channel
   estimate is judged from a `HOUDINI_CSI_DUMP` capture: |H| across the 96
   occupied subcarriers (88 data tones plus 8 pilot tones at R1) RECORDED as its spread (expected well under 1 dB: the
   decimator and interpolator are flat to 0.19 dB over +-24 MHz, W2; more than
   3 dB fails) with no mirror-image structure (single-tone mirror rejection
   measured -87 to -96 dBc). The sounder computes no CSI SNR; none is gated.
7. **Uplink data**: the QPSK constellation shows four clean clusters; the
   recorder's CNS score and the blind EVM from `evm_compare.py` on the dump
   are recorded (first numbers, not gated).
8. **Health**: 5 minutes with no link-health alarm beyond the baseline, and
   app counters `rx_err`, `rx_pad`, `tx_short`, `tx_sat` all zero. The
   interrupt rate is about 46.6k/s on BOTH nodes (HS-207, known: any open TX
   stream at TX 245.76, the BS beacon's replay included; one core in each
   device server).

R2 adds: the X-IF CSI on BS antenna ch2 meets 6 on its own, the two antennas
carry distinct channels (sub-6 and X-IF, no crosstalk structure), and the
X-IF is NOT spectrally inverted under the sounder's conjugation path (W3
checked the sense with tones, not through this path).

## 4a. What to watch on the first run

- **The BS framer's decisions run on the RAW sub-6 lane.** The slots it
  extracts are channel-filtered exactly, but its energy search, presence gate,
  P/U tagging, LTS check and slot alignment now see the capture unfiltered,
  which at mode V carries the channel's 0 dBc real-sampling mirror. Untested
  at mode V. If BS frames are rejected or P/U mis-tagged, A/B a short run with
  `HOUDINI_BS_FILTER_WHOLE=1` (filters the whole capture as before, about 1.3x
  real time at R1, so short runs only).
- **A deliberate detune** (AP-33) puts `mts_phase_stale(RFDC_NCO_REARM)` in
  the preflight's FAIL list, which the monitor reports as new: expected then.
- **Driver messages that are expected** (device `a1c8ecb7`, the SH-422 fold):
  each zone write WARNs that the NCO "reads back as 0.000 MHz against the new
  zone ... re-issue setFrequency" (the bring-up's next step does exactly that);
  an RX setup's per-block `IntrStatus ... WATCH` now names only a bit that
  survives the post-bounce clear (on `.22`: `ADC 0.1 [SUBADC_DCDR]`, the known
  SH-227 class): log it, and treat it as a fault only if the preflight also
  FAILs. A new preflight item `<blk>:rate_unbalanced(...)` FAILs a block whose
  rate differs from its tile's; the sounder writes one rate on every channel
  of each direction, so it should never appear.
- **CPU**: one core per node in the device server for HS-207; the UE's RX
  channel filter costs about 1.2 cores for a continuously read lane.

## 5. Levels to expect

Bare cables, no pads. The software lane's -12 dBFS test tone arrived at about
-27 dBm on sub-6 and -33 dBm on the X-IF, 99 and about 90 dB above the capture
floor. What the sounder transmits: the pilot and uplink data normalized to 0.5
of full scale peak (config.cc), about -16 to -18 dBFS RMS at the OFDM's crest
factor; the beacon at `sync.beacon.tx_full_scale` 0.34 peak = -16 dBFS RMS at
its 6.6 dB PAPR (the plan's back-off); the TX interpolator warns if a burst
clips. All arrive far above every SNR bar here. A BS ADC reading near full
scale means something is wrong, not the level plan.

## 6. Calibrations

Derived before the run (tests/comms-func/detection_calibration_test.cc, a
simulated link calibrated to this bench's legacy measurement), to CHECK on the
rig with the `syncSearch` and re-sync lines:

- the coherence bar 0.2 (`corr_scale` 5; noise-only maximum 0.080 of 400
  windows, a beacon 30 dB under bench level still 0.90), held at 0.1 by
  `detector.min_bar` through the resync retries;
- the in-window SNR floor 25 dB (legacy's 30 dB shifted by the measured
  -5.2 dB for nr_pss_bl at 0.34 peak on the filtered lane).

Still carried, to MEASURE at the first run:

- `tx_advance` (247 at 122.88 / NCO 500): the mode-V converter pipelines
  differ. Record where the UE pilot lands in the BS window at R1; at R3 the
  window tolerates only about 15 samples late (the 32-tick prefix less the
  filter's 17), against about 111 at R1.
