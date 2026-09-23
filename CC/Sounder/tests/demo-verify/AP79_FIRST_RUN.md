# AP-79 first dual-band run: launch, what to watch, pass criteria

The first run of the mode-V dual-band sounder on silicon, run jointly with the
software lane (they watch both nodes' journals and CPU with `node_watch.sh`).
Branch `feat/sub6-xband-demo`. Roles and wiring follow the HS-202 plan
section 6: `.22` is the BS, `.21` is the UE, and the rig host `.26` runs the
sounder, which drives both nodes over their topology file. Nothing here runs
until the software lane hands over the rig.

## 0. Before the run

1. Confirm the stack on both nodes, and write it down: `fpga_commit` (the
   identity; `fpga_version` is cosmetic), `device_build` (SH-422 build),
   `host_build`. The sounder prints them at bring-up and warns
   `VERSION SKEW:` if the nodes differ.
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
( cd build && ctest )                                   # 14/14 before any run
strings build/sounder | grep -c "link health: baseline taken"   # 1 = this build, not a stale one
```

## 2. Launch

One sounder process drives both nodes. Mode B (two terminals) is the one for a
first run, so the sounder's own log is in front of us.

Terminal 1, the dashboard backend:

```sh
cd ~/repos/HoudiniLab-ap79/CC/Sounder && python3 csi_gui/csi_server.py
```

Terminal 2, the sounder, rung R1 first:

```sh
cd ~/repos/HoudiniLab-ap79/CC/Sounder
source ~/houdini_test/bin/activate
export LD_LIBRARY_PATH=$VIRTUAL_ENV/lib
export SOAPY_SDR_PLUGIN_PATH=$VIRTUAL_ENV/lib/SoapySDR/modules0.8-3
export HOUDINI_MAX_FRAME=2000000000
export HOUDINI_UE_TX_DEBUG=1 HOUDINI_CFO_LOG_EVERY=100 HOUDINI_LINK_HEALTH_S=5
./build/sounder --view --conf_file files/houdini-dualband-r1.json 2>&1 | tee ap79_r1_$(date +%H%M).log
```

To split the nodes while debugging, run the same command twice with
`--bs_only` and `--client_only`.

The ladder, one change per rung; run each only after the one before passes:

| Rung | Config | Adds |
|---|---|---|
| R1 | `files/houdini-dualband-r1.json` | mode-V clocks, TX 245.76, sub-6 at 2425, nr_pss_bl, fft 256, 1 ms frame |
| R2 | `files/houdini-dualband-r2.json` | the X-band IF channel (UE TX B to BS RX C at 4380) |
| R3 | `files/houdini-dualband.json` | the 5G-like numerology (fft 4096, 0.5 ms slots, 10 ms frame) |
| R3-40 | `files/houdini-dualband-40.json` | the 40 MHz rollback of R3 |

Before R3, steer the UE clock with no sounder running (the steering tool opens
its own connection, which would reset a running session), then run R3 on the
steered hold: `tests/demo-verify/clock_steer_loop.py`.

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

Per frame and periodically:

- UE: `Re-sync frame N: beacon alive on the anchored grid (resid .., snr .. dB)`
  on every targeted re-sync, and `Re-sync frame N: detection idx .. snr ..` at
  acquisition.
- UE, every 100 frames: `Beacon CFO frame N: tracked <Hz> (<ppm>) | beacon ...`,
  the carrier offset from the tracked clock.
- UE: `UE pilot burst: scheduled N frames up to <tick> (pad ..)` (the TX queue).
- BS: `BS: UE PILOT LOST for N consecutive frames` / `BS: UE pilot RETURNED`.
- Both nodes: `<label> link health: baseline taken; preflight FAILs standing at
  start: ...` once, then a line every 5 s that has an alarm and a clean line
  once a minute: `[..] 5.0 s: irq <n>/s, preflight <verdict>: clean | app:
  rx_err +0, rx_short +0, rx_pad +0, tx_short +0, tx_sat +0`.
- A TX saturation warning names the channel if the interpolated burst clips.
- The backend prints `[csi] N datagrams, antennas=[..]` every 5 s.

## 4. Pass criteria for the first run (R1)

Stated before the run. Each needs three runs that agree (plan rule 5); an
anomaly is explained before it is called a pass.

1. **Bring-up**: both nodes complete the mode-V bring-up with every readback
   equal to what was written, and the streams open with no SH-422 refusal.
2. **Preflight baseline**: with SH-421's fix deployed the standing FAIL list
   is EMPTY on both nodes, apart from the HS-207 item in the "known" part. An
   ADC0.1 or ADC2.1 FAIL means SH-421's fix did not take: report it to the
   software lane.
3. **Acquisition**: the UE acquires the beacon within 10 s of start.
4. **Tracking**: for 5 minutes, `beacon alive` on every re-sync, `resid`
   within +-2 samples, beacon SNR above 20 dB on the bare cables, and no
   `UE PILOT LOST`.
5. **Clock**: the tracked offset reads a steady value near the calibrated
   hold (0.3 to 0.5 ppm was the 2026-09-01 reading). Recorded, not gated.
6. **CSI (sub-6 uplink, BS ch0)**: the `[csi]` counter climbs steadily for
   antenna 0; CSI SNR above 25 dB; |H| across the 96 data subcarriers is
   RECORDED as its spread, expected well under 1 dB (the decimator and the
   interpolator are flat to 0.19 dB over +-24 MHz, W2), and more than 3 dB
   fails; no mirror-image structure (single-tone mirror rejection measured
   -87 to -96 dBc).
7. **Uplink data**: the QPSK constellation shows four clean clusters; EVM
   recorded (first number, not gated).
8. **Health**: 5 minutes with no link-health alarm beyond the baseline, and
   app counters `rx_err`, `rx_pad`, `tx_short`, `tx_sat` all zero. The
   interrupt rate is about 46.6k/s on BOTH nodes: HS-207 keys on any open TX
   stream at TX 245.76, the BS beacon's replay as much as the UE's live TX
   (known; one core in each device server).

R2 adds: the X-IF CSI on BS antenna ch2 meets 6 on its own, and the two
antennas carry distinct channels (sub-6 and X-IF, no crosstalk structure).

## 5. Levels to expect

Bare cables, no pads. The software lane's -12 dBFS test tone arrived at about
-27 dBm on sub-6 and -33 dBm on the X-IF, 99 and about 90 dB above the capture
floor. The OFDM at 0.3 to 0.4 peak (about -18 to -20 dBFS RMS) arrives about 6
to 8 dB lower, still far above every SNR bar here. A BS ADC reading near full
scale means something is wrong, not the level plan.

## 6. Calibrations to take at the first run

Carried from the pre-mode-V demo and not yet valid here:

- `tx_advance` (247 at 122.88 / NCO 500): the mode-V converter pipelines differ.
- `corr_scale` / `corr_scale_init`: measured on the legacy beacon.
- The in-window SNR floor, which the sync layer says was derived for legacy
  and rejects a quieter beacon wholesale (DEMO_VERIFICATION 8.157).
