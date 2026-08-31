# Lab bench runbook: exactly what runs where

This file is the bench-specific companion to `CSI_DEMO_WALKTHROUGH.md`. The
walkthrough stays neutral (placeholders and discovery commands) so it travels
to any bench; this runbook fills in every placeholder for OUR lab setup so a
run can be reproduced or debugged without archaeology. If the bench changes,
change this file in the same commit.

Evidence for everything below lives in `DEMO_VERIFICATION.md`.

## 1. The machines

| Machine | Address | Role | What runs on it |
|---|---|---|---|
| Build VM | (this repo's checkout at `/space/vmshare/repos/HoudiniLab`) | Development only | Editing, compile checks, offline analysis. NOTHING of the demo executes here. Code ships to the rig as a git bundle over ssh, never as a copied tree. Note the vboxsf trap: `touch` changed sources before a local build or cmake may relink nothing. |
| Rig host ("rig B") | `168.6.244.64`, user `houdini` | Runs the entire host side | The sounder binary AND the dashboard backend (details in section 2). |
| Base-station board | `168.6.244.21`, user `houdini`, serial `575524` | BS radio | `SoapySDRServer --bind` (systemd unit) serving the device plugin `libHoudiniSDRDevice.so`; FPGA bitstream v1.30 `c88e0b5f`; device software 0.2.2. |
| Client board | `168.6.244.22`, user `houdini`, serial `6596d2` | UE radio | Identical stack to the BS board. |
| Your workstation | anywhere | Viewer | A browser, through an ssh tunnel to the rig (section 4). |

Both boards share one 10 MHz reference (frequency lock; there is no
cross-board phase lock, so the corrected-phase panel re-anchors per run).

## 2. What runs on the rig host

One sounder process drives BOTH radios. There is no per-board host process:
`sounder` opens the BS radio (remote to `.21`) and the UE radio (remote to
`.22`) from the same process over the SoapyRemote control plane, and runs its
own UDP data planes to each board.

- Checkout: `~/repos/HoudiniLab-rx` (a worktree of this repo, branch
  `feat/csi-gui-tabler`). This is the ONLY checkout the demo should run from.
  `~/repos/HoudiniLab` also exists on the rig and contains an unrelated
  workstream with its own stale sounder build. Always pass `--sounder-dir`
  (section 3) and verify a fresh ship with
  `strings build/sounder | grep <a-string-only-the-new-code-logs>`.
- Binary: `~/repos/HoudiniLab-rx/CC/Sounder/build/sounder`, built on the rig
  with `/usr/bin/cmake --build build --target sounder -j`.
- Dashboard backend: `~/repos/HoudiniLab-rx/CC/Sounder/csi_gui/csi_server.py`
  (HTTP on 8080, CSI datagrams in on UDP 9999, both localhost).
- SoapySDR host stack: the validated houdini HOST plugin lives ONLY at
  `/home/houdini/houdini_test/lib/SoapySDR/modules0.8-3/`. The system
  SoapySDR at `/usr/local` does NOT have it, so every launch must carry
  `SOAPY_SDR_PLUGIN_PATH=/home/houdini/houdini_test/lib/SoapySDR/modules0.8-3`
  (the backend's `--venv` default `~/houdini_test` sets this when launching
  through it). Without it every radio open fails with
  `SoapySDR::Device::make() no match`.
- Teardown helper: `csi_gui/teardown_framer.py` runs on the rig (it opens the
  boards, so it is device-touching).
- Logs land under `~/repos/HoudiniLab-rx/CC/Sounder/logs/`.

## 3. The exact launch used for the live demo

On the rig, in one shell:

```sh
cd ~/repos/HoudiniLab-rx/CC/Sounder
export HOUDINI_BS_RX_DEBUG=1 HOUDINI_UE_TX_DEBUG=1 HOUDINI_CSI_R_DEBUG=1
export HOUDINI_CNS_DUMP_LOW=logs/cnslow
python3 csi_gui/csi_server.py --launch --conf files/houdini-ul.json \
    --sounder-dir ~/repos/HoudiniLab-rx/CC/Sounder \
    --mag-top 85 --mag-span 5
```

The backend sets the plugin path from its `--venv` default, runs the framer
teardown, then starts `sounder --view` and retries the flaky cold start. The
debug exports are optional but cheap, and they are what every verification in
`DEMO_VERIFICATION.md` greps for.

## 4. Viewing

From your workstation:

```sh
ssh -L 8080:localhost:8080 houdini@168.6.244.64
```

then open `http://localhost:8080/`. After any backend restart, refresh the
page once: the magnitude axis and page structure are baked in at page load.

## 5. Stopping, and the two recoveries that actually happen

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
  the environment (see section 2).

A bare `SoapySDRUtil --find` proves nothing about board health (the device
factory answers only when the filter carries `show=1`). Probe a board with:

```sh
SoapySDRUtil --find="remote=tcp://168.6.244.21:55132,show=1"
```

## 6. Stack identity this runbook was written against

fpga 1.30 `c88e0b5f` (2026-08-28), device 0.2.2 `71bcbc6b`, host 0.2.2
`c20d7975`, protocol 1.0, SoapySDR 0.8.1, SoapyRemote 0.6.0. Both boards
report `clock_ref: external`. Config: `files/houdini-ul.json`
(30 slots x 4096 samples = exactly 1 ms per frame, beacon slot 0, pilot slot
16, uplink data slot 18, `tx_advance` 247).
