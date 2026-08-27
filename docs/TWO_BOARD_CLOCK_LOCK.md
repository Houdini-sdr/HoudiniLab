# Two RFSoC4x2 boards free-run — frequency-lock needs the external CLK IN mux select

**Lane:** application (HoudiniLab) investigation; **root-cause/fix lane:** software
(SoapyHoudiniSDR device firmware `clock_driver.cpp`), deployed via the os lane
(`deploy-fw`). This doc is evidence + options ruled in/out for a
`[→ propose SH ticket]` hand-off (AP-9, now RESOLVED — see below). It is **not** a
fix directive — the board clock tree and its bring-up are the software+fpga lanes'
design.

## RESOLUTION — both boards locked to a common external 10 MHz on `CLK IN`

**Status: RESOLVED.** The firmware `CLK_SEL0/1` external-mux path (described below)
landed via the software/os lanes. Verified app-side (`.21` beacon → `.22`, coherent
per-4096-period phase over a 20 ms capture):

- **CFO = +0.1 Hz** (0.0002 ppm at the 500 MHz NCO), was **+447 kHz / 894 ppm**.
- **Sample-clock drift ~0.0000 samples/frame**, was **~110 samples per 1-ms frame**.
- The gold-corr DDC offset moved **+447 kHz → +0.0 MHz**, and the beacon folds
  ISOLATED at period 4096.

Sample clocks are now identical (shared reference), so the beacon-synced UE pilot no
longer walks the BS frame — the blocker this doc was written about is gone. Residual
`SYNC IN`/SYSREF phase determinism (power-cycle-repeatable epochs, coherent/MIMO) is a
separate later step, not needed for the sounder loop.

Everything below is the ORIGINAL hand-off, kept as the investigation record; read it
through the resolution above.

## The need

The sounder closed loop (BS `.21` beacon → UE `.22` sync → UE pilot → BS rx_gate →
record) needs the two boards **frequency-locked**. Everything else in the loop now
works; the ONLY blocker is that the two boards free-run on independent references, so
the UE pilot drifts across the BS frame and can't be seated in the rx_gate.

## Evidence (application-lane, on the live `.21`/`.22` pair via the DGX)

Measured, current:

- **CFO = +447 kHz** between the boards (`.21→.22` beacon, lag-128 self-corr on the
  two Gold reps; consistent ~440–453 kHz across trials). At the 500 MHz NCO that is
  **≈894 ppm** — far beyond crystal tolerance, i.e. two fully independent references.
- **Sample-clock drift = ~110 samples per 1-ms frame** (894 ppm × 122880). The
  beacon-synced UE pilot therefore walks the entire 122880-sample BS frame in
  **~1.1 s**; measured pilot frame-offsets jump 10k–90k samples between scans. A
  FIXED `ue_tx_advance_ticks` cannot hold it in the fixed rx_gate → the recorded
  `Pilot_Samples` stays at the noise floor.
- **The rest of the loop is proven good** and is NOT the blocker: beacon sync locks
  (client re-syncs, `corr_scale=100`), the UE pilot emits (`ch1:acked` climbs, no
  underflow), the reverse cable carries it (tone rms ~1000), and a whole-frame BS
  scan finds the pilot on-air at peak-rms 400–600.
- **Frame geometry:** 122880 samples = 1.000 ms; TDD symbol = 61440 = 0.5 ms;
  rx_gate capture = one 4096-sample slot (33 µs).

Two-board scope check (application-lane `houdini_two_board_tone.py`): with both boards
commanded to the same 50 MHz tone, the scope shows two tones that **beat / slide** at
the CFO — the direct "not locked" signature. (After the fix, that beat should stop.)

## What the firmware shows (`SoapyHoudiniSDR/device/fpga/clock_driver.cpp`)

- **The LMK04828 config already expects a 10 MHz reference.** Header comment +
  registers: PLL1 references **CLKin1 with R-divider 125** → 80 kHz PFD from 10 MHz
  (`0x154/0x156 = 0x7D = 125`; `kLmkRegPll1Nlo` PLL1_N=96 for the 7.68 MHz nested
  zero-delay feedback). So **10 MHz is the correct reference frequency** — no
  reference-frequency change is needed. (Ruled in.)
- **The LMK takes CLKin1 by register** (`0x147 = 0x1A`, CLKin_SEL_MODE = CLKin1
  manual), so the LMK's own CLKin choice is fixed to CLKin1.
- **The firmware selects the ONBOARD reference via the mux-select pins.**
  `ClaimClockGpios()` requests `CLK_SEL0` (MIO 8) and `CLK_SEL1` (MIO 12) — the board's
  **"clock mux-select lines"** (`clock_driver.h`, `clock_driver.cpp:500`) — as outputs
  at initial value **0,0**, and holds them for the process lifetime. Since the LMK
  always takes CLKin1, these pins drive the on-board mux that decides *what feeds
  CLKin1*, and 0,0 routes the onboard 10 MHz.

**Consequence:** connecting a common 10 MHz to the external `CLK IN` SMA does nothing
on its own — the firmware never switches the mux to the external input. (We tried it:
CFO stayed 447 kHz, pilot still jittered.) The lock is a *firmware select*, not a
cable.

## Fix direction (options — the software lane owns the design)

- Feed a common 10 MHz into **both** boards' external `CLK IN` (application/hardware
  side; the user has this connected).
- In `clock_driver.cpp`, drive `CLK_SEL0/CLK_SEL1` to the value that routes the
  **external** SMA into CLKin1 instead of 0,0 (the initial values in
  `ClaimClockGpios`, and the held values in `ClockHoldSelects`/SH-269). If the
  external input is on a *different* LMK CLKin than CLKin1 on this board, then instead
  the CLKin selection (`0x147`) + that CLKin's R-divider (must be 125 for 10 MHz) move
  with it — but the mux-select reading above is the more likely wiring.
- No other register change is expected: CLKin1 / R=125 / PLL1_N=96 already target
  10 MHz. Build + deploy via `deploy-fw`, re-init both boards.

## Open item (the one board-specific unknown)

**Which `CLK_SEL0/1` value selects the external `CLK IN`.** This is the RFSoC4x2
board wiring (RealDigital schematic); it is NOT documented in the firmware, which just
hardcodes 0,0. It is one of (0,1)/(1,0)/(1,1). Resolve from the RealDigital RFSoC4x2
clocking reference, or empirically: with 10 MHz connected, try each combination and
watch (a) `ClockVerifyLocks()` PLL1/PLL2 DLD, (b) the application tone-test beat.

## Verification (how the app lane will confirm the lock closed the loop)

1. **Firmware:** `ClockVerifyLocks()` reports PLL1 + PLL2 locked (LMK DLD bits) against
   the external reference.
2. **App scope test:** `houdini_two_board_tone.py` — the two 50 MHz tones stop beating
   (same frequency).
3. **App RF:** re-measure CFO (→ ~Hz) and re-run the pilot scan — the pilot frame-offset
   should be STABLE across frames instead of walking. Then a single measured
   `ue_tx_advance_ticks` seats the pilot in the rx_gate and the recorded
   `Pilot_Samples` goes non-zero — closing the loop.

## Ruled out / not this

- **Reference frequency:** 10 MHz is correct (matches CLKin1 R=125). Not the problem.
- **Beacon / pilot signal / detector:** all verified good (sync, emission, reverse
  cable, on-air pilot). The blocker is purely the drift from free-running clocks.
- **`SYNC IN` / SYSREF phase alignment:** a *separate, later* step for deterministic,
  power-cycle-repeatable phase (cross-board MTS). Frequency lock alone unblocks the
  sounder; phase determinism is only needed for coherent/MIMO and reproducible epochs.

## Related

- App-lane closed-loop analysis + all the "everything-else-works" evidence lives in the
  application memory (`houdini-find-beacon-dense-corr-scale`,
  `houdini-hil-comb-channel`). AP-8 (`UE_TX_FINE_GRID_TIMING.md`) was the prior loop
  hand-off (resolved by SH-301).
