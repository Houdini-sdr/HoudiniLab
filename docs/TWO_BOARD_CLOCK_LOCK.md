# Two RFSoC4x2 boards free-run — frequency-lock needs the external CLK IN reference select

**Lane:** application (HoudiniLab) investigation; **root-cause/fix lane:** software
(SoapyHoudiniSDR device firmware `clock_driver.cpp`), deployed via the os lane
(`deploy-fw`). This doc is evidence + options ruled in/out for a
`[→ propose SH ticket]` hand-off (AP-9, now RESOLVED — see below). It is **not** a
fix directive — the board clock tree and its bring-up are the software+fpga lanes'
design.

## RESOLUTION — both boards locked to a common external 10 MHz on `CLK IN`

**Status: RESOLVED.** The firmware external-reference select landed via the
software/os lanes as SH-302 — but NOT by the mechanism this document guessed at;
see the correction below before reading the investigation record. Verified app-side (`.21` beacon → `.22`, coherent
per-4096-period phase over a 20 ms capture):

- The two-tone scope beat STOPPED, and the beacon folds ISOLATED at period 4096.
- The UE pilot stopped walking the BS frame: frame-offsets that had jumped tens of
  thousands of samples between scans went stable.

> **The absolute CFO figures this doc used to quote (+447 kHz / 894 ppm before,
> +0.1 Hz after) have been REMOVED as untrustworthy, not merely old.** They came
> from a lag-128 self-correlation with no coarse stage. At lag 128 and
> fs = 122.88 MHz the unambiguous range is only ±480 kHz, so a 447 kHz reading
> sits at 93 % of the fold limit and is one of a family of aliases spaced
> fs/128 = 960 kHz apart, with nothing in that measurement able to say which.
> The beacon's STS also repeats every 16 samples and 128 = 8×16, so a misaligned
> window makes the STS self-correlate at that same lag (seen directly on
> 2026-08-31 as a spurious peak at δ = −255). And 894 ppm is far outside any
> crystal spec, which the original text noticed and explained away rather than
> distrusted. The estimator was rewritten on 2026-08-31 (two-stage Schmidl-Cox,
> coarse STS lag 16 resolving the fine lag-128 ambiguity) and validated against
> injected offsets; **any absolute CFO number predating that rewrite should be
> treated as unmeasured.** AP-33 is the row that measures it properly.

Sample clocks are now identical (shared reference), so the beacon-synced UE pilot no
longer walks the BS frame — the blocker this doc was written about is gone. Residual
`SYNC IN`/SYSREF phase determinism (power-cycle-repeatable epochs, coherent/MIMO) is a
separate later step, not needed for the sounder loop.

Everything below is the ORIGINAL hand-off, kept as the investigation record; read it
through the resolution above.

## CORRECTION — the mux theory below is WRONG (SH-302)

Everything this document says about `CLK_SEL0/1` driving a board-level clock mux
is **wrong**, and it is left in place only because it is the record of what we
believed. The software lane established the real mechanism when it shipped the
fix:

- There is **no board mux**. `CLK_SEL0/1` go to the LMK04828's own pin-select
  inputs and are **don't-care** in this manual-mode config — which is why the
  cable-only test could never have worked, and why nothing we did to those pins
  would ever have mattered.
- The reference select is **LMK register `0x147` (CLKin_SEL_MODE)** alone:
  `CLKin1` = the onboard Si5395 10 MHz, `CLKin0` = the front-panel `CLK IN` SMA.
- It is an operator setting, not a code change: `sdr.conf clock_ref =
  internal|external` via `houdini-provision --set-clock-ref`, applied by re-running
  `houdini-provision` (or a reboot). Observed live on both nodes 2026-09-01:

  ```
  clock: programming PLL1 reference internal (LMK 0x147=0x1A)
  clock verify: PLL1 reference confirmed internal (LMK 0x147=0x1A readback)
  ```

  external programs `0x147=0x0A`. The readback verify matters: PLL1 locks either
  way, since both CLKin carry 10 MHz at the same R=125 / 80 kHz PFD, so a lock
  alone cannot tell you which reference is in force.

**Lesson worth keeping:** the mux reading came from `ClaimClockGpios()` requesting
two lines named "clock mux-select" and holding them at 0,0. The name plus the
hold looked like mechanism; it was neither. The hand-off did at least mark it as
the *likely* wiring with an explicit open item rather than a certainty, and the
owning lane resolved it from the part datasheet — which is the process working.

## The need

The sounder closed loop (BS `.21` beacon → UE `.22` sync → UE pilot → BS rx_gate →
record) needs the two boards **frequency-locked**. Everything else in the loop now
works; the ONLY blocker is that the two boards free-run on independent references, so
the UE pilot drifts across the BS frame and can't be seated in the rx_gate.

## Evidence (application-lane, on the live `.21`/`.22` pair via the DGX)

Measured, current:

- **The two boards are not frequency-locked.** Direct observation, independent of
  any CFO estimate: measured pilot frame-offsets jump 10k–90k samples between
  scans, so the beacon-synced UE pilot walks the entire 122880-sample BS frame. A
  FIXED `ue_tx_advance_ticks` cannot hold it in the fixed rx_gate → the recorded
  `Pilot_Samples` stays at the noise floor. The scope two-tone check below shows
  the same thing qualitatively (two tones beating). The absolute offset is NOT
  quoted here on purpose; see the note in the RESOLUTION section.
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
- **(WRONG — see the CORRECTION above.) The firmware selects the ONBOARD
  reference via the mux-select pins.**
  `ClaimClockGpios()` requests `CLK_SEL0` (MIO 8) and `CLK_SEL1` (MIO 12) — the board's
  **"clock mux-select lines"** (`clock_driver.h`, `clock_driver.cpp:500`) — as outputs
  at initial value **0,0**, and holds them for the process lifetime. Since the LMK
  always takes CLKin1, these pins drive the on-board mux that decides *what feeds
  CLKin1*, and 0,0 routes the onboard 10 MHz.

**Consequence:** connecting a common 10 MHz to the external `CLK IN` SMA does nothing
on its own — the firmware never switches the mux to the external input. (We tried it:
the beat and the pilot jitter both persisted.) The lock is a *firmware select*, not
a cable.

## Fix direction (options — the software lane owns the design)

> Superseded: the actual fix was `0x147`, not these pins. See the CORRECTION.

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

## Open item (the one board-specific unknown) — CLOSED, and the premise was wrong

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
