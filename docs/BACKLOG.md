# HoudiniLab BACKLOG — application lane (`AP-###`)

Cross-lane needs are filed here tagged for the receiving lane per
`houdini-agents/conventions/handoffs.md` (`[→ propose SH ticket]` = software /
SoapyHoudiniSDR, `[→ propose HS ticket]` = fpga / Houdini-Streaming,
`[→ propose AG ticket]` = coordinator). The evidence lives in the linked
`docs/<TOPIC>.md`; the receiving lane files its own row with a back-pointer and the
canonical id lives there from then on.

---

## AP-2 — Frequency-lock the two RFSoC4x2 boards (external CLK IN mux select) `[→ propose SH ticket]`

**Status:** RESOLVED — both boards locked to a common external 10 MHz on `CLK IN`.
Verified app-side (`.21` beacon → `.22`, coherent per-4096-period phase over a 20 ms
capture): **CFO = +0.1 Hz (0.0002 ppm at the 500 MHz NCO), drift ~0.0000 samp/frame**
(was +447 kHz / 894 ppm / ~110 samp-frame free-running). The gold-corr DDC offset moved
+447 kHz → +0.0 MHz and the beacon folds ISOLATED at period 4096. Sample clocks are now
identical (shared ref), so the beacon-synced UE pilot no longer walks the BS frame. The
firmware `CLK_SEL0/1` external-mux path (per the doc) landed via the os/firmware lane.
Residual `SYNC IN`/SYSREF phase determinism (power-cycle-repeatable epochs, coherent/MIMO)
is a separate later step, not needed for the sounder loop.

<details><summary>original OPEN handoff (kept for history)</summary>

**Receiver:** software (SoapyHoudiniSDR device firmware `clock_driver.cpp`), deployed by
the os lane (`deploy-fw`).
**Doc:** [`docs/TWO_BOARD_CLOCK_LOCK.md`](TWO_BOARD_CLOCK_LOCK.md)

The two boards free-run on independent references — measured **CFO ≈447 kHz (894 ppm)**,
sample-clock drift **~110 samples/1-ms frame** — so the beacon-synced UE pilot walks the
whole BS frame (~1.1 s) and can't be seated in the fixed rx_gate (recorded pilot = noise).
Everything else in the sounder loop is proven good (beacon sync, pilot emission, reverse
cable, on-air pilot). The firmware's LMK04828 config **already expects 10 MHz** (CLKin1,
R=125) but `ClaimClockGpios` holds the **`CLK_SEL0/1` mux-select pins at 0,0 = onboard**,
so a common 10 MHz on the external `CLK IN` is ignored. Fix = drive `CLK_SEL0/1` to the
**external** mux value (open item: which value — RFSoC4x2 schematic / empirical). Verify
via `ClockVerifyLocks()` PLL lock + the app tone-test beat stopping + CFO→~0. `SYNC IN`/
SYSREF phase alignment is a separate, later step. See the doc for full evidence + options.
</details>

---

## AP-1 — UE fine-grained timed TX (whole-ms scheduling limit) `[→ propose SH ticket]`

**Status:** RESOLVED by **SH-301** (relay) — the capability already existed; the unblock
was app-side (`tdd=1` on the UE TX stream arg, streaming mode). Residual device
replay-mode whole-ms path is optional/deferred.
**Doc:** [`docs/UE_TX_FINE_GRID_TIMING.md`](UE_TX_FINE_GRID_TIMING.md)
