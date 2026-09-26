# Archived design and investigation notes

Finished work kept for the reasoning behind decisions that still hold. Nothing
here is a procedure to follow; the current operator docs are
`CC/Sounder/CSI_DEMO_WALKTHROUGH.md` and `CC/Sounder/DEMO_BENCH_RUNBOOK.md`, and
the evidence is `CC/Sounder/DEMO_VERIFICATION.md`.

| File | What it was | Where its result lives now |
|---|---|---|
| `TWO_BOARD_CLOCK_LOCK.md` | The two-board clock investigation (AP-9): why the boards need a shared reference or a calibrated hold | The walkthrough's clock plan (section 2.1); the clock steering in the sounder (`sync.steer`) |
| `UE_TX_FINE_GRID_TIMING.md` | The UE's fine-grained timed TX (AP-8): the 3125 ns anchor grid and the zero-padded burst | The walkthrough's frame geometry (section 3); `tx_advance` and the config notes |
| `UE_TIME_FREQ_SYNC.md` | The UE time and frequency sync campaign summary; its one-line state (no frequency correction on the data path) is superseded | The pre-FFT carrier correction (`bs_cfo_pre_fft`, AP-90) and the clock steering; `DEMO_VERIFICATION.md` section 9 |
