# UE fine-grained timed TX — the whole-ms scheduling limit blocks the sounder closed loop

**Lane:** application (HoudiniLab) investigation; **root-cause/fix lane:** software
(SoapyHoudiniSDR). This doc is evidence + options ruled in/out for a
`[→ propose SH ticket]` hand-off (AP-8). It is not a fix directive — the driver
timing contract is the software+fpga lanes' design.

## RESOLUTION (SH-301 → relay to AP-8) — the capability already exists

The software lane filed **SH-301**, grounded this doc against current `develop`, and
relayed it back: **the fix is APP-SIDE, and this doc's premise was partly stale.**

- **Streaming TX — the mode the sounder uses — ALREADY accepts the 3.125 µs TDD
  window grid**, shipped in SH-248. The host `TxTickAnchor::NextTick` validates with
  `IsTddWindowTimeAligned` + exact `TddGridNsToTicks` **when the TX stream is opened
  with the `tdd=1` stream arg** (`host/stream/tx_tick_anchor.h:~92`; arg →
  `cfg.tx_tdd_grid` at `host/SoapyHoudiniSDR_streaming.cpp:~483`, advertised `:~212`).
  Verified in the DEPLOYED driver on the DGX (SH-248 in history, `.so` built Jul 22).
- **My evidence #3 mis-cited the device file.** The whole-ms guard at
  `device/…:1187` is the **replay** branch (AP-8 explicitly ruled replay out); the
  live-TX activate imposes no time grid (plays host-stamped ticks, `:1130`); and
  `:1846` is `setHardwareTime`, not the streaming write. The streaming-TX timing
  lives in the **host `TxTickAnchor`**, which I missed — so "TX doesn't accept the
  fine grid" was wrong.
- **The unblock (app-side):** the sounder's UE `txStreamArgs` (`CC/Sounder/Radio.cc`
  path via `ClientRadioSet`) does not set `tdd`, so it got the whole-ms reject the
  doc observed. Setting `tdd=1` on the UE TX stream (streaming mode) enables sub-ms
  timed pilots on the 3.125 µs grid — no driver change needed.
- **Residual software-lane gap (OPTIONAL, non-blocking, deferred):** the device
  TX-**replay** timed path (`:1187`) is still whole-ms; mirror RX@1538 onto it only
  if replay-mode sub-ms TX is ever wanted. Not needed for AP-8 (streaming).

Everything below is the ORIGINAL hand-off, kept as the investigation record; read it
through the correction above.

---

## The application need

The RENEW Sounder closed loop needs the UE (client) to transmit its uplink pilot
at a **beacon-referenced time** so it lands inside the BS's native-TDD `rx_gate`
window. The BS receive side is done: the sounder's `BaseRadioSet`/`loopRecv` arms
the FPGA TDD framer (beacon replay strobe + `rx_gate` per pilot slot) and records
only the gated slot — verified non-zero (`Data/Pilot_Samples` absmax≈1220).

**The placement tolerance.** Two figures, don't conflate them:
- The `rx_gate` **symbol** (ADC receptive) is `SYM` = 61440 ticks = **0.5 ms**.
- Of that, the BS **records** only `samps_per_slot` = 4096 samples = **33 µs**
  (`4096 / 122.88 MSPS`; 4096 is pinned by the RENEW `genPilots` TX_RAM ≤ 4096
  rule). This 33 µs read is placed inside the 0.5 ms symbol; today it sits at the
  symbol start, but the BS window is deterministic and per-call placeable (v1), so
  the read can be positioned anywhere in the 0.5 ms gate.

So the UE-side timing tolerance is **≤ 0.5 ms** (place the pilot anywhere in the
`rx_gate` symbol; the BS read is aligned to it), NOT ~33 µs. The fix therefore
does **not** need microsecond resolution — it needs (a) sub-0.5-ms placement and,
critically, (b) **no 1 ms discontinuity** as inter-board drift moves the
beacon-referenced target across a boundary. The 3.125 µs TDD grid amply satisfies
both, but a coarser sub-ms grid would too; the hard requirement is losing the 1 ms
cliff, not the exact grid pitch.

## Evidence (what we observed, and where)

1. **The UE can't place a sub-ms pilot via streaming TX.** With `tx_mode=stream`
   (the natural `ue_hw_framer=false` mode), a timed `writeStream`/`activateStream`
   is rejected off-millisecond, observed live as:
   `HoudiniStream::Write rejected: TX start time must be a positive whole millisecond`.
   A 33 µs gate at a sub-ms position in the frame cannot be hit on a 1 ms grid.

2. **The 1 ms is a software contract, not an FPGA limit.**
   `shared/houdini_streaming_math.h`: `kScheduleQuantumNs = 1000000` +
   `IsScheduleTimeAligned()`. The comment gives the rationale: a whole ms
   (122,880 ticks) is an exact multiple of every RX conv-beat and TX start grid at
   every rate, so co-armed DAC/ADC land on the SAME tick (a per-converter snap
   would not); fail-loud, reject-don't-snap. The FPGA's real TX start grid is
   `TxGridTicks(rate_code) = 8>>rate_code = {8,4,2,1}` ticks (`shared/houdini_rate_ladder.h`)
   — i.e. as fine as ~8 ns. The gateware is not the constraint.

3. **A finer grid already exists — and RX already accepts it.**
   `kTddScheduleQuantumNs = 3125` ns = 384 ticks = **3.125 µs** ("the finest
   ns-exact grid, still on every conv-beat/TX grid"; strictly generalizes the ms
   rule — every whole ms is on it). The **RX** timed path accepts it when a TDD
   schedule is engaged: `device/SoapyHoudiniSDR_streaming.cpp:1538`
   (`tdd_grid_ok = !IsScheduleTimeAligned(time_ns) && IsTddWindowTimeAligned(time_ns) && TddEngaged()`).
   The **TX** timed paths do **not** — they still hard-require whole ms:
   `:1187` (TX replay `activateStream(HAS_TIME)`) and `:1846` (streaming-TX write).
   The design comments appear to intend TX support ("host TX: the tdd stream arg"),
   but that refinement is present on RX only. (Observation, not a prescription.)

4. **Drift makes the whole-ms path a hard cliff, not just coarse.** The two RFSoC
   boards free-run (no shared clock); the beacon-referenced target time drifts.
   On a 1 ms grid, when it drifts across a ms boundary the quantization jumps a
   full millisecond → the pilot leaves the `rx_gate` window (the jump, 1 ms, is
   twice the whole 0.5 ms symbol). So whole-ms streaming is not merely imprecise;
   it fails intermittently as drift crosses a boundary. This 1 ms discontinuity —
   not the grid pitch — is the core problem.

## Options ruled OUT on the application side

- **Whole-ms streaming + place the BS gate to catch it** — fails on the
  ms-boundary drift cliff (evidence 4); also can't express the sub-ms UE↔BS clock
  offset.
- **TDD replay strobe, live re-placement** — rewriting `TDD_REPLAY_STROBE` on a
  running framer to chase drift is non-deterministic in test
  (`tests/hil/houdini_strobe_move.py`: same offset → scattered burst positions,
  ~65 k-sample rms; the strobe is deterministic only when set *at arm*, cf. the
  stable v2 result `tests/hil/houdini_tdd_loopback.py`). Not usable for per-frame
  tracking.
- **Replay strobe set-at-arm + re-arm to track drift** — deterministic but each
  re-arm churns the framer epoch and is heavy; not viable for continuous per-frame
  tracking.

## What the application needs (capability, not mechanism)

With a TDD schedule engaged, the UE needs **timed TX (streaming and/or replay) to
accept a start time finer than whole-ms** — enough to place the pilot within the
0.5 ms `rx_gate` and, above all, to move the beacon-referenced target **without a
1 ms discontinuity**. The 3.125 µs TDD grid the RX path already accepts (@1538)
does this and strictly generalizes the ms rule, so mirroring that acceptance onto
TX is the natural candidate — but a coarser sub-ms grid would also clear the bar;
the pitch is not the constraint, the cliff is. Whether/how the scoping is done
(stream arg, device TDD state, which of the streaming vs replay TX paths) is the
software lane's design; RX @1538 is the working precedent to weigh against.

## What would validate it (application-side test, ready to run)

Once TX accepts the TDD grid: run the full sounder — BS native-TDD receive
(`sounder --conf files/houdini-1u.json`, `bs_hw_framer=true`) + UE burst-streaming
pilot placed at the beacon-referenced time on the 3.125 µs grid — sweep the
beacon-referenced advance and confirm the recorder captures the UE pilot **in the
gate** (non-zero, correlating) and **holds it across frames** through a
ms-boundary crossing (the whole-ms path's failure mode). The BS gated-window
receive is already proven deterministic, so this isolates the TX-timing change.

## Application-side references

- BS native-TDD receive fold-in: `CC/Sounder/BaseRadioSet.cc` (`armHoudiniTdd`,
  `houdiniTddRx`), `CC/Sounder/Radio.cc` (`recvTddWindow`), config
  `CC/Sounder/files/houdini-1u.json`.
- Prototypes: `tests/hil/houdini_tdd_bs_rx.py` (RX gating), `houdini_tdd_loopback.py`
  (v2 timing advance), `houdini_strobe_move.py` (live-rewrite non-determinism).
