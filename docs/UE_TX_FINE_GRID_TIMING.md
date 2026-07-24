# UE fine-grained timed TX — the whole-ms scheduling limit blocks the sounder closed loop

**Lane:** application (HoudiniLab) investigation; **root-cause/fix lane:** software
(SoapyHoudiniSDR). This doc is evidence + options ruled in/out for a
`[→ propose SH ticket]` hand-off (AP-1). It is not a fix directive — the driver
timing contract is the software+fpga lanes' design.

## The application need

The RENEW Sounder closed loop needs the UE (client) to transmit its uplink pilot
at a **beacon-referenced time** so it lands inside the BS's native-TDD `rx_gate`
capture window. The BS receive side is done: the sounder's `BaseRadioSet`/`loopRecv`
arms the FPGA TDD framer (beacon replay strobe + `rx_gate` per pilot slot) and
records only the gated slot — verified non-zero (`Data/Pilot_Samples` absmax≈1220).
The capture window is `samps_per_slot` = 4096 samples ≈ **33 µs**. To close the
loop, the UE pilot must be placed with **sub-slot (≤ ~µs) timing precision**, and
must **track slow inter-board drift** without a catastrophic jump.

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
   full millisecond → the pilot leaves the 33 µs gate entirely. So whole-ms
   streaming is not merely imprecise; it fails intermittently as drift crosses a
   boundary.

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
accept a start time on the 3.125 µs TDD grid** — the same acceptance the RX path
already has — so the pilot can be placed at sub-slot resolution and track drift by
grid steps (no 1 ms cliff). Whether/how that scoping is done (stream arg, device
TDD state, which of the streaming vs replay TX paths) is the software lane's
design; RX @1538 is the working precedent to weigh against.

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
