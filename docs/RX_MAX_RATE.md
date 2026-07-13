# Max-rate RX capture (AP-3/AP-4) — budgets, copy chain, and the zero-copy ladder

Application-lane design notes (2026-07-10) for sustaining the RX maximum
streaming rate — **1966.08 MSPS CS16 ≈ 62.9 Gbps ≈ 7.86 GB/s**
(`RX_FAB_CLK` 245.76 MHz × 8 samples/cycle) — through `rx-recorder` on
rig B. Companion to `docs/RX_GAP_AWARENESS.md` (loss accounting; AP-2).

## Fixed budgets (measured / verified)

- **Rig B (GB10) disk**: 4 TB TLC on a Phison PS5027-E27T (PCIe Gen4,
  DRAM-less). **Measured: 4.3 GB/s sustained over 40 GiB O_DIRECT**
  (pSLC not exhausted at that size) — ladder-rate captures have >4x
  headroom; re-measure with a >100 GiB write before trusting multi-minute
  245.76 MSPS runs. Max rate (7.86 GB/s) exceeds even that: **disk is not
  in the max-rate capture path.**
- **Rig B RAM**: 119 GiB (~110 free) unified LPDDR5x, ~273 GB/s. At max
  rate that is a **~14 s capture ceiling**; a 60-80 GB ring ≈ 8-10 s is
  the practical envelope. `rx-recorder`'s `buffer_slots` ring already
  supports capture-faster-than-drain; max-rate capture = RAM-resident
  ring, drain to disk after deactivate.
- **Known socket-path strain**: ~10-15 % kernel-datagram drops at
  sustained ~31.5 Gbps on a single UDP socket (SoapyHoudiniSDR host
  README) — before any host-plugin or app inefficiency.

## The copy chain today

```
NIC ──DMA──► kernel skb ──(1) recvmmsg──► plugin ring slot
      ──(2) readStream memcpy──► app chunk ──(3) staging──► HDF5 row buffer
      ──(4) H5Dwrite / page cache──► disk
```

Each user-space copy at 7.86 GB/s costs ~15.7 GB/s of memory bandwidth
(read+write) and roughly one core of memcpy. (1) is the kernel/user
boundary; (2) is the plugin's `Read()`; (3) is the AP-2 grid-resync
staging buffer; (4) is deferred out of the capture window by RAM-resident
operation.

## Recommendation: the standard direct-buffer API (the "good enough" rung)

> **STATUS 2026-07-12: LANDED driver-side + consumed app-side; validation
> staged.** The software lane shipped the acquire/release API (SH-254) on
> `feat/sh258-afxdp-ingest` exactly as recommended below — one acquire =
> one packet with its own per-packet tick stamp, probe =
> `getNumDirectAccessBuffers > 0` (no hardware-info key). The same branch
> ships AF_XDP zero-copy ingest (SH-258, `rx_xsk` stream kwarg) and the
> `HOUDINI_MTU` make-arg (SH-259) — the "kitchen sink" tier arrived
> together with this rung, and their silicon leg measured **99.92 %
> captured at 1966.08 MSPS** (paired geometry `HOUDINI_MTU=3512`, NIC MTU
> 3498, `rx_xsk=require`) vs ~18 % NIC-ring loss on the socket path.
> rx-recorder now has the acquire/release consumer (`direct_rx`
> auto/require/off, `SampleSource` in `include/rx_recorder_capture.h`,
> radio-free unit tests) and a releasable demo config
> (`CC/Sounder/files/rx-record-demo.json`). Rig validation matrix:
> §validation below.

> **STATUS 2026-07-12 (later): rig-validated on the TDD baseline
> (fpga v1.20 `da4c9d4c` / device+host v0.2.0, `tdd=1`).** Build + 4/4
> radio-free ctest on aarch64; 245.76 MSPS 2 s clean (7500/7500, 0 loss,
> `READ_PATH=direct` even on the socket ingest); AP-5 loopback tone
> verified on silicon (§validation V5). **New max-rate finding:** the
> demo config's zero-copy chain ENGAGES correctly (reconcile
> NIC→3498, `zero-copy RX engaged q19`), but end-to-end capture at
> 1.96608 GSPS loses **~22–25 % (reproducible: 24.6/21.9/22.7 % over 3×
> 2 s runs)** — the AF_XDP fill-ring starves (`RX ring full — hardware
> dropping`), NOT fixed by `taskset` onto the fast-core cluster, so it is
> the recorder-consumer's ~15.7 GB/s throughput, not app-thread core
> placement. The **99.92 % is the driver-ingest boundary** and does not
> carry through our HDF5 consumer at max rate. Every dropped sample is
> exactly accounted in `/Data/Gaps` (sample-exact, `direct`) and
> cross-verified program↔inspect. Isolating/fixing the consumer bound is
> the open V4 work (UMEM/fill-ring depth vs. writer→RAM-ring drain
> coupling vs. memory-bandwidth); until then the demo is a
> **honest-accounting** max-rate capture, not a lossless one.
> *(Superseded same day — V4 root-caused and resolved; next block.)*

> **STATUS 2026-07-12 (V4 root-cause session): SOLVED — max-rate capture
> is near-lossless end-to-end.** The ~22–25 % was NOT raw consumer copy
> throughput and NOT core placement. Three coupled causes, isolated by
> /Data/Gaps forensics on the lossy captures + config-only A/B runs:
>
> 1. **Per-packet acquire/release overhead at the zc wire quantum.** The
>    paired geometry carries **848 samples/packet** (every gap length is
>    a multiple of 848, ±1 stamp rounding) ⇒ **2.32 M acquires/s** at
>    1.96608 GSPS — a 431 ns/packet budget. The direct-path consumer
>    pays two virtual API calls + a per-packet grid stamp check and
>    sustains only ~78 % of wire; the granularity estimate above
>    (~975 k/s, "fits with headroom") was computed for 8 KB packets and
>    does not transfer to the 3.4 KB zc geometry. **`direct_rx=off`
>    (readStream, 64 Ki-sample chunks — per-packet work amortized in the
>    plugin's tight internal loop; still sample-exact via rx_gap_break)
>    removes the whole deficit: 2 s at max rate = 99.989 % on the STOCK
>    256 MB ring.** The extra staging copy is cheaper than 2.32 M/s of
>    API-call overhead — at this quantum the "zero-copy" rung inverts.
> 2. **The 256 MB UMEM (= driver ring) is only a 28 ms absorb window.**
>    At a ~20 % deficit it fills from empty in ~140 ms — the loss was a
>    ring-occupancy **sawtooth** (0 %-loss stretches of 145–162 ms
>    alternating with 50–65 %-loss churn at 100 ms scale), not a steady
>    22 %. Depth lever = `ring_bytes`: direct-path 2 s with
>    `ring_bytes=4 GiB` captured 99.983 % (whole deficit absorbed).
>    Depth masks the deficit for bounded windows; it does not fix rate.
> 3. **Starvation-exit dead air = the worker's 50 ms poll timeout.** Max
>    gap 50.2–50.5 ms in EVERY lossy run: once the fill ring hits zero
>    the NIC delivers nothing, so the xsk worker sleeps its full
>    `recv_timeout_ms=50` with nothing to wake it while app releases
>    accumulate unposted. `[→ propose SH ticket]` — kick/short-poll on
>    starvation exit; expose `recv_timeout_ms`.
>
> Ruled out by A/B at 256 MB + direct: worker moved off the NAPI core
> (19.4 %), + app confined to fast cores 15–17 (20.9 %) — placement is
> hygiene, not the cause; memory bandwidth (the two-copy readStream path
> sustains 62.9 Gbps against the same concurrent HDF5 writer). Rig-B
> fact behind the hygiene: **mlx5_comp N → CPU N identity IRQ mapping**,
> so the old `cpu_affinity=19` put the SCHED_FIFO-50 worker on q19's own
> NAPI core — prefer 18. Diagnostic fingerprints for the future: modal
> clean-run of exactly 64 packets during churn (NAPI budget slivers) and
> the 50 ms max-gap cap — `CC/Sounder/tools/gap_forensics.py` extracts
> all of them from a capture's `/Data/Gaps`.
>
> **V3 validated the full demo:** 8 s, readStream + `rx_xsk=require` +
> `ring_bytes=1 GiB` + worker on 18 (app `taskset -c 15,16,17`):
> 15000/15000 slots, ONE 0.166 ms gap 1.3 ms into the capture =
> **99.9979 % captured**, NIC-ledger exact (`rx_out_of_buffer` +385 =
> 385 × 848). Without any pinning (worker 19, no taskset): 99.987 %.
> The demo config now ships this recipe.

SoapySDR sanctions zero-copy reads: `getNumDirectAccessBuffers` /
`getDirectAccessBufferAddrs` / `acquireReadBuffer` / `releaseReadBuffer`
(base class defaults to unsupported). **The Houdini host plugin's ring is
already shaped for it**: `stream/ring_buffer.h` is an SPSC power-of-two
slot ring with publish/release semantics and per-slot metadata
(tick, flags, payload length) — `Read()` is internally built on exactly
the primitives `acquire`/`release` would expose. Implementing the API is
close to a refactor of `Read()`'s slot-arm loop into public surface:
acquire = consumer snapshot → slot payload pointer (past the 16-B
header) + per-slot `timeNs`/flags; release = `ring_.Release(1)`.
`[→ propose SH ticket]` — plumbing is the software lane's design.

What the app gains (rx-recorder consumer path, readStream fallback kept):

- **Copy (2) and the staging copy (3) collapse into one assembly copy**
  (ring slot → HDF5 row at the grid offset), because assembly is also
  where AP-2's placeholder-zero insertion happens naturally.
- **Sample-exact gap accounting without any contract change**: one
  acquire = one packet = the atomic continuity unit, each with its own
  hardware tick. The app compares per-packet ticks directly — exactness
  no longer depends on the break-at-gap readStream behavior (which
  remains right for generic readStream consumers; the two driver asks
  touch the same slot-arm code and could land as one work package).
- Granularity check: ~8 KB packets ⇒ ~975 k acquires/s at max rate,
  ~1 µs budget each; an acquire is a couple of atomics and the 8 KB
  assembly memcpy is ~200-400 ns — fits with headroom. Slots are
  released immediately after assembly (≤1 held), so worker/overflow
  behavior is unchanged.

## Single-socket drop diagnosis (measured 2026-07-10, rig B)

- **Every kernel drop is a socket-buffer-full event**: `netstat -su` shows
  packet receive errors == receive buffer errors (97M+). No backlog/softirq
  drops. The wall is the DRAIN of the socket buffer (the plugin worker's
  recvmmsg chain — dominated by the kernel->user copy on the worker's
  core), not kernel protocol processing.
- **`net.core.rmem_max` on .64 is 256 MB** — half the value HOST_TUNING.md
  prescribes, so the plugin's 512 MB SO_RCVBUF request is silently clamped.
- **Buffer size buys a lossless WINDOW, not sustained rate**: window =
  size / (offered - drained) ~= 180 ms at 512 MB and today's ~22.9 Gbps
  deficit; 4 GB ~= 1.4 s. Bursts shorter than the window are lossless
  TODAY — the duty-cycle capture strategy.
- **Rig-B core map**: cores 15-18 @ 3.98 GHz, 19 @ 4.0 GHz, 5-9 @ 3.9 GHz,
  0-4/10-14 @ ~2.8 GHz. Worker/IRQ placement across the fast cores is the
  first experiment (`tools/rx_drop_probe.sh` is the harness; the plugin's
  `cpu_affinity` stream kwarg pins the worker, no root needed).

### Experiment plan T4 (root one-liners, [user])

```sh
# larger clamp for the plugin's SO_RCVBUF request (window x4):
sudo sysctl -w net.core.rmem_max=2147483647
# let the unprivileged worker actually get SCHED_FIFO (verify first with
# ps -eLo psr,policy,rtprio,comm during a run — the plugin requests RT 50
# but needs this to succeed without CAP_SYS_NICE):
echo 'houdini - rtprio 50' | sudo tee -a /etc/security/limits.conf
# pin the data-NIC queue IRQs away from the worker's core (find them via
# grep <iface-driver> /proc/interrupts), e.g. queue IRQ N -> core 17:
# echo 20000 | sudo tee /proc/irq/N/smp_affinity
```

Run `tools/rx_drop_probe.sh files/rx-record-max.json --duration 1
--worker-cpus 19 --app-cpu 18` before/after each change: it reports the
exact kernel delivered/dropped delta per run.

### The rung between today and AF_XDP (driver-side, software lane)

> **STATUS 2026-07-12:** AF_XDP itself landed (SH-258) before this rung
> was picked up, so the rung is now the **socket-fallback** improvement
> path only (rigs without the caps bundle / paired MTU, `rx_xsk=off|auto`
> fallback). Busy-poll and UDP_GRO remain valid there; port-striped
> multi-socket is likely mooted (the multi-queue answer is xsk now).
> Handoff stands at reduced priority.

`[-> propose SH ticket]` — with the diagnosis above, the cheap
driver-side lifts before AF_XDP, all in the host module's socket setup
(`stream/udp_comm.cc`):

- `SO_BUSY_POLL` / `SO_PREFER_BUSY_POLL` (+ `napi_defer_hard_irqs` /
  `gro_flush_timeout` sysctls): the consumer core drives NAPI directly —
  the classic single-flow UDP win.
- `UDP_GRO`: kernel coalesces ~8 KB datagrams into ~64 KB segments,
  dividing per-packet stack cost ~8x (recvmmsg + cmsg segment sizes).
- **Port-striped multi-socket** (needs HS for the fh_core side): device
  alternates dst ports across N sockets -> N independent RSS queues,
  softirq cores, and workers. Matches the observed multi-socket aggregate
  >= 70 Gbps; the structural single-node answer short of AF_XDP.

## Ruled out / deferred

- **App-donated recv buffers** (app allocates the memory `recvmmsg`
  fills): no standard SoapySDR surface — it would be a bespoke
  shared-memory kwarg contract, coupling app and plugin for the same
  copy count the sanctioned borrow (acquire/release) already achieves.
- **moodycamel queue in the data plane**: the plugin ring is SPSC —
  the hand-rolled power-of-two ring is the cheaper, already-correct
  primitive there; an MPMC queue adds cost for no need. At the app layer
  the queue helpers already do the handle-passing job (`Event_data`
  carries buffer + offset through `RecorderThread`, samples are never
  copied by the queue). No change wanted.
- **AF_XDP / io_uring registered buffers / DPDK** (kill copy (1), spread
  load across queues): the real kitchen sink. Deferred until a max-rate
  measurement shows the kernel→user copy or single-socket recv is the
  actual wall — likely eventually true for *sustained* 62.9 Gbps, but
  RAM-window captures may land first without it. Escalate on evidence,
  not anticipation. **RESOLVED 2026-07-12: the measured 35 % single-socket
  wall was that evidence — the software lane shipped AF_XDP zero-copy
  (SH-258) on their own initiative; consumed via the `rx_xsk` stream
  kwarg with no app-side code (the consumer surface is unchanged on both
  ingest paths).**

## Measured results (2026-07-10, rig B, fpga v1.19 / device 2695498b)

The step-2 max-rate RAM-window attempt ran END-TO-END at **1.96608 GSPS**
(8 s, 15000 x 1M-sample slots, 64 GiB ring, ~59 GiB file; repeated 2 s
run confirms). Findings:

- **The RX_FAB_CLK walk works as designed** — three doublings
  (30.72 -> 245.76 MHz), and `listSampleRates` DOES rescale with the
  fabric clock (advertised top tracked 245.76 -> 491.52 -> 983.04 MSPS),
  so the coverage check is live, not static. Fresh `make()` resets the
  clock (documented; observed — each run starts at the 245.76 default).
- **Loss is steady-state ~35 %, NOT startup-heavy**: flat per-second
  profile across all 8 s => sustained single-socket goodput ~= **40 Gbps**
  of the 62.9 offered. Extents: ~84k gaps, nearly all > 4096 samples
  (mean ~67k, max 13.3M ~= 6.8 ms).
- **Zero host-ring drops, zero plugin-ring overflows, zero readStream
  overflows**: the app + plugin drained everything the kernel delivered.
  The wall is the kernel-socket recv path, upstream of the plugin —
  confirming the ladder: AP-4 (fewer copies) buys CPU headroom, but
  sustained 62.9 Gbps needs the AF_XDP/multi-queue tier.
- **Grid tolerance was rate-corrected**: integer-ns `timeNs` quantization
  (0.51 ns/sample at this rate) produced 5434 false backward-jump events
  under the original +-1-sample tolerance; now ceil(2 ns x rate) — re-run
  shows zero. Unit-tested.
- **Observation for the fpga/software lanes**: RFDC `ADC0.1` interrupt
  storm — `IntrStatus=0x8000000F [FIFOUSRDAT_OF/UF FIFOMRGNIND_OF/UF
  FIFO_OVR]`, ~500k interrupts in one session, log lines interleaved
  with the RX_FAB_CLK walk phase (before capture start). Our capture
  channel (ADC0.0) was unaffected — loss fully accounted by socket
  drops. **Untraced hypothesis [user]:** the storm correlates with the
  clock change: tiles power up at TILE granularity, so the sibling
  block ADC0.1 is live through the MMCM relock + DynamicPLLConfig
  retune with nobody draining or resetting its FIFO — it rides the
  clock change "confused". For the software lane's eyes alongside the
  SH tickets (AP-2/AP-4 handoffs).

Additional validated points (2026-07-10, symmetric-walk build):

- **491.52 MSPS (one clock up-step): 7500/7500 slots, ZERO gaps** — a
  fully clean rung beyond the default ladder (~2 GB/s to disk sustained).
- **The clock menu is exact**: `fabric = PL_CLK/2^code`, menu
  15.36/30.72/61.44/122.88/245.76 MHz (device fail-loud names it).
- **`listSampleRates` after a clock move advertises ONLY the effective
  rate** (fabric x vld) — the decim rungs exist at the default clock
  only. Consequence: sub-30.72 MSPS rates are genuinely not offered
  (15.36 MSPS probe: the walk stepped 30.72 -> 15.36 MHz, span collapsed
  to {122.88}, next halving rejected at the floor — clean fail-loud,
  device untouched).

Conclusion: **AP-3 capture support is DONE and honest at max rate** —
the file stays time-true with exact accounting under 35 % loss. Lossless
sustained 62.9 Gbps is gated on driver-side work (AP-4 and beyond).

## Order of operations

1. **[DONE — AP-1 validation]** Live rig validation at ladder rates — includes `fio`
   disk-sustain measurement and HDF5 writer throughput check
   (single-threaded writer needs ~1 GB/s at the 245.76 MSPS rung;
   first lever if short: bigger slots = fewer, larger H5 writes).
2. **[DONE — results above]** Max-rate RAM-window capture attempt with the *current* readStream
   path (measure, don't assume: the copy chain may already hold for
   8-10 s windows — 3 copies ≈ 47 GB/s of ~273 GB/s).
3. **[DONE driver + app 2026-07-12 — validation staged]** Direct-buffer
   API (AP-4 → SH-254, landed) + rx-recorder acquire/release consumer
   (implemented, radio-free-tested; silicon validation below).
4. **[DONE driver-side — SH-258]** AF_XDP zero-copy ingest, consumed via
   `rx_xsk` + the `HOUDINI_MTU=3512` paired geometry (demo config).

## Validation matrix (next quiet rig window) {#validation}

Ship the arc to the rig worktree first (bundle procedure), rebuild
`rx-recorder`, and re-run the radio-free ctest suite on aarch64. Then,
in order — each run inspected with `tools/inspect_rx_record.py` and,
for loss runs, bracketed by `tools/rx_drop_probe.sh` deltas:

- **V1 (no root): SH-254 consumer regression on the socket path.**
  Default geometry, `direct_rx` auto (expect `READ_PATH=direct`,
  `GAPS_EXACT=1`) at 245.76 and 491.52 MSPS (the known-clean rungs):
  expect `TOTAL_UNTRUSTED_SAMPLES=0`, byte-identical trust reports vs a
  `direct_rx=off` control run.
- **V2 (no root): exact-extent validation at a lossy rate** (the staged
  AP-2 tail, now two independent exact paths): max-rate socket-path
  capture ~1 s; extents must equal pads (`inspect_rx_record.py` trust
  report + gapstats-style sum check); compare `direct_rx=off` (SH-253
  break-at-gap exactness) vs `direct_rx=require` (per-packet exactness)
  — the two gap tables should agree to the detection-resolution bound.
- **V3 ([user] root, coordinate the NIC MTU with the correlator work —
  3498 breaks the default-8192 geometry while set):** provision the caps
  bundle on `rx-recorder` + `ip link set <data-iface> mtu 3498`, then
  `files/rx-record-demo.json` (rx_xsk=require + direct_rx=require, 8 s):
  target near-lossless (driver leg measured 99.92 % captured; every
  residual drop must appear in `/Data/Gaps`).
- **V4: drop-probe matrix** at max rate — {socket, xsk} x {readstream,
  direct} x worker `cpu_affinity` {default, 19} (+ the staged T4 root
  one-liners for the socket legs): kernel delivered/dropped deltas and
  app CPU. Decides the demo config's final affinity numbers and whether
  the socket-fallback rung (busy-poll/GRO) is still worth an SH ask.
- **V5: AP-5 tone verification** (`tx_replay` section implemented;
  prereq: confirm rig-B DAC→ADC loopback cabling): first
  `files/rx-record-tone.json` at the clean 245.76 MSPS rung
  (`--tone auto` verdict clean on a gap-free capture), then a demo-rate
  capture with the tone — slot-boundary phase continuity cross-checks
  the gap table under real loss.

### Results — 2026-07-12 rig B (TDD baseline fpga v1.20 / v0.2.0)

Ran on `arc/rx-recorder` @ `6f392c9`, worktree rebuilt from the bundle;
4/4 radio-free ctest green on aarch64. Records (evidence — not deleted
if later superseded):

- **V1 — DONE (clean rungs).** 245.76 MSPS 2 s, `direct_rx=auto`:
  7500/7500, `TOTAL_UNTRUSTED_SAMPLES=0`, `READ_PATH=direct`,
  `GAPS_EXACT=1`. Note the direct-buffer path engaged even on the
  **kernel-socket** ingest (rx_xsk auto→socket at the default 8192
  geometry) — the SH-254 and SH-258 paths compose. 491.52-rung control
  not re-run this session.
- **V5 — DONE (clean-rung leg).** `files/rx-record-tone.json` at
  245.76 MSPS, tone armed on the capture's own handle (19.92 MHz
  bin-snapped from 20 MHz), `inspect --tone auto`: detected −19.92 MHz
  @ 87.9 dB, 7499 boundaries, **0 unexplained phase jumps**, verdict
  clean (exit 0). First on-silicon exercise of the AP-5 arm path. The
  demo-rate (lossy) tone leg was not run.
- **ZC max-rate (the demo config) — CHAIN OK, NOT LOSSLESS.** require-mode
  AF_XDP + direct at 1.96608 GSPS, `rx_xsk_nic_reconcile=1` (driver set
  NIC 9000→3498, set-and-leave). Zero-copy engaged (q19, 4 KiB chunks);
  **~22–25 % loss, reproducible** (969.0 / 861.6 / 894.0 M samples over
  3× 2 s), steady-state (s0 27.5 % / s1 21.8 %), cause = NIC fill-ring
  starvation (`RX ring full — hardware dropping`). `taskset -c 15-19` did
  NOT help ⇒ not app-thread placement. All loss exactly in `/Data/Gaps`,
  cross-verified. Confirmed the require-mode fail-loud: the same config at
  the default NIC 9000 (no reconcile) refuses the XDP attach and aborts
  without touching the NIC.
- **V2 / V3-demo-8s / V4 matrix — NOT RUN.** V4 (UMEM depth, writer/ring
  drain coupling, memory-BW) is now the priority: it owns the max-rate
  consumer-loss root cause above, ahead of the socket-fallback SH ask.
  *(V4 + V3 completed later the same day — records below.)*

### Results — 2026-07-12 later session (V4 root cause + V3)

Same TDD baseline, same worktree/binary. Full causal analysis in the V4
STATUS block above; run ledger (all 1.96608 GSPS, loss cross-checked
against `ethtool rx_out_of_buffer` deltas):

- **V4 — DONE (root-caused).** Gap-table forensics on the three lossy
  captures: 848-sample packet quantum; bimodal loss (0 % stretches of
  145–162 ms = the 256 MB ring filling at the deficit, alternating with
  50–65 % churn); modal clean-run exactly 64 packets (NAPI budget);
  max gap = the 50 ms worker poll timeout in every run. Config-only
  A/B ladder (2 s runs): baseline direct+256 MB **21.9–24.6 %** loss →
  worker `cpu_affinity=18` (off the q19 NAPI core; rig-B maps
  mlx5_comp N→CPU N) **19.4 %** → + app `taskset -c 15,16,17`
  **20.9 %** (placement ruled out) → direct + `ring_bytes=4 GiB`
  **0.017 %** (99.983 %, deficit absorbed) → **`direct_rx=off`, stock
  256 MB ring: 0.011 %** (99.989 %) — the per-packet direct-path
  consumer was the deficit. Residual gaps in clean runs sit in the
  first ~50 ms (start transient).
- **V3 — DONE.** Full 8 s demo recipe (readStream + xsk +
  `ring_bytes=1 GiB`, worker 18, app 15–17): 15000/15000 slots, one
  0.166 ms gap at 1.3 ms = **99.9979 %**; NIC ledger exact (+385 pkts
  × 848). Shipped-shape control (worker 19, no taskset, 2 s):
  99.987 %. Evidence files kept in rig `~/rx_logs`:
  `rx_record_20260712_180113.h5` (2 s readStream clean),
  `rx_record_20260712_180254.h5` (8 s V3).
- **V2 — residual (deprioritized).** Both exact-extent paths now have
  clean and lossy validated legs; the same-run direct-vs-readstream
  gap-table cross-compare remains unrun.
- **Rig left:** NIC `enp1s0f1np1` at **MTU 3498** (reconcile set-and-leave;
  restore `sudo ip link set enp1s0f1np1 mtu 9000` before the correlator's
  8192 geometry runs). rmem_max still 256 MB (moot on the xsk path).
