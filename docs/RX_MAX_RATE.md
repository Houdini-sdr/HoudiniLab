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
  not anticipation.

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
3. Direct-buffer API (AP-4 → SH) + rx-recorder acquire/release consumer.
4. AF_XDP-class work only if (2)/(3) measurements demand it.
