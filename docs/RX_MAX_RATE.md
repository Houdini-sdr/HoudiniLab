# Max-rate RX capture (AP-3/AP-4) — budgets, copy chain, and the zero-copy ladder

Application-lane design notes (2026-07-10) for sustaining the RX maximum
streaming rate — **1966.08 MSPS CS16 ≈ 62.9 Gbps ≈ 7.86 GB/s**
(`RX_FAB_CLK` 245.76 MHz × 8 samples/cycle) — through `rx-recorder` on
rig B. Companion to `docs/RX_GAP_AWARENESS.md` (loss accounting; AP-2).

## Fixed budgets (measured / verified)

- **Rig B (GB10) disk**: 4 TB TLC on a Phison PS5027-E27T (PCIe Gen4,
  **DRAM-less**) — expect ~3-4 GB/s into pSLC cache, roughly
  **1-1.6 GB/s sustained post-cache**. Verify with
  `fio --rw=write --bs=1M --size=100G --direct=1` before trusting long
  captures at the 245.76 MSPS rung (983 MB/s ≈ within 2x of the floor).
  Max rate (7.86 GB/s) exceeds even the drive's burst peak: **disk is not
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
- **Observation for the fpga/software lanes** (behavior only): RFDC
  `ADC0.1` interrupt storm during the high-Fs configuration —
  `IntrStatus=0x8000000F [FIFOUSRDAT_OF/UF FIFOMRGNIND_OF/UF FIFO_OVR]`,
  ~500k interrupts in one session. Our capture channel's data was
  unaffected (loss fully accounted by socket drops), but the storm is
  worth their eyes when the SH tickets get picked up.

Conclusion: **AP-3 capture support is DONE and honest at max rate** —
the file stays time-true with exact accounting under 35 % loss. Lossless
sustained 62.9 Gbps is gated on driver-side work (AP-4 and beyond).

## Order of operations

1. Live rig validation at ladder rates (AP-1 close) — includes `fio`
   disk-sustain measurement and HDF5 writer throughput check
   (single-threaded writer needs ~1 GB/s at the 245.76 MSPS rung;
   first lever if short: bigger slots = fewer, larger H5 writes).
2. Max-rate RAM-window capture attempt with the *current* readStream
   path (measure, don't assume: the copy chain may already hold for
   8-10 s windows — 3 copies ≈ 47 GB/s of ~273 GB/s).
3. Direct-buffer API (AP-4 → SH) + rx-recorder acquire/release consumer.
4. AF_XDP-class work only if (2)/(3) measurements demand it.
