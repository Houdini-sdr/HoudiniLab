# RX sample-loss awareness — evidence and the cross-lane ask

Application-lane analysis (AP-2, 2026-07-10) of how sample loss surfaces
through the SoapyHoudiniSDR host plugin, what the HoudiniLab `rx-recorder`
now does about it, and what only the driver can fix. The driver half is a
`[→ propose SH ticket]` handoff — this doc is the evidence package; the
software lane owns the design.

## Observed driver behavior (evidence, not a fix directive)

Read from `SoapyHoudiniSDR/host/stream/houdini_stream.cc` (`Read()`,
~lines 874-1000) and the worker ingest path, at `develop` as of this
analysis:

1. **The ground truth exists per packet.** Every RX datagram's framer64
   header carries a 64-bit `tickCount`; the ingest worker parses it into
   each ring slot's metadata (`m.tick_count`). Samples *within* one packet
   are contiguous by construction (beat-aligned `frame_words`), so the
   packet is the atomic unit of time continuity — a gap can only exist
   *between* packets.
2. **`Read()` stamps time only for the buffer's first sample** — exact at
   a slot head, correctly extrapolated when resuming mid-slot (the code is
   careful about this, including refusing to extrapolate without a sample
   rate).
3. **`Read()` concatenates consecutive ring slots with no continuity
   check.** When it drains one slot and arms the next, it does not compare
   the new slot's `tick_count` against the previous slot's tick + samples
   delivered. Two packets that were never adjacent on the air are spliced
   adjacently into the user buffer.

Consequences, in increasing order of severity:

- A **kernel-socket drop** (`SO_RCVBUF` exhaustion, before `recvmmsg`) is
  invisible to every plugin counter — no overflow, no status event. Its
  only artifact is the tick discontinuity between the packets that *were*
  received. (Known behavior: the HIL suite Row 9 recovers loss rate from
  per-read `timeNs` gaps; ~10-15 % kernel datagram drops at sustained
  full rate per the host README.)
- A **host-ring drop** (drop-newest when the userspace ring is full) is
  counted and surfaces an OVERFLOW status event, but the stored stream is
  spliced the same way.
- Because the splice is silent, a `readStream` buffer that contains a
  mid-buffer gap presents *all samples after the gap at the wrong time*
  under the buffer's single leading `timeNs`. An application tracking
  per-read timestamps (as `rx-recorder` does) detects the loss only at the
  **next read boundary** and cannot locate it more precisely than "somewhere
  in the previous read". An application that doesn't track timestamps gets
  silently time-shifted data. For timing-sensitive consumers (channel
  sounding) this is data corruption, not just loss.

## The design tension (the software lane's problem to solve)

The ingest worker's job is to drain the kernel socket at line rate
(`recvmmsg` batch 32, RT priority) — gap awareness must not slow that
down. The evidence suggests it doesn't have to: the header (including
`tickCount`) is **already parsed per packet at ingest**, and `Read()`
already walks per-slot metadata, so a continuity check is one
predicted-tick compare against data already in cache. Two placements both
appear compatible with keeping the ring as pure kernel-offload:

- **Read-side (zero ingest cost):** when `Read()` arms the next slot with
  `bytes_written > 0`, compare the slot's tick against expected
  (prev tick + delivered × ticks-per-sample); on mismatch return short
  (break-at-gap). The next read then stamps the gap's far edge exactly,
  from the header. Every returned buffer becomes time-contiguous **by
  contract**, and gap extents are exactly computable host-side from
  consecutive read timestamps — no new API surface.
- **Ingest-side:** compute `gap_before` at ingest and store it in slot
  metadata; `Read()` consumes it (same short-read behavior) and/or a
  status event carries the extent. Slightly more worker arithmetic;
  enables reporting even for gaps the app never reads past.

Notes for the ticket, from the app side:

- **Break-at-gap is the lossless channel.** `readStreamStatus` events ride
  a 256-deep drop-oldest FIFO — fine as garnish, lossy under exactly the
  drop storms that matter.
- **Advertise the capability** (e.g. a `getHardwareInfo` kwarg
  `rx_gap_break=1`). `rx-recorder` already probes this key: with it,
  recorded untrusted extents are sample-exact; without it they are widened
  by one read span. Apps can then distinguish exact from conservative
  accounting without version sniffing.
- Suggested validation shape: induce loss (shrink `rx_buf_bytes` / raise
  rate), assert every returned buffer is internally tick-contiguous
  (`debug_headers` cross-check) and that the sum of read-boundary deltas
  equals the induced loss; regression-guard the clean path.

## What the application does today (landed with AP-2)

`CC/Sounder/rx-recorder` (see `CC/Sounder/RX_RECORDER.md`):

- Anchors a sample-time grid at the first stamped read
  (`FIRST_SAMPLE_TIME_NS`): the file promises sample `k` lives at
  `t0 + k/RATE`.
- At every stamped read, compares the emit position against the timestamp;
  a forward jump inserts that many **placeholder zeros** before the read's
  samples, so one gap cannot time-shift the remainder of the file.
- Records every untrusted region in a `/Data/Gaps` table
  `{start_sample, n_samples, start_time_ns, cause}` covering stream gaps,
  host-ring drops, and HDF5 write errors, plus a `TOTAL_UNTRUSTED_SAMPLES`
  attribute.
- Precision: the driver shipped break-at-gap + the `rx_gap_break=1`
  capability (SH-253, observed landed 2026-07-10) and later the SH-254
  direct-buffer path (per-packet stamps) — on current drivers extents
  are sample-exact via either path, with the read-widened accounting
  kept as the older-driver fallback. Current behavior is documented in
  `CC/Sounder/RX_RECORDER.md` (drop accounting); this doc remains the
  evidence package behind the contract.
