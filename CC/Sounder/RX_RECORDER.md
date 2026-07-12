# rx-recorder — timed RX capture to HDF5

A lean CC/Sounder tool for the Houdini SDR: open the device, start a
continuous RX stream **now**, record a fixed duration of raw CS16 samples
to an HDF5 file, report drops. No beacons, no schedules, no TDD — this is
the plain receive path, which is the validated Houdini path today
(single-channel continuous RX; see the SoapyHoudiniSDR host README).

## Run

```sh
./build/rx-recorder -conf_file files/rx-record.json -storepath logs
```

## Live node (`files/rx-record-live.json`)

On a rig host the Soapy stack lives in the `houdini_test` venv — activate
it and pin the module path first (see the SoapyHoudiniSDR host README):

```sh
source ~/houdini_test/bin/activate
export SOAPY_SDR_PLUGIN_PATH=$VIRTUAL_ENV/lib/SoapySDR/modules0.8-3
./build/rx-recorder -conf_file files/rx-record-live.json
```

Live-node config facts (deployed fine-I/Q bitstream, fpga >= 1.8):

- `"remote": "<node ip>"` in `device` pins the node explicitly — with two
  RFSoCs on the bench network, discovery alone is ambiguous.
- `rate` selects a rung of the RX runtime ladder
  **{30.72, 61.44, 122.88, 245.76} MSPS** (SH-175); the device applies it
  on a closed stream, which is why rx-recorder sets it before
  `setupStream`. Disk rate is 4 bytes/sample: 30.72 MSPS ≈ 123 MB/s,
  122.88 ≈ 492 MB/s, 245.76 ≈ 983 MB/s (the last is the known
  single-socket kernel-drop regime — watch the time-gap report).
- `freq` tunes the per-block fine NCO (range ±Fs/2); the mixer conjugates
  the frequency axis (+RF → −baseband) — a convention, not a bug.
- `gain` / `antenna` are omitted: they are only applied when present, and
  the RFDC path exposes no Iris-style gain stages.
- The host needs `net.core.rmem_max >= 536870912` (the plugin's SO_RCVBUF
  request); `setupStream` warns when the grant is short.

## Max-rate RAM-window capture (`files/rx-record-max.json`)

`"rate": 1966080000` (1.96608 GSPS CS16 ~= 62.9 Gbps) is beyond the
standard ladder: rx-recorder walks `RX_FAB_CLK` 30.72 -> 245.76 MHz
(effective rate = fabric x vld, vld = 8 on the deployed bitstream). At
this rate the disk is NOT in the capture path (~7.86 GB/s demand): the
config sizes the host ring as the capture window — 16384 slots x 1 M
samples = 64 GiB of RAM, an ~8 s window — and the HDF5 writer drains it
during + after the capture ("draining queued slots..." until the summary
prints). Budgets + design + measured results: `../../docs/RX_MAX_RATE.md`.
Measured at this rate over the kernel-socket path: ~35 % steady-state
loss (single UDP socket sustains ~40 Gbps) — the file stays time-true
with the loss exactly accounted in `/Data/Gaps`. The zero-copy demo
config below reaches **99.998 %** at this rate (validated 8 s,
2026-07-12) — use it for max-rate work.

## Zero-copy demo capture (`files/rx-record-demo.json`) — 1 ch @ 1.96608 GSPS

The releasable single-channel max-rate configuration: the driver's
AF_XDP zero-copy ingest (SH-258) consumed through `readStream`.
Validated 2026-07-12 (TDD baseline): **8 s at 1.96608 GSPS =
99.9979 % captured** (one 0.166 ms start-transient gap), NIC-ledger
exact. The pieces:

- `"HOUDINI_MTU": 3512` (device make-arg, SH-259) — the paired wire
  geometry. Zero-copy needs the whole frame inside one 4 KiB UMEM chunk
  (wire <= 3840 bytes); the default 8192 geometry cannot engage it.
- `"rx_xsk": "require"` (stream kwarg, SH-258) — AF_XDP zero-copy
  ingest, NIC DMA directly into the driver ring (`auto` falls back to
  the kernel socket with one INFO line; `off` never probes).
- `"direct_rx": "off"` (rx-recorder key) — **deliberate at max rate.**
  The paired geometry carries only 848 samples/packet ⇒ 2.32 M
  acquires/s; the per-packet acquire/release consumer sustains ~78 %
  of wire and loses 20+ % (measured), while `readStream` amortizes the
  per-packet work inside the plugin's tight loop and keeps up with
  headroom. Extents stay sample-exact via the driver's break-at-gap
  contract (`rx_gap_break`). Keep `direct_rx` auto/require at ladder
  rates, where the per-packet budget is ample (V1) and per-packet
  stamps are structurally exact.
- `"ring_bytes": 1073741824` (stream kwarg) — deepens the UMEM/driver
  ring from the 256 MB default (a 28 ms absorb window at max rate) to
  ~112 ms, riding out scheduler/writer jitter.
- `cpu_affinity` / `rt_priority` — pin the driver's recv worker to a
  fast core **that does not own the NIC queue's IRQ**: rig B maps
  mlx5_comp N → CPU N and the plugin engages q19, so core 19 belongs
  to q19's NAPI — pin the worker to 18 (SCHED_FIFO via the capability
  bundle below). Optionally confine the app to other fast cores
  (`taskset -c 15,16,17`); measured worth ~0.01 % — hygiene.

Host prerequisites (one-time provisioning, root — see
`SoapyHoudiniSDR/docs/RX_XSK_INGEST.md`):

```sh
# capability bundle on the binary that loads the plugin:
sudo setcap cap_net_raw,cap_net_admin,cap_bpf,cap_ipc_lock,cap_sys_nice+ep \
    ./build/rx-recorder
# data-NIC MTU must cover the paired wire frame (3512 - 14):
sudo ip link set <data-iface> mtu 3498
```

Caveats: file capabilities trigger glibc secure-exec, which ignores
`LD_LIBRARY_PATH`/`LD_PRELOAD` — the plugin is found via
`SOAPY_SDR_PLUGIN_PATH` (a plain getenv), which the run recipe above
already sets. Do NOT add `rx_xsk_nic_reconcile` on a shared rig: it
would silently move the NIC MTU under every other tool using the
default 8192 geometry (the correlator work runs at MTU 9000).

History: the first on-silicon runs of this config (with
`direct_rx=require`) lost ~22–25 % — root-caused the same day to the
per-packet acquire/release consumer at the 848-sample zc quantum plus
the 256 MB ring's short absorb window, with a 50 ms starvation-exit
stall from the worker's poll timeout (NOT core placement, NOT memory
bandwidth). The recipe above closes it. Full causal record, the A/B
ladder, and the validation matrix: `../../docs/RX_MAX_RATE.md`
§validation.

## Config (`files/rx-record.json`)

| Key | Default | Meaning |
| --- | --- | --- |
| `device` | `{"driver": "houdinisdr"}` | `Device::make()` kwargs. Add `serial`, `remote`, `HOUDINI_MTU` (wire geometry, SH-259), etc. here. |
| `stream` | `{}` | `setupStream()` kwargs forwarded verbatim (`ring_bytes`, `cpu_affinity`, `rt_priority`, `rx_xsk`, ...). Recorded in the file's `STREAM_ARGS`. |
| `channels` | `[0]` | RX channel. Exactly one for now (the combined multi-channel readStream merge is WIP device-side, SH-142/SH-159). |
| `rate` | `0` | Requested sample rate in Hz; `0` keeps the device rate. A rate the device already advertises goes through `setSampleRate`; a rate **outside** the advertised set makes rx-recorder step `RX_FAB_CLK` (doubling toward faster, halving toward slower; device fail-loud = ceiling and floor) until the request is covered, then select it. The file records the **actual** rate read back. |
| `freq` | — | RF tune in Hz (fine NCO on Houdini). Only applied when present. |
| `gain` | — | RX gain in dB. Only applied when present. |
| `antenna` | — | Antenna name. Only applied when present. |
| `duration_sec` | `1.0` | Capture length; converted to slots with the actual rate. |
| `samps_per_slot` | `65536` | CS16 samples per slot (= one HDF5 row). |
| `read_chunk_samps` | `16384` | Max samples per `readStream` call; bounds how precisely a stream gap is located (see drop accounting). |
| `buffer_slots` | `512` | Host ring between the RX loop and the HDF5 writer thread. |
| `rx_timeout_us` | `1000000` | Per-read (readStream or acquire) timeout. 10 consecutive timeouts abort. |
| `direct_rx` | `"auto"` | Zero-copy read path (SH-254 `acquireReadBuffer`/`releaseReadBuffer`): `auto` engages it when the driver advertises direct-access buffers, `require` fails loud when it doesn't, `off` forces `readStream`. Same value idiom as the driver's `rx_xsk`. |
| `tx_replay` | absent | Loopback verification tone (AP-5, test-only): `{"freq": <Hz baseband>, "amp": <0..1, default 0.25>, "channel": <0=DAC0.0 / 1=DAC2.0, default 0>, "n_addrs": <loop length <= 4096, default 4096>}`. See the section below. |
| `output_file` | auto | Defaults to `<storepath>/rx_record_<timestamp>.h5`. |

## Loopback verification tone (`files/rx-record-tone.json`) — AP-5, test-only

With a `tx_replay` config section, the capture's **own device handle**
arms a `tx_mode=replay` TX stream before the RX stream opens: the
requested baseband tone is bin-snapped to the `n_addrs` replay-RAM loop
(seamless phase at the wrap), loaded via `writeStream` (one element =
one complex CS16 sample = one replay-RAM word, depth 4096/channel), and
looped continuously by the RTL from `activateStream` until teardown —
the validated SH-124/126 path (`rx_tx_loopback.ipynb`). Single-handle is
load-bearing: the tone's lifetime IS the replay stream's lifetime
(closeStream releases the DAC tile refcount), and a concurrent second
handle is out-of-contract (SH-251).

Prereq: the rig's DAC→ADC loopback cable (DAC0.0 → the ADC feeding RX
channel 0 — confirm rig-B cabling state first). The file records
`TX_REPLAY_FREQ_ACTUAL` (the snapped frequency actually playing),
`TX_REPLAY_AMP`, `TX_REPLAY_CHANNEL`. Verification:

```sh
tools/inspect_rx_record.py capture.h5 --tone auto
```

checks the tone's slot-boundary phase continuity against `/Data/Gaps` —
an unexplained phase jump means samples were lost WITHOUT being
accounted, the exact failure mode AP-2's gap accounting must never have
(exit 1). The RX-observed tone frequency depends on the mixer/NCO
mapping (the fine-I/Q mixer conjugates the frequency axis), so `--tone
auto` detection is the normal mode; the armed DAC-baseband value is
printed for context.

## Output layout

HDF5 group `Data` with two datasets: `Samples`, dims
`{slot, 1, 1, channel, 2*samps_per_slot}` (interleaved I,Q int16 — the
Houdini wire format, HS-25), and `Gaps`, the `(n, 4)` int64
untrusted-extent table (see drop accounting below). Attributes record the
*actual* device state: `FREQ`, `RATE`, `GAIN`, `ANTENNA`, `CHANNELS`,
`SLOT_SAMP_LEN`, `DURATION_SEC_REQUESTED`, `FORMAT`, `HW_KEY`, `HW_INFO`.

Capture-path provenance attributes: `READ_PATH` (`direct` or
`readstream` — which read path actually engaged), `GAPS_EXACT` (int
1/0 — whether `/Data/Gaps` extents are sample-exact or read-widened),
`STREAM_ARGS` (the `setupStream` kwargs as sent, e.g. the `rx_xsk`
request). When a `tx_replay` tone was armed: `TX_REPLAY_FREQ_ACTUAL`
(Hz, bin-snapped), `TX_REPLAY_AMP`, `TX_REPLAY_CHANNEL`.

Capture bookkeeping attributes (written at finalize):

- `SLOTS_RECORDED` — exact number of slot rows written (the dataset shape
  grows in 2000-row windows and overstates the capture).
- `WRITE_ERRORS` — slots lost to HDF5 write failures (0 on a clean run).
- `FIRST_SAMPLE_TIME_NS` — the grid t0: the hardware time of file
  sample 0, back-projected from the first stamped `readStream` (equals
  that read's `timeNs` only when no unstamped reads precede it). Use
  this for time alignment.
- `START_HW_TIME_NS` — `getHardwareTime()` right after stream activation.
  Approximate (excludes in-flight latency); prefer `FIRST_SAMPLE_TIME_NS`.

Both time attributes are strings: int64 ns does not fit the numeric
attribute overloads.

## Drop accounting and the sample-time grid

The file promises a linear time grid: sample `k` lives at
`FIRST_SAMPLE_TIME_NS + k/RATE`. Three loss signals feed it, all reported
in the end-of-run summary AND recorded as extents in `/Data/Gaps`:

- **Stream gaps** — per-read `timeNs` discontinuities vs. the emit
  position. On Houdini this is the *only* signal for kernel-level UDP
  drops (they never appear in overflow counters). A detected forward gap
  inserts that many **placeholder zeros** before the late samples, so one
  gap cannot time-shift the rest of the file. Backward jumps are flagged,
  never padded or subtracted.
  **Detection resolution:** `timeNs` is integer nanoseconds, so drops of
  `<= ceil(2ns x RATE)` samples (1 at ladder rates, 4 at 1.966 GSPS) are
  indistinguishable from stamp rounding — they are absorbed as a bounded
  standing offset and surface (padded in full) once cumulative drift
  exceeds that tolerance.
  **Time-base jumps:** a stamp more than ~10 s off the grid is a hardware
  time-base change (e.g. a concurrent `setHardwareTime`), not loss — the
  grid re-anchors and a `cause=4` (resync) marker records where; absolute
  `FIRST_SAMPLE_TIME_NS`-based times are invalid past that marker.
- **Dropped slots (host)** — the HDF5 writer fell behind and the host ring
  wrapped. The slot's row stays zeroed in the file; grid intact.
- **Write errors** — HDF5 failed a slot write; the row's content is not
  trustworthy.

`/Data/Gaps` is an `(n, 4)` int64 table, columns
`{start_sample, n_samples, start_time_ns, cause}` with cause codes
`0=time_jump, 1=host_ring, 2=write_error, 3=backward, 4=resync` (also in the
`GAP_COLUMNS` attribute). `TOTAL_UNTRUSTED_SAMPLES` (float64) is the union
length for quick screening: trust the capture iff it is 0.

Precision: extents are sample-exact (`GAPS_EXACT=1`) on either of two
independent paths, both auto-probed. On the `direct_rx` path exactness
is structural — one acquire = one wire packet with its own hardware
`tickCount` stamp, so a gap can only lie at a span boundary. On the
`readStream` path it comes from the driver's break-at-gap contract
(SH-253, advertised as `rx_gap_break=1` in `getHardwareInfo`): every
returned buffer is time-contiguous. Against an older driver with
neither, a stream gap is located only to the enclosing `readStream`
call and its extent is widened by up to `read_chunk_samps` (default
16384; smaller = finer localization, more calls) —
`docs/RX_GAP_AWARENESS.md` documents the evidence and the contract.
The schema is identical in all cases.

## Design notes

Reuses the sounder recorder pipeline: `RecorderThread` (event queue +
writer thread) drives an `RxRecorderWorker` through
`RecorderWorkerInterface` — the same interface the full sounder's
`RecorderWorker` implements. The binary deliberately links only
SoapySDR + HDF5 + gflags (no muFFT, no comms-lib, no radio-set classes).

The capture loop itself is a source-agnostic state machine
(`include/rx_recorder_capture.h`): `fillSlot()` assembles slots from a
`SampleSource` — `ReadStreamSource` (staging chunk) or
`DirectBufferSource` (borrowed driver-ring slots, ≤1 held, released the
moment a span drains) — and is unit-tested radio-free against scripted
sources (`tests/rx-recorder/test_capture_paths.cc`: gaps, widening,
resyncs, abort thresholds, both span shapes).
