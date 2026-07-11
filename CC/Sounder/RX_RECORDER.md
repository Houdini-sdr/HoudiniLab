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
Measured at this rate: ~35 % steady-state kernel-socket loss (single
UDP socket sustains ~40 Gbps) — the file stays time-true with the loss
exactly accounted in `/Data/Gaps`; lossless max-rate capture is gated
on driver-side transport work (AP-4 and beyond).

## Config (`files/rx-record.json`)

| Key | Default | Meaning |
| --- | --- | --- |
| `device` | `{"driver": "houdinisdr"}` | `Device::make()` kwargs. Add `serial`, etc. here. |
| `stream` | `{}` | `setupStream()` kwargs forwarded verbatim (`ring_bytes`, `cpu_affinity`, ...). |
| `channels` | `[0]` | RX channel. Exactly one for now (the combined multi-channel readStream merge is WIP device-side, SH-142/SH-159). |
| `rate` | `0` | Requested sample rate in Hz; `0` keeps the device rate. A rate the device already advertises goes through `setSampleRate`; a rate **beyond** the advertised set makes rx-recorder step `RX_FAB_CLK` up (doubling per MMCM code, device fail-loud = the ceiling) until the request is covered, then select it. The file records the **actual** rate read back. |
| `freq` | — | RF tune in Hz (fine NCO on Houdini). Only applied when present. |
| `gain` | — | RX gain in dB. Only applied when present. |
| `antenna` | — | Antenna name. Only applied when present. |
| `duration_sec` | `1.0` | Capture length; converted to slots with the actual rate. |
| `samps_per_slot` | `65536` | CS16 samples per slot (= one HDF5 row). |
| `read_chunk_samps` | `16384` | Max samples per `readStream` call; bounds how precisely a stream gap is located (see drop accounting). |
| `buffer_slots` | `512` | Host ring between the RX loop and the HDF5 writer thread. |
| `rx_timeout_us` | `1000000` | Per-`readStream` timeout. 10 consecutive timeouts abort. |
| `output_file` | auto | Defaults to `<storepath>/rx_record_<timestamp>.h5`. |

## Output layout

HDF5 group `Data` with two datasets: `Samples`, dims
`{slot, 1, 1, channel, 2*samps_per_slot}` (interleaved I,Q int16 — the
Houdini wire format, HS-25), and `Gaps`, the `(n, 4)` int64
untrusted-extent table (see drop accounting below). Attributes record the
*actual* device state: `FREQ`, `RATE`, `GAIN`, `ANTENNA`, `CHANNELS`,
`SLOT_SAMP_LEN`, `DURATION_SEC_REQUESTED`, `FORMAT`, `HW_KEY`, `HW_INFO`.

Capture bookkeeping attributes (written at finalize):

- `SLOTS_RECORDED` — exact number of slot rows written (the dataset shape
  grows in 2000-row windows and overstates the capture).
- `WRITE_ERRORS` — slots lost to HDF5 write failures (0 on a clean run).
- `FIRST_SAMPLE_TIME_NS` — the **exact** hardware time of file sample 0:
  the `timeNs` of the first stamped `readStream`. Use this for time
  alignment.
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
  gap cannot time-shift the rest of the file. Backward jumps
  (hardware-time resync) are flagged, never padded or subtracted.
- **Dropped slots (host)** — the HDF5 writer fell behind and the host ring
  wrapped. The slot's row stays zeroed in the file; grid intact.
- **Write errors** — HDF5 failed a slot write; the row's content is not
  trustworthy.

`/Data/Gaps` is an `(n, 4)` int64 table, columns
`{start_sample, n_samples, start_time_ns, cause}` with cause codes
`0=time_jump, 1=host_ring, 2=write_error, 3=backward` (also in the
`GAP_COLUMNS` attribute). `TOTAL_UNTRUSTED_SAMPLES` (float64) is the union
length for quick screening: trust the capture iff it is 0.

Precision: today a stream gap is located to the enclosing `readStream`
call, so its extent is widened by up to `read_chunk_samps` (default 16384;
smaller = finer localization, more calls). The driver knows the exact
missing samples (per-packet `tickCount`) but the readStream API cannot
express mid-read gaps — `docs/RX_GAP_AWARENESS.md` documents the behavior
and the proposed break-at-gap driver contract (AP-2 `[→ propose SH
ticket]`). When the driver ships it (capability kwarg `rx_gap_break=1`,
auto-probed), extents become sample-exact with no schema change.

## Design notes

Reuses the sounder recorder pipeline: `RecorderThread` (event queue +
writer thread) drives an `RxRecorderWorker` through
`RecorderWorkerInterface` — the same interface the full sounder's
`RecorderWorker` implements. The binary deliberately links only
SoapySDR + HDF5 + gflags (no muFFT, no comms-lib, no radio-set classes).
