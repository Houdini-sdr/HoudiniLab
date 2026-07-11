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

## Config (`files/rx-record.json`)

| Key | Default | Meaning |
| --- | --- | --- |
| `device` | `{"driver": "houdinisdr"}` | `Device::make()` kwargs. Add `serial`, etc. here. |
| `stream` | `{}` | `setupStream()` kwargs forwarded verbatim (`ring_bytes`, `cpu_affinity`, ...). |
| `channels` | `[0]` | RX channel. Exactly one for now (the combined multi-channel readStream merge is WIP device-side, SH-142/SH-159). |
| `rate` | `0` | Requested sample rate in Hz; `0` keeps the device rate. The file records the **actual** rate read back. |
| `freq` | — | RF tune in Hz (fine NCO on Houdini). Only applied when present. |
| `gain` | — | RX gain in dB. Only applied when present. |
| `antenna` | — | Antenna name. Only applied when present. |
| `duration_sec` | `1.0` | Capture length; converted to slots with the actual rate. |
| `samps_per_slot` | `65536` | CS16 samples per slot (= one HDF5 row = one readStream fill). |
| `buffer_slots` | `512` | Host ring between the RX loop and the HDF5 writer thread. |
| `rx_timeout_us` | `1000000` | Per-`readStream` timeout. 10 consecutive timeouts abort. |
| `output_file` | auto | Defaults to `<storepath>/rx_record_<timestamp>.h5`. |

## Output layout

HDF5 group `Data`, dataset `Samples` with dims
`{slot, 1, 1, channel, 2*samps_per_slot}` (interleaved I,Q int16 — the
Houdini wire format, HS-25). Attributes record the *actual* device state:
`FREQ`, `RATE`, `GAIN`, `ANTENNA`, `CHANNELS`, `SLOT_SAMP_LEN`,
`DURATION_SEC_REQUESTED`, `FORMAT`, `HW_KEY`, `HW_INFO`.

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

## Drop accounting

Three separate loss signals, all reported in the end-of-run summary:

- **Time-gap events** — per-read `timeNs` discontinuities vs. the sample
  counter. On Houdini this is the *only* reliable signal for kernel-level
  UDP drops (they never appear in overflow counters). Only forward jumps
  count as loss; backward jumps (hardware-time resync) are counted and
  reported separately, never subtracted.
- **Dropped slots (host)** — the HDF5 writer fell behind and the host ring
  wrapped. The slot's row stays zeroed in the file so time alignment is
  preserved.
- **Overflows** — device/ring overflow events from `readStream` /
  `readStreamStatus`.

Known limit: a gap is located to the nearest `readStream` boundary, not
the exact samples — the driver knows more (per-packet `tickCount`) than
the readStream API can express. AP-2 tracks the improvement.

## Design notes

Reuses the sounder recorder pipeline: `RecorderThread` (event queue +
writer thread) drives an `RxRecorderWorker` through
`RecorderWorkerInterface` — the same interface the full sounder's
`RecorderWorker` implements. The binary deliberately links only
SoapySDR + HDF5 + gflags (no muFFT, no comms-lib, no radio-set classes).
