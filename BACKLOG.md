# HoudiniLab BACKLOG — AP-### (RENEWLab application suites)

The application lane's tracker: the Python / MATLAB / C++ Sounder dev suites
and the WEBGUI dashboard, adapted for Houdini SDR. Conventions live in the
houdini-agents repo (`conventions/tracker.md`) — one canonical id per item
everywhere; durable items only; closed rows collapse to a one-line ledger and
point at continuations; failure rows record behaviors only unless a cause was
traced. Cross-lane needs are AP rows tagged for the receiving lane
(`[→ propose SH ticket]` driver, `[→ propose HS ticket]` gateware,
`[→ propose OS ticket]` image, `[→ propose AG ticket]` workspace) — never
written into another lane's tracker.

| Pri | Id | Item | Status |
|---|---|---|---|
| P1 | AP-1 | **rx-recorder: timed RX capture to HDF5 via the Houdini SDR.** New lean CC/Sounder tool (`rx-recorder` binary) that opens the SoapyHoudiniSDR device (`driver=houdinisdr`), applies rate/freq/gain from a JSON config, activates a continuous RX stream NOW, records X seconds of CS16 samples to an HDF5 file, and reports timing gaps/overruns. No beacon/TDD/Iris machinery — single-channel continuous RX is the validated device path. Includes a small refactor: `RecorderThread` decoupled from the sounder `RecorderWorker`/`Config` via a worker interface, so the recorder pipeline is reusable. Branch `feat/rx-recorder`. | IN PROGRESS |
| P2 | AP-2 | **Sample-drop recording fidelity: app side landed; driver break-at-gap proposed.** [user] Investigation confirmed the driver's `Read()` (houdini_stream.cc) splices ring slots across packet-tick discontinuities with no continuity check — kernel-UDP loss silently time-shifts every sample after a mid-read gap under the buffer's single leading `timeNs`; per-packet `tickCount` truth exists in slot metadata but is never compared. Evidence + options: `docs/RX_GAP_AWARENESS.md`. **App side (landed on `feat/rx-recorder`):** sample-time grid anchored at `FIRST_SAMPLE_TIME_NS`, forward gaps repaired with placeholder zeros (grid never drifts), all untrusted regions recorded in `/Data/Gaps` `{start_sample,n_samples,start_time_ns,cause}` + `TOTAL_UNTRUSTED_SAMPLES`; extents read-widened today, auto-tighten to sample-exact via proposed `rx_gap_break=1` capability kwarg. **`[→ propose SH ticket]`** (data-integrity class): break-at-gap readStream contract per the doc — detection is one predicted-tick compare on already-parsed headers, preserving the ring's kernel-offload role. | app DONE / SH handoff OPEN |
| P2 | AP-3 | **Max-rate RX capture: support + test 1.96608 GSPS (~62.9 Gbps).** [user] Goal: rx-recorder sustains the RX max streaming rate — `RX_FAB_CLK` 245.76 MHz × 8 samples/cycle = 1966.08 MSPS CS16 ≈ 62.9 Gbps (vs the 245.76 MSPS top rung of the standard `setSampleRate` ladder). Knowns: sustained ~31.5 Gbps already sees 10–15 % kernel-socket drops on a single UDP socket (SoapyHoudiniSDR README), so 62.9 Gbps needs host-path work (socket strategy is driver-side; app side: RAM-resident ring sizing — disk cannot sustain ~7.9 GB/s, capture-then-drain), and AP-2's gap accounting is the honesty mechanism at any loss rate. Test on rig B (GB10). Multi-channel concurrent capture (2 independent single-channel streams, ports 10001/10002) is a related later extension — pipeline already antenna-indexed. | OPEN |

## Closed

Landed rows collapse here (one line; the verbose body stays in git):

- _(none yet)_
