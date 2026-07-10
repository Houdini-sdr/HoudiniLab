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
| P2 | AP-2 | **Sample-drop recording fidelity — discuss, then propose driver support.** [user] Today rx-recorder infers loss from per-`readStream` `timeNs` gaps (count + estimated samples), but the Houdini UDP stream is per-packet timestamped (framer64 `tickCount`), so the host plugin *knows exactly which samples are missing* — the readStream API just cannot express mid-read gaps: it returns contiguous buffers and one leading `timeNs`, and kernel-level drops surface only as a coarser next-read timestamp jump. Discuss what the capture file should record (per-gap `(start_tick, n_samples)` extents? zero-fill with a gap map?) and what the Soapy driver would need to surface (e.g. gap-extent events via `readStreamStatus`, or a break-at-gap readStream contract so each read is gap-free with an exact leading timestamp). Driver-side change belongs to the software lane `[→ propose SH ticket]` after the discussion settles the contract. | OPEN — discuss |

## Closed

Landed rows collapse here (one line; the verbose body stays in git):

- _(none yet)_
