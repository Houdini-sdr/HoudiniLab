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
| Hi | AP-1 | **UE fine-grained timed TX for the sounder closed loop.** BS native-TDD receive done (records the gated pilot non-zero); the UE needs to place its pilot in the BS `rx_gate` (0.5 ms symbol, 33 µs recorded, ≤0.5 ms tolerance) at a beacon-referenced time without the whole-ms 1 ms drift-cliff. **RESOLVED by SH-301 (relay): the capability already exists — streaming TX accepts the 3.125 µs TDD grid via the `tdd=1` stream arg (SH-248, in the deployed driver); no driver change needed.** My hand-off premise was partly stale (mis-cited the device replay path `:1187` / `setHardwareTime` `:1846`; the streaming timing is the host `TxTickAnchor`, missed). **Remaining = app-side:** set `tdd=1` on the UE `tx_stream_args` in `ClientRadioSet` (streaming mode; drop the method-1 replay-strobe changes), then run the full sounder closed loop (UE pilot on the 3.125 µs grid → BS gate). See `docs/UE_TX_FINE_GRID_TIMING.md` (RESOLUTION at top). ⇄ SH-301, SH-248. | app-side, open |

## Closed

Landed rows collapse here (one line; the verbose body stays in git):

- _(none yet)_
