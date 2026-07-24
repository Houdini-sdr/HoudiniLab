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
| Hi | AP-1 | **UE fine-grained timed TX for the sounder closed loop.** The BS native-TDD receive is done (records the gated pilot slot non-zero), but the UE can't place its pilot inside the ~33 µs BS `rx_gate`: streaming TX is whole-ms-quantized (`kScheduleQuantumNs`), and as the beacon-referenced target drifts across a ms boundary the quantization jumps 1 ms → the pilot leaves the gate (hard cliff, not just coarse). The 1 ms is a software contract, not the FPGA (`TxGridTicks`={8,4,2,1}); a 3.125 µs TDD grid already exists and **RX accepts it** (`streaming.cpp:1538`) but **TX does not** (`:1187`, `:1846`). Need: TDD-scoped fine-grid acceptance on the TX timed path. Evidence + options ruled in/out in `docs/UE_TX_FINE_GRID_TIMING.md`. `[→ propose SH ticket]` | open |

## Closed

Landed rows collapse here (one line; the verbose body stays in git):

- _(none yet)_
