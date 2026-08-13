# loopback_ofdm.py — OFDM closure test over a single-board DAC_B→ADC_D loopback

Self-contained reproduction of the sounder's OFDM signal chain in one script, used to
localize the constellation/equalization blocker (see
`houdini-agents/.../houdini-csi-live-gui.md`). It builds an app-rate frame in replay RAM,
loops it over one board's own `DAC_B → ADC_D` loopback cable, receives it, and runs a full
receiver: **GOLD beacon sync → fine CFO (from the two identical pilots) → LTS channel
estimate → zero-forcing equalize → QPSK constellation + EVM**.

Frame (122.88 MSPS, in replay RAM, looped):

```
[128 GOLD beacon][160 pilot = 2×80 LTS syms][NDATA×80 QPSK data syms][zero pad]
```

## Run (on the DGX)

```bash
source ~/houdini_test/bin/activate
export LD_LIBRARY_PATH=~/houdini_test/lib
export SOAPY_SDR_PLUGIN_PATH=~/houdini_test/lib/SoapySDR/modules0.8-3
pkill -9 -f build/sounder                 # free the boards

python3 loopback_ofdm.py                          # hardware, app-rate replay (strong beacon)
python3 loopback_ofdm.py --rate max               # 8×-upsampled DAC-rate replay
python3 loopback_ofdm.py --selftest               # offline receiver self-test (no radio)
python3 loopback_ofdm.py --html                   # + write a visualization to /tmp/loopback_ofdm.html
```

**Visualization (`--html`):** writes a self-contained HTML page (no external libs — inline
canvas, opens in any browser) with the channel `|H|` (dB), channel phase, and the equalized
constellation vs. ideal QPSK, plus a pass/fail verdict banner. Defaults to
`/tmp/loopback_ofdm.html`; pass a path to override (`--html out.html`). Works with `--selftest`
too (renders the clean simulated reference). View it over SSH with
`scp …:/tmp/loopback_ofdm.html .` or an `ssh -L` tunnel to a local web server.

Useful flags: `--board` (default `168.6.244.22`; bare `.21` RX may return 0 samples),
`--tx-ch`/`--rx-ch` (default `0`/`0` = DAC_B→ADC_D on `.22`), `--nco` (MHz, default 500),
`--ndata`, `--secs`. Add `2>&1 | grep -vE '^\[INFO\]'` to hide driver chatter.

## Interpreting the output

| Signal | Clean chain | Degraded |
|---|---|---|
| beacon peak/median | high (100s–1000s) | still high — the wideband beacon is robust |
| channel `\|H\|` spread | small (≤ ~10 dB) | deep comb (20–45 dB) → ring |
| channel adjacent-phase autocorr | > 0.5 (smooth) | ~0 (**per-subcarrier random**) |
| data-aided EVM | low (single-digit %) | tens of % (**ring** if data-aided ≫ blind) |

**RX decode (important).** The RFSoC R2C mixer returns a spectrally-**inverted** baseband, so the
tool decodes `x = conj(iq_from_cs16(buf))` (standard CS16 `[I0,Q0,I1,Q1]` → `even + j·odd`, then
conjugate). An earlier version mis-paired the wire lanes (`L0 + j·L2`) and **fabricated** a
"per-subcarrier-random `\|H\|` comb / 140 % ring" that looked like an RFDC hardware defect — it was
purely the host decode. With the correct decode the single-board loopback resolves to **~25 % EVM**
(residual = a known ADC interleave spur near fs/4 + mild `\|H\|` ripple). A *data-aided* EVM far
above the *blind* EVM means the subcarriers are mirrored (decode/inversion); the two agreeing means
the mapping is right and any remaining spread is a real channel.

`--selftest` proves the receiver itself is correct: **~0 % EVM** on an ideal simulated channel and
**< 40 %** on mild in-CP multipath. Run it first — if it passes but a hardware run rings, check the
decode/`conj` and then the RF channel (a deep `\|H\|` comb from **cross-board cable reflections** is
real multipath, dampened with attenuators — not an RFDC defect). The hardware constellation is saved
to `/tmp/loopback_constellation.npy`.
