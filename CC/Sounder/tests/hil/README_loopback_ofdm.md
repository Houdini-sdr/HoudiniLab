# loopback_ofdm.py: OFDM closure test over a single-board DAC_B to ADC_D loopback

A self-contained reproduction of the sounder's legacy OFDM signal chain in one
script, for checking one board's converters and receive decode without the
sounder. It builds an app-rate frame in replay RAM, loops it over one board's
own `DAC_B` to `ADC_D` loopback cable, receives it, and runs a full receiver:
**GOLD beacon sync, fine CFO (from the two identical pilots), LTS channel
estimate, zero-forcing equalize, QPSK constellation and EVM**.

Frame (122.88 MSPS, in replay RAM, looped):

```
[128 GOLD beacon][160 pilot = 2x80 LTS syms][NDATA x 80 QPSK data syms][zero pad]
```

## Run (on the host that reaches the board)

1. Activate the SoapyHoudiniSDR host environment and point SoapySDR at its
   plugin:

   ```bash
   source <your-houdini-venv>/bin/activate
   export LD_LIBRARY_PATH=$VIRTUAL_ENV/lib
   export SOAPY_SDR_PLUGIN_PATH=$VIRTUAL_ENV/lib/SoapySDR/modules0.8-3
   ```

2. Make sure nothing else holds the board: no sounder or dashboard is running
   against it (`pgrep -cx sounder` prints 0). On a shared host, stop only your
   own runs; `CC/Sounder/tools/rig_release_holders.py` stops every sounder and
   dashboard on the host. Do not use `pkill -f`: its pattern matches its own
   caller.

3. Run the self-test first, then the hardware test against the board that
   carries the loopback cable:

   ```bash
   python3 loopback_ofdm.py --selftest                      # offline receiver self-test (no radio)
   python3 loopback_ofdm.py --board <board-ip>              # hardware, app-rate replay (strong beacon)
   python3 loopback_ofdm.py --board <board-ip> --rate max   # 8x-upsampled DAC-rate replay
   python3 loopback_ofdm.py --board <board-ip> --html       # plus a visualization in /tmp/loopback_ofdm.html
   ```

   `--board` is required for a hardware run: the tool has no default board.

**Visualization (`--html`):** writes a self-contained HTML page (no external
libraries, inline canvas, opens in any browser) with the channel `|H|` (dB),
the channel phase, and the equalized constellation against ideal QPSK, plus a
pass/fail verdict banner. It defaults to `/tmp/loopback_ofdm.html`; pass a path
to override (`--html out.html`). It works with `--selftest` too (it renders the
clean simulated reference). View it over SSH by copying the file to your
workstation or through an `ssh -L` tunnel to a local web server.

Other flags: `--tx-ch`/`--rx-ch` (default `0`/`0`, DAC_B to ADC_D), `--nco`
(MHz, default 500), `--ndata`, `--secs`, `--tx-scale`. Add
`2>&1 | grep -vE '^\[INFO\]'` to hide driver chatter. A board with nothing on
its receive cable can return 0 samples.

## Interpreting the output

| Signal | Clean chain | Degraded |
|---|---|---|
| beacon peak/median | high (100s to 1000s) | still high: the wideband beacon is robust |
| channel `\|H\|` spread | small (at most about 10 dB) | deep comb (20 to 45 dB), a ring |
| channel adjacent-phase autocorr | above 0.5 (smooth) | about 0 (**per-subcarrier random**) |
| data-aided EVM | low (single-digit %) | tens of % (a **ring** if data-aided is far above blind) |

**RX decode (important).** The RFSoC R2C mixer returns a spectrally
**inverted** baseband, so the tool decodes `x = conj(iq_from_cs16(buf))`
(standard CS16 `[I0,Q0,I1,Q1]` to `even + j*odd`, then conjugate). Mis-pairing
the wire lanes (`L0 + j*L2`) fabricates a per-subcarrier-random `|H|` comb and
a ring of about 140 % EVM that looks like an RFDC hardware defect but is purely
the host decode. With the correct decode the single-board loopback resolves to
about 25 % EVM (the residual is a known ADC interleave spur near fs/4 plus mild
`|H|` ripple). A *data-aided* EVM far above the *blind* EVM means the
subcarriers are mirrored (decode or inversion); the two agreeing means the
mapping is right and any remaining spread is a real channel.

`--selftest` proves the receiver itself is correct: **about 0 % EVM** on an
ideal simulated channel and **under 40 %** on mild in-CP multipath. Run it
first. If it passes but a hardware run rings, check the decode and `conj`, then
the RF channel (a deep `|H|` comb from **cross-board cable reflections** is
real multipath, dampened with attenuators, not an RFDC defect). The hardware
constellation is saved to `/tmp/loopback_constellation.npy`.
