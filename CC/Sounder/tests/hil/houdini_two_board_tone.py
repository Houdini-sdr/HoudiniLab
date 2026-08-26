#!/usr/bin/env python3
"""
houdini_two_board_tone.py -- CW tone from BOTH RFSoC4x2 boards, for an oscilloscope
RF phase-lock check.

Both boards (.21 and .22) transmit a CONTINUOUS CW tone at the SAME RF frequency on the
same DAC channel. Put each board's DAC SMA on a scope channel (or combine them with a
power combiner/tee into one channel) and watch:

  * PHASE-LOCKED (boards share the 10 MHz reference): the two tones are EXACTLY the same
    frequency, so their relative phase is CONSTANT --
      - 2 channels: trigger on board-1; board-2's trace sits STILL.
      - combiner:   the summed amplitude is STEADY (no beat).
  * FREE-RUNNING (not locked): the tones differ by the CFO (~440 kHz measured here), so
    the relative phase SLIDES continuously --
      - 2 channels: board-2's trace drifts left/right continuously.
      - combiner:   the amplitude BEATS at the CFO (~440 kHz -> ~2.3 us envelope). This
        beat is the most obvious "not locked" tell.

The beat/slide RATE is the CFO itself, so this also measures how far off they are: a
slow crawl = nearly locked, a ~440 kHz beat = free-running.

Notes:
  * --ch 1 = DAC_A, --ch 0 = DAC_B (RFSoC4x2 SMAs are reversed). DAC_A/ch1 is the one
    cross-cabled to the other board's ADC -- unplug those cables (or use --ch 0 / DAC_B,
    which is free) before connecting the scope.
  * Pick --rf-mhz within BOTH your scope's bandwidth AND the DAC's usable output (the RF
    baluns roll off very low frequencies; if the tone looks weak, raise --rf-mhz).
  * The tone plays until --secs elapses or you Ctrl-C.

Run on the DGX (after: source houdini_test/bin/activate):
    python3 houdini_two_board_tone.py                    # 50 MHz, both boards, 300 s
    python3 houdini_two_board_tone.py --rf-mhz 100 --ch 0
"""
import argparse
import os
import sys
import time

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)
_EX = os.environ.get("HOUDINI_EXAMPLES",
                     os.path.expanduser("~/repos/SoapyHoudiniSDR/host/examples"))
if _EX not in sys.path:
    sys.path.insert(0, _EX)

import SoapySDR  # noqa: E402
from SoapySDR import SOAPY_SDR_TX  # noqa: E402
import houdini_setup as hs  # noqa: E402
from houdini_setup import tx_iq_tone  # noqa: E402
from beacon_tdd import _teardown  # noqa: E402


def start_tone(ip, ch, nco_hz, bb_hz, amp, n=4096):
    """Bring up a continuous replay CW tone on one board; return (sdr, stream)."""
    ctx = hs.open_device(node=ip, ch=ch, verbose=False)
    sdr, native = ctx["sdr"], ctx["native_fmt"]
    _teardown(sdr)
    # A prior run may have left the replay strobe armed on this channel, which
    # FREEZES the replay RAM (fill REFUSED) -> the board would TX stale samples.
    # Explicitly disarm the strobe + abort the framer before loading the tone.
    for setting, val in (("TDD_REPLAY_STROBE", f"ch{ch}:off"), ("TDD_CMD", "abort")):
        try:
            sdr.writeSetting(setting, val)
        except Exception:  # noqa: BLE001
            pass
    sdr.setSampleRate(SOAPY_SDR_TX, ch, 122.88e6)
    rate = float(sdr.getSampleRate(SOAPY_SDR_TX, ch))
    sdr.setFrequency(SOAPY_SDR_TX, ch, nco_hz)
    i16, act_bb = tx_iq_tone(bb_hz, rate, n, amp_frac=amp)
    cs16 = np.ascontiguousarray(i16, dtype=np.int16).view(np.int32)
    txs = sdr.setupStream(SOAPY_SDR_TX, native, [ch], {"tx_mode": "replay"})
    for tryi in range(3):
        try:
            sdr.writeStream(txs, [cs16], n, 0, 0)   # load the replay RAM
            break
        except Exception as e:  # noqa: BLE001
            if tryi == 2:
                raise
            try:
                sdr.writeSetting("TDD_REPLAY_STROBE", f"ch{ch}:off")
                sdr.writeSetting("TDD_CMD", "abort")
            except Exception:  # noqa: BLE001
                pass
            time.sleep(0.3)
    sdr.activateStream(txs)                 # free-running continuous replay
    got_nco = float(sdr.getFrequency(SOAPY_SDR_TX, ch))
    print(f"  {ip}: DAC ch{ch}  RF = {(got_nco + act_bb)/1e6:.3f} MHz "
          f"(NCO {got_nco/1e6:.3f} + baseband {act_bb/1e6:.3f}), amp {amp}")
    return sdr, txs


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--boards", default="168.6.244.21,168.6.244.22")
    ap.add_argument("--rf-mhz", type=float, default=50.0, help="RF tone (= NCO + baseband)")
    ap.add_argument("--bb-mhz", type=float, default=1.0,
                    help="baseband tone offset; NCO = rf - bb (keeps NCO off DC)")
    ap.add_argument("--ch", type=int, default=1, help="DAC channel (1=DAC_A, 0=DAC_B)")
    ap.add_argument("--amp", type=float, default=0.5)
    ap.add_argument("--secs", type=float, default=300.0)
    a = ap.parse_args()

    boards = [b.strip() for b in a.boards.split(",") if b.strip()]
    nco_hz = (a.rf_mhz - a.bb_mhz) * 1e6
    bb_hz = a.bb_mhz * 1e6
    print(f"Starting {a.rf_mhz} MHz CW tone on {len(boards)} board(s), DAC ch{a.ch}:")
    handles = []
    try:
        for ip in boards:
            handles.append((ip, *start_tone(ip, a.ch, nco_hz, bb_hz, a.amp)))
        print(f"\nBoth boards transmitting {a.rf_mhz} MHz. On the scope:")
        print("  LOCKED   -> board-2 trace steady vs a board-1 trigger; combiner amplitude flat.")
        print("  UNLOCKED -> board-2 trace slides; combiner amplitude BEATS at the CFO (~440 kHz).")
        print("  (the slide/beat RATE = the frequency offset between the boards)")
        print(f"\nHolding for {a.secs:.0f} s -- Ctrl-C to stop early ...")
        t0 = time.time()
        while time.time() - t0 < a.secs:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\nstopping (Ctrl-C)")
    finally:
        for ip, sdr, txs in handles:
            try:
                sdr.deactivateStream(txs)
                sdr.closeStream(txs)
            except Exception:  # noqa: BLE001
                pass
            try:
                _teardown(sdr)
            except Exception:  # noqa: BLE001
                pass
        print("tones off, boards torn down.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
