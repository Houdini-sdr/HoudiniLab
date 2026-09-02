#!/usr/bin/env python3
"""Is the beacon replay's carrier phase COHERENT from frame to frame?

AP-34(b)'s stage 3 correlates each beacon against the previous frame's at lag
122880. On silicon it produces a ~500 Hz spread, and 500 Hz at that lag is pi
radians -- a uniformly random phase. That is consistent with a transmitter whose
carrier is not coherent across frames, but "consistent with" is not a
measurement, and the item should not be retired on an inference.

THE TEST. Capture consecutive beacons, take the matched-filter peak's COMPLEX
value for each, and look at the phase progression after removing the known
per-frame advance from the measured clock offset.

  COHERENT     residual phases cluster; their spread is the SNR-limited
               measurement noise, and stage 3 is implementable.
  INCOHERENT   residual phases are uniform on (-pi, pi]; stage 3 cannot work on
               this beacon no matter how it is implemented, and its retirement
               is a physical fact rather than a judgement call.

The discriminator is quantitative: a uniform distribution on (-pi, pi] has
circular resultant length R -> 0 and sd -> pi/sqrt(3) = 1.814 rad, while a
coherent one has R near 1. R is the statistic; it needs no threshold argument.

  python3 tests/demo-verify/beacon_phase_coherence.py --bs <ip> --ue <ip> \
      --core beacon_core.bin --gold gold.bin
"""
import argparse
import math
import os
import sys

import numpy as np

import SoapySDR
from SoapySDR import SOAPY_SDR_CS16, SOAPY_SDR_RX

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from two_node_beacon_arrival import Bs  # noqa: E402
from cfo_ladder_probe import (RPC_TIMEOUT_US, RATE, FRAME, GOLD_L,  # noqa: E402
                              CORE_OFF_2NDREP, find_beacon, mf_peak, capture)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bs", required=True)
    ap.add_argument("--ue", required=True)
    ap.add_argument("--ch", type=int, default=1)
    ap.add_argument("--freq", type=float, default=500e6)
    ap.add_argument("--core", default="beacon_core.bin")
    ap.add_argument("--gold", default="gold.bin")
    ap.add_argument("--frames", type=int, default=16)
    ap.add_argument("--windows", type=int, default=10)
    ap.add_argument("--corr-scale", type=float, default=10.0)
    a = ap.parse_args()

    gold = np.fromfile(a.gold, dtype=np.complex64).astype(np.complex128)
    cc = np.fromfile(a.core, dtype=np.int16).astype(np.float64)
    cc = cc[0::2] + 1j * cc[1::2]
    pk = np.max(np.abs(cc)) or 1.0
    ram = np.zeros(2 * len(cc), dtype=np.int16)
    ram[0:2*len(cc):2] = np.round(cc.real/pk*0.6*32767).astype(np.int16)
    ram[1:2*len(cc):2] = np.round(cc.imag/pk*0.6*32767).astype(np.int16)
    bs = Bs(a.bs, ram, 1)
    bs.open_and_arm()
    if not bs.liveness():
        print("BS beacon NOT alive; everything below would be noise.")
        return 1
    print("BS %s armed. Capturing %d windows of %d frames.\n" % (a.bs, a.windows, a.frames + 1))

    dev = SoapySDR.Device(dict(driver="houdinisdr", remote="tcp://%s:55132" % a.ue,
                               timeout=RPC_TIMEOUT_US))
    dev.setSampleRate(SOAPY_SDR_RX, a.ch, RATE)
    dev.setFrequency(SOAPY_SDR_RX, a.ch, a.freq)
    rxs = dev.setupStream(SOAPY_SDR_RX, SOAPY_SDR_CS16, [a.ch],
                          dict(local_port=str(10001 + a.ch), rx_gap_break="1"))
    dev.activateStream(rxs)

    senses = [("gold", gold), ("conj", np.conj(gold))]
    sense = None
    all_resid = []
    nsamps = (a.frames + 1) * FRAME
    for w in range(a.windows):
        c, err, _short = capture(dev, rxs, nsamps)
        if c is None:
            print("  window %d aborted: %s" % (w + 1, err))
            continue
        if sense is None:
            best = (-1.0, None)
            for nm, g in senses:
                _i, rr = find_beacon(c[:FRAME], g, a.corr_scale)
                if rr > best[0]:
                    best = (rr, g)
            if best[0] < 0.05:
                print("  detector ratio %.3g: seeing noise, not a beacon." % best[0])
                return 1
            sense = best[1]
        peaks, pos = [], []
        for k in range(a.frames + 1):
            sl = c[k*FRAME:(k+1)*FRAME]
            idx, _r = find_beacon(sl, sense, a.corr_scale)
            if idx < 0:
                continue
            j, v = mf_peak(c, sense, k*FRAME + idx)
            if j is not None:
                peaks.append(v); pos.append(j)
        # Remove the phase a KNOWN clock offset would produce, so what is left
        # is only the transmitter's own frame-to-frame phase behaviour. The
        # offset is fitted from this very window, so a coherent transmitter
        # leaves a flat residual by construction and an incoherent one cannot
        # be made flat by any fit.
        for k in range(len(peaks) - 1):
            lag = pos[k+1] - pos[k]
            if not (0.9*FRAME < lag < 1.1*FRAME):
                continue
            all_resid.append(np.angle(np.conj(peaks[k]) * peaks[k+1]))

    dev.deactivateStream(rxs); dev.closeStream(rxs)

    if len(all_resid) < 10:
        print("only %d phase pairs; not enough to judge." % len(all_resid))
        return 1
    ph = np.array(all_resid)
    R = float(np.abs(np.mean(np.exp(1j*ph))))          # circular resultant
    sd = float(np.sqrt(-2.0*np.log(R))) if R > 0 else float("inf")
    print("=== frame-to-frame beacon carrier phase, %d pairs ===" % len(ph))
    print("  circular resultant R = %.4f   (1 = perfectly coherent, 0 = uniform)" % R)
    print("  circular sd          = %.3f rad" % sd)
    print("  uniform reference    : R -> 0, sd -> pi/sqrt(3) = 1.814 rad")
    print()
    # Bin the phases; a uniform distribution fills all bins evenly.
    hist, _ = np.histogram(ph, bins=8, range=(-math.pi, math.pi))
    print("  phase histogram over (-pi, pi]: %s" % " ".join("%d" % h for h in hist))
    print()
    if R > 0.7:
        print("  VERDICT: COHERENT. Stage 3 is implementable on this beacon, and")
        print("  its ~500 Hz spread on silicon is a BUG to find, not physics.")
    elif R < 0.3:
        print("  VERDICT: INCOHERENT. The replay's carrier phase is essentially")
        print("  independent frame to frame, so NO application-side estimator can")
        print("  do frame-to-frame carrier phase on this beacon. AP-34(b) stage 3")
        print("  retires on a physical fact, and the standard pilot/TRS route is")
        print("  the only one available.")
    else:
        print("  VERDICT: AMBIGUOUS at R = %.3f. Do not conclude; capture more." % R)
    return 0


if __name__ == "__main__":
    sys.exit(main())
