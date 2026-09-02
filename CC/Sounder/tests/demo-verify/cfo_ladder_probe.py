#!/usr/bin/env python3
"""AP-34(b) on silicon: the three-stage CFO ladder from ONE contiguous capture.

The model (cfo_ladder_model.py) establishes what the ladder can do and what it
needs. This runs it on real samples.

WHAT IT DOES. Reads a contiguous window spanning K+1 frames, finds every beacon
in it, and forms:

  stage 1/2  receiver.cc's own two-stage estimate, once per beacon
  stage 2a   those averaged, which is what unwraps stage 3
  stage 3    consecutive-beacon carrier phase, lag = the ACTUAL sample distance
             between the two cores (known exactly from their detected indices,
             so a drifting frame period costs nothing)

WHY A CONTIGUOUS CAPTURE AND NOT THE LIVE CLIENT. Stage 3 needs beacons one
frame apart. The client resyncs every 260 frames (lag 31.9 M samples, range
+-1.92 Hz, useless) and could not be sped up anyway, because its loop runs
412-746 iter/s against 1000 frames/s and cannot observe consecutive frames.

THE TRUTH IT IS MEASURED AGAINST. Precision is not accuracy. Stage 3 is
predicted to resolve ~0.26 Hz at this bench's noise, roughly 70x finer than the
timing channel's 0.036 ppm residual, but its ZERO POINT is unknown -- the
beacon estimator has a configuration-dependent bias (+280 to +1753 Hz across
the campaign legs before AP-39, DEMO_VERIFICATION 8.6). The timing channel was
validated to <= 0.05 ppm against an RF-free hardware clock ratio, so IT is the
truth here. Run this alongside a clock_drift_probe leg and compare; that
comparison, not this script's own numbers, is what decides AP-41's fusion.

  python3 tests/demo-verify/cfo_ladder_probe.py --ue <ip> --gold gold.bin \
      --frames 8 --windows 20
"""
import argparse
import math
import os
import sys

import numpy as np

import SoapySDR
from SoapySDR import SOAPY_SDR_CS16, SOAPY_SDR_RX

# THE PROBE MUST ARM THE BS BEACON ITSELF. An earlier cut opened only the UE and
# read, on the assumption that a beacon would be on the air. Nothing puts one
# there unless the sounder is running or a probe arms it, so the first two runs
# on silicon found 0 beacons in 24 windows with detector ratios of ~3e-7, six
# orders below the 0.36-5.8 the sibling probes measure. That is the signature of
# correlating against noise, not of a weak link, and reporting it as "0 beacons
# found" would have read as a dead bench. Reuse the same BS helper
# clock_drift_probe uses rather than writing a second one.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from two_node_beacon_arrival import Bs  # noqa: E402

# SoapyRemote's `timeout` device arg is MICROSECONDS and bounds the make() RPC.
# A cold make runs the full RFDC bring-up and measures 3.34 s on this bench, so
# the long-standing 1 s sat inside the normal spread (AP-36).
RPC_TIMEOUT_US = "30000000"

RATE = 122.88e6
FRAME = 122880
GOLD_L = 128
# Beacon geometry, Config::genBeacon: 15 x STS(16) then 2 x gold(128).
STS_LEN, STS_REPS, GOLD_REPS = 16, 15, 2
CORE = STS_LEN * STS_REPS + GOLD_L * GOLD_REPS       # 496
# find_beacon returns the start of the SECOND gold rep, so the core starts
# CORE_OFF_2NDREP earlier. Same convention as two_node_beacon_arrival.py.
CORE_OFF_2NDREP = 368


def find_beacon(raw, gold, corr_scale):
    """Replica of CommsLib::find_beacon_avx, as two_node_beacon_arrival uses."""
    L = len(gold)
    if len(raw) < 2 * L + 8:
        return -1, 0.0
    n = 1 << int(np.ceil(np.log2(len(raw) + L)))
    gc = np.fft.ifft(np.fft.fft(raw, n) * np.conj(np.fft.fft(gold, n)))
    gc = gc[:len(raw) - L + 1]
    ac = np.zeros(len(gc), dtype=np.complex128)
    ac[L:] = gc[L:] * np.conj(gc[:-L])
    peak = np.abs(ac) ** 2
    ca = np.abs(gc) ** 2
    csum = np.concatenate(([0.0], np.cumsum(ca)))
    idx = np.arange(len(gc))
    thresh = csum[idx] - csum[np.maximum(0, idx - L)]
    ratio = peak / (thresh + 1e-30)
    valid = np.where(corr_scale * peak > thresh)[0]
    if len(valid) == 0:
        return -1, float(ratio.max())
    best = int(np.argmax(peak))
    if corr_scale * peak[best] <= thresh[best]:
        best = int(valid[np.argmax(ratio[valid])])
    return int(best), float(ratio[best])


def stage12(core):
    """receiver.cc estimateCFO, verbatim. Returns normalized CFO, or nan."""
    g1 = STS_LEN * STS_REPS
    g2 = g1 + GOLD_L
    r_fine = np.vdot(core[g1:g1 + GOLD_L], core[g2:g2 + GOLD_L])
    r_coarse = 0j
    for k in range(STS_REPS - 1):
        a = core[k * STS_LEN:(k + 1) * STS_LEN]
        b = core[(k + 1) * STS_LEN:(k + 2) * STS_LEN]
        r_coarse += np.vdot(a, b)
    if r_fine == 0 or r_coarse == 0:
        return float("nan")
    f_fine = np.angle(r_fine) / (2 * math.pi * GOLD_L)
    f_coarse = np.angle(r_coarse) / (2 * math.pi * STS_LEN)
    amb = 1.0 / GOLD_L
    f = f_fine + round((f_coarse - f_fine) / amb) * amb
    return -f          # the matched-NCO RX mixer delivers baseband conjugated


def stage3(core_a, core_b, lag, coarse_norm):
    r = np.vdot(core_a, core_b)
    if r == 0:
        return float("nan")
    f = -np.angle(r) / (2 * math.pi * lag)
    amb = 1.0 / lag
    return f + round((coarse_norm - f) / amb) * amb


def sd(xs):
    return float(np.std(xs, ddof=1)) if len(xs) > 1 else 0.0


def capture(dev, rxs, nsamps, chunk=1 << 16):
    """One CONTIGUOUS window.

    A SHORT READ IS NORMAL AND IS NOT A GAP. readStream truncates on this
    platform as a matter of course -- clock_drift_probe.py counts them
    (ret=2032 against a 12288 request, observed live) and
    two_node_beacon_arrival.py simply uses sr.ret. An earlier cut of this
    function aborted the window on any short read, which meant every window
    aborted and the probe could never produce the measurement it exists for.
    Take what the read returned and continue; contiguity is preserved because
    the next read resumes where this one stopped.

    What DOES invalidate stage 3 is a dropped-packet GAP, which is a different
    thing: the driver is opened with rx_gap_break=1 so a gap ENDS the read with
    ret <= 0 rather than splicing across it, and that is what aborts here.
    """
    out = np.zeros(2 * nsamps, dtype=np.int16)
    got = 0
    shorts = 0
    buf = np.zeros(2 * chunk, dtype=np.int16)
    while got < nsamps:
        want = min(chunk, nsamps - got)
        sr = dev.readStream(rxs, [buf], want, timeoutUs=2000000)
        if sr.ret <= 0:
            return None, "read returned %d after %d/%d samples" % (sr.ret, got, nsamps), shorts
        if sr.ret != want:
            shorts += 1
        out[2 * got:2 * (got + sr.ret)] = buf[:2 * sr.ret]
        got += sr.ret
    w = out.astype(np.float64)
    return (w[0::2] + 1j * w[1::2]) / 32767.0, None, shorts


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ue", required=True, help="UE board IP")
    ap.add_argument("--bs", required=True, help="BS board IP (this probe arms "
                                                "the beacon itself)")
    ap.add_argument("--core", default="beacon_core.bin",
                    help="beacon core IQ, loaded into the BS TX RAM")
    ap.add_argument("--tx-ch", type=int, default=1)
    ap.add_argument("--ch", type=int, default=1)
    ap.add_argument("--gold", default="gold.bin")
    ap.add_argument("--freq", type=float, default=500e6)
    ap.add_argument("--frames", type=int, default=8,
                    help="consecutive-frame PAIRS per window (window = N+1 frames)")
    ap.add_argument("--windows", type=int, default=20)
    # 10.0 is what clock_drift_probe.py and two_node_beacon_arrival.py both
    # use. find_beacon admits a peak only when ratio > 1/corr_scale, so a
    # default of 1.0 is a floor of 1.0 against ratios measured 0.36 to 5.8 on
    # this very link: the probe would report "0 beacons found" on a link the
    # siblings lock cleanly. The ledger already records that trap -- the
    # detector ratio is not an absolute, it moves with link level.
    ap.add_argument("--corr-scale", type=float, default=10.0)
    a = ap.parse_args()

    gold = np.fromfile(a.gold, dtype=np.complex64).astype(np.complex128)
    if len(gold) != GOLD_L:
        print("gold is %d samples, expected %d" % (len(gold), GOLD_L))
        return 2
    # BOTH SENSES. The TX RAM holds the conjugated core, so which replica
    # matches is a property of the bench rather than a constant, and both
    # sibling probes try the pair and print which won. Trying only one makes a
    # sense mismatch indistinguishable from a dead link.
    senses = [("gold", gold), ("conj", np.conj(gold))]

    # Arm the BS first, and CHECK it is actually playing. A silent BS makes
    # every downstream number a measurement of noise.
    cc = np.fromfile(a.core, dtype=np.int16).astype(np.float64)
    cc = cc[0::2] + 1j * cc[1::2]
    pk = np.max(np.abs(cc)) or 1.0
    ram = np.zeros(2 * len(cc), dtype=np.int16)
    ram[0:2 * len(cc):2] = np.round(cc.real / pk * 0.6 * 32767).astype(np.int16)
    ram[1:2 * len(cc):2] = np.round(cc.imag / pk * 0.6 * 32767).astype(np.int16)
    bs = Bs(a.bs, ram, a.tx_ch)
    bs.open_and_arm()
    if not bs.liveness():
        print("BS beacon is NOT alive. Everything below would be a measurement")
        print("of noise, so stopping here rather than reporting zero beacons.")
        return 1
    print("BS %s beacon armed and playing" % a.bs)

    dev = SoapySDR.Device(dict(driver="houdinisdr",
                               remote="tcp://%s:55132" % a.ue,
                               timeout=RPC_TIMEOUT_US))
    hi = dict(dev.getHardwareInfo())
    print("UE %s label=%s ip=%s" % (a.ue, hi.get("label", "?"),
                                    hi.get("ip_address", "")))
    dev.setSampleRate(SOAPY_SDR_RX, a.ch, RATE)
    dev.setFrequency(SOAPY_SDR_RX, a.ch, a.freq)
    rxs = dev.setupStream(SOAPY_SDR_RX, SOAPY_SDR_CS16, [a.ch],
                          dict(local_port=str(10001 + a.ch), rx_gap_break="1"))
    dev.activateStream(rxs)

    nsamps = (a.frames + 1) * FRAME
    print("window = %d frames = %d samples (%.1f MB), %d windows\n"
          % (a.frames + 1, nsamps, nsamps * 4 / 1e6, a.windows))

    s2_all, s3_all, aborted, shorts_total = [], [], 0, 0
    sense_name, sense_gold = None, None
    for w in range(a.windows):
        c, err, shorts = capture(dev, rxs, nsamps)
        shorts_total += shorts
        if c is None:
            aborted += 1
            print("  window %d ABORTED: %s" % (w + 1, err))
            continue
        # Settle the replica sense once, on the first window that yields a
        # detection either way, then hold it: switching mid-run would make the
        # per-window results incomparable.
        if sense_gold is None:
            best = (-1.0, None, None)
            for nm, g in senses:
                _idx, rr = find_beacon(c[:FRAME], g, a.corr_scale)
                if rr > best[0]:
                    best = (rr, nm, g)
            # A ratio orders below the 0.36-5.8 this bench measures means there
            # is nothing to lock onto. Say that, rather than reporting "0
            # beacons found" window after window as though the link were dead.
            if best[0] < 0.05:
                print("  best detector ratio %.3g is far below anything this "
                      "bench produces (0.36-5.8 measured): the UE is seeing "
                      "noise, not a beacon. Check the BS arm and the cabling."
                      % best[0])
                return 1
            _, sense_name, sense_gold = best
            print("  replica sense = %s (best ratio %.3g)" % (sense_name, best[0]))
        # Find one beacon per frame slice, then convert the detector index to
        # the core start. A slice is one frame long, so exactly one beacon lands
        # in it -- but the beacon can straddle the slice boundary, so skip any
        # detection too close to either edge rather than reading past it.
        cores, pos = [], []
        for k in range(a.frames + 1):
            sl = c[k * FRAME:(k + 1) * FRAME]
            idx, ratio = find_beacon(sl, sense_gold, a.corr_scale)
            if idx < 0:
                continue
            start = k * FRAME + idx - CORE_OFF_2NDREP
            if start < 0 or start + CORE > len(c):
                continue
            cores.append(c[start:start + CORE])
            pos.append(start)
        if len(cores) < 2:
            aborted += 1
            print("  window %d: only %d beacon(s) found" % (w + 1, len(cores)))
            continue
        s2 = [stage12(x) for x in cores]
        s2 = [v for v in s2 if v == v]
        if not s2:
            aborted += 1
            continue
        s2avg = sum(s2) / len(s2)
        s2_all += [v * RATE for v in s2]
        for k in range(len(cores) - 1):
            lag = pos[k + 1] - pos[k]
            # Only ADJACENT frames carry a usable stage-3 range. A missed
            # detection makes the lag 2+ frames and shrinks the window below
            # what stage 2 can unwrap, so drop the pair rather than report it.
            if not (0.9 * FRAME < lag < 1.1 * FRAME):
                continue
            v = stage3(cores[k], cores[k + 1], lag, s2avg)
            if v == v:
                s3_all.append(v * RATE)

    dev.deactivateStream(rxs)
    dev.closeStream(rxs)

    print("\n=== ladder over %d window(s), %d aborted, %d short read(s) "
          "stitched ===" % (a.windows, aborted, shorts_total))
    if not s2_all or not s3_all:
        print("NO RESULT: stage2 n=%d stage3 n=%d" % (len(s2_all), len(s3_all)))
        return 1
    m2, m3 = float(np.mean(s2_all)), float(np.mean(s3_all))
    d2, d3 = sd(s2_all), sd(s3_all)
    print("  stage 2  n=%4d  mean %+10.2f Hz  sd %9.2f Hz  sem %7.2f"
          % (len(s2_all), m2, d2, d2 / math.sqrt(len(s2_all))))
    print("  stage 3  n=%4d  mean %+10.4f Hz  sd %9.4f Hz  sem %7.4f"
          % (len(s3_all), m3, d3, d3 / math.sqrt(len(s3_all))))
    if d3 > 0:
        print("  precision gain per shot: %.0fx" % (d2 / d3))
    print("  stage 3 vs stage 2 mean:  %+.2f Hz" % (m3 - m2))
    print()
    print("  eps = df/df_sc   %+.6f   (df_sc = fs/N_fft = %.2f Hz)"
          % (m3 / (RATE / 64), RATE / 64))
    print("  ppm vs carrier   %+.4f ppm" % (m3 / a.freq * 1e6))
    print()
    print("  NOT AN ACCURACY FIGURE. Compare the ppm above against the timing")
    print("  channel's eps from a clock_drift_probe leg taken on the same link.")
    print("  The difference IS stage 3's zero-point bias, and that number is")
    print("  what decides whether AP-41 may fuse this channel.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
