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


def mf_peak(win, gold_rep, near, span=16):
    """Matched-filter peak near `near`: returns (index, complex value).

    STAGE 3 MUST NOT TRUST THE DETECTOR INDEX. Correlating raw beacon cores at
    the index find_beacon reports is destroyed by that index's jitter: modelled
    2026-09-02, ONE sample of misalignment costs 200-350 Hz against a 130 Hz
    truth, and random jitter of sd 0.5-3 samples gives a per-shot spread of
    244-300 Hz. That matches the 279-477 Hz this probe measured on silicon
    exactly, and it is why the first five runs disagreed by 13x. The core is a
    structured sequence (15 x STS(16) + 2 x gold(128)), so a one-sample shift
    decorrelates it against itself.

    Searching for the matched-filter peak instead is index-INSENSITIVE: the
    peak sits at the true position wherever the search starts, so the phase
    read there does not carry the detector's error. Modelled at 0.0000 Hz error
    and 0.0000 spread for index jitter up to 5 samples.
    """
    best = (-1.0, None, None)
    lo = max(0, near - span)
    hi = min(len(win) - len(gold_rep), near + span)
    for j in range(lo, hi + 1):
        v = np.vdot(gold_rep, win[j:j + len(gold_rep)])
        a = abs(v)
        if a > best[0]:
            best = (a, j, v)
    return best[1], best[2]


def stage3_phase(v_a, v_b):
    """RAW frame-to-frame phase difference, radians. No unwrap, no scaling."""
    r = np.conj(v_a) * v_b
    return float("nan") if r == 0 else float(-np.angle(r))


def stage3_unwrap(phases, lag, coarse_norm):
    """AVERAGE IN THE PHASE DOMAIN, THEN UNWRAP ONCE.

    The previous version unwrapped every pair independently against stage 2 and
    then averaged the results. That is backwards and it was the whole bug. Each
    unwrap does `round((coarse - f) / amb)` with amb = 1/lag, so it must decide
    which 1000 Hz ambiguity bin the pair belongs to using a stage-2 estimate
    whose per-shot spread is 400-1500 Hz on this bench. It gets that decision
    wrong often enough that the ROUNDING, not the measurement, dominates: the
    output spread came out ~474 Hz, roughly half an ambiguity step, which is the
    signature of occasional bin errors rather than of noisy phase.

    The phases themselves are excellent. Measured directly on silicon
    2026-09-02: circular resultant R = 0.9919 and 0.9952 over 160 pairs,
    circular sd 0.10-0.13 rad, which at this lag is about 20 Hz per pair.

    So: average the phases first, where they are tightly clustered and a
    circular mean is well conditioned, and unwrap that ONE well-determined value
    against stage 2. One rounding decision for the whole run instead of one per
    pair, and it is made on a quantity with ~1/sqrt(n) less noise.
    """
    ph = np.asarray([p for p in phases if p == p])
    if len(ph) == 0:
        return float("nan"), 0.0, 0
    # Circular mean: the right average for angles, and it degrades gracefully
    # if the cluster ever does spread out (R falls, which the caller reports).
    z = np.mean(np.exp(1j * ph))
    R = float(np.abs(z))
    f = float(np.angle(z)) / (2 * math.pi * lag)
    amb = 1.0 / lag
    return f + round((coarse_norm - f) / amb) * amb, R, len(ph)


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
    ap.add_argument("--coarse-hz", type=float, default=0.0,
                    help="unwrap reference in Hz. Default 0 is correct while "
                         "|offset| < 500 Hz (|eps| < 1 ppm at 500 MHz). For a "
                         "free-running pair supply the TIMING channel's eps * "
                         "carrier; never stage 2, which is too noisy.")
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
    s3_phase, s3_lag = [], []      # raw phases, unwrapped ONCE at the end
    ramp_drift = []                # per-window arrival drift, samples
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
        cores, pos, peaks, ppos = [], [], [], []
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
            # Stage 3's own observable: the matched-filter peak, FOUND rather
            # than assumed. `idx` is the detector's estimate of the 2nd gold
            # rep; search around it and keep where the peak actually is.
            jj, vv = mf_peak(c, sense_gold, k * FRAME + idx)
            if jj is not None:
                peaks.append(vv)
                ppos.append(jj)
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
        # The beacon POSITIONS in this same capture are an arrival ramp, and
        # its slope is the timing eps -- measured simultaneously, on the same
        # samples, for free. That is a far better unwrap reference than stage 2.
        # The arrival ramp within ONE window is NOT usable as a reference here,
        # and the arithmetic says so before any data does: at 0.3 ppm the slip is
        # 0.037 samples per frame, so across a 9-frame window the total drift is
        # 0.3 samples -- below the integer quantisation of the detected position.
        # The fit then returns ~0 and only LOOKS right because 0 Hz happens to
        # sit inside the ambiguity window at this eps. Measured 2026-09-02: it
        # printed "+0.0 Hz" on four consecutive runs. Recorded, and the drift is
        # reported so the caller can see when a longer window would make it
        # usable (about 290 frames at this eps for 10 samples of drift).
        if len(ppos) >= 3:
            drift = float(ppos[-1] - ppos[0]) - FRAME * (len(ppos) - 1)
            ramp_drift.append(drift)
        for k in range(len(peaks) - 1):
            lag = ppos[k + 1] - ppos[k]
            # Only ADJACENT frames carry a usable stage-3 range. A missed
            # detection makes the lag 2+ frames and shrinks the window below
            # what stage 2 can unwrap, so drop the pair rather than report it.
            if not (0.9 * FRAME < lag < 1.1 * FRAME):
                continue
            ph = stage3_phase(peaks[k], peaks[k + 1])
            if ph == ph:
                s3_phase.append(ph)
                s3_lag.append(lag)

    dev.deactivateStream(rxs)
    dev.closeStream(rxs)

    print("\n=== ladder over %d window(s), %d aborted, %d short read(s) "
          "stitched ===" % (a.windows, aborted, shorts_total))
    # Check the RAW phase list, not s3_all: the latter is not populated until
    # the single unwrap below, which is the whole point of the new ordering.
    if not s2_all or not s3_phase:
        print("NO RESULT: stage2 n=%d stage3 phases n=%d"
              % (len(s2_all), len(s3_phase)))
        return 1
    # UNWRAP AGAINST THE TIMING CHANNEL, NOT STAGE 2. Stage 2's mean wanders
    # by more than half an ambiguity step on this bench (+200, -142, +336 Hz
    # measured across three runs), and one bad run then puts the whole answer
    # exactly 1000 Hz out -- observed 2026-09-02, a paired run reading
    # +2.2275 ppm against a timing truth of +0.2571 while its neighbour read
    # +0.2581. The timing eps from THIS capture's own beacon arrivals is good to
    # ~0.001 ppm, which is 0.5 Hz at 500 MHz and comfortably inside the +-500 Hz
    # window, and it costs no extra device time because the positions are
    # already in hand. Fall back to stage 2 only if the ramp is unavailable.
    # WHAT THE UNWRAP NEEDS is a reference within +-RATE/(2*FRAME) = +-500 Hz of
    # the truth. It does NOT need precision, and it must not come from a noisy
    # source, because being wrong by more than the window puts the whole answer
    # exactly 1000 Hz out.
    #
    # STAGE 2 IS NEVER THAT SOURCE. Its mean wandered +200 / -142 / +336 / +764 /
    # +647 Hz across five runs on a link whose true offset was 120-150 Hz. An
    # earlier version of this code used stage 2 whenever |stage2| > 500 Hz, on
    # the reasoning that zero might then be unsafe -- which inverted the logic
    # and caused exactly the failure it meant to prevent: two of three paired
    # runs came out 1000 Hz wrong (+2.2849 and +2.2409 ppm against timing truths
    # of +0.3004 and +0.2402) while the one run that used zero was right.
    #
    # So: ZERO by default, which is correct whenever |eps| * f_carrier < 500 Hz
    # (|eps| < 1.0 ppm at 500 MHz) and, unlike stage 2, cannot wander. For a
    # free-running pair at 8.5 ppm the offset is 4260 Hz, four steps out, and the
    # caller must supply --coarse-hz from a source that is actually accurate --
    # the timing channel, not this beacon's own stage 2.
    amb_hz = RATE / (2.0 * FRAME)
    coarse_hz = a.coarse_hz
    coarse_src = ("--coarse-hz" if a.coarse_hz != 0.0
                  else "zero (valid while |offset| < %.0f Hz)" % amb_hz)
    s2m = float(np.mean(s2_all))
    if abs(s2m) > 3 * amb_hz and a.coarse_hz == 0.0:
        # Stage 2 is too noisy to unwrap with, but 3x the window is far enough
        # out to be worth saying. Warn; do NOT silently switch to it.
        print("  NOTE: stage 2 reads %+.0f Hz, past 3x the +-%.0f Hz window a"
              % (s2m, amb_hz))
        print("  zero reference covers. Stage 2 is too noisy to unwrap with, so")
        print("  this run still uses zero. If the link really is that far off,")
        print("  pass --coarse-hz from the timing channel; a wrong bin shows up")
        print("  as an answer exactly %.0f Hz out." % (2 * amb_hz))
    s2avg_norm = coarse_hz / RATE
    lag_med = float(np.median(s3_lag)) if s3_lag else FRAME
    f3, R3, n3 = stage3_unwrap(s3_phase, lag_med, s2avg_norm)
    m3 = f3 * RATE
    # The per-pair spread, reported in Hz, is now the honest measurement noise
    # rather than the rounding noise the old code produced.
    ph = np.asarray(s3_phase)
    d3 = float(np.std(ph, ddof=1)) * RATE / (2 * math.pi * lag_med) if len(ph) > 1 else 0.0
    print("  unwrap reference: %+.1f Hz from %s" % (coarse_hz, coarse_src))
    print("  stage 3 phase cluster: R = %.4f over %d pairs (1 = coherent, "
          "0 = uniform)" % (R3, n3))
    if R3 < 0.7:
        print("  WARNING: the phases are NOT tightly clustered, so the single")
        print("  unwrap below is not well conditioned. Treat the result as")
        print("  suspect and look at the beacon before believing it.")
    s3_all = [m3]
    m2 = float(np.mean(s2_all))
    d2 = sd(s2_all)
    print("  stage 2  n=%4d  mean %+10.2f Hz  sd %9.2f Hz  sem %7.2f"
          % (len(s2_all), m2, d2, d2 / math.sqrt(len(s2_all))))
    print("  stage 3  n=%4d  mean %+10.4f Hz  per-pair sd %9.4f Hz  sem %7.4f"
          % (n3, m3, d3, d3 / math.sqrt(max(1, n3))))
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
