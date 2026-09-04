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
import time

import numpy as np

import SoapySDR
from SoapySDR import SOAPY_SDR_CS16, SOAPY_SDR_RX, SOAPY_SDR_TX

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import two_node_beacon_arrival as tn  # noqa: E402
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
    ap.add_argument("--ue-mts", default="on", choices=["on", "off"],
                    help="put the UE's receive stream in the MTS group via a "
                         "never-activated TX replay stream first, as the "
                         "sounder's UE does (default on). off reproduces the "
                         "per-arm converter skew of 8.172: smeared, tilted "
                         "beacon and phase hops at half-sample timing.")
    ap.add_argument("--mts", default="on", choices=["on", "off", "on-power0"],
                    help="open the streams with mts=true as the sounder does "
                         "(default on). off reproduces the probes' pre-2026-09-03 "
                         "behaviour, whose transmitted beacon varied per arm (8.171).")
    ap.add_argument("--settle", type=float, default=0.0,
                    help="seconds to wait after arming the base station before "
                         "capturing. The sounder's windows are taken seconds "
                         "after its arm; this probe captured within ~1 s and "
                         "saw a per-arm varying transmit spectrum (8.171).")
    ap.add_argument("--bs-rx", action="store_true",
                    help="also configure the base station's RECEIVE side (rate, "
                         "frequency, an RX stream) before arming, as the sounder "
                         "does. The probe's transmit-only arming produced a "
                         "smeared, spectrally tilted beacon in some 2026-09-03 "
                         "runs (8.169/8.171); this is the A/B for it.")
    ap.add_argument("--sense", default="conj", choices=["conj", "gold", "auto"],
                    help="which replica sense to correlate with. This probe "
                         "transmits the dumped core as is and the Houdini "
                         "receive mixer conjugates, so the received beacon "
                         "matches conj(replica): conj is the physical answer "
                         "and the default. (The sounder pre-conjugates its "
                         "transmit and therefore correlates with the replica "
                         "itself.) auto picks by lag-product ratio and chose "
                         "conj in every 2026-09-03 run; gold is here to show "
                         "the wrong sense barely detects at all.")
    ap.add_argument("--dump-raw", default=None,
                    help="write the FIRST captured window as complex64 so the "
                         "received waveform itself can be examined offline")
    ap.add_argument("--dump", default=None,
                    help="write one CSV row per detected beacon (window, frame, "
                         "position, peak magnitude and phase, both neighbours) "
                         "so an odd summary can be traced to its frames")
    ap.add_argument("--lag-max", type=int, default=1,
                    help="also report the phase INNOVATION at lags 2..lag-max "
                         "frames (AP-67): how far the beacon phase can be "
                         "predicted ahead, and how the error grows with time")
    a = ap.parse_args()
    tn.STREAM_MTS[0] = a.mts != "off"
    tn.MTS_POWER_CH0[0] = a.mts == "on-power0"

    gold = np.fromfile(a.gold, dtype=np.complex64).astype(np.complex128)
    cc = np.fromfile(a.core, dtype=np.int16).astype(np.float64)
    cc = cc[0::2] + 1j * cc[1::2]
    pk = np.max(np.abs(cc)) or 1.0
    ram = np.zeros(2 * len(cc), dtype=np.int16)
    ram[0:2*len(cc):2] = np.round(cc.real/pk*0.6*32767).astype(np.int16)
    ram[1:2*len(cc):2] = np.round(cc.imag/pk*0.6*32767).astype(np.int16)
    bs = Bs(a.bs, ram, 1)
    bs.open_and_arm(tx_only=not a.bs_rx)
    if not bs.liveness():
        print("BS beacon NOT alive; everything below would be noise.")
        return 1
    if a.settle > 0:
        time.sleep(a.settle)
    print("BS %s armed%s. Capturing %d windows of %d frames.\n"
          % (a.bs, (", settled %.1f s" % a.settle) if a.settle > 0 else "", a.windows, a.frames + 1))

    dev = SoapySDR.Device(dict(driver="houdinisdr", remote="tcp://%s:55132" % a.ue,
                               timeout=RPC_TIMEOUT_US))
    dev.setSampleRate(SOAPY_SDR_RX, a.ch, RATE)
    tn.tune(dev, SOAPY_SDR_RX, a.ch, a.freq)
    ue_dummy_tx = None
    if a.ue_mts == "on":
        # The sounder's UE (ClientRadioSet + Radio.cc): its TX replay stream is
        # set up first, which acquires the DAC tiles and satisfies the driver's
        # first-up rule, then its RX stream joins the MTS group. Mirror that
        # with a never-activated TX replay stream (and the ch0 aux for tile-0
        # membership, as the Bs class does).
        if a.ch != 0:
            ue_aux = dev.setupStream(SOAPY_SDR_TX, SOAPY_SDR_CS16, [0],
                                     dict(tx_mode="replay", mts="true"))
        ue_dummy_tx = dev.setupStream(SOAPY_SDR_TX, SOAPY_SDR_CS16, [a.ch],
                                      dict(tx_mode="replay", mts="true"))
        tn.STREAM_MTS_RX[0] = True
    rxs = dev.setupStream(SOAPY_SDR_RX, SOAPY_SDR_CS16, [a.ch], tn.rx_stream_args(a.ch))
    dev.activateStream(rxs)

    senses = [("gold", gold), ("conj", np.conj(gold))]
    sense = None
    all_resid = []
    all_resid_i = []   # the same pairs, fractional-peak phase
    image_ratios = []  # wrong-sense over right-sense peak, per beacon
    spectra = []       # (centroid MHz, low-minus-high edge dB) per window
    core_off_2nd = CORE_OFF_2NDREP
    iq_stats = []      # (Q/I rms ratio, normalised <IQ>) per window
    lobe_steps = []    # phase(strong neighbour) - phase(peak), per beacon
    all_recs = []
    lag_pairs = []
    nsamps = (a.frames + 1) * FRAME
    for w in range(a.windows):
        c, err, _short = capture(dev, rxs, nsamps)
        if c is None:
            print("  window %d aborted: %s" % (w + 1, err))
            continue
        # Replica-independent chain statistics on the RAW samples: I/Q power
        # balance and I/Q correlation. On a real-sampled converter with a
        # digital mixer these should read 1.00 and 0.00 up to noise; a per-run
        # departure would be the first place an "image" could physically come
        # from, and would be measured without any replica assumption.
        # MEANS REMOVED: a DC offset would otherwise read as imbalance and
        # correlation. The offset itself is reported separately, in units of the
        # window's rms, because a per-arm DC offset is a chain state too.
        # BEACON-FREE STRETCHES ONLY. Over a whole window the beacon dominates
        # (47 dB above noise for 0.4 % of the samples is 200x the noise energy),
        # so the earlier whole-window version measured the beacon's own I/Q
        # product at the carrier phase, not the chain. Take the middle 80 % of
        # each frame, far from any beacon.
        quiet = np.concatenate([c[k*FRAME + FRAME//10 : k*FRAME + FRAME - FRAME//10]
                                for k in range(a.frames + 1)])
        mi, mq = float(np.mean(quiet.real)), float(np.mean(quiet.imag))
        ri, rq = quiet.real - mi, quiet.imag - mq
        ii = float(np.mean(ri**2)); qq = float(np.mean(rq**2))
        iq = float(np.mean(ri*rq)) / math.sqrt(ii*qq) if ii > 0 and qq > 0 else 0.0
        rms = math.sqrt(ii + qq)
        iq_stats.append((math.sqrt(qq/ii) if ii > 0 else 0.0, iq,
                         math.hypot(mi, mq) / rms if rms > 0 else 0.0))
        if a.dump_raw and w == 0:
            c.astype(np.complex64).tofile(a.dump_raw)
            print("  raw window 0 (%d samples, %d short reads) -> %s" % (len(c), _short, a.dump_raw))
        if sense is None:
            ratios = {}
            for nm, g in senses:
                _i, rr = find_beacon(c[:FRAME], g, a.corr_scale)
                ratios[nm] = rr
            if max(ratios.values()) < 0.05:
                print("  detector ratio %.3g: seeing noise, not a beacon." % max(ratios.values()))
                return 1
            if a.sense == "auto":
                chosen = max(ratios, key=ratios.get)
            else:
                chosen = a.sense
            sense = dict(senses)[chosen]
            print("  replica sense: %s (lag-product ratios gold %.3g, conj %.3g%s)"
                  % (chosen, ratios["gold"], ratios["conj"],
                     "" if a.sense != "auto" else "; auto-selected"))
        peaks, pos, ipeaks, recs = [], [], [], []
        for k in range(a.frames + 1):
            sl = c[k*FRAME:(k+1)*FRAME]
            idx, _r = find_beacon(sl, sense, a.corr_scale)
            if idx < 0:
                continue
            j, v = mf_peak(c, sense, k*FRAME + idx)
            if j is None:
                continue
            # The integer peak's neighbours, and a fractional-peak phase from
            # them: a parabola through |v| at j-1, j, j+1 gives the sub-sample
            # offset, and the complex value is interpolated linearly toward
            # the nearer neighbour. If a flip in the integer-peak phase is the
            # peak wandering between two samples under fractional timing, this
            # estimator does not flip; if the flip is physical, both do.
            vm = np.vdot(sense, c[j-1:j-1+len(sense)]) if j >= 1 else 0j
            vp = np.vdot(sense, c[j+1:j+1+len(sense)]) if j+1+len(sense) <= len(c) else 0j
            am, a0, ap_ = abs(vm), abs(v), abs(vp)
            den = (am - 2*a0 + ap_)
            tau = 0.5*(am - ap_)/den if den != 0 else 0.0
            tau = max(-0.5, min(0.5, tau))
            vi = v*(1-abs(tau)) + (vp if tau > 0 else vm)*abs(tau)
            peaks.append(v); pos.append(j); ipeaks.append(vi)
            # Two chain statistics, reported per run so the receive chain's
            # state travels with the result. READ THEM AGAINST THE NULL in
            # phase_probe_null.py, not against a rule of thumb: for this replica
            # a pure beacon gives a wrong/right peak ratio of 0.16 to 0.28
            # (tau-dependent) and a lobe phase step of ~0.01 rad; real windows
            # read 0.18 to 0.34 and 0.06 to 0.30. The one run that hopped by
            # 2.4 rad had two equal-magnitude adjacent samples 2.4 rad apart,
            # which a single lobe cannot produce; its waveform was not kept, so
            # --dump-raw is worth arming on every run until one is.
            other = np.conj(sense)
            vo = max((abs(np.vdot(other, c[jj:jj+len(sense)])) for jj in range(j-2, j+3)), default=0.0)
            image_ratios.append(vo / a0 if a0 else 0.0)
            nbv = vm if am >= ap_ else vp
            if abs(nbv) > 0.3 * a0:
                lobe_steps.append(float(np.angle(nbv / v)))
            recs.append((w, k, j, k*FRAME + idx, a0, float(np.angle(v)), am,
                         float(np.angle(vm)) if am else 0.0, ap_,
                         float(np.angle(vp)) if ap_ else 0.0, tau,
                         float(np.angle(vi))))
        all_recs.extend(recs)
        # THE TRANSMITTED BEACON'S SPECTRUM, replica-free: a 512-point FFT of
        # the first detected core in this window. Centroid in MHz and the
        # low-edge minus high-edge band power in dB. A flat beacon reads
        # centroid ~0 and tilt ~0; the 2026-09-03 hop run read +22 MHz and
        # -13 dB while its noise floor stayed flat (8.169), and ten plain runs
        # spread from -6 to +13 MHz. The tuning calls are the sounder's, so the
        # variation is in the base station's transmit chain per arm.
        if pos:
            j0 = pos[0] - core_off_2nd
            seg = np.array(c[j0:j0 + 496], dtype=np.complex128)
            if len(seg) == 496:
                X = np.fft.fftshift(np.fft.fft(seg, 512))
                P = np.abs(X) ** 2
                f = (np.arange(512) - 256) * (RATE / 512) / 1e6
                cen = float(np.sum(P * f) / np.sum(P))
                lo = float(np.sum(P[16:96])); hi = float(np.sum(P[416:496]))
                spectra.append((cen, 10 * math.log10(lo / hi) if lo > 0 and hi > 0 else 0.0))
        for k in range(len(ipeaks) - 1):
            lag = pos[k+1] - pos[k]
            if 0.9*FRAME < lag < 1.1*FRAME:
                all_resid_i.append(np.angle(np.conj(ipeaks[k]) * ipeaks[k+1]))
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
        # AP-67: the same peaks, at every lag up to --lag-max frames. Kept as
        # (lag_frames, raw phase difference) pairs; the per-frame advance is
        # removed at the end from the lag-1 population so every lag is
        # corrected by ONE fitted rate, not by its own.
        for m in range(2, a.lag_max + 1):
            for k in range(len(peaks) - m):
                lag = pos[k+m] - pos[k]
                if not ((m - 0.1)*FRAME < lag < (m + 0.1)*FRAME):
                    continue
                lag_pairs.append((m, np.angle(np.conj(peaks[k]) * peaks[k+m])))

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
    # THE SECOND ESTIMATOR, AND THE FRAMES BEHIND A BAD SUMMARY. Same pairs,
    # phase read at the fractional peak instead of the integer one.
    if len(all_resid_i) >= 10:
        phi = np.array(all_resid_i)
        Ri = float(np.abs(np.mean(np.exp(1j*phi))))
        sdi = float(np.sqrt(-2.0*np.log(Ri))) if Ri > 0 else float("inf")
        hi_, _ = np.histogram(phi, bins=8, range=(-math.pi, math.pi))
        print("  fractional-peak phase: R = %.4f  sd = %.3f rad  hist %s"
              % (Ri, sdi, " ".join("%d" % h for h in hi_)))
        print("  (integer-peak R above %.4f; a large gap says the integer peak"
              " wanders between samples)" % R)
    if spectra:
        print("  transmitted-beacon spectrum: centroid %+.1f..%+.1f MHz, low-edge minus high-edge %+.1f..%+.1f dB over %d windows"
              % (min(x[0] for x in spectra), max(x[0] for x in spectra),
                 min(x[1] for x in spectra), max(x[1] for x in spectra), len(spectra)))
    if iq_stats:
        qi = [x[0] for x in iq_stats]; xc = [x[1] for x in iq_stats]; dc = [x[2] for x in iq_stats]
        print("  raw I/Q (means removed): Q/I rms %.4f..%.4f, corr %+.4f..%+.4f, DC offset %.3f..%.3f of rms, %d windows"
              % (min(qi), max(qi), min(xc), max(xc), min(dc), max(dc), len(qi)))
        print("  (beacon-free stretches; circular noise: Q/I 1.000 +- %.3f, corr 0 +- %.3f)"
              % (1.0/math.sqrt(2*len(quiet)), 1.0/math.sqrt(len(quiet))))
    if image_ratios:
        ls = np.array(lobe_steps) if lobe_steps else np.array([0.0])
        print("  receive-chain state: image ratio mean %.3f (min %.3f max %.3f); "
              "lobe phase step to the strong neighbour mean %+.2f rad, sd %.2f, n %d"
              % (np.mean(image_ratios), np.min(image_ratios), np.max(image_ratios),
                 float(np.mean(ls)), float(np.std(ls)), len(lobe_steps)))
        print("  (null for a pure beacon: ratio 0.16-0.28 by tau, lobe step ~0.01;"
              " see phase_probe_null.py. A hop between two half-sample-split"
              " samples moves the integer-peak phase by the lobe step.)")
    if all_recs:
        # Flag the pairs outside 1 rad of the main cluster and show what the
        # frames on both sides looked like: did the peak position step, did
        # the magnitude dip (a split peak), did the neighbour hold the energy.
        adv0 = float(np.angle(np.mean(np.exp(1j*ph))))
        byw = {}
        for r in all_recs:
            byw.setdefault(r[0], []).append(r)
        nflag = 0
        print("\n  frames behind the outliers (window, frame, dpos, |v|/|v|max, "
              "|v-1|/|v|, |v+1|/|v|, tau, resid rad):")
        for w_, rs in sorted(byw.items()):
            vmax = max(r[4] for r in rs) or 1.0
            for i in range(1, len(rs)):
                r0, r1 = rs[i-1], rs[i]
                lag = r1[2] - r0[2]
                if not (0.9*FRAME < lag < 1.1*FRAME):
                    continue
                d = float(np.angle(np.exp(1j*(r1[5] - r0[5] - adv0))))
                if abs(d) > 1.0 and nflag < 24:
                    nflag += 1
                    print("   w%-2d k%-2d dpos %+d  |v| %.2f  nb- %.2f  nb+ %.2f  tau %+.2f  resid %+.2f"
                          % (w_, r1[1], lag - FRAME, r1[4]/vmax, r1[6]/r1[4],
                             r1[8]/r1[4], r1[10], d))
        print("  (%d outlier pairs in total)" % sum(
            1 for w_, rs in byw.items() for i in range(1, len(rs))
            if 0.9*FRAME < rs[i][2]-rs[i-1][2] < 1.1*FRAME and
            abs(float(np.angle(np.exp(1j*(rs[i][5]-rs[i-1][5]-adv0))))) > 1.0))
    if a.dump:
        with open(a.dump, "w") as f:
            f.write("window,frame,pos,det_pos,mag,phase,mag_m1,phase_m1,"
                    "mag_p1,phase_p1,tau,phase_interp\n")
            for r in all_recs:
                f.write(",".join("%s" % x for x in r) + "\n")
        print("  dumped %d records to %s" % (len(all_recs), a.dump))
    if a.lag_max > 1 and lag_pairs:
        # The per-frame phase advance, from the lag-1 population (circular
        # mean), removed k times from a lag-k difference. What remains is the
        # phase the beacon did NOT predict: the innovation a tracker holding
        # frequency fixed would see after k frames.
        adv = float(np.angle(np.mean(np.exp(1j*ph))))
        print("\n=== phase innovation against lag (AP-67): advance/frame %+.4f rad ===" % adv)
        print("  %-4s %6s %10s %10s %12s" % ("lag", "pairs", "circ sd", "sd deg", "sd/sqrt(lag)"))
        for m in range(1, a.lag_max + 1):
            v = ph if m == 1 else np.array([d for (mm, d) in lag_pairs if mm == m])
            if len(v) < 5:
                continue
            inn = np.angle(np.exp(1j*(v - m*adv)))
            Rm = float(np.abs(np.mean(np.exp(1j*inn))))
            sdm = float(np.sqrt(-2.0*np.log(Rm))) if Rm > 0 else float("inf")
            print("  %-4d %6d %10.3f %10.1f %12.3f" % (m, len(v), sdm, math.degrees(sdm),
                                                      sdm/math.sqrt(m)))
        print("  a random-walk phase gives sd/sqrt(lag) constant; a frequency error")
        print("  gives sd growing linearly with lag; the floor at lag 1 is the")
        print("  estimator's own noise.")
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
