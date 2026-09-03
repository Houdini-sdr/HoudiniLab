#!/usr/bin/env python3
"""Which beacon works best, measured on silicon, on the same link, interleaved.

[user 2026-09-02] "try each of the beacon types and report on tracking
frequency / CFO / timing correction based on each. ie which one works the best,
which one we want to keep as default."

Four candidates, all built by include/beacon_shapes.h and dumped by
beacon_shape_dump, so the waveform this probe transmits is sample-for-sample the
waveform Config::genPilots will build. That is not a nicety: AP-34(a) cost a
bench session because the bench and the build disagreed about a beacon.

  legacy        15 x STS(16) + 2 x gold(128)      -- what we ship
  legacy_guard  the same, with an 802.11 GI2 cyclic guard before the gold
  dot11         the actual 802.11 legacy preamble: STF(160) + GI2 + 2 x LTS(64)
  nr            NR PSS (38.211 7.4.2.2) + CP + 2 x a 38.211 5.2.1 CSI-RS symbol

WHAT IS MEASURED, per shape:
  timing     residual of the beacon arrival against the predicted frame grid --
             sd and worst case. This IS the timing correction quality: it is the
             number the scatter tolerance has to cover.
  frequency  the residual slope over the run, in ppm. Two clocks free-running,
             so this is the tracked clock difference the sync loop has to hold.
  CFO        from the fine field's repeated pair, the same estimator
             Receiver::estimateCFO uses, plus the circular resultant R across
             detections. R is the honest part: a stable-looking mean CFO with
             R near 0 is noise that happens to average.
  detection  fraction of predicted arrivals that produced a detection, the
             detector ratio distribution, and the in-window SNR.

INTERLEAVED, NOT BACK TO BACK. Shapes run round-robin with the order rotated
each round. A previous campaign on this bench produced a confident "2.8x
instrument disagreement" that was entirely an artifact of comparing two
non-interleaved runs, and it was reported to another lane before it was
retracted. Anything that drifts with time or temperature now hits all four
shapes equally instead of loading onto whichever ran last.

Reuses two_node_beacon_arrival's device wiring so there is one BS arm path and
one UE read path on this bench, not two that can drift apart.

  python3 beacon_shape_compare.py --shapes-dir DIR --rounds 3 --matches 60
"""
import argparse
import json
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import two_node_beacon_arrival as tn  # noqa: E402

RATE = tn.RATE
FRAME = tn.FRAME


def load_shape(d, name, geom):
    """Read one shape's transmit core and correlator replica off disk.

    The core arrives as the ci16 the DAC would replay; it is CONJUGATED here for
    the same reason beacon_tx_gold conjugates: the matched-NCO real->complex
    mixer delivers the beacon conjugated and the detector correlates raw RX, so
    pre-conjugating the transmit cancels it.
    """
    core = np.fromfile(os.path.join(d, "%s_core.bin" % name), dtype=np.int16)
    cc = core[0::2].astype(np.float64) - 1j * core[1::2]
    if len(cc) != geom["core_len"]:
        raise RuntimeError("%s_core.bin has %d samples, shapes.json says %d"
                           % (name, len(cc), geom["core_len"]))
    pk = np.abs(cc).max()
    ram = np.zeros(2 * 4096, dtype=np.int16)
    ram[0:2 * len(cc):2] = np.round(cc.real / pk * 0.6 * 32767).astype(np.int16)
    ram[1:2 * len(cc):2] = np.round(cc.imag / pk * 0.6 * 32767).astype(np.int16)
    rep = np.fromfile(os.path.join(d, "%s_replica.bin" % name),
                      dtype=np.complex64).astype(np.complex128)
    if len(rep) != geom["replica_len"]:
        raise RuntimeError("%s_replica.bin has %d taps, shapes.json says %d"
                           % (name, len(rep), geom["replica_len"]))
    return ram, rep


def core_off(geom):
    """Where find_beacon's returned index sits, relative to the core start.

    NOT a constant, and NOT copied from the legacy value. This probe's detector
    correlates so that gc peaks at the START of a matched field, so the lag
    product peaks at the start of the SECOND fine repetition. Derived from the
    shape geometry so it cannot go stale when a shape changes -- the hard-coded
    368 in two_node_beacon_arrival is exactly the legacy case of this.
    """
    return geom["fine_off"] + geom["fine_len"]


def synth_beacon(core_ci16, geom, cfo_hz, snr_db_want, n, pos, seed):
    """A synthetic capture with a KNOWN CFO at a KNOWN position.

    The instrument gets validated against a known-good case before its output is
    believed. The first version of the reader below returned a wrong answer with
    a circular resultant of exactly 1.000 -- maximum apparent confidence.
    """
    rng = np.random.default_rng(seed)
    core = (core_ci16[0::2].astype(np.float64)
            + 1j * core_ci16[1::2].astype(np.float64)) / 32767.0
    core = core[:geom["core_len"]]
    sig_p = float(np.mean(np.abs(core) ** 2))
    c = (rng.standard_normal(n) + 1j * rng.standard_normal(n)) / np.sqrt(2.0)
    c *= np.sqrt(sig_p / (10 ** (snr_db_want / 10.0)))
    t = np.arange(len(core))
    c[pos:pos + len(core)] += core * np.exp(2j * np.pi * cfo_hz * t / RATE)
    return c


def cfo_from_fine(c, core_start, geom, conj_sense):
    """CFO in Hz from the fine field's repeated pair.

    The same estimator Receiver::estimateCFO uses on the gold pair, generalised
    to the shape: correlate repetition 1 against repetition 2 at lag fine_len,
    which is a phase ramp of 2*pi*f*fine_len/RATE.

    Returns (hz, |r|) or (nan, 0). |r| is the normalised correlation magnitude:
    a CFO read off a pair that did not actually correlate is not a measurement,
    and returning 0.0 for it -- as an earlier version of the C++ estimator did --
    lets a fabricated number be averaged in with no symptom.
    """
    g1 = core_start + geom["fine_off"]
    g2 = g1 + geom["fine_len"]
    n = geom["fine_len"]
    if g1 < 0 or g2 + n > len(c):
        return float("nan"), 0.0
    a = c[g1:g1 + n]
    b = c[g2:g2 + n]
    r = np.vdot(a, b)                      # sum conj(a) * b
    na = np.linalg.norm(a) * np.linalg.norm(b)
    if na <= 0:
        return float("nan"), 0.0
    ph = float(np.angle(r))
    if conj_sense:
        ph = -ph      # the received beacon is the conjugate of the reference
    return ph / (2.0 * np.pi * n) * RATE, float(abs(r) / na)


def snr_db(c, core_start, geom):
    """In-window SNR: core power against a same-length window ahead of it."""
    L = geom["core_len"]
    if core_start < L or core_start + L > len(c):
        return float("nan")
    sig = float(np.mean(np.abs(c[core_start:core_start + L]) ** 2))
    noi = float(np.mean(np.abs(c[core_start - L:core_start]) ** 2))
    if noi <= 0 or sig <= 0:
        return float("nan")
    return 10.0 * np.log10(sig / noi)


def run_one(ue, rep, geom, corr_scale, matches, max_windows, dwell_s):
    """Acquire, then track for `dwell_s` seconds. Returns a dict of measurements.

    TIME-BOXED, NOT MATCH-COUNTED. The first version collected a fixed 80
    detections, which on this bench arrive about every 2 frames, so each shape
    was tracked for 0.19 SECONDS. Over that span a 0.01 ppm clock difference
    moves the beacon by a fifth of a sample, so every residual quantised to
    exactly 0 and every eps came out of a straight-line fit through a constant.
    The run looked immaculate -- sd 0.00, max 0 -- and measured nothing about
    tracking. A rate needs a baseline.
    """
    off2 = core_off(geom)
    senses = [("gold", rep), ("conj", np.conj(rep))]
    anchor = None
    sense = None
    windows = 0
    for w in range(400):
        if tn.over_budget():
            break
        windows = w + 1
        tk, c = ue.window()
        if tk is None:
            continue
        for nm, g in senses:
            idx, ratio = tn.find_beacon(c, g, corr_scale)
            if idx >= 0 and ratio >= tn.MIN_RATIO[0]:
                anchor = tk + idx - off2
                sense = (nm, g)
                break
        if anchor is not None:
            break
    if anchor is None:
        return {"locked": False, "acq_windows": windows}

    nm, g = sense
    conj_sense = nm == "conj"
    resids, ks, ratios, cfos, rmags, snrs = [], [], [], [], [], []
    attempts = 0
    t_end = time.time() + dwell_s
    # The slice needs a FULL core length of samples ahead of the beacon, or
    # snr_db has no noise window to measure against and silently returns nan --
    # which is what the first campaign did for every shape in every round.
    lead = geom["core_len"] + 256
    for w in range(max_windows):
        if len(resids) >= matches or tn.over_budget() or time.time() > t_end:
            break
        tk, c = ue.window()
        if tk is None:
            continue
        k = int(round((tk + 2048 - anchor) / FRAME))
        pred = anchor + k * FRAME
        o = pred - tk
        span = geom["core_len"] + 384
        if not (lead <= o < len(c) - span):
            continue
        lo = o - lead
        sl = c[lo:o + span]
        attempts += 1
        idx, rr = tn.find_beacon(sl, g, corr_scale)
        if idx < 0 or rr < tn.MIN_RATIO[0]:
            continue
        meas = tk + lo + idx - off2
        resids.append(meas - pred)
        ks.append(k)
        ratios.append(rr)
        # Core start INSIDE THE SLICE. `meas` is an absolute tick and `lo` is the
        # slice origin, so the slice-relative index is idx - off2 and NOT
        # lo + idx - off2, which is the index into the parent window. Passing the
        # latter alongside the slice fed the CFO and SNR readers a location
        # thousands of samples from the beacon. They did not fail; they returned
        # +682844 Hz with a circular resultant of exactly 1.000.
        cs = idx - off2
        hz, rm = cfo_from_fine(sl, cs, geom, conj_sense)
        cfos.append(hz)
        rmags.append(rm)
        snrs.append(snr_db(sl, cs, geom))

    r = np.array(resids, dtype=np.float64)
    kk = np.array(ks, dtype=np.float64)
    out = {"locked": True, "acq_windows": windows, "sense": nm,
           "n": len(r), "attempts": attempts,
           "detect_frac": (len(r) / attempts) if attempts else 0.0,
           "resid": resids, "k": ks, "ratio": ratios,
           "cfo_hz": [None if np.isnan(x) else x for x in cfos],
           "cfo_r": rmags,
           "snr_db": [None if np.isnan(x) else x for x in snrs]}
    if len(r) >= 3:
        out["span_s"] = float((max(ks) - min(ks)) * FRAME / RATE)
        out["resid_sd"] = float(np.std(r))
        out["resid_max"] = float(np.max(np.abs(r)))
        # ppm: residual grows by (eps * FRAME) samples per frame, and k counts
        # frames. Positive slope = the UE clock is SLOW relative to the BS.
        sl_, ic_ = np.polyfit(kk, r, 1)
        out["eps_ppm"] = float(sl_ / FRAME * 1e6)
        # THE BEACON'S OWN TIMING QUALITY IS THE SCATTER ABOUT THE RAMP, NOT THE
        # RAW SPREAD. The raw spread is dominated by the clock ramp across the
        # leg: measured on this bench, eps -0.0007 / +0.0008 / +0.0102 / +0.0343
        # ppm over four sequential 20 s legs gives 1.7 / 2.0 / 25 / 84 samples of
        # drift, and the raw sd and max reproduced exactly that ordering. Reading
        # it as a beacon property would have ranked the candidates by which
        # minute of the session they happened to run in.
        out["resid_sd_detrended"] = float(np.std(r - (sl_ * kk + ic_)))
        out["resid_max_detrended"] = float(np.max(np.abs(r - (sl_ * kk + ic_))))
        fin = [x for x in cfos if not np.isnan(x)]
        if fin:
            out["cfo_hz_mean"] = float(np.mean(fin))
            out["cfo_hz_sd"] = float(np.std(fin))
            # THE FINE-PAIR COHERENCE, not a circular resultant over the CFO
            # values. The first version computed the latter and it read 1.000 to
            # three decimals for every shape in every round -- including rounds
            # whose CFO spread was 3134 Hz -- because at these frequencies the
            # per-detection phase is under 0.01 rad, so the resultant of a set of
            # near-zero angles is 1 no matter how they scatter. A metric that
            # cannot go down is not a check. |r| from the pair itself can.
            out["cfo_R"] = float(np.mean(rmags))
        out["ratio_med"] = float(np.median(ratios))
        out["ratio_min"] = float(np.min(ratios))
        fs = [x for x in snrs if not np.isnan(x)]
        if fs:
            out["snr_med"] = float(np.median(fs))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bs-ip", default="168.6.244.21")
    ap.add_argument("--ue-ip", default="168.6.244.22")
    ap.add_argument("--shapes-dir", required=True)
    ap.add_argument("--shapes", default="legacy,legacy_guard,dot11,nr")
    ap.add_argument("--rounds", type=int, default=3)
    ap.add_argument("--matches", type=int, default=4000,
                    help="hard cap on detections per shape per round; the "
                         "DWELL is what normally ends a leg")
    ap.add_argument("--dwell", type=float, default=30.0,
                    help="seconds of tracking per shape per round. A rate needs "
                         "a baseline: at 0.01 ppm, one sample of drift takes 0.8 s")
    ap.add_argument("--corr-scale", type=float, default=10.0)
    ap.add_argument("--min-ratio", type=float, default=1e-2)
    ap.add_argument("--budget", type=float, default=2400.0)
    ap.add_argument("--tx-ch", type=int, default=1)
    ap.add_argument("--rx-ch", type=int, default=1)
    ap.add_argument("--out", default="beacon_shape_compare.json")
    ap.add_argument("--self-test", action="store_true",
                    help="validate the CFO and SNR readers against synthetic "
                         "captures with known answers, then exit. No hardware.")
    args = ap.parse_args()
    tn.DEADLINE[0] = time.time() + args.budget
    tn.MIN_RATIO[0] = args.min_ratio

    with open(os.path.join(args.shapes_dir, "shapes.json")) as f:
        shapes_json = json.load(f)["shapes"]
    names = [s for s in args.shapes.split(",") if s]
    for n in names:
        if n not in shapes_json:
            print("no geometry for shape %r in shapes.json" % n)
            return 2
    loaded = {n: load_shape(args.shapes_dir, n, shapes_json[n]) for n in names}

    if args.self_test:
        # ONE NOISE DRAW IS NOT A MEASUREMENT OF AN ESTIMATOR. The first version
        # of this self-test used a single fixed seed, so every "error" it printed
        # was one realisation, and it duly reported legacy_guard biased +78 Hz
        # and nr +140 Hz while legacy sat at -8. Those looked like per-shape
        # systematic offsets. They were the same draw scaled by each shape's own
        # noise sensitivity: at 20 dB every one of them grew by exactly the
        # sqrt(SNR) ratio, which is the signature of noise, not of bias.
        #
        # So: many draws, and separate what a bias and a spread actually are.
        # BIAS is the correctness claim -- an estimator that is systematically
        # off will read a wrong CFO no matter how long you average. SPREAD is a
        # sensitivity, expected to differ between shapes, and is REPORTED rather
        # than gated because a shorter fine field is legitimately noisier.
        bad = 0
        ndraw = 64
        print("\nself-test: known CFO and SNR injected, %d draws each, "
              "no hardware\n" % ndraw)
        print("%-13s %9s %8s %9s %8s %8s %9s %8s" %
              ("shape", "CFO want", "bias Hz", "spread Hz", "R", "SNR want",
               "SNR bias", "verdict"))
        for nm in names:
            g = shapes_json[nm]
            core = np.fromfile(os.path.join(args.shapes_dir, "%s_core.bin" % nm),
                               dtype=np.int16)
            for want_snr in (45.0, 20.0):
                for want_hz in (0.0, 250.0, -1200.0, 4250.0):
                    pos = 700
                    errs, rs, sns = [], [], []
                    for sd in range(ndraw):
                        c = synth_beacon(core, g, want_hz, want_snr, 4096, pos,
                                         1000 + sd)
                        hz, r = cfo_from_fine(c, pos, g, False)
                        errs.append(hz - want_hz)
                        rs.append(r)
                        sns.append(snr_db(c, pos, g))
                    bias = float(np.mean(errs))
                    spread = float(np.std(errs))
                    # The bias must be small compared with the spread it hides
                    # in: a systematic error worth catching is one that survives
                    # averaging, i.e. bigger than the standard error of the mean.
                    sem = spread / np.sqrt(ndraw)
                    ok_cfo = abs(bias) < max(3.0 * sem, 5.0)
                    snb = float(np.mean(sns)) - want_snr
                    ok_snr = abs(snb) < 3.0
                    if not (ok_cfo and ok_snr):
                        bad += 1
                    print("%-13s %9.1f %8.2f %9.1f %8.3f %8.1f %9.2f %8s"
                          % (nm, want_hz, bias, spread, float(np.mean(rs)),
                             want_snr, snb,
                             "ok" if (ok_cfo and ok_snr) else
                             ("CFO-BIAS" if not ok_cfo else "SNR")))
        print("\nself-test: %s (%d failure(s))" % ("PASS" if not bad else "FAIL", bad))
        print("spread is a sensitivity, not a defect: a 64-sample fine field is")
        print("legitimately noisier than a 128-sample one. Only bias is gated.")
        return 1 if bad else 0
    for n in names:
        g = shapes_json[n]
        print("%-13s core %4d  replica %3d  fine@%d x%d  index off %d  PAPR %.2f dB"
              % (n, g["core_len"], g["replica_len"], g["fine_off"],
                 g["fine_len"], core_off(g), g["papr_db"]))

    out = {"rounds": [], "shapes": shapes_json, "argv": vars(args)}
    rc = 0
    ue = None
    try:
        ue = tn.Ue(args.ue_ip, args.rx_ch)
        for rnd in range(args.rounds):
            # Rotate the order every round: whatever drifts with time hits each
            # shape in a different slot instead of always the same one.
            order = names[rnd % len(names):] + names[:rnd % len(names)]
            print("\n=== round %d/%d, order %s ===" % (rnd + 1, args.rounds,
                                                       ",".join(order)))
            rec = {}
            for nm in order:
                if tn.over_budget():
                    print("  (budget) stopping before %s" % nm)
                    break
                ram, rep = loaded[nm]
                bs = tn.Bs(args.bs_ip, ram, args.tx_ch)
                try:
                    bs.open_and_arm()
                    if not bs.liveness(settle=0.6):
                        print("  %-13s BEACON NOT PLAYING -- skipped" % nm)
                        rec[nm] = {"locked": False, "reason": "liveness"}
                        continue
                    r = run_one(ue, rep, shapes_json[nm], args.corr_scale,
                                args.matches, args.matches * 30, args.dwell)
                finally:
                    bs.close()
                rec[nm] = r
                if not r.get("locked"):
                    print("  %-13s NO LOCK in %d windows" % (nm, r["acq_windows"]))
                    continue
                print("  %-13s n=%-4d %5.1fs det=%4.0f%%  jitter sd %5.2f max %4.0f  "
                      "eps %+.4f ppm  CFO %+8.1f Hz (sd %6.1f r %.4f)  "
                      "ratio med %6.2f min %6.2f  SNR %5.1f dB  acq %d win"
                      % (nm, r["n"], r.get("span_s", 0.0), 100 * r["detect_frac"],
                         r.get("resid_sd_detrended", float("nan")),
                         r.get("resid_max_detrended", float("nan")),
                         r.get("eps_ppm", float("nan")),
                         r.get("cfo_hz_mean", float("nan")),
                         r.get("cfo_hz_sd", float("nan")),
                         r.get("cfo_R", float("nan")),
                         r.get("ratio_med", float("nan")),
                         r.get("ratio_min", float("nan")),
                         r.get("snr_med", float("nan")), r["acq_windows"]))
            out["rounds"].append(rec)
    except KeyboardInterrupt:
        print("interrupted")
        rc = 130
    finally:
        if ue is not None:
            ue.close()
        with open(args.out, "w") as f:
            json.dump(out, f, indent=1)
        print("\nwrote %s" % args.out)

    # Summary across rounds. Reported per shape with the ROUND-TO-ROUND spread
    # beside every mean, because a single round is one sample and this bench has
    # already produced one confident conclusion from an unreplicated pair.
    print("\njitter = residual scatter about the fitted clock ramp: the beacon's")
    print("own timing quality. eps = that ramp, a clock property the legs share.\n")
    print("%-13s %6s %6s %7s %9s %9s %10s %9s %8s %8s %7s" %
          ("shape", "rounds", "n", "span s", "jitter sd", "jitter max",
           "eps ppm", "CFO sd", "pair r", "SNR dB", "det %"))
    for nm in names:
        rs = [r[nm] for r in out["rounds"] if nm in r and r[nm].get("n", 0) >= 3]
        if not rs:
            print("%-13s %6d  -- no usable round --" % (nm, 0))
            continue
        def col(k):
            v = [x[k] for x in rs if k in x]
            return (np.mean(v), np.std(v)) if v else (float("nan"),) * 2
        sd = col("resid_sd_detrended"); mx = col("resid_max_detrended")
        ep = col("eps_ppm")
        cs = col("cfo_hz_sd"); cr = col("cfo_R"); df = col("detect_frac")
        sp = col("span_s"); sn = col("snr_med")
        print("%-13s %6d %6d %7.1f %9.2f %9.1f %+10.4f %9.1f %8.4f %8.1f %7.0f"
              % (nm, len(rs), int(np.sum([x["n"] for x in rs])), sp[0], sd[0],
                 mx[0], ep[0], cs[0], cr[0], sn[0], 100 * df[0]))
        print("%-13s %6s %6s %7.1f %9.2f %9.1f %10.4f %9.1f %8.4f %8.1f %7.0f   (spread)"
              % ("", "", "", sp[1], sd[1], mx[1], ep[1], cs[1], cr[1], sn[1],
                 100 * df[1]))
    return rc


if __name__ == "__main__":
    sys.exit(main())
