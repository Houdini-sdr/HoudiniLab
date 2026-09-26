#!/usr/bin/env python3
"""AP-34(b): the three-stage beacon CFO resolution ladder, offline and synthetic.

WHY THIS IS AN OFFLINE INSTRUMENT AND NOT A CHANGE TO receiver.cc.
Stage 3 correlates the beacon carrier phase between CONSECUTIVE frames, lag =
one frame. The live client cannot supply that: its resync cadence is 260 frames,
which makes the lag 31.9 M samples and the unambiguous range +-1.92 Hz, and it
could not be sped up to fix that because the client loop runs 412-746 iter/s
against 1000 frames/s and physically cannot observe two consecutive frames. One
contiguous multi-frame capture, post-processed, supplies every stage at once.
So the ladder costs the sync path nothing and cannot regress it.

THE LADDER. Precision scales with the correlation lag; unambiguous range scales
inversely. Each stage unwraps the next.

  stage 1  STS pairs, lag 16          +-3.84 MHz     ~1.7 kHz
  stage 2  gold rep2 vs rep1, lag 128 +-480 kHz      ~405 Hz measured (AP-39)
  stage 2a stage 2 averaged over M    same range     405/sqrt(M) Hz
  stage 3  frame to frame, lag 122880 +-500 Hz       ~1 Hz

Stages 1 and 2 are exactly receiver.cc's estimateCFO, replicated here (and in
cfo_model.py, which validates that pair on its own). This file adds 2a and 3 and
checks the thing that actually decides whether the ladder works: whether the
averaged stage 2 is accurate enough to unwrap stage 3.

THE PART THAT MATTERS FOR SEQUENCING. Averaging kills stage 2's NOISE and not
its BIAS. Before AP-39's index guard that bias was +1487 Hz, three ambiguity
steps of stage 3, so no amount of averaging would have unwrapped it. AP-39 is a
PREREQUISITE for AP-34(b), not an unrelated improvement, and this model
demonstrates that rather than asserting it.

    python3 tests/demo-verify/cfo_ladder_model.py
"""
import cmath
import math
import random
import sys

# Beacon geometry, Config::genBeacon. Identical to cfo_model.py on purpose.
STS_LEN, STS_REPS, GOLD_LEN, GOLD_REPS = 16, 15, 128, 2
CORE = STS_LEN * STS_REPS + GOLD_LEN * GOLD_REPS
FS = 122.88e6
N_FFT = 64
FRAME = 122880           # 30 slots x 4096
F_CARRIER = 500e6
HOUDINI = True           # both shipped configs are radio_type houdini
DF_SC = FS / N_FFT       # subcarrier spacing, 1.92 MHz

random.seed(11)
_sts = [complex(random.gauss(0, 1), random.gauss(0, 1)) for _ in range(STS_LEN)]
_gold = [complex(random.gauss(0, 1), random.gauss(0, 1)) for _ in range(GOLD_LEN)]
BEACON = _sts * STS_REPS + _gold * GOLD_REPS
assert len(BEACON) == CORE == 496


# ---------------------------------------------------------------- stages 1, 2
def stage12(core):
    """receiver.cc estimateCFO, verbatim arithmetic. Returns normalized CFO."""
    g1 = STS_LEN * STS_REPS
    g2 = g1 + GOLD_LEN
    r_fine = sum(core[g1 + i].conjugate() * core[g2 + i] for i in range(GOLD_LEN))
    r_coarse = 0j
    for k in range(STS_REPS - 1):
        for i in range(STS_LEN):
            r_coarse += core[k * STS_LEN + i].conjugate() * core[(k + 1) * STS_LEN + i]
    if abs(r_fine) == 0.0 or abs(r_coarse) == 0.0:
        return float("nan")
    f_fine = cmath.phase(r_fine) / (2 * math.pi * GOLD_LEN)
    f_coarse = cmath.phase(r_coarse) / (2 * math.pi * STS_LEN)
    amb = 1.0 / GOLD_LEN
    f = f_fine + round((f_coarse - f_fine) / amb) * amb
    return -f if HOUDINI else f


# -------------------------------------------------------------------- stage 3
def stage3(core_a, core_b, lag, coarse_norm):
    """Frame-to-frame carrier phase.

    `lag` is the ACTUAL sample distance between the two cores, which we know
    exactly from their detected positions, so a drifting frame period costs
    nothing here. `coarse_norm` unwraps the result into the right ambiguity bin;
    it must be accurate to better than half of 1/lag, i.e. +-500 Hz at lag =
    one frame. That requirement is the whole reason stage 2a exists.
    """
    r = sum(core_a[i].conjugate() * core_b[i] for i in range(CORE))
    if abs(r) == 0.0:
        return float("nan")
    f = cmath.phase(r) / (2 * math.pi * lag)
    if HOUDINI:
        f = -f
    amb = 1.0 / lag
    return f + round((coarse_norm - f) / amb) * amb


# ------------------------------------------------------------------ the stream
def synth(eps, n_frames, snr_db=None, stage2_bias_hz=0.0):
    """N frames of beacons on a clock offset by `eps` (fractional).

    SCO and CFO are ONE number here, because both derive from the one LMK
    reference: the frame period stretches by eps AND the carrier shifts by
    eps*f_c. Modelling them separately would let the ladder pass while the
    physical claim behind it was wrong.

    Returns (cores, positions). The carrier rotation is applied against ABSOLUTE
    stream position, so phase is continuous across frames -- without that,
    stage 3 has nothing to measure.
    """
    period = FRAME * (1.0 + eps)
    fn = eps * F_CARRIER / FS          # normalized carrier offset
    bias_n = stage2_bias_hz / FS
    cores, pos = [], []
    for k in range(n_frames):
        p = int(round(k * period))
        c = []
        for i in range(CORE):
            n = p + i
            # `bias_n` models an index-misalignment bias as a small extra
            # rotation INSIDE the core: it moves stages 1/2, which read one
            # core, and not stage 3, which differences two cores a frame apart.
            # That asymmetry is exactly why the bias survives averaging and
            # still breaks the unwrap.
            x = BEACON[i] * cmath.exp(2j * math.pi * (fn * n + bias_n * i))
            if HOUDINI:
                x = x.conjugate()
            c.append(x)
        if snr_db is not None:
            sp = sum(abs(v) ** 2 for v in c) / CORE
            s = math.sqrt((sp / (10 ** (snr_db / 10.0))) / 2)
            c = [v + complex(random.gauss(0, s), random.gauss(0, s)) for v in c]
        cores.append(c)
        pos.append(p)
    return cores, pos


def ladder(eps, n_frames=100, snr_db=None, stage2_bias_hz=0.0):
    cores, pos = synth(eps, n_frames, snr_db, stage2_bias_hz)
    s2 = [stage12(c) for c in cores]
    s2 = [v for v in s2 if v == v]
    s2_avg = sum(s2) / len(s2)
    s3 = []
    for k in range(len(cores) - 1):
        lag = pos[k + 1] - pos[k]
        v = stage3(cores[k], cores[k + 1], lag, s2_avg)
        if v == v:
            s3.append(v)
    return {
        "truth_hz": eps * F_CARRIER,
        "s2_each_hz": [v * FS for v in s2],
        "s2_avg_hz": s2_avg * FS,
        "s3_avg_hz": (sum(s3) / len(s3)) * FS if s3 else float("nan"),
        "s3_each_hz": [v * FS for v in s3],
        "lag": pos[1] - pos[0],
    }


def sd(xs):
    if len(xs) < 2:
        return 0.0
    m = sum(xs) / len(xs)
    return math.sqrt(sum((x - m) ** 2 for x in xs) / (len(xs) - 1))


fails = []


def check(ok, what):
    print(("PASS  " if ok else "FAIL  ") + what)
    if not ok:
        fails.append(what)


def main():
    print("beacon core %d samples, frame %d, fs %.2f MSPS, carrier %.0f MHz"
          % (CORE, FRAME, FS / 1e6, F_CARRIER / 1e6))
    print("stage 3 lag = one frame -> unambiguous range +-%.1f Hz\n"
          % (FS / (2.0 * FRAME)))

    # ---- 1. the ranges are what the row claims ----------------------------
    print("=== stage ranges ===")
    for name, lag in (("stage 1 (STS, lag 16)", STS_LEN),
                      ("stage 2 (gold, lag 128)", GOLD_LEN),
                      ("stage 3 (frame, lag %d)" % FRAME, FRAME)):
        print("  %-32s +-%12.1f Hz" % (name, FS / (2.0 * lag)))
    check(abs(FS / (2.0 * FRAME) - 500.0) < 1.0,
          "stage 3's unambiguous range is +-500 Hz")
    check(FS / (2.0 * GOLD_LEN) > 4.5e5,
          "stage 2's range (+-480 kHz) covers any offset stage 3 cannot")
    print()

    # ---- 2. noiseless: the ladder recovers a known offset ------------------
    print("=== noiseless recovery, eps swept ===")
    for eps_ppm in (0.0, 0.06, 1.0, 8.52, 40.0):
        r = ladder(eps_ppm * 1e-6, n_frames=20)
        err2 = r["s2_avg_hz"] - r["truth_hz"]
        err3 = r["s3_avg_hz"] - r["truth_hz"]
        print("  eps %7.2f ppm -> truth %+9.1f Hz | stage2 err %+9.2f | "
              "stage3 err %+9.4f" % (eps_ppm, r["truth_hz"], err2, err3))
        check(abs(err3) < 0.5,
              "noiseless stage 3 is within 0.5 Hz at eps = %g ppm" % eps_ppm)
    print()

    # ---- 3. with noise: precision, and that stage 3 beats stage 2 ---------
    print("=== at 47 dB SNR (the measured cabled-bench figure), eps 8.52 ppm ===")
    r = ladder(8.52e-6, n_frames=100, snr_db=47.0)
    s2sd, s3sd = sd(r["s2_each_hz"]), sd(r["s3_each_hz"])
    n2 = len(r["s2_each_hz"])
    print("  truth              %+10.2f Hz" % r["truth_hz"])
    print("  stage 2 per-shot   sd %8.2f Hz over %d shots" % (s2sd, n2))
    print("  stage 2 averaged   %+10.2f Hz  (err %+.2f, sem %.2f)"
          % (r["s2_avg_hz"], r["s2_avg_hz"] - r["truth_hz"], s2sd / math.sqrt(n2)))
    print("  stage 3 per-shot   sd %8.4f Hz" % s3sd)
    print("  stage 3 averaged   %+10.4f Hz  (err %+.4f)"
          % (r["s3_avg_hz"], r["s3_avg_hz"] - r["truth_hz"]))
    gain = s2sd / s3sd if s3sd else float("inf")
    print("  precision gain stage 2 -> stage 3: %.0fx (lag ratio %.0fx)"
          % (gain, FRAME / GOLD_LEN))
    check(s3sd < s2sd / 100.0,
          "stage 3 is at least 100x more precise per shot than stage 2")
    check(abs(r["s3_avg_hz"] - r["truth_hz"]) < 1.0,
          "stage 3 lands within 1 Hz of truth at 47 dB SNR")
    print()

    # ---- 3b. CALIBRATE AGAINST THE BENCH BEFORE BELIEVING THE PREDICTION --
    # The leg above is thermal noise only, and it gives stage 2 a 62 Hz scatter
    # where the bench measured 405 Hz (AP-39). The bench carries phase noise and
    # detector index jitter this model does not, so quoting stage 3's precision
    # from the 47 dB leg would be quoting an instrument against itself. Find the
    # SNR that REPRODUCES the measured stage 2 scatter, and read stage 3 there:
    # that is the figure that predicts what the rig will do.
    print("=== calibrated to the MEASURED stage 2 scatter (405 Hz, AP-39) ===")
    best = None
    for snr in [x / 2.0 for x in range(40, 100)]:
        r = ladder(8.52e-6, n_frames=40, snr_db=snr)
        d = abs(sd(r["s2_each_hz"]) - 405.0)
        if best is None or d < best[0]:
            best = (d, snr, r)
    _, snr_cal, r = best
    s2sd, s3sd = sd(r["s2_each_hz"]), sd(r["s3_each_hz"])
    print("  SNR reproducing the bench     %6.1f dB" % snr_cal)
    print("  stage 2 per-shot sd           %8.1f Hz   (bench: 405)" % s2sd)
    print("  stage 2 averaged over 40      %8.1f Hz sem" % (s2sd / math.sqrt(40)))
    print("  stage 3 per-shot sd           %8.3f Hz" % s3sd)
    print("  stage 3 err vs truth          %+8.3f Hz" % (r["s3_avg_hz"] - r["truth_hz"]))
    check(abs(s2sd - 405.0) < 120.0,
          "a calibration point reproducing the bench's stage 2 scatter exists")
    check(s3sd < 5.0,
          "AT THE BENCH'S OWN NOISE, stage 3 still resolves to under 5 Hz")
    # The unwrap margin is what decides whether the ladder is usable at all: the
    # averaged stage 2 must land inside +-500 Hz of truth.
    margin = 500.0 - abs(r["s2_avg_hz"] - r["truth_hz"])
    print("  unwrap margin                 %+8.1f Hz of the +-500 Hz window" % margin)
    check(margin > 0.0,
          "the averaged stage 2 lands inside stage 3's ambiguity window")
    print()

    # ---- 4. THE SEQUENCING RESULT: stage 2's bias breaks the unwrap -------
    print("=== can stage 2 unwrap stage 3? bias, not noise, decides ===")
    print("  %-42s %12s %12s" % ("stage 2 bias", "s3 err (Hz)", "unwrapped?"))
    for label, bias in (("AP-39 applied, measured -31.8 Hz", -31.8),
                        ("half an ambiguity step, 250 Hz", 250.0),
                        ("pre-AP-39, measured +1487.5 Hz", 1487.5)):
        r = ladder(8.52e-6, n_frames=60, snr_db=47.0, stage2_bias_hz=bias)
        err = r["s3_avg_hz"] - r["truth_hz"]
        ok = abs(err) < 10.0
        print("  %-42s %+12.2f %12s" % (label, err, "yes" if ok else "NO"))
        if abs(bias) < 500.0:
            check(ok, "the unwrap survives a %g Hz stage 2 bias" % bias)
        else:
            check(not ok,
                  "the unwrap FAILS at a %g Hz bias, as it must (this is why "
                  "AP-39 gates AP-34(b))" % bias)
    print()

    # ---- 5. report in the literature's units -------------------------------
    print("=== reported as the literature does (eps = df / df_sc) ===")
    r = ladder(8.52e-6, n_frames=60, snr_db=47.0)
    df = r["s3_avg_hz"]
    eps_norm = df / DF_SC
    print("  df                 %+10.4f Hz" % df)
    print("  df_sc = fs/N_fft   %10.2f Hz" % DF_SC)
    print("  eps = df/df_sc     %+10.6f  (integer %+d, fractional %+.6f)"
          % (eps_norm, int(round(eps_norm)), eps_norm - round(eps_norm)))
    print("  ppm vs carrier     %+10.4f ppm" % (df / F_CARRIER * 1e6))
    print("  SCO ppm            %+10.4f ppm  (one reference, so SCO == CFO)"
          % (df / F_CARRIER * 1e6))
    check(abs(eps_norm) < 0.5,
          "the offset is a pure FRACTIONAL subcarrier offset, no integer part")
    check(abs(df / F_CARRIER * 1e6 - 8.52) < 0.01,
          "stage 3 recovers the 8.52 ppm pair to better than 0.01 ppm")

    print("\nRESULT: %s (%d failure(s))" % ("FAIL" if fails else "PASS", len(fails)))
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
