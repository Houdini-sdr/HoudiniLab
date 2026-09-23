#!/usr/bin/env python3
"""AP-79 filter staging: the in-channel |H| change between two stages, per tone
and in 10 MHz bins, for both BS antennas at R2 (ant 0 sub-6, ant 1 X-IF).

usage: fstage_hratio.py <from_stage_dir> <to_stage_dir> [out.csv]

Each stage dir holds fstage_run.sh run dirs; only the r2_* runs are used. Each
stage's |H| is the mean over its runs of 20log10|H| per data tone (the CSI dump),
so the difference cancels everything the two stages share: the sounder's own TX
and RX filters, the cables, the converters. What is left is the change the stage
made, per tone. The unchanged link is the control: it should read flat at the
run-to-run level (F1 against F0: -0.02 dB, rms 0.006 dB).

RF is assigned by tone index, NCO + (k - N/2) x Fs/N; that the index rises with
RF on both links is an assumption until a filter of known tilt direction shows it.
The fits separate a slope from ripple: a quadratic that beats the line means the
change turns inside the band (F1: a ripple, DEMO_VERIFICATION 9.18).
"""
import glob, os, sys
import numpy as np

LINKS = (("cns_dump_ant1.bin", "xif", 4380.0), ("cns_dump.bin", "sub6", 2425.0))
FS_MHZ = 122.88


def hdb(run, name):
    b = open(os.path.join(run, name), "rb").read()
    N, _, _, _, nd = (int(v) for v in np.frombuffer(b[:20], np.int32))
    H = np.frombuffer(b[20:20 + 8 * N], np.float32).reshape(-1, 2); H = H[:, 0] + 1j * H[:, 1]
    di = np.frombuffer(b[20 + 8 * N:20 + 8 * N + 4 * nd], np.int32)
    return N, di, 20 * np.log10(np.abs(H[di]))


def stage(d, name):
    runs = sorted(r for r in glob.glob(os.path.join(d, "r2_*")) if os.path.exists(os.path.join(r, name)))
    arr = [hdb(r, name) for r in runs]
    if not arr:
        sys.exit("no R2 runs with %s in %s" % (name, d))
    N, di = arr[0][0], arr[0][1]
    for a in arr:
        if a[0] != N or not np.array_equal(a[1], di):
            sys.exit("runs in %s differ in numerology" % d)
    v = np.array([a[2] for a in arr])
    return N, di, v.mean(0), float(np.median(v.max(0) - v.min(0))), len(arr)


def main():
    a, b = sys.argv[1], sys.argv[2]
    out = open(sys.argv[3], "w") if len(sys.argv) > 3 else None
    if out:
        out.write("link,tone_index,rf_mhz_by_tone_index,from_mean_db,to_mean_db,to_minus_from_db\n")
    for name, lab, nco in LINKS:
        N, di, m0, sp0, n0 = stage(a, name)
        N1, di1, m1, sp1, n1 = stage(b, name)
        if N1 != N or not np.array_equal(di1, di):
            sys.exit("the two stages differ in numerology")
        d = m1 - m0
        f = nco + (di - N // 2) * FS_MHZ / N
        print("== %s: %s minus %s (%d and %d runs), %d tones %.2f..%.2f MHz, per-tone run spread %.3f / %.3f dB"
              % (lab, os.path.basename(b.rstrip("/")), os.path.basename(a.rstrip("/")), n1, n0, len(di), f.min(),
                 f.max(), sp0, sp1))
        lo = f.min()
        while lo <= f.max():
            m = (f >= lo) & (f < lo + 10.0)
            if m.sum():
                print("   %7.1f-%7.1f MHz: %+.3f dB (%d tones)" % (lo, min(lo + 10.0, f.max()), d[m].mean(), m.sum()))
            lo += 10.0
        lin = np.polyfit(f, d, 1); q = np.polyfit(f, d, 2)
        print("   mean %+.3f dB; line %+.3f dB across the band (rms residual %.3f); quadratic rms residual %.3f"
              % (d.mean(), lin[0] * (f.max() - f.min()), np.std(d - np.polyval(lin, f)), np.std(d - np.polyval(q, f))))
        if out:
            for k, fk, v0, v1 in zip(di, f, m0, m1):
                out.write("%s,%d,%.2f,%.3f,%.3f,%+.3f\n" % (lab, k, fk, v0, v1, v1 - v0))
    if out:
        out.close()


if __name__ == "__main__":
    main()
