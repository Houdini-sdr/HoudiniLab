#!/usr/bin/env python3
"""The seat at zero carrier offset, from runs where the offset moves.

Each pre-FFT carrier line (the BS's reading of the UE pilot's carrier, Hz) is
paired with the mean logged pilot_grid_off over the BS_RX lines within
+-<half> frames of it, after adding (tx_advance - 198) so runs at other
tx_advance values read on the 198 scale. A least-squares line seat = a + b*Hz
per run and over each group of runs gives a, the seat at zero offset, with its
standard error. Group = the label before ':' in each argument.
usage: seat_at_zero_cfo.py [--half N] <group>:<tx_advance>:<run dir> ...
"""
import bisect, os, re, statistics as st, sys

args = sys.argv[1:]
half = 100
if args[:1] == ["--half"]:
    half = int(args[1]); args = args[2:]


def pairs(run, txadv):
    log = os.path.join(run, os.path.basename(run) + ".log")
    seat_f, seat_v, pre = [], [], []
    frame = 0
    for l in open(log, errors="replace"):
        m = re.search(r"HOUDINI_BS_RX: frame=(\d+) .*pilot_grid_off=(-?\d+)", l)
        if m:
            frame = int(m.group(1))
            seat_f.append(frame); seat_v.append(int(m.group(2)) + txadv - 198)
            continue
        m = re.search(r"after the pilot's pre-FFT (-?[0-9]+) Hz", l)
        if m:
            pre.append((frame, int(m.group(1))))
    order = sorted(range(len(seat_f)), key=lambda i: seat_f[i])
    seat_f = [seat_f[i] for i in order]; seat_v = [seat_v[i] for i in order]
    out = []
    for f, hz in pre:
        lo = bisect.bisect_left(seat_f, f - half); hi = bisect.bisect_right(seat_f, f + half)
        if hi - lo >= 3:
            out.append((hz, st.mean(seat_v[lo:hi])))
    return out


def fit(p):
    x = [a for a, _ in p]; y = [b for _, b in p]
    n = len(p); mx, my = st.mean(x), st.mean(y)
    sxx = sum((a - mx) ** 2 for a in x)
    if n < 3 or sxx == 0:
        return None
    b = sum((a - mx) * (c - my) for a, c in zip(x, y)) / sxx
    a = my - b * mx
    r = [c - (a + b * xi) for xi, c in zip(x, y)]
    s2 = sum(e * e for e in r) / (n - 2)
    se_a = (s2 * (1 / n + mx * mx / sxx)) ** 0.5
    return a, se_a, b * 1000, s2 ** 0.5, n, min(x), max(x)


groups = {}
for arg in args:
    g, tx, run = arg.split(":", 2)
    p = pairs(run.rstrip("/"), int(tx))
    groups.setdefault(g, []).extend(p)
    f = fit(p)
    if f:
        print(f"{g:5s} {os.path.basename(run.rstrip('/')):16s} seat@0Hz {f[0]:+.2f} (se {f[1]:.2f})  "
              f"slope {f[2]:+.2f}/kHz  resid sd {f[3]:.2f}  n {f[4]:3d}  Hz {f[5]}..{f[6]}")
    else:
        print(f"{g:5s} {os.path.basename(run.rstrip('/')):16s} no fit (n {len(p)})")
for g, p in groups.items():
    f = fit(p)
    if f:
        print(f"== {g}: seat@0Hz {f[0]:+.2f} (se {f[1]:.2f})  slope {f[2]:+.2f}/kHz  resid sd {f[3]:.2f}  "
              f"n {f[4]}  Hz {f[5]}..{f[6]}  -> tx_advance + seat at 0 Hz = {198 + f[0]:.2f}")
# The direct comparison: the paired seats in bands of carrier offset, no line.
BANDS = [(-400, -100), (-100, 100), (100, 300), (300, 500), (500, 700), (700, 1300)]
print("== seat by carrier band (mean, n)")
print("  band Hz      " + "".join(f"{g:>16s}" for g in groups))
for lo, hi in BANDS:
    row = f"  {lo:+5d}..{hi:+5d}"
    for g, p in groups.items():
        v = [s for hz, s in p if lo <= hz < hi]
        row += f"{(f'{st.mean(v):+.2f} (n {len(v)})' if v else '-'):>16s}"
    print(row)
