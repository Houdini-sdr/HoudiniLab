#!/usr/bin/env python3
"""The seat over a run beside the carrier and the clock steering.

Reads a run's log in order and keeps the last HOUDINI_BS_RX frame number, so
each pre-FFT carrier line and each steering push is placed at a frame. Prints
the logged pilot_grid_off's mean in bins of a set number of frames, with the
pre-FFT carrier and the pushes that fall in each bin, then the run's mean seat
and the seat over its last 40 % of frames.
usage: seat_vs_cfo.py <run dir> [<bin frames>]
"""
import glob, os, re, statistics as st, sys

run = sys.argv[1].rstrip("/")
binf = int(sys.argv[2]) if len(sys.argv) > 2 else 500
log = os.path.join(run, os.path.basename(run) + ".log")
seat, pre, push = {}, {}, {}
frame = 0
for l in open(log, errors="replace"):
    m = re.search(r"HOUDINI_BS_RX: frame=(\d+) .*pilot_grid_off=(-?\d+)", l)
    if m:
        frame = int(m.group(1))
        seat.setdefault(frame // binf, []).append(int(m.group(2)))
        continue
    m = re.search(r"after the pilot's pre-FFT (-?[0-9]+) Hz", l)
    if m:
        pre.setdefault(frame // binf, []).append(int(m.group(1)))
        continue
    m = re.search(r"Clock steering \[\d+\]: .*-> push ([-+]\d+)", l)
    if m:
        push.setdefault(frame // binf, []).append(m.group(1))
print(f"== {os.path.basename(run)} (bins of {binf} frames)")
for b in sorted(seat):
    s = seat[b]
    print(f"  frames {b*binf:6d}+: seat mean {st.mean(s):+5.2f} (n {len(s):3d}, {min(s):+d}..{max(s):+d})"
          f"  pre-FFT Hz {pre.get(b, '')}  pushes {push.get(b, '')}")
allv = [v for b in sorted(seat) for v in seat[b]]
tail = allv[int(len(allv) * 0.6):]
print(f"  run: seat mean {st.mean(allv):+.2f} over {len(allv)}; last 40 % {st.mean(tail):+.2f} over {len(tail)}")
