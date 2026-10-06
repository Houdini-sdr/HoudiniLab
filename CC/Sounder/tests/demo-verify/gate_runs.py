#!/usr/bin/env python3
"""The fabric packet gate seen in the BS's raw reads: gate_runs.py <dump dir>
(the BS landing dumps of a run with --bs_dump_frame set; needs numpy).
DEMO_VERIFICATION.md 9.70 (SM3) is this tool's reading of fpga 1.34.

HoudiniFramer's landing dump (--bs_dump_frame) writes lane 0 of one read
exactly as recv built it: every sample at its stamp's position, the gaps
between delivered packets zero-filled. Real samples are ADC noise or signal,
never an exact 0+0j run of a packet's length, so each delivered run of samples
is a run of packets. For each run: where it starts against the schedule (the
tick's offset from the rx slot boundary, expected 0 for the first stamp of a
window) and how long it is (expected 2 slots = 64 packets of 1920 for P+U,
shorter only at the read's two ends).
"""
import glob, os, sys
import numpy as np

d = sys.argv[1]
PKT = 1920
for mp in sorted(glob.glob(os.path.join(d, "bsframe_*.txt"))):
    meta = {}
    for line in open(mp):
        p = line.split()
        if p and p[0] != "rx_slots":
            meta[p[0]] = float(p[1])
    raw = np.fromfile(mp.replace(".txt", ".bin"), dtype=np.int16).reshape(-1, 2)
    nz = (raw[:, 0] != 0) | (raw[:, 1] != 0)
    n, fr, ep = int(meta["n"]), int(meta["frame_ticks"]), int(meta["epoch"])
    t0 = int(round(meta["ft_ns"] * meta["tick_rate"] / 1e9))
    # runs of real samples, bridging zero stretches shorter than a packet (a
    # sample can be an exact 0 inside real data only for a moment)
    idx = np.flatnonzero(nz)
    if idx.size == 0:
        print(os.path.basename(mp), "no real samples")
        continue
    br = np.flatnonzero(np.diff(idx) > PKT) + 1
    starts, ends = idx[np.r_[0, br]], idx[np.r_[br - 1, idx.size - 1]] + 1
    rows = []
    for s, e in zip(starts, ends):
        off = (t0 + s - ep) % fr
        slot, into = off // n, off % n
        if into >= n // 2:  # just before a boundary reads as that boundary minus
            slot, into = (slot + 1) % (fr // n), into - n
        rows.append("[slot %d%+d, %d samples = %.3f pkts]" % (slot, into, e - s, (e - s) / PKT))
    print(os.path.basename(mp), "read start slot %d%+d:" % (((t0 - ep) % fr) // n, ((t0 - ep) % fr) % n),
          " ".join(rows))
