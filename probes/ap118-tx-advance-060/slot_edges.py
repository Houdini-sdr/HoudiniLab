#!/usr/bin/env python3
"""Where the UE burst sits in each BS rx slot: the leading and trailing gaps
around its content, from the BS frame dumps (--bs_dump_frame DIR:
bsframe_NN.bin, lane 0's raw read, and bsframe_NN.txt, the grid).

Each UE slot is [ofdm_tx_zero_prefix zeros | 14 x (cp + fft) | postfix zeros],
filling the slot exactly (32 + 61376 + 32 = 61440 at the demo config). Centred
means a leading and a trailing gap equal to the pads. The dump carries lane 0
(the sub-6 receive) raw; its p_start is the reference lane's placed pilot, so
the script also prints the gaps the reference lane's seat implies (32 + off,
32 - off) beside lane 0's measured ones, which compares the two lanes.

Content edges: the first and last sample of the slot whose 4-sample mean power
clears the slot's median power by more than -20 dB (the pads hold only the
receiver's noise, about 45 dB under the content). The recomputed
pilot_grid_off must equal the run log's HOUDINI_BS_RX value for the frame: that
checks the grid arithmetic (HoudiniFramer.cc, slot_align.h pilotGridOff).

usage: slot_edges.py DIR [--prefix 32] [--postfix 32]"""
import argparse
import glob
import os

import numpy as np


def read_meta(path):
    meta = {}
    for line in open(path):
        p = line.split()
        if not p:
            continue
        if p[0] == "rx_slots":
            meta["rx_slots"] = [int(x) for x in p[1:]]
        else:
            v = float(p[1])
            meta[p[0]] = int(v) if v.is_integer() else v
    return meta


def grid_off(stamp, p_start, epoch, slot, n, fr):
    """slot_align.h pilotGridOff."""
    rel = ((stamp + p_start - epoch) % fr + fr) % fr - slot * n
    if rel > fr // 2:
        rel -= fr
    if rel < -fr // 2:
        rel += fr
    return rel


def edges(seg, win=4, rel_db=-20.0):
    pw = np.abs(seg) ** 2
    sm = np.convolve(pw, np.ones(win) / win, mode="same")
    thr = np.median(pw) * 10 ** (rel_db / 10)
    hit = np.nonzero(sm > thr)[0]
    if hit.size == 0:
        return None, None
    return int(hit[0]), int(len(seg) - 1 - hit[-1])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("dir")
    ap.add_argument("--prefix", type=int, default=32)
    ap.add_argument("--postfix", type=int, default=32)
    a = ap.parse_args()
    rows = []
    for txt in sorted(glob.glob(os.path.join(a.dir, "bsframe_*.txt"))):
        m = read_meta(txt)
        raw = np.fromfile(txt[:-4] + ".bin", dtype=np.int16).astype(np.float64)
        iq = raw[0::2] + 1j * raw[1::2]
        n, fr, ep = m["n"], m["frame_ticks"], m["epoch"]
        stamp = int(round(m["ft_ns"] * m["tick_rate"] / 1e9))
        g = grid_off(stamp, m["p_start"], ep, m["pilot_slot"], n, fr)
        rel = (stamp + np.arange(len(iq)) - ep) % fr
        out = ["frame %d: pilot_grid_off %+d (ref lane %d)" % (m["frame"], g, m["ref_lane"])]
        # The read spans more than a frame (cg > frame_ticks), so a slot can
        # appear more than once: each whole occurrence is a burst of its own
        # frame, and each is measured.
        for k in m["rx_slots"]:
            idx = np.nonzero((rel >= k * n) & (rel < (k + 1) * n))[0]
            runs = np.split(idx, np.nonzero(np.diff(idx) != 1)[0] + 1) if idx.size else []
            name = "P" if k == m["pilot_slot"] else "U" if k == m["pilot_slot"] + 1 else "slot %d" % k
            for run in runs:
                if run.size != n:
                    out.append("  %s (slot %d) at %d: %d of %d samples in the read: skipped" % (name, k, run[0], run.size, n))
                    continue
                lead, trail = edges(iq[run])
                out.append("  %s (slot %d) at %d, lane 0: lead %s trail %s | the ref lane's seat implies lead %d trail %d"
                           % (name, k, run[0], lead, trail, a.prefix + g, a.postfix - g))
                rows.append((name, lead, trail, g))
        print("\n".join(out))
    for name in ("P", "U"):
        r = [x for x in rows if x[0] == name and x[1] is not None]
        if r:
            L = np.array([x[1] for x in r]); T = np.array([x[2] for x in r]); G = np.array([x[3] for x in r])
            print("%s over %d frames: lane 0 lead %.1f (%d..%d), trail %.1f (%d..%d); ref-lane seat %+.1f"
                  % (name, len(r), L.mean(), L.min(), L.max(), T.mean(), T.min(), T.max(), G.mean()))


if __name__ == "__main__":
    main()
