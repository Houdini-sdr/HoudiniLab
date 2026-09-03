#!/usr/bin/env python3
"""AP-15 correlation campaign: restart the demo N times; per run, collect the
sounder's own equalized-constellation datagrams (CNS1), score them with the
4th-power concentration metric (~1 clean QPSK, ~0 ring), and pair the verdict
with that run's per-restart timing draws (pilot_grid_off, pu_spacing_err from
the BS-side rederivation). No csi_server, no dashboard.

Run on the rig from the HoudiniLab-rx/CC/Sounder directory.
"""
import argparse
import os
import re
import socket
import struct
import subprocess
import sys
import time

import numpy as np

CNS_HDR = struct.Struct("<IIIII")
MAGIC_CNS = 0x434E5331


def one_run(i, args):
    subprocess.run(["python3", "csi_gui/teardown_framer.py"],
                   capture_output=True, timeout=120)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("127.0.0.1", args.port))
    sock.settimeout(1.0)
    env = dict(os.environ)
    env["HOUDINI_CSI_UDP"] = "127.0.0.1:%d" % args.port
    env["HOUDINI_MAX_FRAME"] = str(args.frames)
    env["HOUDINI_BS_RX_DEBUG"] = "1"
    env["HOUDINI_CSI_DUMP"] = "60"   # settled dump: CNS datagrams are
    # ~1 per 33 ms (each carries UP TO kMaxPts=600 points, symbols ~6..18 of
    # the slot -- NOT the whole constellation), so 60 sends = ~2 s after
    # pilots start; needs --frames >= ~2500
    dump = os.path.join(os.environ.get("HOUDINI_DUMP_DIR", "/tmp"), "cns_dump.bin")
    try:
        os.remove(dump)
    except OSError:
        pass
    os.makedirs("logs", exist_ok=True)
    logf = open("logs/ap15_run%d.log" % i, "wb")
    p = subprocess.Popen(["./build/sounder", "--view", "--conf_file",
                          args.conf, "--storepath", "logs"],
                         stdout=logf, stderr=subprocess.STDOUT, env=env)
    pts = []
    t0 = time.time()
    while p.poll() is None and time.time() - t0 < args.run_timeout:
        try:
            d, _ = sock.recvfrom(65536)
        except socket.timeout:
            continue
        if len(d) < CNS_HDR.size:
            continue
        magic, frame, ant, npts, mod = CNS_HDR.unpack_from(d)
        if magic != MAGIC_CNS or frame < args.settle:
            continue
        if len(d) < CNS_HDR.size + 8 * npts:
            continue  # truncated datagram: skip, don't abort the campaign
        body = np.frombuffer(d, dtype=np.float32,
                             offset=CNS_HDR.size, count=2 * npts)
        pts.append(body[0::2] + 1j * body[1::2])
    try:
        p.wait(timeout=60)
    except subprocess.TimeoutExpired:
        p.kill()
    sock.close()
    logf.close()
    if os.path.exists(dump):
        os.replace(dump, "logs/ap15_run%d_dump.bin" % i)
    log = open("logs/ap15_run%d.log" % i, "rb").read().decode(errors="replace")
    # The run's timing draw, taken from the first debug line AT OR AFTER the
    # settle boundary (the very first line of a run is pre-settle and can
    # differ, Opus review), and checked constant across the run.
    draws = re.findall(r"HOUDINI_BS_RX: frame=(\d+) .*?pilot_grid_off=(-?\d+) "
                       r"pu_spacing_err=(-?\d+)", log)
    settled = [(int(f), g, u) for f, g, u in draws if int(f) >= args.settle]
    pgo, pu = (settled[0][1], settled[0][2]) if settled else ("?", "?")
    drift_note = ""
    if settled:
        pgos = sorted(set(int(g) for _, g, _ in settled))
        if pgos[-1] - pgos[0] > 8:
            drift_note = " PGO-DRIFTED(%d..%d)" % (pgos[0], pgos[-1])
    if not pts:
        return "run %2d: NO CNS POINTS (rc=%s) pgo=%s pu=%s" % (
            i, p.returncode, pgo, pu)
    # PER-DATAGRAM phase-only 4th-power scores (Opus review H8: one score
    # over the whole run's pooled points cannot see a rare bad-frame class --
    # a 5%% garbage fraction still averaged 0.949 "CLUSTERS"). The run report
    # carries the median, the worst datagram, and the low fraction; CLUSTERS
    # now also requires (almost) no low datagrams.
    dscores = []
    for body in pts:
        zz = body[np.abs(body) > 1e-9]
        if zz.size < 16:
            continue
        uu = zz / np.abs(zz)
        dscores.append(float(np.abs(np.mean(uu ** 4))))
    z = np.concatenate(pts)
    z = z[np.abs(z) > 1e-9]
    np.save("logs/ap15_run%d_cns.npy" % i, z)
    if not dscores:
        return "run %2d: no scorable datagrams pgo=%s pu=%s" % (i, pgo, pu)
    med = float(np.median(dscores))
    worst = float(np.min(dscores))
    low = sum(1 for v in dscores if v < 0.7)
    low_frac = low / len(dscores)
    # NB with fewer than 200 scorable datagrams the 0.5% gate is
    # zero-tolerance (1/N > 0.005): a short run reports partial on a single
    # bad datagram, which is the conservative direction.
    if med > 0.8 and low_frac <= 0.005:
        verdict = "CLUSTERS"
    elif med < 0.3:
        verdict = "ring"
    else:
        verdict = "partial"
    return ("run %2d: med=%.3f worst=%.3f low=%d/%d %-8s pts=%d "
            "pgo=%s pu=%s%s") % (i, med, worst, low, len(dscores), verdict,
                                 z.size, pgo, pu, drift_note)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--runs", type=int, default=12)
    ap.add_argument("--frames", type=int, default=3000)
    ap.add_argument("--settle", type=int, default=60)
    ap.add_argument("--port", type=int, default=9911)
    ap.add_argument("--conf", default="files/houdini-ul.json")
    ap.add_argument("--run-timeout", type=float, default=120.0)
    args = ap.parse_args()
    for i in range(1, args.runs + 1):
        print(one_run(i, args), flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
