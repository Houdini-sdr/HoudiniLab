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
    env["HOUDINI_CSI_DUMP"] = "90"   # settled raw U-slot + H dump per run
    try:
        os.remove("/tmp/cns_dump.bin")
    except OSError:
        pass
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
        body = np.frombuffer(d, dtype=np.float32,
                             offset=CNS_HDR.size, count=2 * npts)
        pts.append(body[0::2] + 1j * body[1::2])
    try:
        p.wait(timeout=60)
    except subprocess.TimeoutExpired:
        p.kill()
    sock.close()
    logf.close()
    if os.path.exists("/tmp/cns_dump.bin"):
        os.replace("/tmp/cns_dump.bin", "logs/ap15_run%d_dump.bin" % i)
    log = open("logs/ap15_run%d.log" % i, "rb").read().decode(errors="replace")
    m = re.search(r"HOUDINI_BS_RX:.*?pilot_grid_off=(-?\d+) pu_spacing_err=(-?\d+)",
                  log)
    pgo, pu = (m.group(1), m.group(2)) if m else ("?", "?")
    if not pts:
        return "run %2d: NO CNS POINTS (rc=%s) pgo=%s pu=%s" % (
            i, p.returncode, pgo, pu)
    z = np.concatenate(pts)
    z = z[np.abs(z) > 1e-9]
    np.save("logs/ap15_run%d_cns.npy" % i, z)
    score = float(np.abs(np.mean(z ** 4)) / (np.mean(np.abs(z) ** 4) + 1e-30))
    verdict = "CLUSTERS" if score > 0.8 else ("ring" if score < 0.3
                                              else "partial")
    return "run %2d: score=%.3f %-8s pts=%d pgo=%s pu=%s" % (
        i, score, verdict, z.size, pgo, pu)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--runs", type=int, default=12)
    ap.add_argument("--frames", type=int, default=450)
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
