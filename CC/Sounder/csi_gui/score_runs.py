#!/usr/bin/env python3
"""Restart the demo N times and score each run's constellation. The AP-15 instrument.

AP-15: the equalized constellation is clean QPSK on some restarts and a uniform
ring on others, decided per run and stable within a run. Eyeballing the dashboard
cannot measure that, and a single run proves nothing either way -- so this restarts
the demo, samples several frames per run, and scores each one.

The score is the 4th-power concentration |E[z^4]| / E[|z|^4]: about 1.0 for tight
QPSK clusters, about 0 for a uniform ring. It needs no reference symbols, because
raising QPSK to the 4th power cancels the modulation.

    python3 score_runs.py 6                       # 6 restarts, score each
    python3 score_runs.py 6 --dump 60             # also keep a settled CSI dump
    python3 score_runs.py 6 --conf files/houdini-ul.json

WARNING: this restarts the sounder every ~90 s. If anyone is watching the
dashboard they will see it flip between good and bad, which looks exactly like a
real intermittency. Tell them before you start it.

The dumps it keeps (with --dump) are readable offline; see AP-15 for the two traps
that make them easy to misread (bins 11/25/39/53 are pilot tones carrying signal,
not guard bins; and the dump must skip the first frames or every run looks alike).
"""
import argparse
import json
import math
import os
import signal
import subprocess
import sys
import time
import urllib.request


def snap(url, timeout=45):
    """One SSE frame from the dashboard, or None if it never comes up."""
    t0 = time.time()
    while time.time() - t0 < timeout:
        try:
            r = urllib.request.urlopen(url, timeout=8)
            buf = b""
            while True:
                buf += r.read(16384)
                if buf.startswith(b"data: ") and b"\n\n" in buf:
                    r.close()
                    return json.loads(buf.split(b"\n\n", 1)[0][6:])["ant"]["0"]
                if buf.startswith(b": "):        # keepalive, drop and keep reading
                    i = buf.find(b"\n\n")
                    buf = buf[i + 2:] if i >= 0 else buf
            r.close()
        except Exception:
            pass
        time.sleep(2)
    return None


def concentration(pts):
    z = [complex(x, y) for x, y in pts]
    s4 = sum(p ** 4 for p in z) / len(z)
    m4 = sum(abs(p) ** 4 for p in z) / len(z)
    return abs(s4) / m4 if m4 else 0.0


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("runs", type=int, nargs="?", default=6)
    ap.add_argument("--conf", default="files/houdini-ul.json")
    ap.add_argument("--sounder-dir", default=os.path.expanduser("~/repos/HoudiniLab-rx/CC/Sounder"))
    ap.add_argument("--venv", default=os.path.expanduser("~/houdini_test"))
    ap.add_argument("--http-port", type=int, default=8080)
    ap.add_argument("--frames", type=int, default=14, help="frames scored per run")
    ap.add_argument("--dump", type=int, metavar="N", default=0,
                    help="also keep a CSI dump, skipping N constellation frames "
                         "(must be >1: the first frames are pre-settling)")
    args = ap.parse_args()
    url = "http://localhost:%d/stream" % args.http_port

    env = dict(os.environ)
    if args.dump:
        env["HOUDINI_CSI_DUMP"] = str(args.dump)

    print("run  conc    verdict    pilot_peak%s" % ("  dump" if args.dump else ""))
    results = []
    for run in range(1, args.runs + 1):
        subprocess.run("pkill -f 'csi_serv[e]r.py'", shell=True)   # bracketed: see AP-15 notes
        time.sleep(4)
        if args.dump:
            subprocess.run("rm -f /tmp/cns_dump.bin", shell=True)
        proc = subprocess.Popen(
            ["python3", "csi_gui/csi_server.py", "--launch", "--conf", args.conf,
             "--sounder-dir", args.sounder_dir, "--venv", args.venv,
             "--http-port", str(args.http_port)],
            cwd=args.sounder_dir, stdout=open("/tmp/score_run_%d.log" % run, "w"),
            stderr=subprocess.STDOUT, start_new_session=True, env=env)

        cs, peaks = [], []
        if snap(url) is not None:
            for _ in range(args.frames):
                d = snap(url, 15)
                if not d:
                    continue
                if "cns" in d:
                    cs.append(concentration(d["cns"]["pts"]))
                if "adc" in d:
                    peaks.append(d["adc"]["peak"])
                time.sleep(0.3)
        med = lambda v: sorted(v)[len(v) // 2] if v else float("nan")
        c = med(cs)
        verdict = "CLUSTERS" if c > 0.8 else ("ring" if c < 0.3 else "partial")
        kept = ""
        if args.dump and os.path.exists("/tmp/cns_dump.bin"):
            kept = "/tmp/dump_%d_%s.bin" % (run, verdict)
            subprocess.run("cp /tmp/cns_dump.bin %s" % kept, shell=True)
        print("%3d  %-7.3f %-10s %-11.0f %s"
              % (run, c, verdict, med(peaks), kept))
        results.append(c)
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
        except Exception:
            pass
        time.sleep(2)

    ok = [c for c in results if c == c and c > 0.8]
    print("\nlocked to clusters: %d of %d" % (len(ok), len(results)))
    print("(1 of 6 and 1 of 5 were the readings that opened AP-15)")


if __name__ == "__main__":
    main()
