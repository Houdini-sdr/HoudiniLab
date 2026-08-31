#!/usr/bin/env python3
"""SYN1 wire-path test for the beacon-sync panel: NO hardware required.

Starts csi_server.py on spare ports, sends synthetic SYN1 datagrams covering
every state (LOCKED / HOLD / ESCALATING), then reads the SSE stream back and
checks the page would receive exactly what was sent.

Covers the seams that silently break: the struct layout agreeing between
receiver.cc and csi_server.py, the history surviving into the SSE payload, the
escalation shift, and the geometry fields the page needs to compute ppm without
hardcoding config values.

Note on reading SSE: a CSI payload is several KB, so a fixed-size read can cut a
data line mid-JSON and look like "no data" -- read until a blank-line delimiter.

    python3 tests/demo-verify/syn1_wire_test.py
"""
import json
import os
import signal
import socket
import struct
import subprocess
import sys
import time
import urllib.request

HDR = struct.Struct("<IIIIiffiIf")   # must match receiver.cc sendSyncTelemetry
MAGIC = 0x53594E31
HTTP_PORT, UDP_PORT = 8099, 9989
SFR, FC = 122880, 500e6

# frame, tid, state, resid, cfo, snr, shift
CASES = [
    (1000, 0, 1,    0, -1147.0, 48.0,    0),
    (1100, 0, 1,    3,  -900.0, 47.6,    0),
    (1250, 0, 2, 1500,  -800.0, 47.9,    0),   # HOLD: off-grid, not acted on
    (1260, 0, 3,    0,     0.0,  0.0, -412),   # ESCALATING: anchor shift applied
    (1400, 0, 1,   -2, -1200.0, 48.1,    0),
]


def read_one_event(url, budget=5.0):
    req = urllib.request.urlopen(url, timeout=budget)
    buf, t0 = b"", time.time()
    while time.time() - t0 < budget:
        buf += req.read(1)
        if buf.endswith(b"\n\n"):
            for line in buf.decode("utf-8", "replace").split("\n"):
                if line.startswith("data: "):
                    return json.loads(line[6:])
            buf = b""
    return None


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    server = os.path.join(here, "..", "..", "csi_gui", "csi_server.py")
    srv = subprocess.Popen(
        [sys.executable, server, "--http-port", str(HTTP_PORT),
         "--udp-port", str(UDP_PORT)],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        preexec_fn=os.setsid)
    try:
        time.sleep(2.0)
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        for (f, t, st, r, c, n, sh) in CASES:
            sock.sendto(HDR.pack(MAGIC, f, t, st, r, c, n, sh, SFR, FC),
                        ("127.0.0.1", UDP_PORT))
        time.sleep(1.0)
        rec = read_one_event("http://127.0.0.1:%d/stream" % HTTP_PORT)
        if rec is None:
            print("FAIL: no complete SSE data line")
            return 1
        sync = rec.get("sync")
        if sync is None:
            print("FAIL: payload carries no 'sync' key")
            return 1
        hist = sync["hist"]
        ok = len(hist) == len(CASES)
        if not ok:
            print("FAIL: got %d points, sent %d" % (len(hist), len(CASES)))
        for got, exp in zip(hist, CASES):
            fields = (got["frame"], got["state"], got["resid"], got["shift"])
            want = (exp[0], exp[2], exp[3], exp[6])
            if fields != want:
                ok = False
                print("FAIL: %s != %s" % (fields, want))
        if hist and (hist[0]["sfr"] != SFR or abs(hist[0]["fc"] - FC) > 1):
            ok = False
            print("FAIL: geometry not carried (sfr/fc)")
        print("points %d  states %s  shift %s  sfr=%s fc=%s  age_ms=%s"
              % (len(hist), [p["state"] for p in hist],
                 [p["shift"] for p in hist if p["state"] == 3],
                 hist[0]["sfr"] if hist else "-",
                 hist[0]["fc"] if hist else "-", sync["age_ms"]))
        print("RESULT:", "PASS" if ok else "FAIL")
        return 0 if ok else 1
    finally:
        os.killpg(os.getpgid(srv.pid), signal.SIGTERM)


if __name__ == "__main__":
    sys.exit(main())
