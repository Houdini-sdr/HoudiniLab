#!/usr/bin/env python3
"""SYN1 wire-path test for the beacon-sync panel: NO hardware required.

Two independent checks:

1. LAYOUT. receiver.cc hand-assembles the datagram with std::memcpy at literal
   offsets, with no header shared with Python and no compiler to catch drift.
   So this parses those offsets straight out of receiver.cc and asserts they
   tile the buffer and match csi_server.py's struct field-by-field. An earlier
   version of this test packed with the parser's OWN format string, which made
   it tautological across precisely the seam it advertised: reordering fields in
   the C++ still printed PASS.

2. TRANSPORT. Sends synthetic datagrams covering every state through a server
   this test starts itself, on a port chosen at runtime so it can never pass by
   talking to a stale server someone left running, and reads the SSE back.

Note on reading SSE: a CSI payload is several KB, so a fixed-size read can cut a
data line mid-JSON and look like "no data" -- read to the blank-line delimiter.

    python3 tests/demo-verify/syn1_wire_test.py
"""
import json
import os
import re
import signal
import socket
import struct
import subprocess
import sys
import time
import urllib.request

HERE = os.path.dirname(os.path.abspath(__file__))
SOUNDER = os.path.normpath(os.path.join(HERE, "..", ".."))
RECEIVER_CC = os.path.join(SOUNDER, "receiver.cc")
SERVER_PY = os.path.join(SOUNDER, "csi_gui", "csi_server.py")

SFR, FC, TOL = 122880, 500e6, 1024

# The canonical wire order, by the C++ variable names sendSyncTelemetry copies.
# Types alone cannot catch a reorder: five fields are uint32 and two are int32,
# so swapping frame and tid -- which would put tid on the panel's frame axis --
# is invisible to an offset/width/type comparison.
CC_FIELD_ORDER = ["magic", "fr", "ti", "state", "rs", "cf", "sn", "sh",
                  "sfr", "carrier", "scatter_tol"]

# frame, tid, state, resid, cfo, snr, shift
CASES = [
    (1000, 0, 1,    0, -1147.0, 48.0,    0),   # LOCKED
    (1100, 0, 1,    3,  -900.0, 47.6,    0),
    (1250, 0, 2, 1500,  -800.0, 47.9,    0),   # HOLD: off-grid, not acted on
    (1260, 0, 4,    0,     0.0, 12.0,    0),   # WEAK: under the SNR floor
    (1270, 0, 3,    0,     0.0,  0.0, -412),   # RE-ANCHORED, step applied
    (1280, 0, 5,    0,     0.0,  0.0,    0),   # re-anchor did NOT confirm
    (1400, 0, 1,   -2, -1200.0, 48.1,    0),
]


# C++ scalar type -> struct format code. Width alone is NOT enough: int32_t and
# float are both 4 bytes, so comparing only offsets lets a type swap through.
CTYPE_CODE = {"uint32_t": "I", "int32_t": "i", "float": "f", "double": "d",
              "uint16_t": "H", "int16_t": "h", "uint64_t": "Q", "int64_t": "q"}


def cc_layout():
    """(total, [(offset, width, code), ...]) parsed out of sendSyncTelemetry."""
    src = open(RECEIVER_CC).read()
    i = src.index("void sendSyncTelemetry(")
    body = src[i:src.index("\n}\n", i)]
    m = re.search(r"uint8_t\s+buf\[(\d+)\]", body)
    if not m:
        raise AssertionError("cannot find the buf[] declaration in sendSyncTelemetry")
    total = int(m.group(1))

    # var -> C++ type, from the parameter list and from local declarations
    # (both `const T a = ..., b = ...;` and plain parameters).
    types = {}
    sig = body[:body.index(") {") + 1]
    for t, n in re.findall(r"\b(%s)\s+(\w+)" % "|".join(CTYPE_CODE), sig):
        types[n] = t
    for t, decl in re.findall(r"\bconst\s+(%s)\s+([^;]+);" % "|".join(CTYPE_CODE),
                              body):
        for part in decl.split(","):
            nm = re.match(r"\s*(\w+)", part)
            if nm:
                types[nm.group(1)] = t

    order = [n for _o, n, _w in re.findall(
        r"std::memcpy\(buf \+ (\d+),\s*&(\w+),\s*(\d+)\)", body)]
    magic = re.search(r"magic\s*=\s*(0x[0-9A-Fa-f]+)u?", body)
    fields = []
    for o, name, w in re.findall(
            r"std::memcpy\(buf \+ (\d+),\s*&(\w+),\s*(\d+)\)", body):
        t = types.get(name)
        if t is None:
            raise AssertionError("cannot resolve the C++ type of '%s'" % name)
        code = CTYPE_CODE[t]
        if struct.calcsize("<" + code) != int(w):
            raise AssertionError(
                "%s declared %s but memcpy copies %s bytes" % (name, t, w))
        fields.append((int(o), int(w), code))
    fields.sort()
    return total, fields, order, (int(magic.group(1), 16) if magic else None)


def py_layout():
    fmt = open(SERVER_PY).read()
    m = re.search(r'SYN_HDR = struct\.Struct\("<([A-Za-z]+)"\)', fmt)
    if not m:
        raise AssertionError("cannot find SYN_HDR in csi_server.py")
    codes = m.group(1)
    out, off = [], 0
    for c in codes:
        w = struct.calcsize("<" + c)
        out.append((off, w, c))
        off += w
    return off, out, codes


def check_layout():
    cc_total, cc_fields, cc_order, cc_magic = cc_layout()
    py_total, py_fields, codes = py_layout()
    ok = True
    if cc_total != py_total:
        ok = False
        print("FAIL: receiver.cc buf[%d] vs csi_server.py struct %d bytes"
              % (cc_total, py_total))
    if cc_fields != py_fields:
        ok = False
        print("FAIL: field layout or TYPES differ")
        print("   receiver.cc  :", cc_fields)
        print("   csi_server.py:", py_fields)
    covered = sum(w for _, w, _c in cc_fields)
    if covered != cc_total:
        ok = False
        print("FAIL: memcpy fields cover %d of %d declared bytes" % (covered, cc_total))
    exp = 0
    for o, w, _c in cc_fields:
        if o != exp:
            ok = False
            print("FAIL: gap/overlap at offset %d (expected %d)" % (o, exp))
        exp = o + w
    if cc_order != CC_FIELD_ORDER:
        ok = False
        print("FAIL: field ORDER changed in receiver.cc")
        print("   got     :", cc_order)
        print("   expected:", CC_FIELD_ORDER)
    py_magic = re.search(r"MAGIC_SYN\s*=\s*(0x[0-9A-Fa-f]+)", open(SERVER_PY).read())
    py_magic = int(py_magic.group(1), 16) if py_magic else None
    if cc_magic is None or py_magic is None or cc_magic != py_magic:
        ok = False
        print("FAIL: magic mismatch receiver.cc=%s csi_server.py=%s"
              % (cc_magic and hex(cc_magic), py_magic and hex(py_magic)))
    print("layout: %d bytes, %d fields, format '<%s', magic %s  -> %s"
          % (cc_total, len(cc_fields), codes,
             cc_magic and hex(cc_magic), "OK" if ok else "MISMATCH"))
    return ok, codes, cc_magic


def free_port():
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    p = s.getsockname()[1]
    s.close()
    return p


def read_one_event(url, budget=6.0):
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


def check_transport(codes, magic):
    hdr = struct.Struct("<" + codes)
    http_port, udp_port = free_port(), free_port()
    srv = subprocess.Popen(
        [sys.executable, SERVER_PY, "--http-port", str(http_port),
         "--udp-port", str(udp_port)],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        preexec_fn=os.setsid)
    try:
        time.sleep(2.0)
        if srv.poll() is not None:
            print("FAIL: server exited immediately (rc=%s)" % srv.returncode)
            return False
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        for (f, t, st, r, c, n, sh) in CASES:
            sock.sendto(hdr.pack(magic, f, t, st, r, c, n, sh, SFR, FC, TOL),
                        ("127.0.0.1", udp_port))
        time.sleep(1.0)
        rec = read_one_event("http://127.0.0.1:%d/stream" % http_port)
        if rec is None:
            print("FAIL: no complete SSE data line")
            return False
        sync = rec.get("sync")
        if not sync or "0" not in sync:
            print("FAIL: payload carries no sync entry for tid 0:", sync)
            return False
        hist = sync["0"]["hist"]
        ok = len(hist) == len(CASES)
        if not ok:
            print("FAIL: got %d points, sent %d" % (len(hist), len(CASES)))
        for got, exp in zip(hist, CASES):
            # cfo and snr are both f32 at adjacent offsets, so check_layout
            # cannot see a swap between them -- assert their VALUES here or the
            # pair is covered by neither half of this test.
            fields = (got["frame"], got["tid"], got["state"], got["resid"],
                      got["shift"], got["tol"],
                      round(got["cfo"], 2), round(got["snr"], 2))
            want = (exp[0], exp[1], exp[2], exp[3], exp[6], TOL,
                    round(exp[4], 2), round(exp[5], 2))
            if fields != want:
                ok = False
                print("FAIL: %s != %s" % (fields, want))
        if hist and (hist[0]["sfr"] != SFR or abs(hist[0]["fc"] - FC) > 1):
            ok = False
            print("FAIL: geometry not carried (sfr/fc)")
        print("transport: %d points, states %s, shift %s, tol %s, age_ms %s -> %s"
              % (len(hist), [p["state"] for p in hist],
                 [p["shift"] for p in hist if p["state"] == 3],
                 hist[0]["tol"] if hist else "-", sync["0"]["age_ms"],
                 "OK" if ok else "MISMATCH"))
        return ok
    finally:
        # srv.poll() above reaps the pid, after which getpgid raises
        # ProcessLookupError and destroys this test's own RESULT line.
        try:
            os.killpg(os.getpgid(srv.pid), signal.SIGTERM)
        except (ProcessLookupError, OSError):
            pass
        try:
            srv.wait(timeout=5)
        except Exception:
            pass


def main():
    lay_ok, codes, magic = check_layout()
    tr_ok = check_transport(codes, magic)
    print("RESULT:", "PASS" if (lay_ok and tr_ok) else "FAIL")
    return 0 if (lay_ok and tr_ok) else 1


if __name__ == "__main__":
    sys.exit(main())
