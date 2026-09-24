#!/usr/bin/env python3
"""Sample the dashboard stream every <period> s for <total> s: each antenna MER
(1 s pooled, as the page shows it), appended to <out>. Stdlib only.
usage: mer_sampler.py <out> <period_s> <total_s> [first_delay_s]"""
import json, sys, time, urllib.request
out, period, total = sys.argv[1], float(sys.argv[2]), float(sys.argv[3])
first = float(sys.argv[4]) if len(sys.argv) > 4 else 0.0
def find(o, path=""):
    if isinstance(o, dict):
        if "mer_db" in o:
            yield path, o
        for k, v in o.items():
            yield from find(v, path + "/" + str(k))
time.sleep(first)
t0 = time.time()
while time.time() - t0 < total:
    stamp = time.strftime("%H:%M:%S", time.gmtime())
    try:
        r = urllib.request.urlopen("http://127.0.0.1:8080/stream", timeout=10)
        line, deadline = b"", time.time() + 10
        while time.time() < deadline:
            line = r.readline()
            if line.startswith(b"data: "):
                break
        r.close()
        d = json.loads(line[6:])
        vals = ["%s MER %.1f dB, EVM %.2f %%, %s pts" % (p, o["mer_db"], o.get("evm_pct", float("nan")), o.get("mer_pts"))
                for p, o in find(d)]
        msg = "; ".join(vals) or "no MER in the stream"
    except Exception as e:
        msg = "sample failed: %s" % e
    with open(out, "a") as f:
        f.write("%s UTC %s\n" % (stamp, msg))
    time.sleep(period)
