#!/usr/bin/env python3
"""Sample the dashboard stream every <period> s for <total> s: each antenna's
MER (1 s pooled, as the page shows it), appended to <out>, with the
constellation's frame and the antenna's age. A sample is STALE when the
constellation did not advance since the previous sample or the antenna is at
or past the stale threshold: the page shows its stale badge then, and the MER
it still carries is the last one before the stall. Exits 1 when any sample was
stale, failed or carried no MER, so a record cannot read healthy over a stall.
Stdlib only.
usage: mer_sampler.py <out> <period_s> <total_s> [first_delay_s] [--url URL] [--stale-ms MS]"""
import argparse, json, sys, time, urllib.request

ap = argparse.ArgumentParser(usage=__doc__.rsplit("usage: ", 1)[1])
ap.add_argument("out")
ap.add_argument("period", type=float)
ap.add_argument("total", type=float)
ap.add_argument("first", type=float, nargs="?", default=0.0)
ap.add_argument("--url", default="http://127.0.0.1:8080/stream")
ap.add_argument("--stale-ms", type=int, default=1500, help="the dashboard's own default")
args = ap.parse_args()


def one_event(url):
    r = urllib.request.urlopen(url, timeout=10)
    try:
        deadline = time.time() + 10
        while time.time() < deadline:
            line = r.readline()
            if line.startswith(b"data: "):
                return json.loads(line[6:])
        raise TimeoutError("no data event in 10 s")
    finally:
        r.close()


time.sleep(args.first)
t0 = time.time()
last, bad, n = {}, 0, 0
while time.time() - t0 < args.total:
    stamp = time.strftime("%H:%M:%S", time.gmtime())
    stale_any = False
    try:
        vals = []
        for a, rec in sorted((one_event(args.url).get("ant") or {}).items()):
            cns = rec.get("cns") or {}
            if "mer_db" not in cns:
                continue
            fr, age = cns.get("frame"), rec.get("age_ms", 0)
            stale = age >= args.stale_ms or last.get(a) == fr
            last[a] = fr
            stale_any |= stale
            vals.append("/ant/%s/cns %sMER %.1f dB, EVM %.2f %%, %s pts, frame %s, age %s ms"
                        % (a, "STALE " if stale else "", cns["mer_db"], cns.get("evm_pct", float("nan")),
                           cns.get("mer_pts"), fr, age))
        msg = "; ".join(vals)
        if not vals:
            msg, stale_any = "no MER in the stream", True
    except Exception as e:  # noqa: BLE001 -- a failed sample is recorded, not fatal
        msg, stale_any = "sample failed: %s" % e, True
    n += 1
    bad += stale_any
    with open(args.out, "a") as f:
        f.write("%s UTC %s\n" % (stamp, msg))
    time.sleep(args.period)
with open(args.out, "a") as f:
    f.write("%d samples, %d stale, failed or empty\n" % (n, bad))
sys.exit(1 if bad else 0)
