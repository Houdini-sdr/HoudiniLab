"""Sample the live dashboard's pilot-slot spectra (SPC1) at given times: per
antenna the in-band mean, peak and total (dBFS per 240 kHz bin), the median
floor outside the band, and the strongest out-of-band bin with its offset.
The occupied half bandwidths are the demo's (antenna 0 sub-6 at 133 RB, antenna
1 X-band at 270 RB); edit HALF for another config. Reads the dashboard at
127.0.0.1:8080, so run it on the rig host beside a run.
usage: spc_sample.py <out_file> <t1,t2,...>   (seconds after start)"""
import json, sys, time, math, urllib.request
out, times = sys.argv[1], [float(x) for x in sys.argv[2].split(",")]
HALF = {"0": 23.94, "1": 48.6}  # occupied half bandwidth, MHz (sub-6 133 RB, X-band 270 RB)
t0 = time.time()
with open(out, "a") as fo:
    for tt in times:
        while time.time() - t0 < tt: time.sleep(0.5)
        try:
            r = urllib.request.urlopen("http://127.0.0.1:8080/stream", timeout=10); a = {}; ts = time.time()
            while time.time() - ts < 8:
                l = r.readline()
                if l.startswith(b"data: "):
                    d = json.loads(l[6:]); a.update(d.get("ant") or {})
                    if len(a) >= 2 and all("spc" in v for v in a.values()): break
            for k, v in sorted(a.items()):
                s = v.get("spc") or {}; db = s.get("db") or []
                n = len(db)
                if n == 0: fo.write("%s ant %s: no spc\n" % (time.strftime("%H:%M:%S", time.gmtime()), k)); continue
                df = 122.88 / n; h = HALF.get(k, 23.94)
                f = [(i - n / 2) * df for i in range(n)]
                inb = [x for x, ff in zip(db, f) if abs(ff) <= h - 1]
                oob = [(x, ff) for x, ff in zip(db, f) if abs(ff) >= h + 5]
                tot = 10 * math.log10(sum(10 ** (x / 10) for x in inb))
                fl = sorted(x for x, _ in oob)[len(oob) // 2] if oob else float("nan")
                mx = max(oob) if oob else (float("nan"), 0)
                fo.write("%s ant %s: in-band mean %.1f peak %.1f total %.1f dBFS | floor(median oob) %.1f | strongest oob %.1f at %+.1f MHz | MER %s | spc keys %s\n" % (
                    time.strftime("%H:%M:%S", time.gmtime()), k, sum(inb) / len(inb), max(inb), tot, fl, mx[0], mx[1],
                    (v.get("cns") or {}).get("mer_db"), sorted(x for x in s if x != "db")))
        except Exception as e:
            fo.write("sample failed: %r\n" % (e,))
        fo.flush()
