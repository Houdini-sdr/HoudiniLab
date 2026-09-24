#!/usr/bin/env python3
"""One demo-length run's END report from its log (AP-79 D runs).
usage: demo_report.py <run dir>   (the log is <run dir>/<basename>.log)
Prints: UE/BS TX totals, the TX status timeline about every 60 s of device
time, saturation and watchdog lines with their device time, CNS low per 300 s,
late-release forensic statistics, the wake-jitter close lines, warning classes,
and the tracked carrier offset about every 5 min."""
import collections, os, re, sys

run = sys.argv[1].rstrip("/")
name = os.path.basename(run)
L = [re.sub(r"\x1b\[[0-9;]*m", "", l) for l in open(os.path.join(run, name + ".log"), errors="replace").read().splitlines()]

tx = collections.Counter()
for l in L:
    m = re.search(r"(UE|BS) [\d.]+ link health: .*", l)
    if m:
        for k, v in re.findall(r"(tx\d\.\w+) \+(\d+)", m.group(0)):
            tx["%s %s" % (m.group(1), k)] += int(v)
print("TX totals: " + (", ".join("%s +%d" % kv for kv in sorted(tx.items())) or "none"))

ev = []
for i, l in enumerate(L):
    m = re.search(r"TX status: .*? at (\d+) ns.*?ch0:acked=\d+,late=(\d+),under=(\d+),seqerr=\d+,zerofill=(\d+).*?ch1:acked=\d+,late=(\d+),under=(\d+)", l)
    if m:
        ev.append((i, int(m.group(1)) / 1e9) + tuple(int(g) for g in m.groups()[1:]))
print("TX status events: %d" % len(ev))
last = -1e9
for e in ev:
    if e[1] - last >= 60 or e is ev[-1]:
        print("  dev %7.1f s  ch0 late %6d under %6d zerofill %10d | ch1 late %5d under %5d" % e[1:])
        last = e[1]

def tdev(i):
    prev = None
    for a in ev:
        if a[0] >= i:
            return a[1] if prev is None else prev[1] + (a[1] - prev[1]) * (i - prev[0]) / max(1, a[0] - prev[0])
        prev = a
    return ev[-1][1] if ev else float("nan")

for i, l in enumerate(L):
    if "SATURATED" in l or "WATCHDOG" in l:
        print("  line %d ~dev %.0f s: %s" % (i + 1, tdev(i), l.strip()[l.find("[WARNING]"):][:150]))

pts = [(i, int(m.group(1)), int(m.group(2))) for i, l in enumerate(L) for m in [re.search(r"\((\d+) datagrams, (\d+) low\)", l)] if m]
if pts and ev:
    prev, edge = (0, 0), 300
    for i, d, lo in pts:
        t = tdev(i)
        if t >= edge or (i, d, lo) == pts[-1]:
            print("CNS to ~%4.0f s: datagrams %5d low %4d (%.1f %%)" % (t, d - prev[0], lo - prev[1], 100.0 * (lo - prev[1]) / max(1, d - prev[0])))
            prev, edge = (d, lo), edge + 300
elif pts:
    print("CNS total: %d low of %d (no TX status events to time the windows)" % (pts[-1][2], pts[-1][1]))

rows = [l[l.find("[WARNING] HoudiniStream TX late-release"):] for l in L if "HoudiniStream TX late-release" in l]
c = collections.Counter()
for t in rows:
    c["ledger 0.0" if "ledger 0.0" in t else "ledger other"] += 1
    c["rail room" if "rail room" in t else "rail other"] += 1
    m = re.search(r"held (\d+) us", t)
    h = int(m.group(1)) if m else -1
    c["held>=5ms" if h >= 5000 else ("held 1-5ms" if h >= 1000 else "held<1ms")] += 1
print("late-release lines: %d %s" % (len(rows), dict(c)))
for i, l in enumerate(L):
    if "wake-jitter" in l:
        print("  %d %s" % (i + 1, l[l.find("wake-jitter"):][:230]))
w = collections.Counter(re.sub(r"\d{2,}", "N", l[l.find("[WARNING]"):])[:110] for l in L if "[WARNING]" in l and "late-release" not in l)
print("warning classes:")
for k, v in w.most_common():
    print("  %4d %s" % (v, k))
cfo = [l for l in L if "Beacon CFO frame" in l]
for l in cfo[:: max(1, len(cfo) // 8)] + cfo[-1:]:
    m = re.search(r"Beacon CFO frame (\d+): tracked ([+-][\d.]+) Hz \(([+-][\d.]+) ppm\)", l)
    if m:
        print("  CFO frame %s: tracked %s Hz (%s ppm)" % m.groups())
