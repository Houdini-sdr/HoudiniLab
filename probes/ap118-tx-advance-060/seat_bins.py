"""The logged pilot seat (HOUDINI_BS_RX pilot_grid_off, every 20th frame) of a
run in bins of BIN seconds, binned by frame number (50 frames/s) so that the
lines the log prints without a time prefix count too; the UTC of each bin comes
from the prefixed lines (epoch-mod-100 prefix walked from the run start).
usage: seat_bins.py <log> <start UTC> [bin s] [frames/s]"""
import re, sys, datetime as dt
import statistics as st
log, start = sys.argv[1], sys.argv[2]
B = float(sys.argv[3]) if len(sys.argv) > 3 else 60.0
FPS = float(sys.argv[4]) if len(sys.argv) > 4 else 50.0
t0 = dt.datetime.strptime(start, "%Y-%m-%d %H:%M:%S").replace(tzinfo=dt.timezone.utc).timestamp()
base = int(t0) - int(t0) % 100
prev = None
pre = re.compile(r"^(?:\[\w+\] )?(\d{1,2}):(\d{6}) ")
pat = re.compile(r"HOUDINI_BS_RX: frame=(\d+) .*pilot_grid_off=(-?\d+)")
rows, anchor = [], []
for line in open(log, errors="replace"):
    m = pre.match(line)
    t = None
    if m:
        s = int(m.group(1)) + int(m.group(2)) / 1e6
        if prev is None:
            if base + s < t0 - 5:
                base += 100
        elif s < prev - 50:
            base += 100
        prev = s
        t = base + s
    g = pat.search(line)
    if g:
        f, v = int(g.group(1)), int(g.group(2))
        rows.append((f, v))
        if t is not None:
            anchor.append(t - f / FPS)
z = st.median(anchor)   # the UTC of frame 0
bins = {}
for f, v in rows:
    bins.setdefault(int((z + f / FPS) // B), []).append(v)
allv = [v for _, v in rows]
for k in sorted(bins):
    v = bins[k]
    print("%s n=%3d mean %+5.2f min %+d max %+d" % (dt.datetime.fromtimestamp(k * B, dt.timezone.utc).strftime("%H:%M:%S"), len(v), st.mean(v), min(v), max(v)))
means = [st.mean(bins[k]) for k in sorted(bins)]
print("all n=%d (%d without a time prefix) mean %+.2f sd %.2f min %+d max %+d; bin means %+.2f..%+.2f; frame-0 anchor spread %.3f s"
      % (len(allv), len(rows) - len(anchor), st.mean(allv), st.pstdev(allv), min(allv), max(allv), min(means), max(means), max(anchor) - min(anchor)))
