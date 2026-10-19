"""Absolute UTC of each matching line of a sounder log whose prefix is
'<epoch seconds % 100>:<usec>', walking rollovers from a known start time.
usage: abs_times.py <log> <start 'YYYY-mm-dd HH:MM:SS' UTC> <regex>"""
import re, sys, datetime as dt
log, start, pat = sys.argv[1], sys.argv[2], re.compile(sys.argv[3])
t0 = dt.datetime.strptime(start, "%Y-%m-%d %H:%M:%S").replace(tzinfo=dt.timezone.utc).timestamp()
base = int(t0) - int(t0) % 100   # the hundred-second block the run starts in
prev = None
pre = re.compile(r"^(?:\[\w+\] )?(\d{1,2}):(\d{6}) ")
for line in open(log, errors="replace"):
    m = pre.match(line)
    if not m:
        continue
    s = int(m.group(1)) + int(m.group(2)) / 1e6
    if prev is None:
        if base + s < t0 - 5:  # first stamp already past a rollover
            base += 100
    elif s < prev - 50:        # rolled over
        base += 100
    prev = s
    if pat.search(line):
        print(dt.datetime.fromtimestamp(base + s, dt.timezone.utc).strftime("%H:%M:%S.%f")[:-3], line.strip()[:170])
