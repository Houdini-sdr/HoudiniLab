#!/usr/bin/env python3
"""SH-427 leg report: every "[WARNING] HoudiniStream TX late-release" line
verbatim (up to 30 per stream) with its log line number and the nearest MLPD
stamps before and after it, then the UE per-channel TX totals.
usage: late_release_report.py <run.log>"""
import collections, re, sys
import sounder_log as sl

L = sl.read_lines(sys.argv[1])
stamp = re.compile(r"^(\d+:\d{6}) (INFOR|WARNG|ERROR)")
KEY = "[WARNING] HoudiniStream TX late-release"
per = collections.defaultdict(list)
for i, l in enumerate(L):
    k = l.find(KEY)
    if k < 0:
        continue
    m = re.search(r"\bch(\d+)\b|tx_stream\[(\d+)\]|stream (\d+)", l[k:])
    ch = next((g for g in m.groups() if g is not None), "?") if m else "?"
    before = next((stamp.match(L[j]).group(1) for j in range(i - 1, -1, -1) if stamp.match(L[j])), "-")
    after = next((stamp.match(L[j]).group(1) for j in range(i + 1, len(L)) if stamp.match(L[j])), "-")
    per[ch].append((i + 1, before, after, l[k:].strip()))
print("late-release lines: %s" % ", ".join("stream %s: %d" % (c, len(v)) for c, v in sorted(per.items())) or "none")
for c, v in sorted(per.items()):
    print("== stream %s (%d lines, first %d shown)" % (c, len(v), min(30, len(v))))
    for n, b, a, t in v[:30]:
        print("line %d [%s .. %s] %s" % (n, b, a, t))
tx = sl.tx_totals(L)
print("UE totals: " + ", ".join("%s +%d" % (k, v) for (r, k), v in sorted(tx.items()) if r == "UE"))
cns = sl.cns_total(L)
print("CNS low %s" % ("%d/%d" % (cns[1], cns[0]) if cns else "none"))
