#!/usr/bin/env python3
"""SH-427 leg report: every "[WARNING] HoudiniStream TX late-release" line
verbatim (up to 30 per stream) with its log line number and the nearest MLPD
stamps before and after it, then the UE per-channel TX totals.
usage: late_release_report.py <run.log>"""
import collections, re, sys

L = open(sys.argv[1], errors="replace").read().splitlines()
L = [re.sub(r"\x1b\[[0-9;]*m", "", l) for l in L]
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
tx = collections.Counter()
for l in L:
    m = re.search(r"UE [\d.]+ link health: .*", l)
    if m:
        for k, v in re.findall(r"(tx\d\.\w+) \+(\d+)", m.group(0)):
            tx[k] += int(v)
print("UE totals: " + ", ".join("%s +%d" % kv for kv in sorted(tx.items())))
cns = re.findall(r"\((\d+) datagrams, (\d+) low\)", "\n".join(L))
print("CNS low %s" % ("%s/%s" % (cns[-1][1], cns[-1][0]) if cns else "none"))
