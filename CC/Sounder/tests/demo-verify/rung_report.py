#!/usr/bin/env python3
"""AP-79 rung report: every pass-criterion number from one sounder log.
Throttled warnings (powers of two, or every Nth) are read by their LAST
occurrence number, never by counting lines."""
import re, sys, collections, statistics as st
L = open(sys.argv[1], errors="replace").read().splitlines()
def last_occ(pat):
    v = 0
    for l in L:
        m = re.search(pat + r".*?occurrence (\d+)", l)
        if m: v = max(v, int(m.group(1)))
    return v
def grab(pat, conv=float):
    return [conv(m.group(1)) for l in L for m in [re.search(pat, l)] if m]
print("== %s (%d lines)" % (sys.argv[1], len(L)))
acq = grab(r"syncSearch: detection #\d+ statistic ([\d.]+) vs bar 0\.2")
print("acquisition statistics (coherence, bar 0.2):", acq[:5])
alive = [(int(m.group(1)), int(m.group(2)), float(m.group(3))) for l in L
         for m in [re.search(r"Re-sync frame (\d+): beacon alive .*?resid ([+-]?\d+).*?snr ([\d.]+) dB", l)] if m]
if alive:
    r = [a[1] for a in alive]; s = [a[2] for a in alive]
    w2 = sum(abs(x) <= 2 for x in r); w4 = sum(abs(x) <= 4 for x in r)
    print("re-syncs alive %d; resid %d..%d, |r|<=2 %.0f%%, |r|<=4 %.0f%%; beacon SNR %.1f/%.1f/%.1f dB" %
          (len(r), min(r), max(r), 100.0 * w2 / len(r), 100.0 * w4 / len(r), min(s), st.median(s), max(s)))
print("BAD SYNC %d, PILOT LOST %d, escalations %d" % (sum("BAD SYNC" in l for l in L),
      sum("UE PILOT LOST" in l for l in L), sum(bool(re.search(r"escalat(e|ion) (to|#)|ESCALAT", l)) for l in L)))
print("BS no-UE-burst skips >= %d; LTS-untrusted >= %d; CNS low %s" % (
      last_occ("no UE burst in frame read"), last_occ("failed the LTS check"),
      # the periodic SUMMARY carries the true total; the per-event warning's
      # 'low occurrence K of D' is throttled on powers of two (AP-58)
      (lambda m: "%s of %s (summary)" % (m[-1][1], m[-1][0]) if m else "none")(re.findall(r"\((\d+) datagrams, (\d+) low\)", "\n".join(L)))))
ev = [(int(m.group(1)), int(m.group(2))) for l in L for m in [re.search(r"UE pilot burst: scheduled (\d+) frames up to (\d+)", l)] if m]
if len(ev) > 2:
    fr = 122880.0
    m = re.search(r'"frame_schedule":\["([A-Z]+)"', "\n".join(L[:10]))
    if m: fr = 122880.0 if len(m.group(1)) == 30 else 1228800.0 if len(m.group(1)) == 20 else fr
    span = (ev[-1][1] - ev[0][1]) / fr
    print("UE pilot coverage %.1f%%" % (100.0 * (sum(a for a, _ in ev) - ev[0][0]) / span))
pu = grab(r"pu_spacing_err=([-\d]+)", int); seat = grab(r"pilot_grid_off=([-\d]+)", int)
if pu: print("pu_spacing_err: %s" % dict(collections.Counter(pu).most_common(4)))
if seat: print("pilot seat: mean %.1f, sd %.1f, range %d..%d (n %d)" % (st.mean(seat), st.pstdev(seat), min(seat), max(seat), len(seat)))
cfo = [l for l in L if "Beacon CFO frame" in l]
if cfo: print("clock:", re.sub(r"^.*?Beacon CFO", "Beacon CFO", cfo[-1])[:160])
alarms = [l for l in L if "link health: [" in l and "clean |" not in l]
kinds = collections.Counter()
for l in alarms:
    for m in re.finditer(r"(tx\d\.\w+ \+\d+|preflight new FAIL [^|;]+|rx_\w+ \+[1-9]\d*|tx_\w+ \+[1-9]\d*|host\.\w+ \+\d+)", l):
        kinds[re.sub(r"\+\d+", "+N", m.group(1))] += 1
sat = sum("saturated" in l for l in alarms)
print("health alarm lines %d (%d carry the standing egress-saturation item); kinds: %s" % (len(alarms), sat, dict(kinds)))
late = grab(r"tx0\.late \+(\d+)", int); und = grab(r"tx0\.under \+(\d+)", int); zf = grab(r"tx0\.zerofill \+(\d+)", int)
print("UE tx0 totals: late %d, under %d, zerofill %d; TX status events %d" % (sum(late), sum(und), sum(zf), sum("TX status:" in l for l in L)))
print("state records:", sum("RFDC state record" in l for l in L), "| CSI dump:", any("CSI dump written" in l for l in L))
