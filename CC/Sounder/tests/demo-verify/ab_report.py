#!/usr/bin/env python3
"""SH-427 A/B (AP-79): one row per leg from fstage_run.sh run dirs, A and B side by side.

usage: ab_report.py <run_dir> [<run_dir> ...]   (in leg order: A1 B1 A2 B2 ...)

Per leg:
  stack    each node's device_build and the host_build, from the sounder's own
           "Node stack" lines (the swap is verified against the announced ids)
  tx       the per-stream TX counter increments summed over the link-health lines
           (UE and BS; the lines print only when a counter moves), e.g. UE tx1.drops
  trim     the host pacer's close-of-stream lines ("trim final bias", SH-427's A symptom)
  clock    the plugin's log lines naming DeviceClock / TX_HOST_STATUS / TIME_ERROR /
           arrival offset (B logs its per-stream arrival offset and clock re-sets)
           (the A/B build never reads TX_HOST_STATUS itself: only what the plugin logs)
  resync   targeted re-syncs, the longest gap in UE frames, the BS pilot seat's
           largest |value| (the AP-80 separator: a walk is AP-80, not the pacer)
  quality  CNS low of all datagrams; MER per BS antenna from the CSI dumps
  warn     device [WARNING] lines by class, and IntrStatus lines (SH-438)
"""
import collections, contextlib, glob, importlib.util, io, os, re, sys

HERE = os.path.dirname(os.path.abspath(__file__))
_argv = sys.argv
sys.argv = ["x"]
spec = importlib.util.spec_from_file_location("fr", os.path.join(HERE, "fstage_report.py"))
fr = importlib.util.module_from_spec(spec)
with contextlib.redirect_stdout(io.StringIO()):
    spec.loader.exec_module(fr)
sys.argv = _argv


def clean(t):
    return re.sub(r"\x1b\[[0-9;]*m", "", t)


def leg(run):
    name = os.path.basename(run.rstrip("/"))
    L = clean(open(os.path.join(run, name + ".log"), errors="replace").read())
    out = {"run": name}
    st = re.findall(r"Node stack (BS|UE) ([\d.]+):.*?device_build=(\S+).*?host_build=(\S+)", L)
    out["stack"] = "; ".join("%s dev %s host %s" % (r, d, h) for r, _, d, h in dict(((s[0], s) for s in st)).values())
    tx = collections.Counter()
    for m in re.finditer(r"(UE|BS) [\d.]+ link health: .*", L):
        for k, v in re.findall(r"(tx\d\.\w+) \+(\d+)", m.group(0)):
            tx["%s %s" % (m.group(1), k)] += int(v)
    out["tx"] = dict(tx)
    out["trim"] = sorted(set(re.findall(r"(trim final bias [^\n(]*)", L)))
    lines = L.splitlines()
    out["clock"] = [l.strip()[:220] for l in lines if re.search(r"DeviceClock|TX_HOST_STATUS|TIME_ERROR|arrival offset", l)]
    # The plugin's lines carry no timestamp: place each by its line number and
    # the nearest sounder (MLPD) stamps around it, seconds within the minute.
    stamp = re.compile(r"^(\d+:\d{6}) (INFOR|WARNG|ERROR)")
    detail = []
    for i, l in enumerate(lines):
        if re.search(r"DeviceClock|TX_HOST_STATUS|TIME_ERROR|arrival offset", l):
            before = next((stamp.match(lines[j]).group(1) for j in range(i - 1, -1, -1) if stamp.match(lines[j])), "-")
            after = next((stamp.match(lines[j]).group(1) for j in range(i + 1, len(lines)) if stamp.match(lines[j])), "-")
            detail.append("line %d [%s .. %s] %s" % (i + 1, before, after, l.strip()[:150]))
    out["clock_detail"] = detail
    fr_ = [int(v) for v in re.findall(r"Re-sync frame (\d+): beacon alive", L)]
    out["resyncs"] = len(fr_)
    out["maxgap"] = max((b - a for a, b in zip(fr_, fr_[1:])), default=0)
    seat = [abs(int(v)) for v in re.findall(r"pilot_grid_off=([-\d]+)", L)]
    out["seatmax"] = max(seat) if seat else None
    out["seat_over30"] = sum(s > 30 for s in seat)
    cns = re.findall(r"\((\d+) datagrams, (\d+) low\)", L)
    out["cns_low"] = "%s/%s" % (cns[-1][1], cns[-1][0]) if cns else "none"
    for k, nm in ((0, "cns_dump.bin"), (1, "cns_dump_ant1.bin")):
        p = os.path.join(run, nm)
        if os.path.exists(p):
            out["mer%d" % k] = round(fr.ul(p)["mer"], 2)
    w = collections.Counter(re.sub(r"0x[0-9A-Fa-f]+|\d+(\.\d+)?", "N", l.strip())[:90]
                            for l in L.splitlines() if "[WARNING]" in l)
    out["warn"] = sum(w.values())
    out["warn_classes"] = dict(w.most_common())
    out["intr"] = [l.strip()[:120] for l in L.splitlines() if "IntrStatus" in l]
    return out


def main():
    rows = [leg(r) for r in sys.argv[1:]]
    for r in rows:
        print("== %s" % r["run"])
        print("   stack   %s" % r["stack"])
        print("   tx      %s" % (", ".join("%s +%d" % kv for kv in sorted(r["tx"].items())) or "no increments"))
        print("   trim    %s" % ("; ".join(r["trim"]) or "-"))
        print("   clock   %d lines" % len(r["clock"]))
        for d in r["clock_detail"]:
            print("           %s" % d)
        print("   resync  %d, max gap %d UE frames; seat max |%s|, >30 in %d frames" % (r["resyncs"], r["maxgap"], r["seatmax"], r["seat_over30"]))
        print("   quality CNS low %s; MER ant0 %s ant1 %s" % (r["cns_low"], r.get("mer0", "-"), r.get("mer1", "-")))
        print("   warn    %d; IntrStatus %d%s" % (r["warn"], len(r["intr"]), (": " + " | ".join(r["intr"])) if r["intr"] else ""))


if __name__ == "__main__":
    main()
