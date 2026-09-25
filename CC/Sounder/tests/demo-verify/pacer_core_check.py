#!/usr/bin/env python3
"""Check that no NIC receive flow lands on the sounder's pinned cores. Run it on
the rig host while a run streams (its first minute is enough). Stdlib only.

The 100G NIC hashes each node's RX flow onto a queue by a key drawn at every
boot, and queue N interrupts CPU N; a flow on a pacer's or the sounder main's
core preempts it (V1/V2: 63-67 late releases per run with a flow on core 16).
The pacers' own TX completions are TX, not RX, so any RX packets on a pinned
core's queue mean a flow sits there.

usage: pacer_core_check.py --cores 15,18,19 [--ports enp1s0f0np0,enp1s0f1np1] [--secs 3]
Exit 0: no RX on those queues. Exit 1: a flow on one (move the pacers to free
isolated cores with HOUDINI_TX_CPU_AFFINITY and restart). Read-only (ethtool -S)."""
import argparse, re, subprocess, sys, time


def rx_counts(text):
    """{queue: rx packets} from `ethtool -S` output."""
    return {int(q): int(n) for q, n in re.findall(r"^\s*rx(\d+)_packets:\s*(\d+)", text, re.M)}


def flows(before, after, cores, secs, min_pps=100.0):
    """[(core, pkt/s)] for the cores whose queue received more than min_pps."""
    out = []
    for c in cores:
        pps = (after.get(c, 0) - before.get(c, 0)) / secs
        if pps > min_pps:
            out.append((c, pps))
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--cores", required=True)
    ap.add_argument("--ports", default="enp1s0f0np0,enp1s0f1np1")
    ap.add_argument("--secs", type=float, default=3.0)
    a = ap.parse_args()
    cores = [int(c) for c in a.cores.split(",")]
    ports = a.ports.split(",")
    read = lambda p: rx_counts(subprocess.run(["ethtool", "-S", p], capture_output=True, text=True).stdout)
    before = {p: read(p) for p in ports}
    time.sleep(a.secs)
    after = {p: read(p) for p in ports}
    bad = []
    for p in ports:
        for c, pps in flows(before[p], after[p], cores, a.secs):
            bad.append("%s queue %d (CPU %d): %.0f RX pkt/s" % (p, c, c, pps))
    if bad:
        print("RX FLOW ON A PINNED CORE: " + "; ".join(bad))
        return 1
    print("ok: no RX on the queues of cores %s over %.0f s" % (a.cores, a.secs))
    return 0


if __name__ == "__main__":
    sys.exit(main())
