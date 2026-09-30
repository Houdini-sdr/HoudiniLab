#!/usr/bin/env python3
"""Check that no NIC receive flow lands on the sounder's pinned cores. Run it on
the rig host while a run streams (its first minute is enough). Stdlib only.

The 100G NIC hashes each node's RX flow onto a queue by a key drawn at every
boot, and each queue's completion interrupt runs on the CPU its IRQ affinity
names; a flow on a pacer's or the sounder main's core preempts it (V1/V2: 63-67
late releases per run with a flow on core 16). The queue-to-CPU map is read
from /proc (the port's mlx5_comp<N> IRQs and their affinity), and only when it
cannot be read is queue N taken to interrupt CPU N (said in the result). The
pacers' own TX completions are TX, not RX, so any RX packets on a queue that
interrupts a pinned core mean a flow sits there.

usage: pacer_core_check.py --cores 15,18,19 --ports <data port>[,<data port>] [--secs 3]
(the data ports: the interfaces the radios stream to, `ip -br link`).
Exit 0: no RX on those cores' queues. Exit 1: a flow on one (move the pacers to
free isolated cores with HOUDINI_TX_CPU_AFFINITY and restart). Exit 2: a port
whose queue counters cannot be read (a wrong name would otherwise read as no
flow). Read-only (ethtool -S, /proc)."""
import argparse, os, re, subprocess, sys, time


def rx_counts(text):
    """{queue: rx packets} from `ethtool -S` output."""
    return {int(q): int(n) for q, n in re.findall(r"^\s*rx(\d+)_packets:\s*(\d+)", text, re.M)}


def cpulist(text):
    """The CPUs of a kernel CPU list ("15", "0-3,8")."""
    out = set()
    for part in text.strip().split(","):
        if "-" in part:
            lo, hi = part.split("-")
            out.update(range(int(lo), int(hi) + 1))
        elif part:
            out.add(int(part))
    return out


def queue_cpus(interrupts, bdf, affinity):
    """{queue: CPUs} for one port's completion IRQs (mlx5_comp<N>@pci:<bdf> in
    /proc/interrupts); `affinity(irq)` returns the IRQ's CPU list text. A queue
    whose affinity cannot be read is left out, so it falls back to queue N on
    CPU N (and is named as such) instead of mapping to no CPU and never being
    flagged."""
    out = {}
    for irq, q, dev in re.findall(r"^\s*(\d+):.*\bmlx5_comp(\d+)@pci:(\S+)\s*$", interrupts, re.M):
        if dev == bdf:
            cpus = cpulist(affinity(irq))
            if cpus:
                out[int(q)] = cpus
    return out


def flows(before, after, cores, secs, qcpus=None, min_pps=100.0):
    """[(queue, pkt/s, CPUs)] for the queues that received more than min_pps
    and interrupt a pinned core; `qcpus` is the queue-to-CPU map (queue N on
    CPU N where it has no entry)."""
    out = []
    for q in sorted(after):
        pps = (after.get(q, 0) - before.get(q, 0)) / secs
        cpus = (qcpus or {}).get(q, {q})
        if pps > min_pps and cpus & set(cores):
            out.append((q, pps, sorted(cpus)))
    return out


def _affinity(irq):
    for name in ("effective_affinity_list", "smp_affinity_list"):
        try:
            with open("/proc/irq/%s/%s" % (irq, name)) as f:
                return f.read()
        except OSError:
            pass
    return ""


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--cores", required=True)
    ap.add_argument("--ports", default="enp1s0f0np0,enp1s0f1np1", help="default: the reference rig's data ports")
    ap.add_argument("--secs", type=float, default=3.0)
    a = ap.parse_args()
    cores = [int(c) for c in a.cores.split(",")]
    ports = a.ports.split(",")

    def read(p):
        try:
            r = subprocess.run(["ethtool", "-S", p], capture_output=True, text=True)
        except OSError as e:  # no ethtool on this host
            return {}, str(e)
        return rx_counts(r.stdout), (r.stderr or "").strip()

    try:
        interrupts = open("/proc/interrupts").read()
    except OSError:
        interrupts = ""
    qmap, identity = {}, []
    for p in ports:
        try:
            bdf = os.path.basename(os.readlink("/sys/class/net/%s/device" % p))
        except OSError:
            bdf = None
        qmap[p] = queue_cpus(interrupts, bdf, _affinity) if bdf else {}
        if not qmap[p]:
            identity.append(p)
    before = {}
    for p in ports:
        before[p], err = read(p)
        if not before[p]:
            print("cannot read %s's RX queue counters (ethtool -S%s): name the data ports with --ports"
                  % (p, ": " + err if err else ""))
            return 2
    time.sleep(a.secs)
    after = {p: read(p)[0] for p in ports}
    # Queues with traffic counters but no readable IRQ affinity, on a port whose
    # map was otherwise read: taken as CPU N, and said.
    # Only queues that received in the window: mlx5 keeps counters for channels
    # no longer active, which have no IRQ at all.
    partial = ["%s queue %s" % (p, ",".join(map(str, qs))) for p in ports if qmap[p]
               for qs in [sorted(q for q in after[p] if q not in qmap[p] and after[p][q] != before[p].get(q))] if qs]
    how = ("IRQ map read from /proc" if not identity
           else "queue N taken as CPU N for %s: its IRQ map was not readable" % ", ".join(identity))
    if partial:
        how += "; queue N taken as CPU N for %s (no readable IRQ affinity)" % "; ".join(partial)
    bad = ["%s queue %d (CPU %s): %.0f RX pkt/s" % (p, q, ",".join(map(str, c)), pps)
           for p in ports for q, pps, c in flows(before[p], after[p], cores, a.secs, qmap[p])]
    if bad:
        print("RX FLOW ON A PINNED CORE: " + "; ".join(bad) + " (" + how + ")")
        return 1
    print("ok: no RX on the queues that interrupt cores %s over %.0f s (%s)" % (a.cores, a.secs, how))
    return 0


if __name__ == "__main__":
    sys.exit(main())
