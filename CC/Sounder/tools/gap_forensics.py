#!/usr/bin/env python3
"""Gap-STRUCTURE forensics for rx-recorder captures.

inspect_rx_record.py answers "how much loss, is the file trustworthy";
this tool answers "what is the loss's SHAPE" — the structure of
/Data/Gaps is a fingerprint of the losing mechanism. It reads only the
gap table and attributes, so it is cheap on multi-GB captures.

Fingerprints validated on rig B 2026-07-12 (docs/RX_MAX_RATE.md, V4
root-cause STATUS + results):

- Gap lengths quantize to the WIRE PACKET QUANTUM (848 samples at the
  zero-copy paired geometry, +/-1 from integer-ns stamp rounding): the
  drop unit is whole packets at the NIC, i.e. fill-ring starvation.
- Modal clean-run of exactly 64 packets between gaps = NAPI-budget
  slivers: delivery is limited to one softirq budget per wake while the
  fill ring runs on fumes.
- Max gap pinned at ~50 ms = the xsk worker's recv_timeout_ms poll
  backstop (dead air on starvation exit).
- 0%-loss stretches alternating with 50-65% churn at ~100 ms scale =
  the ring-occupancy sawtooth: clean-stretch length ~= ring depth /
  consumer deficit (256 MB / 20% deficit ~= 140 ms), NOT a consumer
  whose speed oscillates.

Usage:
  tools/gap_forensics.py capture.h5 [more.h5 ...] [--bucket-ms 100]

Needs h5py + numpy (on rig B: the houdini_test venv).
"""
import argparse

import h5py
import numpy as np

# HDF5 dataset rows grow in MAX_FRAME_INC windows (include/macros.h);
# gaps clustering at this period would implicate the writer's
# closeDataset/openDataset extend cycle.
EXTEND_WINDOW_SLOTS = 2000

CAUSE_NAMES = {0: "time_jump", 1: "host_ring", 2: "write_error",
               3: "backward", 4: "resync"}


def scalar_attrs(group):
    """h5py attributes arrive as 0/1-element arrays; unwrap them."""
    out = {}
    for k in group.attrs:
        v = np.asarray(group.attrs[k]).ravel()
        out[k] = v[0] if v.size == 1 else group.attrs[k]
    return out


def estimate_quantum(lengths, tol):
    """Wire packets/gap-length quantum: the largest q <= 4096 for which
    (almost) every gap length sits within +/-tol of a multiple of q.
    gcd alone fails here because integer-ns stamp rounding perturbs
    lengths by +/-1 sample."""
    uniq = np.unique(lengths.astype(np.int64))
    best_q, best_cover = 1, 0.0
    for q in range(2, 4097):
        r = uniq % q
        cover = float(np.mean((r <= tol) | (r >= q - tol)))
        # Prefer the largest q at (near-)full coverage: multiples of the
        # true quantum also cover, but smaller divisors are not "larger".
        if cover >= 0.99 and q > best_q:
            best_q, best_cover = q, cover
    return best_q, best_cover


def report(path, bucket_ms):
    with h5py.File(path, "r") as f:
        d = f["Data"]
        a = scalar_attrs(d)
        rate = float(a["RATE"])
        slot = int(a["SLOT_SAMP_LEN"])
        total = int(a["SLOTS_RECORDED"]) * slot
        print(f"\n===== {path}")
        for k in ("READ_PATH", "GAPS_EXACT", "STREAM_ARGS",
                  "SLOTS_RECORDED", "TOTAL_UNTRUSTED_SAMPLES"):
            if k in a:
                print(f"  {k} = {a[k]}")

        g = d["Gaps"][:] if "Gaps" in d else np.empty((0, 4), np.int64)
        if g.size == 0:
            print("  no gap extents — nothing to fingerprint")
            return
        starts = g[:, 0].astype(np.int64)
        lens = g[:, 1].astype(np.int64)
        causes = g[:, 3].astype(np.int64)

        vals, cnts = np.unique(causes, return_counts=True)
        breakdown = ", ".join(
            f"{CAUSE_NAMES.get(int(v), v)}:{int(c)}" for v, c in zip(vals, cnts))
        print(f"  extents: {len(g)} ({breakdown})")

        # Structure analysis over positive-length loss extents only
        # (cause 3/4 are zero-length anomaly markers).
        m = lens > 0
        S, L = starts[m], lens[m]
        if L.size == 0:
            print("  only anomaly markers present")
            return
        lost = int(L.sum())
        print(f"  grid ~{total} samples, lost {lost} "
              f"({100.0 * lost / total:.3f} %)")

        tol = max(1, int(round(2e-9 * rate)))  # stamp-rounding bound
        q, cover = estimate_quantum(L, tol)
        if q > 1:
            print(f"  packet quantum estimate: {q} samples/packet "
                  f"({100.0 * cover:.1f} % of unique gap lengths are "
                  f"multiples, +/-{tol})")
        else:
            print("  no clean packet quantum found (mixed or "
                  "non-packet-aligned loss)")
        lv, lc = np.unique(L, return_counts=True)
        order = np.argsort(lc)[::-1][:6]
        top = [(int(lv[i]), int(lc[i])) for i in order]
        print(f"  top gap lengths (len, count): {top}")
        print(f"  gap len: min {L.min()} med {int(np.median(L))} "
              f"mean {L.mean():.0f} max {L.max()} "
              f"({L.max() / rate * 1e3:.3f} ms; ~50 ms = worker poll "
              f"timeout fingerprint)")

        b = int(bucket_ms * 1e-3 * rate)
        nb = total // b + 1
        per = np.bincount(S // b, weights=L.astype(float), minlength=nb)[:nb]
        print(f"  loss % per {bucket_ms} ms bucket:")
        print("   ", np.array2string(100.0 * per / b, precision=1,
                                     max_line_width=96))

        if S.size > 2:
            order = np.argsort(S)
            sp = np.diff(S[order]) - L[order][:-1]
            sv, sc = np.unique(sp, return_counts=True)
            tops = np.argsort(sc)[-6:][::-1]
            if q > 1:
                pkts = [f"{int(sv[i])} (={sv[i] / q:.1f} pkt) x{int(sc[i])}"
                        for i in tops]
            else:
                pkts = [f"{int(sv[i])} x{int(sc[i])}" for i in tops]
            print(f"  top clean-run lengths between gaps: {pkts}")
            print(f"    (a modal run of exactly 64 packets = NAPI-budget "
                  f"sliver fingerprint)")
            print(f"  clean-run max {sp.max()} samples "
                  f"({sp.max() / rate * 1e3:.2f} ms; sawtooth period ~= "
                  f"ring depth / consumer deficit)")

        for name, period in (("slot", slot),
                             ("extend window", EXTEND_WINDOW_SLOTS * slot)):
            h, _ = np.histogram((S % period) / period, bins=10, range=(0, 1))
            flat = h.max() < 2 * max(1, h.min())
            print(f"  gap position within {name} (10 bins, "
                  f"{'flat=unrelated' if flat else 'CLUSTERED'}): {h}")


def main():
    p = argparse.ArgumentParser(
        description="Loss-structure fingerprints from rx-recorder "
                    "/Data/Gaps tables")
    p.add_argument("files", nargs="+", help="rx-recorder HDF5 capture(s)")
    p.add_argument("--bucket-ms", type=float, default=100.0,
                   help="loss-profile bucket width in ms (default 100)")
    args = p.parse_args()
    for path in args.files:
        report(path, args.bucket_ms)


if __name__ == "__main__":
    main()
