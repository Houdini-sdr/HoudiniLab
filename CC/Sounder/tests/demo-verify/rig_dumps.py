"""Readers for the sounder's measurement dumps, one copy shared by the fstage_*
tools: the CNS dump (HOUDINI_CSI_DUMP), the beacon RAM (HOUDINI_DUMP_BEACON)
and the UE's re-sync windows (HOUDINI_DUMP_RESYNC_WIN), plus where the beacon
sits in a window and which samples are clear of it."""
import collections, glob, os
import numpy as np

CnsDump = collections.namedtuple("CnsDump", "N cp es nsym H di x")


def read_cns(path):
    """One CNS dump: a header (N, cp, es, nsym, nd) of int32; H, N complex64 over
    the DC-centred tones; the nd data tone indices, int32; then the U slot, int16
    IQ. x is the slot conjugated, the recorder's sense."""
    b = open(path, "rb").read()
    N, cp, es, nsym, nd = (int(v) for v in np.frombuffer(b[:20], np.int32))
    o = 20
    H = np.frombuffer(b[o:o + 8 * N], np.float32).reshape(-1, 2); H = H[:, 0] + 1j * H[:, 1]; o += 8 * N
    di = np.frombuffer(b[o:o + 4 * nd], np.int32); o += 4 * nd
    s = np.frombuffer(b[o:], np.int16).reshape(-1, 2).astype(np.float64)
    return CnsDump(N, cp, es, nsym, H, di, s[:, 0] - 1j * s[:, 1])


def read_iq(path):
    x = np.fromfile(path, np.int16).astype(float).reshape(-1, 2)
    return x[:, 0] + 1j * x[:, 1]


def beacon_and_windows(run):
    """The beacon RAM's nonzero span and the run's re-sync window files, or None
    when the run has either one missing."""
    rb = os.path.join(run, "beacon_ram.bin")
    wins = sorted(glob.glob(os.path.join(run, "resync", "resyncwin_*.bin")))
    if not os.path.exists(rb) or not wins:
        return None
    r = read_iq(rb)
    nz = np.flatnonzero(np.abs(r) > 0)
    return r[nz[0]:nz[-1] + 1], wins


GUARD = 64  # samples kept clear of each beacon edge (the fitted timing is within one)


def locate(x, refs):
    """The beacon's start in window x: the correlation peak over the candidate
    references; returns (start, the reference that won)."""
    best = None
    for rr in refs:
        c = np.abs(np.correlate(x, rr, "valid"))
        k = int(np.argmax(c))
        if best is None or c[k] > best[0]:
            best = (c[k], k, rr)
    return best[1], best[2]


def beacon_free(n, k, L):
    """The beacon-free regions [a, b) of an n-sample window whose beacon of L
    samples starts at k: before it and after it, GUARD samples clear."""
    return [(0, max(0, k - GUARD)), (min(n, k + L + GUARD), n)]


def segments(regions, M, hop):
    """Starts of the M-sample segments, every hop, that lie wholly inside one region."""
    return [s for a, b in regions for s in range(a, b - M + 1, hop)]
