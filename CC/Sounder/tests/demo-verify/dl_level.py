"""The downlink at the UE's ADC, per run, from the re-sync windows the run dumps
(HOUDINI_DUMP_RESYNC_WIN, set by fstage_run.sh): the beacon's power and the
beacon-free floor in each window (dBFS re a full-scale int16 complex tone), the
beacon-to-floor ratio, the floor's in-band (+-23 MHz) minus out-of-band (beyond
+-30 MHz) split at 122.88 MSPS, and the largest sample. Compare a run against a
wired reference to tell a low signal from a raised floor (over the air: a front
end or antenna fault lowers the beacon; a working RX LNA raises the floor).
Run from CC/Sounder: python3 tests/demo-verify/dl_level.py <run_dir> [<run_dir> ...]
(a run that never locked has no windows)."""
import sys, numpy as np
sys.path.insert(0, "tests/demo-verify")
import rig_dumps as rd
FS = 32767.0 ** 2
for run in sys.argv[1:]:
    bw = rd.beacon_and_windows(run)
    if bw is None:
        print(run, ": no beacon RAM or no windows"); continue
    ref, wins = bw
    L = len(ref); rows = []
    for w in wins:
        x = rd.read_iq(w)
        k, _ = rd.locate(x, [ref])
        b = x[k:k + L]
        free = np.concatenate([x[a:c] for a, c in rd.beacon_free(len(x), k, L) if c > a])
        pb = 10 * np.log10(np.mean(np.abs(b) ** 2) / FS)
        pf = 10 * np.log10(np.mean(np.abs(free) ** 2) / FS)
        # floor spectrum: in the beacon's band (+-23 MHz) vs outside +-30 MHz, 122.88 Msps
        n = 4096; seg = free[: (len(free) // n) * n].reshape(-1, n) * np.hanning(n)
        P = np.mean(np.abs(np.fft.fftshift(np.fft.fft(seg, axis=1), axes=1)) ** 2, axis=0)
        f = (np.arange(n) - n / 2) * 122.88 / n
        inb = 10 * np.log10(np.mean(P[np.abs(f) < 23])); oob = 10 * np.log10(np.mean(P[np.abs(f) > 30]))
        pk = float(np.max(np.abs(x))) / 32767.0
        rows.append((pb, pf, pb - pf, inb - oob, pk))
    r = np.array(rows)
    med = np.median(r, axis=0)
    print("%s: %d windows | beacon %.1f dBFS | floor %.1f dBFS | beacon-to-floor %.1f dB | floor in-band minus out-of-band %.1f dB | peak |x| %.3f FS" % (
        run.rstrip("/").split("/")[-1], len(r), med[0], med[1], med[2], med[3], np.max(r[:, 4])))
