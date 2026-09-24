#!/usr/bin/env python3
"""AP-79 filter staging: the UE ADC's Fs/2 offset spur (2457.6 MHz, DEMO_VERIFICATION 9.8)
per UE bring-up, from the UE's re-sync windows (one run = one bring-up).

usage: fstage_spur.py <stage_dir> [<stage_dir> ...]

The spur sits at +-32.64 MHz from the 2425 MHz NCO (which sign depends on the
DDC's sense, so both are searched). Per run: Welch PSD (1024-sample Hann) over the
beacon-free samples of every window, the spur = the strongest bin within 0.5 MHz
of +-32.64 MHz, reported in dB over the median in-band (|f| < 23 MHz) bin, plus
the run's wideband beacon-free floor (the proxy used before, 9.8)."""
import glob, os, sys
import numpy as np
FS = 122.88
M = 1024
for sd in sys.argv[1:]:
    for run in sorted(d for d in glob.glob(os.path.join(sd, "*")) if os.path.isdir(d)):
        rb = os.path.join(run, "beacon_ram.bin"); wins = sorted(glob.glob(os.path.join(run, "resync", "resyncwin_*.bin")))
        if not os.path.exists(rb) or not wins:
            continue
        r = np.fromfile(rb, np.int16).astype(float).reshape(-1, 2); r = r[:, 0] + 1j * r[:, 1]
        nz = np.flatnonzero(np.abs(r) > 0); ref = r[nz[0]:nz[-1] + 1]; L = len(ref)
        P, nseg, wide = None, 0, []
        w = np.hanning(M)
        for wf in wins:
            x = np.fromfile(wf, np.int16).astype(float).reshape(-1, 2); x = x[:, 0] + 1j * x[:, 1]
            c = np.abs(np.correlate(x, ref, "valid")); k = int(np.argmax(c))
            free = [(0, max(0, k - 64)), (k + L + 64, len(x))]
            for a, b in free:
                for s in range(a, b - M + 1, M // 2):
                    S = np.abs(np.fft.fftshift(np.fft.fft(x[s:s + M] * w))) ** 2
                    P = S if P is None else P + S; nseg += 1
                if b - a > 0:
                    wide.append(np.mean(np.abs(x[a:b]) ** 2))
        if nseg == 0:
            print("%s: no beacon-free segment" % os.path.basename(run)); continue
        P /= nseg
        f = (np.arange(M) - M // 2) * FS / M
        inband = np.median(P[np.abs(f) < 23.0])
        spur = max(P[np.abs(f - s) <= 0.5].max() for s in (32.64, -32.64))
        print("%-8s %-10s spur %+6.1f dB over the in-band floor | wideband floor %6.1f dBFS | segments %d"
              % (os.path.basename(sd.rstrip("/")), os.path.basename(run), 10 * np.log10(spur / inband),
                 10 * np.log10(np.median(wide) / 32767 ** 2), nseg))
