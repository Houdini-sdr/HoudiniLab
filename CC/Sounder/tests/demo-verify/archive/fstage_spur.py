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
import rig_dumps as rd
FS = 122.88  # the UE's receive rate, MHz (sample_rate)
SPUR = 32.64  # the spur's offset from the 2425 MHz NCO, MHz (9.8)
INBAND = 23.0  # R2's data tones span +-23.04 MHz (houdini-dualband-r2.json's numerology note)
M = 1024
for sd in sys.argv[1:]:
    for run in sorted(d for d in glob.glob(os.path.join(sd, "*")) if os.path.isdir(d)):
        bw = rd.beacon_and_windows(run)
        if bw is None:
            continue
        ref, wins = bw
        P, nseg, wide = None, 0, []
        w = np.hanning(M)
        for wf in wins:
            x = rd.read_iq(wf)
            # the beacon RAM's own sense only (fstage_report also tries its conjugate)
            k, _ = rd.locate(x, (ref,))
            free = rd.beacon_free(len(x), k, len(ref))
            for s in rd.segments(free, M, M // 2):
                S = np.abs(np.fft.fftshift(np.fft.fft(x[s:s + M] * w))) ** 2
                P = S if P is None else P + S; nseg += 1
            for a, b in free:
                if b - a > 0:
                    wide.append(np.mean(np.abs(x[a:b]) ** 2))
        if nseg == 0:
            print("%s: no beacon-free segment" % os.path.basename(run)); continue
        P /= nseg
        f = (np.arange(M) - M // 2) * FS / M
        inband = np.median(P[np.abs(f) < INBAND])
        spur = max(P[np.abs(f - s) <= 0.5].max() for s in (SPUR, -SPUR))
        print("%-8s %-10s spur %+6.1f dB over the in-band floor | wideband floor %6.1f dBFS | segments %d"
              % (os.path.basename(sd.rstrip("/")), os.path.basename(run), 10 * np.log10(spur / inband),
                 10 * np.log10(np.median(wide) / 32767 ** 2), nseg))
