#!/usr/bin/env python3
"""AP-79 filter staging (DEMO_FREQUENCY_PLAN 6.1b): per-run link numbers and
per-stage change against the first stage given.

usage: fstage_report.py <stage_dir> [<stage_dir> ...]   (each holds run dirs from fstage_run.sh)

Per run:
  UL, per BS antenna (cns_dump.bin = ant 0 sub-6, cns_dump_ant1.bin = ant 1 X-IF at R2):
    level  = 20log10 median |H| over the data tones (H = received / known pilot,
             so a cable loss moves it 1:1 in dB; relative only)
    flat   = p95 - p5 of 20log10|H| over the data tones (max-min is inflated by
             estimation error, DEMO_VERIFICATION 9.13(d))
    rms/pk = U-slot raw samples, dBFS against the int16 container (32767)
    MER    = median over U symbols, decision-directed QPSK after per-symbol CPE
             removal, the csi_dump_eval.py method (audited, 9.13(d))
  DL (UE re-sync windows, first 6 per run):
    lvl    = 20log10 median fitted beacon amplitude |g| (least squares on the
             dumped beacon waveform, fractional timing), relative only
    lvl_sd = sd of that over the windows, in dB
    floor  = rms of the beacon-free samples, dBFS
  From the log: acquisition coherence (median over the detections), pilot seat
  (mean, sd), beacon SNR (median over the re-syncs), CNS low.
"""
import glob, json, math, os, re, statistics as st, sys
import numpy as np

FS = 32767.0


def db20(x):
    return 20 * math.log10(x) if x > 0 else float("nan")


def inband(x, starts, M, band):
    """Mean in-band power, dBFS: Welch over Hann segments x[a:a+M] for a in starts,
    summing the DC-centred bins whose centre lies in band = (lo, hi) in cycles/sample.
    White noise of variance s2 reads s2 x (bins in band)/M, a tone its own power."""
    w = np.hanning(M); f = (np.arange(M) - M // 2) / M
    sel = (f >= band[0]) & (f <= band[1])
    P = [np.abs(np.fft.fftshift(np.fft.fft(x[a:a + M] * w))) ** 2 / np.sum(w ** 2) for a in starts]
    if not P:
        return float("nan")
    return 10 * np.log10(np.sum(np.mean(P, axis=0)[sel]) / M / FS ** 2)


def chan_band(N, di):
    """The occupied band in cycles/sample: the data tones' DC-centred bins, half a bin wider each side."""
    return ((di.min() - N // 2 - 0.5) / N, (di.max() - N // 2 + 0.5) / N)


def ul(path):
    b = open(path, "rb").read()
    N, cp, es, nsym, nd = (int(v) for v in np.frombuffer(b[:20], np.int32))
    o = 20
    H = np.frombuffer(b[o:o + 8 * N], np.float32).reshape(-1, 2); H = H[:, 0] + 1j * H[:, 1]; o += 8 * N
    di = np.frombuffer(b[o:o + 4 * nd], np.int32); o += 4 * nd
    s = np.frombuffer(b[o:], np.int16).reshape(-1, 2).astype(np.float64)
    x = s[:, 0] - 1j * s[:, 1]  # conj RX, as the recorder
    hd = 20 * np.log10(np.abs(H[di]))
    mer = []
    for k in range(nsym):
        base = es + k * (cp + N) + cp
        if base < 0 or base + N > len(x):
            break
        Y = np.fft.fft(x[base:base + N]); Y = Y[(np.arange(N) + N // 2) % N]
        X = Y[di] / H[di]
        X = X * np.exp(-1j * np.angle(-np.mean(X ** 4)) / 4)
        X /= np.sqrt(np.mean(np.abs(X) ** 2))
        d = (np.sign(X.real) + 1j * np.sign(X.imag)) / np.sqrt(2)
        mer.append(-10 * np.log10(np.mean(np.abs(X - d) ** 2)))
    a = np.abs(x)
    band = chan_band(N, di)
    end = es + nsym * (cp + N)
    sig = inband(x, list(range(es, end - N + 1, N)), N, band)
    # The TX-idle floor: the burst's zero prefix/postfix inside the slot. Found from
    # the data, not from es (the burst's seat moves by tens of samples run to run):
    # 32-sample segments whose power is within 3 dB of the quietest one. About 250
    # idle samples a slot, so a few tenths of a dB of scatter per run.
    M = 32
    starts = list(range(0, len(x) - M + 1, M // 2))
    pw = np.array([np.mean(np.abs(x[a0:a0 + M]) ** 2) for a0 in starts])
    idle = [a0 for a0, q in zip(starts, pw) if q < 2 * pw.min()]
    flo = inband(x, idle, M, band)
    return {"level": float(np.median(hd)), "sig_in": sig, "floor_in": flo, "snr_rx": sig - flo, "nidle": len(idle), "flat": float(np.percentile(hd, 95) - np.percentile(hd, 5)),
            "rms": db20(float(np.sqrt(np.mean(a ** 2))) / FS), "pk": db20(float(a.max()) / FS),
            "mer": float(np.median(mer)) if mer else float("nan"), "nsym": len(mer), "N": N}


def dl(run):
    rb = os.path.join(run, "beacon_ram.bin")
    wins = sorted(glob.glob(os.path.join(run, "resync", "resyncwin_*.bin")))
    if not os.path.exists(rb) or not wins:
        return None
    r = np.fromfile(rb, np.int16).astype(float).reshape(-1, 2); r = r[:, 0] + 1j * r[:, 1]
    nz = np.flatnonzero(np.abs(r) > 0)
    ref = r[nz[0]:nz[-1] + 1]; L = len(ref)
    n2 = 1 << int(np.ceil(np.log2(L + 64)))
    f = np.fft.fftfreq(n2)
    gs, floors, segs, xs = [], [], [], []
    for w in wins:
        x = np.fromfile(w, np.int16).astype(float).reshape(-1, 2); x = x[:, 0] + 1j * x[:, 1]
        best = None
        for rr in (ref, np.conj(ref)):
            c = np.abs(np.correlate(x, rr, "valid"))
            k = int(np.argmax(c))
            if best is None or c[k] > best[0]:
                best = (c[k], k, rr)
        _, k, rr = best
        R = np.fft.fft(np.concatenate([rr, np.zeros(n2 - L)]))
        seg = x[k:k + L]
        bres = None
        for tau in np.arange(-1.0, 1.0001, 0.1):
            rs = np.fft.ifft(R * np.exp(-2j * np.pi * f * tau))[:L]
            g = np.vdot(rs, seg) / np.vdot(rs, rs)
            res = np.sum(np.abs(seg - g * rs) ** 2)
            if bres is None or res < bres[0]:
                bres = (res, g)
        gs.append(abs(bres[1]))
        out = np.concatenate([x[:max(0, k - 64)], x[k + L + 64:]])
        if len(out):
            floors.append(float(np.sqrt(np.mean(np.abs(out) ** 2))))
        xs.append(np.conj(x))  # conj RX, the UE's sense as the recorder's
        segs.append([a0 for a0 in range(0, max(0, k - 64 - 256) + 1, 128)] +
                    [a0 for a0 in range(k + L + 64, len(x) - 256 + 1, 128)])
    gdb = [db20(g) for g in gs]
    out = {"lvl": float(np.median(gdb)), "lvl_sd": float(np.std(gdb)), "nwin": len(gs),
           "floor": db20(float(np.median(floors)) / FS) if floors else float("nan")}
    if DL_BAND is not None:  # in the uplink's channel band (the same numerology on both)
        out["floor_in"] = float(np.median([inband(x, sg, 256, DL_BAND) for x, sg in zip(xs, segs) if sg]))
    return out


DL_BAND = None


def logfig(run):
    logs = [p for p in glob.glob(os.path.join(run, "*.log")) if not re.search(r"_(csi|cpu|threads)_", p)]
    if not logs:
        return {}
    L = open(logs[0], errors="replace").read()
    coh = [float(v) for v in re.findall(r"syncSearch: detection #\d+ statistic ([\d.]+) vs bar", L)]
    seat = [int(v) for v in re.findall(r"pilot_grid_off=([-\d]+)", L)]
    snr = [float(v) for v in re.findall(r"beacon alive .*?snr ([\d.]+) dB", L)]
    cns = re.findall(r"\((\d+) datagrams, (\d+) low\)", L)
    # median, not min: a detection whose beacon straddles the read boundary
    # reads low (0.35 at idx 701 on F0 run 1) while the link is the same
    return {"coh": st.median(coh) if coh else float("nan"),
            "seat": st.mean(seat) if seat else float("nan"), "seat_sd": st.pstdev(seat) if seat else float("nan"),
            "bsnr": st.median(snr) if snr else float("nan"),
            "cns_low": "%s/%s" % (cns[-1][1], cns[-1][0]) if cns else "none",
            "bad": len(re.findall(r"BAD SYNC|UE PILOT LOST", L))}


def run_row(run):
    global DL_BAND
    row = {"run": os.path.basename(run)}
    p0 = os.path.join(run, "cns_dump.bin")
    if os.path.exists(p0):
        b = open(p0, "rb").read(); N, _, _, _, nd = (int(v) for v in np.frombuffer(b[:20], np.int32))
        DL_BAND = chan_band(N, np.frombuffer(b[20 + 8 * N:20 + 8 * N + 4 * nd], np.int32))
    for k, name in ((0, "cns_dump.bin"), (1, "cns_dump_ant1.bin")):
        p = os.path.join(run, name)
        if os.path.exists(p):
            for m, v in ul(p).items():
                row["ul%d_%s" % (k, m)] = v
    d = dl(run)
    if d:
        for m, v in d.items():
            row["dl_" + m] = v
    row.update(logfig(run))
    return row


KEYS = ["ul0_level", "ul0_sig_in", "ul0_floor_in", "ul0_snr_rx", "ul0_mer", "ul0_flat", "ul0_rms", "ul0_pk",
        "ul1_level", "ul1_sig_in", "ul1_floor_in", "ul1_snr_rx", "ul1_mer", "ul1_flat", "ul1_rms", "ul1_pk",
        "dl_lvl", "dl_lvl_sd", "dl_floor", "dl_floor_in", "coh", "seat", "seat_sd", "bsnr"]


def fmt(v):
    return "%7.3f" % v if isinstance(v, float) and not math.isnan(v) else "%7s" % ("-" if not isinstance(v, (int, float)) else v)


stages = []
for sd in sys.argv[1:]:
    runs = sorted(d for d in glob.glob(os.path.join(sd, "*")) if os.path.isdir(d))
    rows = [run_row(r) for r in runs]
    stages.append((os.path.basename(sd.rstrip("/")), rows))
    print("== %s (%d runs)" % (sd, len(rows)))
    for r in rows:
        print("  %-16s " % r["run"] + " ".join("%s=%s" % (k, fmt(r.get(k, float("nan"))).strip()) for k in KEYS if k in r)
              + " cns_low=%s bad=%s" % (r.get("cns_low"), r.get("bad")))
print()
print("%-10s" % "metric" + "".join("%26s" % s for s, _ in stages))
ref = None
for k in KEYS:
    line = "%-10s" % k
    base = None
    for i, (s, rows) in enumerate(stages):
        v = [r[k] for r in rows if k in r and not math.isnan(r[k])]
        if not v:
            line += "%26s" % "-"
            continue
        m = st.mean(v)
        cell = "%7.2f [%5.3f]" % (m, max(v) - min(v))
        if i == 0:
            base = m
        elif base is not None:
            cell += " d%+6.2f" % (m - base)
        line += "%26s" % cell
    print(line)
print("\n[x] = run-to-run range (max - min); dN = change of the stage mean against the first stage.")
json.dump({s: rows for s, rows in stages}, open("/tmp/fstage_report_last.json", "w"), indent=1)
