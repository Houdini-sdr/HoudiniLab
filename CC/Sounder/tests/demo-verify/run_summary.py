#!/usr/bin/env python3
"""Summarise one demo-head run and judge it against DEMO_BENCH_RUNBOOK.md A6.

usage: run_summary.py <run dir or sounder log>

A run directory (fstage_run.sh's, or demo_run.sh's) is read through its
largest *.log, the sounder's; a dashboard session log (csi_server.py
--log-dir) is given as the file. The summary prints the stack, the X-band
front end, slots mode, errors, the end-of-run checks, warning counts, UE
acquisition, the BS framer (with HOUDINI_BS_RX_DEBUG) and the carrier. The
verdict, last, is FAIL on an error, on a nonzero end-of-run count, or on a
missing end-of-run line (they are printed as the radios close, so a run that
did not close cleanly has none); link-health alarms, lost pilots, untrusted
windows and late releases are warnings to read. Exits 1 on FAIL.
"""
import glob, os, re, statistics as st, sys
from collections import Counter

# "Radios Not Found. Will attempt a retry" is the in-process retry of a slow or
# refused open (8.127, 8.131): a warning when the run then closes cleanly; one
# that never recovers has no end-of-run lines and fails on those.
ERRORS = r"what\(\)|terminate called|bs_rx_slots: TDD_RX_SLOTS|mode V bring-up:"


def alarm_kinds(lines):
    """Counter of the alarm kinds in link-health lines, one entry per item, in
    the forms link_health.h writes them: a counter rise '<name> +N', a blind
    egress counter '<name>=N (sticky' or '(saturated', a new preflight FAIL, a
    config drift 'config <section>: old -> new', and the app counters."""
    pat = re.compile(r"((?:tx|rx)\d\.\w+ \+\d+|(?:egress|host)\.\w+ \+\d+|egress\.\w+=\d+ \((?:sticky|saturated)"
                     r"|preflight new FAIL [^|;]+|config [^:|;]+:|(?:rx|tx)_\w+ \+[1-9]\d*)")
    out = Counter()
    for l in lines:
        for m in pat.finditer(l):
            k = re.sub(r"\+\d+", "+N", m.group(1))
            k = re.sub(r"=\d+ \((sticky|saturated)$", r" \1", k)
            out[k.rstrip(":") + (" drift" if k.startswith("config ") else "")] += 1
    return out
HOST_COUNTERS = ("tdd_straddle", "tdd_refused", "rxq_ovfl", "ring_ovfl")


def fpga_version(L):
    """The lowest fpga_version the log's stack lines name, as (major, minor), or None."""
    v = [tuple(int(x) for x in m.group(1).split(".")) for l in L for m in [re.search(r"fpga_version=(\d+\.\d+)", l)] if m]
    return min(v) if v else None


def read_log(path):
    if os.path.isdir(path):
        own = os.path.join(path, os.path.basename(os.path.normpath(path)) + ".log")  # <TAG>_<T>/<TAG>_<T>.log
        logs = sorted(glob.glob(os.path.join(path, "*.log")), key=os.path.getsize)
        if not os.path.isfile(own) and not logs:
            sys.exit("no log in " + path)
        path = own if os.path.isfile(own) else logs[-1]
    with open(path, errors="ignore") as f:
        # the config echo holds every key's text, so it would match anything
        return [re.sub(r"\x1b\[[0-9;]*m", "", l.rstrip("\n")) for l in f if "Config: {" not in l]


def verdict(L):
    """[(level, text)]: FAIL, WARN or PASS items against A6."""
    out = []
    for l in L:
        if re.search(ERRORS, l):
            out.append(("FAIL", "error: " + l.strip()[:200]))
    slots = any("receives only its rx slots" in l for l in L)
    # Up to fpga 1.33 the host plugin drops the non-rx symbols itself, so
    # tdd_drop counts them by design (9.69: 51.7 M a channel); from 1.34 the
    # fabric gates them and tdd_drop must stay 0 (9.70). No version read: judged.
    fv = fpga_version(L)
    host = HOST_COUNTERS + (() if fv is not None and fv < (1, 34) else ("tdd_drop",))
    for role in ("BS", "UE"):
        rc = [l for l in L if re.search(r"\b%s [^:]*: RX read check:" % role, l)]
        if not rc:
            out.append(("FAIL", "%s: no RX read check line (the run did not close cleanly, or the log is cut)" % role))
        for l in rc:
            m = re.search(r"\((\d+) samples lost in rx slots.*?\), (\d+) out of order, (\d+) time jumps", l)
            if not m:
                out.append(("FAIL", "%s: RX read check unreadable: %s" % (role, l.strip()[:160])))
            elif any(int(x) for x in m.groups()):
                out.append(("FAIL", "%s: RX read check: %s lost in rx slots, %s out of order, %s time jumps"
                            % ((role,) + m.groups())))
            else:
                out.append(("PASS", "%s: RX read check clean" % role))
    if slots:
        sc = [l for l in L if "AP-87 slot check:" in l]
        hs = [l for l in L if re.search(r"RX_HOST_STATUS \w+=", l)]
        for l in L:
            if "RX_HOST_STATUS unreadable" in l:
                out.append(("FAIL", "BS: " + l.split("BS", 1)[-1].strip()[:160]))
        if not sc:
            out.append(("FAIL", "BS: no AP-87 slot check line in a slots-mode run"))
        for l in sc:
            m = re.search(r"(\d+) of them outside the rx slots", l)
            if not m or int(m.group(1)):
                out.append(("FAIL", "BS: AP-87 slot check: %s" % l.split("AP-87 slot check:")[1].strip()))
            else:
                out.append(("PASS", "BS: 0 samples outside the rx slots"))
        if not hs:
            out.append(("FAIL", "BS: no RX_HOST_STATUS line in a slots-mode run"))
        for l in hs:
            kv = dict(re.findall(r"(\w+)=(\d+)", l))
            gone = [k for k in host if k not in kv]
            if gone:
                out.append(("FAIL", "BS: RX_HOST_STATUS lacks %s (a host plugin other than the validated one?)"
                            % ", ".join(gone)))
                continue
            bad = ["%s=%s" % (k, kv[k]) for k in host if int(kv[k])]
            out.append(("FAIL", "BS: RX_HOST_STATUS " + ", ".join(bad)) if bad
                       else ("PASS", "BS: RX_HOST_STATUS counters 0 (%s)" % ", ".join(k for k in host if k in kv)))
    alarms = [l for l in L if "WARNG" in l and "link health: [" in l]
    if alarms:
        out.append(("WARN", "%d link-health alarm lines: %s" % (len(alarms), dict(alarm_kinds(alarms)))))
    for name, pat in [("UE PILOT LOST", r"UE PILOT LOST"), ("pilot failed the LTS check", r"failed the LTS check"),
                      ("window marked untrusted", r"marked untrusted"),
                      ("a radio open retried (Radios Not Found)", r"Radios Not Found")]:
        n = sum(bool(re.search(pat, l)) for l in L)
        if n:
            out.append(("WARN", "%s: %d lines" % (name, n)))
    # The host pacer's counters are cumulative, printed every health period:
    # the peak is the count, not the number of lines. late_refusals (the host
    # refusing a TX write stamped behind its time, SH-427) and the plugin's
    # 'Write rejected: HAS_TIME stamp ... behind' line are the software lane's
    # to trace: send them the log window.
    for key, what in (("release_late", "late releases (host pacer)"),
                      ("late_refusals", "late TX writes refused by the host (send the window to the software lane)")):
        peak = {}
        for l in L:
            for ch, v in re.findall(r"\b%s_ch(\d)=(\d+)" % key, l):
                peak[ch] = max(peak.get(ch, 0), int(v))
        if any(peak.values()):
            out.append(("WARN", what + ": " + ", ".join("ch%s %d" % kv for kv in sorted(peak.items()))))
    n = sum("Write rejected: HAS_TIME stamp" in l for l in L)
    if n:
        out.append(("WARN", "%d 'Write rejected: HAS_TIME stamp' lines (send the window to the software lane)" % n))
    return out


def summary(L):
    def grep(pat, n=6, width=220):
        r = [l for l in L if re.search(pat, l)]
        for l in r[:n]:
            print("  " + l[:width])
        if len(r) > n:
            print(f"  ... {len(r)} lines")
        return r

    print("== stack")
    grep(r"VERSION SKEW|device_build|host_build|fpga_commit", 4)
    print("== AP-86 front end")
    if not grep(r"TDD_EXTPIN|front end's static", 8):
        print("  (no TDD_EXTPIN lines: xband_frontend_static off, or the bring-up never reached it)")
    print("== AP-87 slots mode")
    grep(r"receives only its rx slots|TDD_RX_MODE|TDD schedule is not running|packets of [0-9]+ samples|Houdini BS TDD armed", 6)
    print("== errors")
    if not grep(ERRORS + r"|TIMEOUT", 6):
        print("  none")
    print("== end-of-run checks (read check, slot check, host cut)")
    grep(r"RX read check|AP-87 slot check|RX_HOST_STATUS|slot cut dropped", 8, 300)
    print("== warning counts")
    for name, pat in [("no UE burst", r"no UE burst in frame read"), ("UE PILOT LOST", r"UE PILOT LOST"),
                      ("pilot failed the LTS check", r"failed the LTS check"),
                      ("head at slot edge", r"starts at the pilot slot's edge"),
                      ("stray outside rx slots", r"outside the rx slots \(occurrence"),
                      ("window untrusted (order/jump)", r"marked untrusted"),
                      ("CSI view dropped", r"CSI view: dropped"), ("rotation clamped", r"carrier rotation clamped"),
                      ]:
        print(f"  {name}: {sum(bool(re.search(pat, l)) for l in L)}")
    print("== UE acquisition")
    acc = sum("clientSyncBeacon [0]: idx" in l for l in L)
    print(f"  accepted {acc}, locks {sum('lock CONFIRMED' in l for l in L)}, hunts {sum('hunt lock' in l for l in L)}, "
          f"pilot bursts {sum('UE pilot burst' in l for l in L)}")
    snr = [float(m.group(1)) for l in L for m in [re.search(r"Re-sync frame [0-9]+: detection.*snr ([-0-9.]+)", l)] if m]
    if snr:
        print(f"  beacon snr n={len(snr)} median {st.median(snr):.1f} dB, below 10 dB: {sum(s < 10 for s in snr)}")
    rows = [dict(re.findall(r"([a-z_\-]+)=([-0-9.]+)", l)) for l in L if "HOUDINI_BS_RX:" in l]
    if rows:
        print("== BS framer (HOUDINI_BS_RX, every 20th frame)")
        ss = [float(r["selfsim"]) for r in rows if "selfsim" in r]
        rms = [float(r["pilot-rms"]) for r in rows if "pilot-rms" in r]
        go = Counter(int(r["pilot_grid_off"]) for r in rows if "pilot_grid_off" in r)
        lanes = Counter(r.get("ref_lane", "?") for r in rows)
        if ss and rms:
            print(f"  n={len(rows)} selfsim median {st.median(ss):.2f} (min {min(ss):.2f}), pilot-rms median {st.median(rms):.0f}")
        print(f"  pilot_grid_off {dict(go.most_common(6))}   ref_lane {dict(lanes)}")
        fr = [(int(r["frame"]), int(r["stamp_ticks"])) for r in rows if "frame" in r and "stamp_ticks" in r]
        if len(fr) > 2 and fr[-1][1] > fr[0][1]:
            secs = (fr[-1][1] - fr[0][1]) / 122.88e6
            print(f"  BS frames/s: {(fr[-1][0] - fr[0][0]) / secs:.1f} over {secs:.0f} s")
    print("== carrier (pre-FFT Hz when bs_cfo_pre_fft)")
    pre = [int(m.group(1)) for l in L for m in [re.search(r"after the pilot's pre-FFT (-?[0-9]+) Hz", l)] if m]
    res = [float(m.group(1)) for l in L for m in [re.search(r"= ([-+0-9.]+) Hz residual", l)] if m]
    if pre:
        print(f"  pre-FFT Hz n={len(pre)} min {min(pre)} median {st.median(pre)} max {max(pre)}")
    if res:
        print(f"  post residual Hz n={len(res)} median {st.median(res):.1f} max|.| {max(abs(x) for x in res):.1f}")
    print(f"  steering pushes: {sum(bool(re.search(r'steer.*(push|CLOCK_ADJ)', l)) for l in L)}")


def main(path):
    L = read_log(path)
    summary(L)
    v = verdict(L)
    print("== verdict (DEMO_BENCH_RUNBOOK.md A6)")
    for level, text in v:
        print("  %-4s %s" % (level, text))
    nf, nw = sum(l == "FAIL" for l, _ in v), sum(l == "WARN" for l, _ in v)
    print("VERDICT: " + ("FAIL (%d)" % nf if nf else "PASS" + (" with %d warning(s) to read" % nw if nw else "")))
    return 1 if nf else 0


if __name__ == "__main__":
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    sys.exit(main(sys.argv[1]))
