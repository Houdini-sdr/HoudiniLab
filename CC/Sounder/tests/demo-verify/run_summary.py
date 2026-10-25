#!/usr/bin/env python3
"""Summarise one demo-head run and judge it against DEMO_BENCH_RUNBOOK.md A6.

usage: run_summary.py <run dir or sounder log>

A run directory (fstage_run.sh's, or demo_run.sh's) is read through its
largest *.log, the sounder's; a dashboard session log (csi_server.py
--log-dir) is given as the file. The summary prints the stack, the X-band
front end, slots mode, errors, the end-of-run checks, warning counts, UE
acquisition, the BS framer (with --bs_rx_debug) and the carrier. The
verdict, last, is FAIL on an error, on a nonzero end-of-run count, on a
missing end-of-run line (they are printed as the radios close, so a run that
did not close cleanly has none), when no data flowed (the UE never locked
on the beacon, or the BS produced no constellation), or when an antenna
carried no usable data: a tenth or more of the constellation records low, or,
in a run directory with the dashboard's MER samples (mer.txt, demo_run.sh's),
an antenna with no sample or with its median under its band's demo floor;
link-health alarms, lost pilots, untrusted windows and late releases are
warnings to read. Exits 1 on FAIL.
"""
import glob, os, re, statistics as st, sys
from collections import Counter
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from sounder_log import ANSI, alarm_kinds, cns_total  # noqa: E402,F401  alarm_kinds: rung_report and the test read it here too

# "Radio open failed (attempt k of n): <cause>" is the in-process retry of a
# slow or refused open (8.127, 8.131; logs before it read "Radios Not Found"): a
# warning when the run then closes cleanly; one that never recovers has no
# end-of-run lines and fails on those.
ERRORS = r"what\(\)|terminate called|bs_rx_slots: TDD_RX_SLOTS|mode V bring-up:"


HOST_COUNTERS = ("tdd_straddle", "rxq_ovfl", "ring_ovfl")

# An antenna that carries nothing scores every record low, so with two
# antennas half the run's records are (DEMO_VERIFICATION 9.84, W5A1: 2,560 of
# 5,120); healthy runs read 0 to 0.07 %, and older stacks up to 1.76 % (AP-58),
# which no fixed floor this low tells apart. A tenth is a gross failure only.
CNS_LOW_FAIL = 0.10
# Each band's demo floor, in dB of MER (no 15 s sample of a demo-length run
# under it, 9.84 to 9.86); the band is the BS receive channel's NCO, the
# sub-6 carrier (2425 MHz) or the X-band chain's IF (4380 MHz).
MER_FLOOR = {"sub-6": 33.0, "X-IF": 28.0}
XIF_MIN_MHZ = 3000.0


def fpga_version(L):
    """The lowest fpga_version the log's stack lines name, as (major, minor), or None."""
    v = [tuple(int(x) for x in m.group(1).split(".")) for l in L for m in [re.search(r"fpga_version=(\d+\.\d+)", l)] if m]
    return min(v) if v else None


def bs_bands(L):
    """[(rx channel, band)] of the BS's receive channels in their antenna order,
    from the mode V bring-up's 'BS <id> mode V: RX ch<k>: NCO <f> MHz' lines (a
    retried open prints them again)."""
    out = []
    for l in L:
        m = re.search(r"\bBS \S+ mode V: RX ch(\d+): NCO ([0-9.]+) MHz", l)
        if m and int(m.group(1)) not in [ch for ch, _ in out]:
            out.append((int(m.group(1)), "X-IF" if float(m.group(2)) >= XIF_MIN_MHZ else "sub-6"))
    return out


def mer_samples(M):
    """{antenna: ([fresh MER dB], stale count)} from mer_sampler.py's lines
    ('/ant/<i>/cns MER <x> dB', or 'cns STALE MER' when the antenna's
    constellation did not advance since the last sample or is past the stale
    age: its value is an old frame's)."""
    out = {}
    for l in M:
        for a, stale, v in re.findall(r"/ant/(\d+)/cns (STALE )?MER (-?[0-9.]+) dB", l):
            fresh, n_stale = out.get(int(a), ([], 0))
            if stale:
                n_stale += 1
            else:
                fresh.append(float(v))
            out[int(a)] = (fresh, n_stale)
    return out


def read_log(path):
    if os.path.isdir(path):
        own = os.path.join(path, os.path.basename(os.path.normpath(path)) + ".log")  # <TAG>_<T>/<TAG>_<T>.log
        logs = sorted(glob.glob(os.path.join(path, "*.log")), key=os.path.getsize)
        if not os.path.isfile(own) and not logs:
            sys.exit("no log in " + path)
        path = own if os.path.isfile(own) else logs[-1]
    with open(path, errors="ignore") as f:
        # the config echo holds every key's text, so it would match anything
        return [ANSI.sub("", l.rstrip("\n")) for l in f if "Config: {" not in l]


def verdict(L, mer=None):
    """[(level, text)]: FAIL, WARN or PASS items against A6. `mer` is the run's
    mer.txt lines, or None when there is none to judge."""
    out = []
    for l in L:
        if re.search(ERRORS, l):
            out.append(("FAIL", "error: " + l.strip()[:200]))
        elif "no retry left" in l and "Radio open failed" in l:
            m = re.search(r"\): (?:the radios did not come up: )?(.*?)(?:; no retry left.*)?$", l)
            out.append(("FAIL", "the radio open failed on every attempt: " + (m.group(1) if m else l).strip()[:200]))
    # Data flowed (AP-112): every other check below reads clean on an idle
    # link, so a run that carried nothing passed them (DEMO_VERIFICATION 9.83,
    # the nodes cabled in loopback). The UE locks on the beacon first, and the
    # BS's CSI view then scores constellations from its uplink. Only a confirmed
    # lock counts: a detection line is written before the SNR floor and the
    # confirm judge it, and a rejected one is a clientSyncBeacon line too. Every
    # Houdini run acquires through houdiniAcquireAnchor (Config::sync_model).
    if not any("lock CONFIRMED" in l for l in L):
        out.append(("FAIL", "UE: never locked on the beacon (no 'lock CONFIRMED' line): no data flowed"))
    cns = cns_total(L)
    if not cns or cns[0] == 0:
        out.append(("FAIL", "BS: no constellation record (no CNS summary with datagrams): no UE uplink reached the "
                            "CSI view"))
    elif cns[1] >= CNS_LOW_FAIL * cns[0]:
        out.append(("FAIL", "BS: %d of %d constellation records low (%.1f %%): an antenna carries no usable data"
                            % (cns[1], cns[0], 100.0 * cns[1] / cns[0])))
    # Per antenna (AP-117): the dashboard's MER samples, each antenna against
    # its band's floor. The median rides out a cold start's first minutes.
    if mer is not None:
        bands, got = bs_bands(L), mer_samples(mer)
        if not bands:
            out.append(("WARN", "MER samples beside the log, but no 'BS ... mode V: RX ch' line names the antennas' "
                                "bands, so no per-antenna MER floor was judged"))
        for ant, (ch, band) in enumerate(bands):
            v, n_stale = got.get(ant, ([], 0))
            name = "BS antenna %d (%s, RX ch%d)" % (ant, band, ch)
            if not v and not n_stale:
                out.append(("FAIL", name + ": no MER sample in mer.txt: it carried nothing the dashboard could score"))
                continue
            # A stale sample repeats an old frame's MER: an antenna that stopped
            # mid-run reads its last good value until the end, so the stale
            # ones are counted, not averaged in.
            if len(v) < n_stale:
                out.append(("FAIL", name + ": %d of %d MER samples stale: its constellation stopped advancing for "
                                    "most of the run" % (n_stale, n_stale + len(v))))
                continue
            med = st.median(v)
            what = name + ": MER median %.1f dB over %d samples (%.1f..%.1f)%s" % (
                med, len(v), min(v), max(v), ", %d more stale" % n_stale if n_stale else "")
            out.append(("FAIL", what + ", under its demo floor %.0f dB" % MER_FLOOR[band]) if med < MER_FLOOR[band]
                       else ("PASS", what))
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
                      ("UE burst head at the slot cut", r"starts at the pilot slot's edge"),
                      ("UE burst tail at the slot cut", r"runs to the pilot slot's far edge"),
                      ("a radio open failed and was retried", r"Radio open failed \(attempt.*; retrying|Radios Not Found")]:
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
                      ("tail at slot edge", r"runs to the pilot slot's far edge"),
                      ("stray outside rx slots", r"outside the rx slots \(occurrence"),
                      ("window untrusted (order/jump)", r"marked untrusted"),
                      ("CSI view dropped", r"CSI view: dropped"), ("rotation clamped", r"carrier rotation clamped"),
                      ]:
        print(f"  {name}: {sum(bool(re.search(pat, l)) for l in L)}")
    print("== UE acquisition")
    acc = sum("clientSyncBeacon [0]: idx" in l for l in L)
    frames, lines = pilot_frames(L)
    print(f"  accepted {acc}, locks {sum('lock CONFIRMED' in l for l in L)}, hunts {sum('hunt lock' in l for l in L)}, "
          f"pilot frames scheduled {frames} in {lines} schedule lines")
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
    print(f"  steering pushes: {steering_pushes(L)}")


def pilot_frames(L):
    """(frames, lines): each 'UE pilot burst: scheduled N frames' line schedules N frames' bursts."""
    n = [int(m.group(1)) for l in L for m in [re.search(r"UE pilot burst: scheduled (\d+) frames", l)] if m]
    return sum(n), len(n)


def steering_pushes(L):
    """The CLOCK_ADJ pushes the steering made: its '-> push +1' lines only, not the config, ON or release lines."""
    return sum(bool(re.search(r"Clock steering \[\d+\]: .*-> push [-+]\d", l)) for l in L)


def read_mer(path):
    """A run directory's mer.txt lines, or None (a log file, or a run without the sampler)."""
    f = os.path.join(path, "mer.txt")
    if not (os.path.isdir(path) and os.path.isfile(f)):
        return None
    with open(f, errors="ignore") as fh:
        return fh.read().splitlines()


def main(path):
    L = read_log(path)
    summary(L)
    v = verdict(L, read_mer(path))
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
