# Known answers for sounder_log.py and the two report scripts built on it
# (demo_report.py, late_release_report.py). Stdlib only; run from
# tests/demo-verify (ctest does). Each check names the mutation that breaks it.
import os, shutil, subprocess, sys, tempfile
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import sounder_log as sl  # noqa: E402
fails = 0
def check(ok, what):
    global fails; print(("PASS " if ok else "FAIL ") + what); fails += (not ok)

# The sounder's own line formats (RadioHoudini.cc, recorder_worker.cc), and a
# SoapySDR [WARNING] line as its log handler colours it.
L = ["57:000002 WARNG: UE 192.168.10.21 link health: [UE 192.168.10.21] 5.0 s: irq 12/s, preflight ok: tx0.late +3"
     " tx0.under +2 | app: rx_err +0, rx_short +0, rx_pad +0, tx_short +0, tx_sat +0",
     "57:000003 WARNG: BS 192.168.10.22 link health: [BS 192.168.10.22] 5.0 s: irq 12/s, preflight ok: tx0.late +5"
     " | app: rx_err +0, rx_short +0, rx_pad +0, tx_short +0, tx_sat +0",
     "57:000004 WARNG: UE 192.168.10.21 link health: [UE 192.168.10.21] 5.0 s: irq 12/s, preflight ok: tx0.late +4"
     " | app: rx_err +0, rx_short +0, rx_pad +0, tx_short +0, tx_sat +0",
     "57:000005 WARNG: TX status: 1 problem event(s), latest TIME_ERROR (code -6) on tx_stream[0] at 5000000000 ns."
     " TX_BANK_STATUS=ch0:acked=1,late=5,under=2,seqerr=0,zerofill=0,played=9;ch1:acked=1,late=4,under=1,played=9",
     "57:000006 INFOR: CNS score 0.950 rot +1.2 deg at frame 900 (400 datagrams, 3 low); P->U 20.8 us, so +7.5 deg per kHz",
     "57:000007 INFOR: CNS score 0.410 (low, no rotation) at frame 1400 (900 datagrams, 7 low)",
     "57:000008 WARNG: CNS score 0.300 at frame 1401, r=0.2 (low occurrence 8 of 901 datagrams)",
     "\x1b[1;33m[WARNING] HoudiniStream TX late-release ch1: held 1200 us, ledger 0.0, rail room\x1b[0m"]

d = tempfile.mkdtemp(prefix="sounder_log_")
try:
    log = os.path.join(d, "raw.log")
    open(log, "w").write("\n".join(L) + "\n")
    R = sl.read_lines(log)
    check(R[-1] == "[WARNING] HoudiniStream TX late-release ch1: held 1200 us, ledger 0.0, rail room",
          "colour codes are removed (mutation: read the lines raw)")
    t = sl.tx_totals(R)
    check(t[("UE", "tx0.late")] == 7 and t[("UE", "tx0.under")] == 2 and t[("BS", "tx0.late")] == 5,
          "TX increments are summed per role (mutation: key by counter only, the BS's +5 joins the UE's): %s" % dict(t))
    check(sl.cns_summaries(R) == [(4, 400, 3), (5, 900, 7)],
          "every summary is read with its line, the throttled warning is not (mutation: a pattern that also "
          "reads 'low occurrence K of D'): %s" % sl.cns_summaries(R))
    check(sl.cns_total(R) == (900, 7) and sl.cns_total(R[:4]) is None,
          "the run's CNS total is the last summary's, None without one (mutation: the first summary)")

    # The two report scripts, end to end on a run directory as fstage_run.sh files one.
    run = os.path.join(d, "D1_010203")
    os.makedirs(run)
    shutil.copy(log, os.path.join(run, "D1_010203.log"))
    out = subprocess.run([sys.executable, os.path.join(HERE, "demo_report.py"), run],
                         capture_output=True, text=True).stdout
    check("TX totals: BS tx0.late +5, UE tx0.late +7, UE tx0.under +2" in out,
          "demo_report prints each role's TX totals (mutation: drop the role from the key)")
    check("CNS to ~   5 s: datagrams   900 low    7 (0.8 %)" in out,
          "demo_report's CNS window ends at the last summary, the low-score form included (mutation: a summary "
          "pattern that needs the 'rot' form, so the window ends at 400 and 3)")
    out = subprocess.run([sys.executable, os.path.join(HERE, "late_release_report.py"), log],
                         capture_output=True, text=True).stdout
    check("late-release lines: stream 1: 1" in out and "UE totals: tx0.late +7, tx0.under +2" in out
          and "CNS low 7/900" in out,
          "late_release_report prints the stream, the UE's totals only and the CNS low (mutation: count the BS's)")
finally:
    shutil.rmtree(d, ignore_errors=True)
print("%d failure(s)" % fails); sys.exit(1 if fails else 0)
