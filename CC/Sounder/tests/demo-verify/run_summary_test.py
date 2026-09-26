# Known answers for run_summary.py's A6 verdict, on the sounder's real line
# formats (a slots-mode run's end-of-run lines). Stdlib only; run from
# tests/demo-verify (ctest does). Each check names the mutation that breaks it.
import contextlib, importlib.util, io, os, sys, tempfile
spec = importlib.util.spec_from_file_location("rs", os.path.join(os.path.dirname(os.path.abspath(__file__)), "run_summary.py"))
rs = importlib.util.module_from_spec(spec); spec.loader.exec_module(rs)
fails = 0
def check(ok, what):
    global fails; print(("PASS " if ok else "FAIL ") + what); fails += (not ok)

GOOD = [
    "57:000100 INFOR: BS: receives only its rx slots (AP-87): TDD_RX_SLOTS active=1 epoch=91566720 stop=0 symbol",
    "57:004619 INFOR: BS 192.168.10.22: RX read check: 507470 stamped reads, 402636 on the count, 104834 after a gap "
    "(0 samples lost in rx slots, 128820113280 the schedule's gaps), 0 out of order, 0 time jumps",
    "57:004621 INFOR: BS 192.168.10.22: AP-87 slot check: 507470 reads, 25763909760 samples, 0 of them outside the rx "
    "slots (0 reads)",
    "57:004972 INFOR: BS 192.168.10.22: RX_HOST_STATUS rxq_ovfl=0 rxq_ovfl_ch0=0 ring_ovfl=0 tdd_drop=0 tdd_drop_ch0=0 "
    "tdd_straddle=0 tdd_refused=0",
    "57:368354 INFOR: UE 192.168.10.21: RX read check: 2411956 stamped reads, 2411956 on the count, 0 after a gap "
    "(0 samples lost in rx slots, 0 the schedule's gaps), 0 out of order, 0 time jumps",
    "57:000200 INFOR: BS 192.168.10.22 link health: [BS 192.168.10.22] 60.0 s: irq 12/s, preflight ok: clean"
    " | app: rx_err +0, rx_short +0, rx_pad +0, tx_short +0, tx_sat +0",
]
def levels(L):
    return [lv for lv, _ in rs.verdict(L)]
def swap(L, old, new):
    return [l.replace(old, new) for l in L]

check("FAIL" not in levels(GOOD) and "WARN" not in levels(GOOD), "a clean slots-mode run passes with no warnings: %s" % rs.verdict(GOOD))
check("FAIL" in levels(swap(GOOD, "(0 samples lost in rx slots, 128820113280", "(1920 samples lost in rx slots, 128820113280")),
      "samples lost in the rx slots fail (mutation: the lost count not read)")
check("FAIL" in levels([l.replace("0 time jumps", "2 time jumps") if "UE 192" in l else l for l in GOOD]),
      "a time jump fails (mutation: the jump count not read)")
check("FAIL" in levels(swap(GOOD, "0 of them outside", "3 of them outside")), "samples outside the rx slots fail (mutation: the stray count not read)")
check("FAIL" in levels(swap(GOOD, "tdd_drop=0 tdd_drop_ch0", "tdd_drop=5 tdd_drop_ch0")), "a nonzero tdd_drop fails (mutation: tdd_drop not judged)")
check("FAIL" in levels([l for l in GOOD if "UE 192" not in l]),
      "a missing UE read check fails: the run did not close cleanly (mutation: judge only the lines present)")
check("FAIL" in levels([l for l in GOOD if "RX_HOST_STATUS" not in l]),
      "a slots-mode run without RX_HOST_STATUS fails (mutation: host status not required in slots mode)")
allrx = [l for l in GOOD if "rx slots (AP-87)" not in l and "AP-87 slot check" not in l and "RX_HOST_STATUS" not in l]
check("FAIL" not in levels(allrx), "an all-rx run needs no slot check or host status (mutation: slots lines required always)")
check("FAIL" in levels(GOOD + ["57:1 ERROR: terminate called after throwing an instance of 'std::runtime_error'"]),
      "an escaped exception fails (mutation: the error pattern not read)")
app = GOOD + ["57:000300 WARNG: BS 192.168.10.22 link health: [BS 192.168.10.22] 5.0 s: irq 12/s, preflight ok: clean"
              " | app: rx_err +0, rx_short +0, rx_pad +7, tx_short +0, tx_sat +0"]
v = rs.verdict(app)
check("FAIL" not in levels(app) and any(lv == "WARN" and "rx_pad +N" in t for lv, t in v),
      "an app-only health alarm is a warning naming its kind, not a failure (mutation: drop lines reading 'clean |')")
g133 = [l.replace("tdd_drop=0 tdd_drop_ch0=0", "tdd_drop=103400354 tdd_drop_ch0=51700177") for l in GOOD]
check("FAIL" not in levels(g133 + ["57:1 INFOR: stack fpga_version=1.33 fpga_commit=38f662c4"])
      and "FAIL" in levels(g133 + ["57:1 INFOR: stack fpga_version=1.34 fpga_commit=4a19b2f2"]),
      "tdd_drop is judged from fpga 1.34, where the fabric gates; on 1.33 the host drops by design (mutation: "
      "tdd_drop judged on every version, or never)")
lr = GOOD + ["57:%d INFOR: UE TX_HOST_STATUS release_late_ch0=0 release_late_ch1=%d" % (i, min(i, 2)) for i in range(5)]
v = rs.verdict(lr)
check([t for lv, t in v if lv == "WARN"] == ["late releases (host pacer): ch0 0, ch1 2"],
      "late releases are the cumulative counter's peak per channel, not its line count (mutation: count lines)")
# main: the exit status is the verdict's, read from a run directory's largest log
d = tempfile.mkdtemp(prefix="run_summary_")
open(os.path.join(d, "small_cpu.log"), "w").write("x\n")
open(os.path.join(d, "VL1_051734.log"), "w").write("\n".join(GOOD) + "\n" + "57:1 INFOR: filler\n" * 50)
out = io.StringIO()
with contextlib.redirect_stdout(out):
    rc = rs.main(d)
check(rc == 0 and "VERDICT: PASS" in out.getvalue(), "main reads the run directory's sounder log and returns 0 on a pass (mutation: read the first log)")
open(os.path.join(d, "VL1_051734.log"), "a").write(GOOD[1].replace("0 time jumps", "1 time jumps") + "\n")
with contextlib.redirect_stdout(io.StringIO()):
    rc = rs.main(d)
check(rc == 1, "main returns 1 on a FAIL (mutation: always 0)")
import shutil; shutil.rmtree(d)
print("%d failure(s)" % fails); sys.exit(1 if fails else 0)
