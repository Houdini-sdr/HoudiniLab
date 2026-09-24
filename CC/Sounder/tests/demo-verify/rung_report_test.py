# Known answers for rung_report.py's log readers. Stdlib only.
import contextlib, importlib.util, io, os, sys, tempfile
spec = importlib.util.spec_from_file_location("rr", os.path.join(os.path.dirname(os.path.abspath(__file__)), "rung_report.py"))
rr = importlib.util.module_from_spec(spec); spec.loader.exec_module(rr)
fails = 0
def check(ok, what):
    global fails; print(("PASS " if ok else "FAIL ") + what); fails += (not ok)
L = ["12:000001 INFOR syncSearch: detection #1 statistic 0.9712 vs bar 0.3500 (confirm), idx 701 in 4096",
     "12:000002 INFOR syncSearch: detection #2 statistic 0.9650 vs bar 0.3500 (confirm), idx 702 in 4096",
     "12:000003 WARNG BS 168.6.244.22 link health: [WARN] tx0.late +5 tx0.under +1 | app ok",
     "12:000004 WARNG UE 168.6.244.21 link health: [WARN] tx0.late +3 tx0.under +2 tx0.zerofill +100 tx1.late +7 | app ok",
     "12:000005 WARNG UE 168.6.244.21 link health: [WARN] tx0.late +4 | app ok",
     "12:000006 INFOR HOUDINI_BS_RX: frame=20 stamp_ticks=1 cg=2 pilot-rms=3 selfsim=0.99 p_start=4 rx_slots=2 pilot_grid_off=-1 clamped=0",
     "12:000007 INFOR HOUDINI_BS_RX: frame=40 stamp_ticks=1 cg=2 pilot-rms=3 selfsim=0.99 p_start=4 rx_slots=2 pilot_grid_off=0 clamped=1"]
acq = rr.acquisitions(L)
# Fails under: the old pattern pinned to "vs bar 0\.2" (every detection at another bar dropped).
check(acq == [(0.9712, 0.35), (0.965, 0.35)], "detections are read at any bar: %s" % acq)
# Fails under: summing every link-health line (the BS's own tx0 adds +5 late, +1 under).
check(rr.ue_tx0_totals(L) == (7, 2, 100), "UE tx0 totals are the UE's lines only: %s" % (rr.ue_tx0_totals(L),))
log = tempfile.NamedTemporaryFile("w", suffix=".log", delete=False); log.write("\n".join(L) + "\n"); log.close()
out = io.StringIO()
with contextlib.redirect_stdout(out):
    rr.main(log.name)
os.remove(log.name)
check("UE tx0 totals: late 7, under 2, zerofill 100" in out.getvalue() and "vs bar [0.35]" in out.getvalue(),
      "the report prints both")
# Fails under: reading the retired pu_spacing_err field (the line never prints).
check("BS slots clamped past the capture edge: 1 of 2 sampled frames" in out.getvalue(),
      "the report counts the frames whose slots were clamped")
print("%d failure(s)" % fails); sys.exit(1 if fails else 0)
