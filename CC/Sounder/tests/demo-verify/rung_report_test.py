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
     "12:000007 INFOR HOUDINI_BS_RX: frame=40 stamp_ticks=1 cg=2 pilot-rms=3 selfsim=0.99 p_start=4 rx_slots=2 pilot_grid_off=0 clamped=1",
     # the sounder's own formats (receiver.cc, recorder_worker.cc)
     "12:000008 ERROR clientSyncBeacon [0]: BAD SYNC Received (-1/4096) 12345",
     "12:000009 INFOR Re-sync frame 500: beacon alive on the anchored grid (resid -3 within scatter, snr 21.5 dB), tid 3",
     "12:000010 INFOR Re-sync frame 760: beacon alive on the anchored grid (resid +2 within scatter, snr 23.0 dB), tid 3",
     "12:000011 INFOR CNS score 0.950 rot +1.2 deg at frame 900 (400 datagrams, 3 low); P->U 20.8 us, so +7.5 deg per kHz",
     "12:000012 INFOR CNS score 0.410 (low, no rotation) at frame 1400 (900 datagrams, 7 low)",
     "12:000013 WARNG CNS score 0.300 at frame 1401, r=0.2 (low occurrence 8 of 901 datagrams)"]
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
# Fails under: BAD SYNC counted as a constant 0 (or its pattern not matching the line).
check("BAD SYNC 1," in out.getvalue(), "a BAD SYNC line is counted")
# Fails under: the CNS low read from the first summary, or from the throttled
# per-event warning ("low occurrence 8"), instead of the last summary.
check("CNS low 7 of 900 (summary)" in out.getvalue(), "the CNS low is the last summary's total")
# Fails under: the residual pattern dropping a sign (a negative residual then
# never matches and the range reads 2..2 over 1 re-sync).
check("re-syncs alive 2; resid -3..2" in out.getvalue(), "negative re-sync residuals are read")
# Fails under: reading the retired pu_spacing_err field (the line never prints).
check("BS slots clamped past the capture edge: 1 of 2 sampled frames" in out.getvalue(),
      "the report counts the frames whose slots were clamped")
# Real-format health lines (RadioHoudini's health thread, link_health.h line()):
# an app-only alarm reads "clean" on the device side.
H = ["57:000001 WARNG: BS 192.168.10.22 link health: [BS 192.168.10.22] 5.0 s: irq 12/s, preflight ok: clean"
     " | app: rx_err +0, rx_short +0, rx_pad +7, tx_short +0, tx_sat +3",
     "57:000002 WARNG: BS 192.168.10.22 link health: [BS 192.168.10.22] 5.0 s: irq 12/s, preflight ok: rx0.gated +4"
     " | app: rx_err +0, rx_short +0, rx_pad +0, tx_short +0, tx_sat +0",
     "57:000003 INFOR: BS 192.168.10.22 link health: [BS 192.168.10.22] 60.0 s: irq 12/s, preflight ok: clean"
     " | app: rx_err +0, rx_short +0, rx_pad +0, tx_short +0, tx_sat +0"]
log = tempfile.NamedTemporaryFile("w", suffix=".log", delete=False); log.write("\n".join(H) + "\n"); log.close()
out = io.StringIO()
with contextlib.redirect_stdout(out):
    rr.main(log.name)
os.remove(log.name)
o2 = out.getvalue()
check("health alarm lines 2 " in o2 and "'rx_pad +N': 1" in o2 and "'tx_sat +N': 1" in o2 and "'rx0.gated +N': 1" in o2,
      "an app-only alarm and an rx bank alarm both count, the periodic clean line does not (mutation: drop lines "
      "reading 'clean |', or no rx bank pattern): " + o2[o2.find("health alarm"):].split("\n")[0])
print("%d failure(s)" % fails); sys.exit(1 if fails else 0)
