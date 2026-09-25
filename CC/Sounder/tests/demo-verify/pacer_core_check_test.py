# Known answers for pacer_core_check.py. Each check names the mutation that breaks it.
import importlib.util, os, sys
spec = importlib.util.spec_from_file_location("pc", os.path.join(os.path.dirname(os.path.abspath(__file__)), "pacer_core_check.py"))
pc = importlib.util.module_from_spec(spec); spec.loader.exec_module(pc)
fails = 0
def check(ok, what):
    global fails; print(("PASS " if ok else "FAIL ") + what); fails += (not ok)
A = "NIC statistics:\n     rx_packets: 999\n     rx7_packets: 1000\n     rx16_packets: 5000\n     rx18_packets: 0\n     tx18_packets: 700\n"
B = "NIC statistics:\n     rx_packets: 999999\n     rx7_packets: 181000\n     rx16_packets: 185000\n     rx18_packets: 0\n     tx18_packets: 90000\n"
a, b = pc.rx_counts(A), pc.rx_counts(B)
check(a == {7: 1000, 16: 5000, 18: 0}, "per-queue rx counters parse; rx_packets and tx counters are not queues (mutation: match tx or the total)")
check(pc.flows(a, b, [15, 16, 18], 3.0) == [(16, 60000.0)], "a flow on a pinned core's queue is named with its rate (mutation: absolute counts instead of the delta)")
check(pc.flows(a, b, [15, 18], 3.0) == [], "cores with no RX pass even when their TX counts move (mutation: count tx packets)")
check(pc.flows({16: 0}, {16: 200}, [16], 3.0) == [], "a trickle under 100 pkt/s is not a flow (mutation: drop the threshold)")
print("%d failure(s)" % fails); sys.exit(1 if fails else 0)
