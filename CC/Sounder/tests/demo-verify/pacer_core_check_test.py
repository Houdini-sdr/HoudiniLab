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
check(pc.flows(a, b, [15, 16, 18], 3.0) == [(16, 60000.0, [16])], "a flow on a pinned core's queue is named with its rate (mutation: absolute counts instead of the delta)")
check(pc.flows(a, b, [15, 18], 3.0) == [], "cores with no RX pass even when their TX counts move (mutation: count tx packets)")
check(pc.flows({16: 0}, {16: 200}, [16], 3.0) == [], "a trickle under 100 pkt/s is not a flow (mutation: drop the threshold)")
# The queue-to-CPU map from /proc/interrupts (the rig host's line format) and the
# IRQs' affinity lists: queue 7's IRQ on CPU 18 makes its flow a pinned-core flow.
IRQ = ("           CPU0       CPU1\n"
       " 396:       1411          0 ITS-PCI-MSIX-0000:01:00.0   1 Edge      mlx5_comp7@pci:0000:01:00.0\n"
       " 397:          0     530029 ITS-PCI-MSIX-0000:01:00.0   2 Edge      mlx5_comp16@pci:0000:01:00.0\n"
       " 398:          0          0 ITS-PCI-MSIX-0000:01:00.1   1 Edge      mlx5_comp7@pci:0000:01:00.1\n")
aff = {"396": "18\n", "397": "0-3,8\n", "398": "15\n"}
qc = pc.queue_cpus(IRQ, "0000:01:00.0", lambda irq: aff[irq])
check(qc == {7: {18}, 16: {0, 1, 2, 3, 8}}, "the port's own completion IRQs map its queues to their affinity CPUs, the other port's are not its (mutation: ignore the PCI address): %s" % qc)
qu = pc.queue_cpus(IRQ, "0000:01:00.0", lambda irq: "" if irq == "397" else aff[irq])
check(qu == {7: {18}} and pc.flows(a, b, [16], 3.0, qu) == [(16, 60000.0, [16])],
      "an IRQ whose affinity cannot be read falls back to queue N on CPU N, so its flow is still flagged "
      "(mutation: keep it with no CPUs, never flagged): %s" % qu)
check(pc.flows(a, b, [18], 3.0, qc) == [(7, 60000.0, [18])] and pc.flows(a, b, [16], 3.0, qc) == [],
      "with the IRQ map a queue is judged by the CPU it interrupts, not by its number (mutation: queue N as CPU N)")
print("%d failure(s)" % fails); sys.exit(1 if fails else 0)
