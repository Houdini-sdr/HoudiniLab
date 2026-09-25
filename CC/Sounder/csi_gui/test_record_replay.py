# Known answers for the canned-data fallback: the record format, the recorder's
# cap, the replay schedule and window, and the dashboard's UDP loop recording
# what it receives. Stdlib only; run from csi_gui/ (ctest does). Each check names
# the mutation that breaks it.
import importlib.util, os, socket, struct, sys, tempfile, threading, time
sys.argv = ["x"]
import csi_record as cr
import replay_feed as rf
spec = importlib.util.spec_from_file_location("cs", "csi_server.py"); cs = importlib.util.module_from_spec(spec); spec.loader.exec_module(cs)
fails = 0
def check(ok, what):
    global fails; print(("PASS " if ok else "FAIL ") + what); fails += (not ok)
td = tempfile.mkdtemp(prefix="csi_rec_")
recs = [(10.0, b"\x31\x49\x53\x43" + bytes(range(40))), (10.25, b"abc"), (11.5, bytes(3000))]

p = os.path.join(td, "a.rec")
r = cr.Recorder(p, 1 << 20, log=lambda m: None)
for t, d in recs: r.write(t, d)
r.close()
back = list(cr.read_recording(p))
check(back == recs, "records read back byte for byte with their times (mutation: drop the length field)")

logs = []
p2 = os.path.join(td, "cap.rec")
r = cr.Recorder(p2, len(cr.REC_MAGIC) + 2 * cr.REC_HDR.size + 44 + 3, log=logs.append)
for t, d in recs: r.write(t, d)
r.close()
back = list(cr.read_recording(p2))
check(back == recs[:2] and len(logs) == 1, "the cap stops after the last record that fits, says so once, and the file stays readable (mutation: no cap check)")

with open(p, "ab") as f:
    f.write(cr.REC_HDR.pack(12.0, 500) + b"short")
check(list(cr.read_recording(p)) == recs, "a record cut short at the end ends the recording, no exception (mutation: raise on a short read)")

bad = os.path.join(td, "bad.rec"); open(bad, "wb").write(b"NOTAREC!")
try:
    list(cr.read_recording(bad)); ok = False
except ValueError:
    ok = True
check(ok, "a file without the header is refused by name (mutation: skip the header check)")

check(rf.schedule([10.0, 10.25, 11.5], 2.0) == [0.0, 0.125, 0.75], "the schedule keeps the recorded spacing scaled by speed (mutation: ignore speed)")
check([t for t, _ in rf.load(p, 0.25, 1.25)] == [10.25], "a window takes [start, start + duration) from the first datagram (mutation: an inclusive end)")
check(len(rf.load(p, 0.0, 0.0)) == 3, "duration 0 plays to the end (mutation: treat 0 as empty)")

# End to end: the dashboard's own UDP loop records what it receives, and a replay of
# that recording delivers the same datagrams in the same order.
def free_port():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM); s.bind(("127.0.0.1", 0)); n = s.getsockname()[1]; s.close(); return n
port = free_port(); p3 = os.path.join(td, "loop.rec")
rec3 = cr.Recorder(p3, 1 << 20, log=lambda m: None)
threading.Thread(target=cs._udp_loop, args=("127.0.0.1", port, rec3), daemon=True).start()
time.sleep(0.3)
tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sent = [struct.pack("<I", 0x12345678) + bytes([i]) * (i + 1) for i in range(5)]
for d in sent:
    tx.sendto(d, ("127.0.0.1", port)); time.sleep(0.02)
deadline = time.time() + 3
while rec3.n < len(sent) and time.time() < deadline: time.sleep(0.05)
rec3.close()
got = [d for _, d in cr.read_recording(p3)]
check(got == sent, "the dashboard's UDP loop records every datagram as received, unknown magic included (mutation: record after the magic filter)")
rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM); rx.bind(("127.0.0.1", 0)); rx.settimeout(2)
rf.play(tx, rx.getsockname(), list(cr.read_recording(p3)), 10.0)
out = [rx.recvfrom(65535)[0] for _ in sent]
check(out == sent, "a replay delivers the recorded datagrams unchanged and in order (mutation: send the header with the payload)")

print("%d failure(s)" % fails)
sys.exit(1 if fails else 0)
