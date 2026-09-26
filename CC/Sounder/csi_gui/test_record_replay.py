# Known answers for the canned-data fallback: the record format, the recorder's
# cap, the replay schedule, window and pace, and the dashboard's UDP loop recording
# what it receives; plus the loop surviving a malformed datagram, each antenna's
# age in the snapshot, and the SSE event shared between pages. Stdlib only; run
# from csi_gui/ (ctest does).
# Each check names the mutation that breaks it.
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

class _Full:
    def write(self, b): raise OSError(28, "No space left on device")
    def flush(self): pass
    def close(self): pass
logs2 = []
r = cr.Recorder(os.path.join(td, "full.rec"), 1 << 20, log=logs2.append); r.f.close(); r.f = _Full()
try:
    r.write(1.0, b"x"); r.write(2.0, b"y"); ok = r.f is None and len(logs2) == 1
except OSError:
    ok = False
check(ok, "a write error (a full disk) stops the recording once and never raises into the receive loop (mutation: drop the try)")

before = open(p, "rb").read(); logs3 = []
r = cr.Recorder(p, 1 << 20, log=logs3.append); r.write(20.0, b"zz"); r.close()
check(open(p, "rb").read() == before and r.f is None and len(logs3) == 1,
      "an existing recording is never truncated or appended to, and the refusal is said once (mutation: open with wb)")
logs4 = []
try:
    r = cr.Recorder(os.path.join(td, "no_such_dir", "x.rec"), 1 << 20, log=logs4.append)
    r.write(1.0, b"x"); ok = r.f is None and len(logs4) == 1
except OSError:
    ok = False
check(ok, "a recording that cannot be created records nothing and never raises into the dashboard's start (mutation: let the OSError through)")

check(rf.schedule([10.0, 10.25, 11.5], 2.0) == [0.0, 0.125, 0.75], "the schedule keeps the recorded spacing scaled by speed (mutation: ignore speed)")
check([t for t, _ in rf.load(p, 0.25, 1.25)] == [10.25], "a window takes [start, start + duration) from the first datagram (mutation: an inclusive end)")
check(len(rf.load(p, 0.0, 0.0)) == 3, "duration 0 plays to the end (mutation: treat 0 as empty)")
read = [0]; real_read = rf.read_recording
def counting_read(path):
    for r in real_read(path):
        read[0] += 1; yield r
rf.read_recording = counting_read
try:
    got_w = [t for t, _ in rf.window(p, 0.0, 0.2)]
finally:
    rf.read_recording = real_read
check(got_w == [10.0] and read[0] == 2, "a window stops reading at the first datagram past its end (mutation: read the "
      "whole recording, then filter)")

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
# A short datagram with a known magic raises inside its parser; the loop must
# count it and keep going (the port is open to the network).
tx.sendto(struct.pack("<II", cs.MAGIC_CSI2, 7), ("127.0.0.1", port)); time.sleep(0.05)
tx.sendto(cs.MET_HDR.pack(cs.MAGIC_MET, 5, 1, 4096, 3276, 10.0e9, 30e3, 98.28e6), ("127.0.0.1", port))
deadline = time.time() + 3
while 5 not in cs._latest and time.time() < deadline: time.sleep(0.05)
check(cs._bad_dgram[0] == 1 and cs._latest.get(5, {}).get("met", {}).get("fc_mhz") == 10.0e3,
      "a short datagram is counted and the receive loop parses the next one (mutation: drop the try around the parse)")
rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM); rx.bind(("127.0.0.1", 0)); rx.settimeout(2)
rf.play(tx, rx.getsockname(), list(cr.read_recording(p3)), 10.0)
out = [rx.recvfrom(65535)[0] for _ in sent]
check(out == sent, "a replay delivers the recorded datagrams unchanged and in order (mutation: send the header with the payload)")
# The replay keeps the recorded pace: three datagrams 0.3 s apart take at least
# 0.6 s at speed 1 (a lower bound only, so a loaded host cannot fail it).
paced = [(50.0, b"p0"), (50.3, b"p1"), (50.6, b"p2")]
t0 = time.monotonic()
rf.play(tx, rx.getsockname(), paced, 1.0)
took = time.monotonic() - t0
got = [rx.recvfrom(65535)[0] for _ in paced]
check(took >= 0.5 and got == [d for _, d in paced],
      "a replay of datagrams 0.3 s apart takes %.2f s, at least 0.5 (mutation: delete the pacing sleep, every "
      "datagram at once)" % took)

# Each antenna's record carries its age, from the last datagram seen for it:
# a frozen lane must read as old on the page, not as a static channel.
t_in = time.monotonic()
with cs._lock:
    cs._latest[41] = {"cns": {"frame": 1}, "t": t_in - 2.0}
_seq, snap, _sync = cs._snapshot()
after = time.monotonic()
age = snap.get("41", {}).get("age_ms")
check(age is not None and 1999 <= age <= int((after - t_in + 2.0) * 1000) + 1 and "t" not in snap["41"],
      "a lane last heard 2 s ago reads age_ms %r, 2000 (less 1 ms of rounding) up to the call's own time (mutation: age_ms forced to 0, "
      "or the stamp leaked into the record)" % age)
with cs._lock:
    del cs._latest[41]

# One serialisation per seq, shared by every open page; a stale re-push (seq
# unchanged) past BODY_REUSE_S rebuilds, so the ages it carries still move.
calls = [0]; real = cs.json.dumps
def counting(*a, **k):
    calls[0] += 1; return real(*a, **k)
cs.json.dumps = counting
e1 = cs._shared_event(900, {"0": {"age_ms": 1}}, {}, 100.0)
e2 = cs._shared_event(900, {"0": {"age_ms": 2}}, {}, 100.1)
check(e1 is e2 and calls[0] == 1, "a second page within BODY_REUSE_S gets the same event, serialised once (mutation: no cache)")
e3 = cs._shared_event(900, {"0": {"age_ms": 400}}, {}, 100.0 + cs.BODY_REUSE_S + 0.01)
check(calls[0] == 2 and b'"age_ms": 400' in e3, "a re-push of the same seq past BODY_REUSE_S carries fresh ages (mutation: reuse on seq alone)")
e4 = cs._shared_event(901, {"0": {"age_ms": 0}}, {}, 100.0 + cs.BODY_REUSE_S + 0.02)
check(calls[0] == 3 and e4.startswith(b"data: ") and e4.endswith(b"\n\n"), "a new seq is serialised as an SSE event (mutation: reuse regardless of seq)")
check(cs._shared_event(902, {"0": {"x": float("nan")}}, {}, 200.0) is None, "a non-finite value gives no event rather than invalid JSON (mutation: allow_nan)")
cs.json.dumps = real

import shutil; shutil.rmtree(td, ignore_errors=True)  # no temp dir left per run
print("%d failure(s)" % fails)
sys.exit(1 if fails else 0)
