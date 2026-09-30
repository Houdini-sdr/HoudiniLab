# mer_sampler against a live dashboard fed by fake_feed: a feed that stalls must
# give STALE samples and exit 1; a healthy feed none and exit 0. Stdlib only;
# run from tests/demo-verify (ctest does).
# Mutations: drop the frame-advance test and the age test (the stalled run then
# reads healthy); mark every sample stale (the healthy run fails).
import os, socket, subprocess, sys, tempfile, time
GUI = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "csi_gui")
fails = 0
def check(ok, what):
    global fails; print(("PASS " if ok else "FAIL ") + what, flush=True); fails += (not ok)
def free_port(kind):
    s = socket.socket(socket.AF_INET, kind); s.bind(("127.0.0.1", 0)); p = s.getsockname()[1]; s.close(); return p

def run(stall_after):
    udp, http = free_port(socket.SOCK_DGRAM), free_port(socket.SOCK_STREAM)
    out = tempfile.mktemp(prefix="mer_sampler_", suffix=".txt")
    srv = subprocess.Popen([sys.executable, "csi_server.py", "--udp-host", "127.0.0.1", "--udp-port", str(udp),
                            "--http-host", "127.0.0.1", "--http-port", str(http)], cwd=GUI,
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    feed = subprocess.Popen([sys.executable, "fake_feed.py", "--port", str(udp), "--fps", "10", "--samps", "512"]
                            + (["--stall-after", str(stall_after)] if stall_after else []), cwd=GUI,
                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    try:
        time.sleep(1.5)
        rc = subprocess.run([sys.executable, "mer_sampler.py", out, "1", "5", "--url",
                             "http://127.0.0.1:%d/stream" % http], timeout=90).returncode
        text = open(out).read()
    finally:
        for p in (feed, srv):
            p.kill(); p.wait()
        if os.path.exists(out):
            os.unlink(out)
    return rc, text

rc, text = run(stall_after=25)  # 10 fps: data for 2.5 s, then nothing
print(text)
check(rc == 1 and "STALE" in text, "a feed that stalls gives STALE samples and exit 1 (mutation: no frame or age test)")
rc, text = run(stall_after=0)
print(text)
check(rc == 0 and "STALE" not in text and "MER" in text, "a healthy feed gives MER samples, none stale, exit 0 (mutation: every sample stale)")
print("%d failure(s)" % fails); sys.exit(1 if fails else 0)
