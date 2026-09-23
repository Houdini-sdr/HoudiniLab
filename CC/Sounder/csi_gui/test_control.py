# Dashboard control (--control) against a stand-in sounder: the supervisor runs
# on the main thread as in main(); commands arrive from another thread, and the
# HTTP route is exercised end to end. Stdlib only; run from csi_gui/ (ctest does).
import json, os, sys, tempfile, threading, time, types, urllib.request, urllib.error
sys.argv = ["x"]
import importlib.util
spec = importlib.util.spec_from_file_location("cs", "csi_server.py"); cs = importlib.util.module_from_spec(spec); spec.loader.exec_module(cs)
fails = 0
def check(ok, what):
    global fails; print(("PASS " if ok else "FAIL ") + what, flush=True); fails += (not ok)

sd = tempfile.mkdtemp(prefix="csi_ctl_")
log = os.path.join(sd, "log")
os.makedirs(os.path.join(sd, "build")); os.makedirs(os.path.join(sd, "csi_gui")); os.makedirs(os.path.join(sd, "files"))
with open(os.path.join(sd, "build", "sounder"), "w") as f:  # records its config, then runs until killed
    f.write("#!/bin/sh\necho \"start $3\" >> %s\nexec sleep 60\n" % log)
os.chmod(os.path.join(sd, "build", "sounder"), 0o755)
with open(os.path.join(sd, "csi_gui", "teardown_framer.py"), "w") as f:
    f.write("open(%r, 'a').write('teardown\\n')\n" % log)
for n in ("houdini-a.json", "houdini-b.json", "other.json"):
    open(os.path.join(sd, "files", n), "w").write("{}")
args = types.SimpleNamespace(sounder_dir=sd, max_frame=1, csi_fps=0, venv=sd,
                             conf="files/houdini-a.json", storepath=sd)
sup = cs.SounderSupervisor(args, "127.0.0.1:1")
sup.SETTLE_AFTER_TEARDOWN_S = 0.2; sup.RETRY_DELAY_S = 0.2; sup.STOP_GRACE_S = 2.0

srv = cs.ThreadingHTTPServer(("127.0.0.1", 0), cs.Handler); srv.daemon_threads = True
srv.control = sup
threading.Thread(target=srv.serve_forever, daemon=True).start()
url = "http://127.0.0.1:%d/control" % srv.server_address[1]
def post(obj):
    req = urllib.request.Request(url, data=json.dumps(obj).encode(), method="POST",
                                 headers={"Content-Type": "application/json"})
    try:
        with urllib.request.urlopen(req) as r: return r.status, json.load(r)
    except urllib.error.HTTPError as e:
        body = e.read()
        return e.code, (json.loads(body) if body.startswith(b"{") else None)
def get(): 
    with urllib.request.urlopen(url) as r: return json.load(r)
def wait_for(pred, t=5.0):
    end = time.time() + t
    while time.time() < end:
        if pred(): return True
        time.sleep(0.05)
    return False
def starts(): return [l.split()[1] for l in open(log).read().splitlines() if l.startswith("start")] if os.path.exists(log) else []
def alive(pid):
    try: os.kill(pid, 0); return True
    except ProcessLookupError: return False

seen = {}
def driver():
    try:
        st = get()
        check(st["enabled"] and st["state"] == "stopped", "not autostarted: stopped until asked")
        check(st["configs"] == ["files/houdini-a.json", "files/houdini-b.json"], "only files/houdini*.json are offered")
        check(post({"cmd": "start", "conf": "files/other.json"})[0] == 400, "a config outside the list is refused")
        check(post({"cmd": "start", "conf": "../../etc/passwd"})[0] == 400, "a path outside the sounder is refused")
        check(post({"cmd": "reboot"})[0] == 400, "an unknown command is refused")
        check(starts() == [], "nothing launched by refused requests")
        check(post({"cmd": "start"})[0] == 202, "start is accepted")
        check(wait_for(lambda: get()["state"] == "running"), "start: running")
        pid1 = get()["pid"]; seen["pid1"] = pid1
        check(starts() == ["files/houdini-a.json"], "start ran the current config after a teardown")
        post({"cmd": "restart", "conf": "files/houdini-b.json"})
        check(wait_for(lambda: get()["state"] == "running" and get()["pid"] != pid1), "restart: a new sounder is running")
        check(not alive(pid1), "restart killed the old sounder")
        check(get()["conf"] == "files/houdini-b.json" and starts()[-1] == "files/houdini-b.json", "restart switched config")
        check(open(log).read().count("teardown") == 2, "every start is preceded by a teardown")
        pid2 = get()["pid"]
        post({"cmd": "stop"})
        check(wait_for(lambda: get()["state"] == "stopped"), "stop: stopped")
        check(wait_for(lambda: not alive(pid2)), "stop killed the sounder")
        time.sleep(0.5)
        check(get()["state"] == "stopped" and len(starts()) == 2, "stop stays stopped (no retry)")
        post({"cmd": "start"})
        check(wait_for(lambda: get()["state"] == "running"), "start after stop works")
        seen["pid3"] = get()["pid"]
    finally:
        sup.stop()

threading.Thread(target=driver, daemon=True).start()
sup.serve(autostart=False)  # main thread, as in main(); returns once sup.stop() is called
check("pid3" in seen and wait_for(lambda: not alive(seen["pid3"])), "shutdown stop() ends serve() and kills the sounder")
# Without --control the route does not exist.
srv.control = None
check(get() == {"enabled": False}, "no --control: GET reports disabled")
check(post({"cmd": "start"})[0] == 404, "no --control: POST is 404")
srv.shutdown()
print("%d failure(s)" % fails); sys.exit(1 if fails else 0)
