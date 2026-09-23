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
    # plus a child that ignores SIGTERM: only a group SIGKILL ends it
    f.write("#!/bin/sh\necho \"start $3\" >> %s\n"
            "sh -c 'trap \"\" TERM; while :; do sleep 0.2; done' &\necho $! >> %s.kids\nexec sleep 60\n" % (log, log))
os.chmod(os.path.join(sd, "build", "sounder"), 0o755)
with open(os.path.join(sd, "csi_gui", "teardown_framer.py"), "w") as f:
    f.write("open(%r, 'a').write('teardown\\n')\n" % log)
# Stand-in setup check: logs its form, and FAILs while the flag file exists.
flag = os.path.join(sd, "fail_check")
with open(os.path.join(sd, "csi_gui", "check_setup.py"), "w") as f:
    f.write("import json, os, sys\nq = '--quick' in sys.argv\n"
            "open(%r, 'a').write('check %%s\\n' %% ('quick' if q else 'full'))\n"
            "bad = os.path.exists(%r)\n"
            "print(json.dumps({'ok': not bad, 'quick': q, 'conf': sys.argv[2], 'results': "
            "[{'level': 'FAIL' if bad else 'PASS', 'what': 'stand-in', 'detail': '', 'fix': ''}]}))\n" % (log, flag))
for n in ("houdini-a.json", "houdini-b.json", "other.json"):
    open(os.path.join(sd, "files", n), "w").write('{"_description": "desc of %s"}' % n)
args = types.SimpleNamespace(sounder_dir=sd, max_frame=1, csi_fps=0, venv=sd,
                             conf="files/houdini-a.json", storepath=sd)
# The operator's own --conf is offered even when it is not files/houdini*.json.
check(cs.SounderSupervisor(types.SimpleNamespace(**dict(vars(args), conf="files/other.json")),
                           "x").configs()[-1] == "files/other.json", "the --conf config is always in the list")
sup = cs.SounderSupervisor(args, "127.0.0.1:1")
sup.SETTLE_AFTER_TEARDOWN_S = 0.2; sup.RETRY_DELAY_S = 0.2; sup.STOP_GRACE_S = 2.0

srv = cs.ThreadingHTTPServer(("127.0.0.1", 0), cs.Handler); srv.daemon_threads = True
srv.control = sup
threading.Thread(target=srv.serve_forever, daemon=True).start()
url = "http://127.0.0.1:%d/control" % srv.server_address[1]
def post(obj, headers=None, raw=None):
    h = {"Content-Type": "application/json"}; h.update(headers or {})
    req = urllib.request.Request(url, data=raw if raw is not None else json.dumps(obj).encode(),
                                 method="POST", headers=h)
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
def events(): return [l for l in open(log).read().splitlines() if not l.startswith("start")] if os.path.exists(log) else []
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
        check(st["desc"]["files/houdini-b.json"] == "desc of houdini-b.json", "each config's _description is served")
        # A failing quick check blocks Start and is shown; nothing launches.
        open(flag, "w").close()
        post({"cmd": "start"})
        check(wait_for(lambda: get()["state"] == "check failed"), "a failing quick check blocks Start")
        ck = get()["check"]
        check(ck and ck["quick"] and not ck["ok"] and starts() == [] and "teardown" not in events(),
              "the failed check is reported and neither teardown nor sounder ran")
        post({"cmd": "check"})
        check(wait_for(lambda: get()["state"] == "stopped" and not get()["check"]["quick"]),
              "Check runs the full form")
        os.remove(flag)
        open(log, "w").close()
        check(post({"cmd": "start", "conf": "files/other.json"})[0] == 400, "a config outside the list is refused")
        check(post({"cmd": "start", "conf": "../../etc/passwd"})[0] == 400, "a path outside the sounder is refused")
        check(post({"cmd": "reboot"})[0] == 400, "an unknown command is refused")
        check(post({"cmd": "start"}, {"Content-Type": "text/plain"})[0] == 403,
              "a non-JSON POST (what a cross-site no-cors fetch sends) is refused")
        check(post({"cmd": "start"}, {"Origin": "http://evil.example"})[0] == 403, "a foreign Origin is refused")
        check(post(None, raw=b"[1]")[0] == 400 and post(None, raw=b"{bad")[0] == 400,
              "a body that is not a JSON object is a 400")
        check(post(None, raw=b"{" + b" " * 5000 + b"}")[0] == 400, "an oversized body is a 400")
        check(starts() == [], "nothing launched by refused requests")
        check(post({"cmd": "start"})[0] == 202, "start is accepted")
        check(wait_for(lambda: get()["state"] == "running"), "start: running")
        pid1 = get()["pid"]; seen["pid1"] = pid1
        check(post({"cmd": "start"})[0] == 400, "Start while running is refused")
        sup.cmds.put(("start", None)); time.sleep(0.6)  # one queued before the state showed running
        check(get()["pid"] == pid1 and alive(pid1), "a queued Start does not restart a live session")
        check(starts() == ["files/houdini-a.json"] and events()[:2] == ["check quick", "teardown"],
              "start ran a quick check, a teardown, then the current config")
        check(post({"cmd": "check"})[0] == 400, "Check while running is refused")
        n_checks = events().count("check full")
        sup.cmds.put(("check", None)); time.sleep(0.6)
        check(get()["pid"] == pid1 and events().count("check full") == n_checks,
              "a queued Check neither runs nor disturbs a live session")
        post({"cmd": "restart", "conf": "files/houdini-b.json"})
        check(wait_for(lambda: get()["state"] == "running" and get()["pid"] != pid1), "restart: a new sounder is running")
        check(not alive(pid1), "restart killed the old sounder")
        kid1 = int(open(log + ".kids").read().split()[0])
        check(not alive(kid1), "restart killed the old sounder's SIGTERM-proof child (group SIGKILL)")
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
