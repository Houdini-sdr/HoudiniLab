# check_setup.py against a fake host: a stand-in SoapySDRUtil, a listening
# socket for each radio's server, a process named `sounder`, and a stand-in
# SoapySDR module that serves each radio's hardware info. Stdlib only; run
# from csi_gui/ (ctest does).
import json, os, shutil, socket, subprocess, sys, tempfile, threading, time
fails = 0
def check(ok, what):
    global fails; print(("PASS " if ok else "FAIL ") + what, flush=True); fails += (not ok)

root = tempfile.mkdtemp(prefix="csi_chk_")
sd = os.path.join(root, "Sounder"); venv = os.path.join(root, "venv"); fake = os.path.join(root, "fake")
for d in ("files", "build", "include"):
    os.makedirs(os.path.join(sd, d))
os.makedirs(os.path.join(venv, "bin")); os.makedirs(os.path.join(venv, "lib", "SoapySDR", "modules0.8-3")); os.makedirs(fake)
ex = os.path.join(root, "examples"); os.makedirs(ex); open(os.path.join(ex, "houdini_setup.py"), "w").close()

# Both "radios" on one listening port: 127.0.0.1 and 127.0.0.2 (all of 127/8 is
# loopback). Accept and close, or the backlog fills and later connects time out.
srv = socket.socket(); srv.bind(("0.0.0.0", 0)); srv.listen(8); port = srv.getsockname()[1]
def _accept():
    while True:
        try: srv.accept()[0].close()
        except OSError: return
threading.Thread(target=_accept, daemon=True).start()
json.dump({"BaseStations": {"BS0": {"sdr": ["127.0.0.1"]}}, "Clients": {"sdr": ["127.0.0.2"]}},
          open(os.path.join(sd, "files", "topo.json"), "w"))
json.dump({"serial_file": "files/topo.json", "remote_port": str(port), "_description": "the demo"},
          open(os.path.join(sd, "files", "houdini-x.json"), "w"))
open(os.path.join(sd, "files", "houdini-bad.json"), "w").write("{nope")
open(os.path.join(sd, "a.cc"), "w").close()
time.sleep(0.05)
exe = os.path.join(sd, "build", "sounder"); open(exe, "w").write("#!/bin/sh\n"); os.chmod(exe, 0o755)
open(os.path.join(venv, "lib", "SoapySDR", "modules0.8-3", "libHoudiniSDRSupport.so"), "w").close()
util = os.path.join(venv, "bin", "SoapySDRUtil")
open(util, "w").write("#!/bin/sh\necho 'Available factories... houdinisdr, remote'\n"); os.chmod(util, 0o755)
# Stand-in SoapySDR: hardware info per radio from a JSON file keyed by address.
info_file = os.path.join(root, "info.json")
open(os.path.join(fake, "SoapySDR.py"), "w").write(
    "import json\nclass Device:\n"
    "    def __init__(self, a): self.ip = a['remote'].split('//')[1].split(':')[0]\n"
    "    def getHardwareInfo(self): return json.load(open(%r))[self.ip]\n" % info_file)
same = {k: "v1" for k in ("fpga_version", "fpga_commit", "fpga_board", "device_version",
                          "device_build", "host_version", "host_build", "proto_version")}
json.dump({"127.0.0.1": same, "127.0.0.2": same}, open(info_file, "w"))

env = dict(os.environ, HOUDINI_EXAMPLES=ex, PYTHONPATH=fake)
def run(*extra, conf="files/houdini-x.json"):
    out = subprocess.run([sys.executable, os.path.abspath("check_setup.py"), "--sounder-dir", sd,
                          "--venv", venv, "--conf", conf, "--json"] + list(extra),
                         env=env, stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=120)
    rep = json.loads(out.stdout)
    return out.returncode, rep, {r["what"]: r["level"] for r in rep["results"]}

rc, rep, lv = run()
check(rc == 0 and rep["ok"], "a ready host passes (rc 0)")
check(lv.get("stack match") == "PASS" and lv.get("server 127.0.0.2") == "PASS", "full form: servers answer and stacks match")
check(all(l != "WARN" for l in lv.values()), "no warnings on a ready host: %s" % lv)
rc, rep, lv = run("--quick")
check(rc == 0 and "stack match" not in lv, "--quick does not open the radios")

# Each broken piece, one at a time, is a FAIL (or the stated WARN) under its own name.
json.dump({"127.0.0.1": same, "127.0.0.2": dict(same, fpga_commit="v2")}, open(info_file, "w"))
rc, rep, lv = run(); check(rc == 1 and lv["stack match"] == "FAIL", "a node on a different gateware fails 'stack match'")
check("fpga_commit" in [r for r in rep["results"] if r["what"] == "stack match"][0]["detail"], "and names the differing key")
json.dump({"127.0.0.1": same, "127.0.0.2": same}, open(info_file, "w"))
rc, rep, lv = run(conf="files/houdini-bad.json"); check(rc == 1 and lv["config"] == "FAIL", "a config that is not JSON fails 'config'")
rc, rep, lv = run(conf="files/none.json"); check(rc == 1 and lv["config"] == "FAIL", "a missing config fails 'config'")
os.utime(os.path.join(sd, "a.cc"), None); os.utime(exe, (time.time() - 60, time.time() - 60))
rc, rep, lv = run("--quick"); check(rc == 0 and lv["build"] == "WARN", "a source newer than the binary is a WARN, not a FAIL")
os.utime(exe, None)
os.rename(exe, exe + ".x"); rc, rep, lv = run("--quick"); check(rc == 1 and lv["build"] == "FAIL", "no binary fails 'build'"); os.rename(exe + ".x", exe)
open(util, "w").write("#!/bin/sh\necho 'Available factories... remote'\n")
rc, rep, lv = run("--quick"); check(rc == 1 and lv["plugin"] == "FAIL", "SoapySDR not loading the Houdini module fails 'plugin'")
open(util, "w").write("#!/bin/sh\necho 'Available factories... houdinisdr, remote'\n")
env["HOUDINI_EXAMPLES"] = root; rc, rep, lv = run("--quick")
check(rc == 0 and lv["teardown"] == "WARN", "missing host examples is a WARN"); env["HOUDINI_EXAMPLES"] = ex
snd = os.path.join(root, "sounder"); shutil.copy("/bin/sleep", snd)
p = subprocess.Popen([snd, "30"]); time.sleep(0.2)
rc, rep, lv = run("--quick"); check(rc == 1 and lv["radios free"] == "FAIL", "another sounder running fails 'radios free'")
p.kill(); p.wait()
srv.shutdown(socket.SHUT_RDWR); srv.close()
rc, rep, lv = run(); check(rc == 1 and lv["server 127.0.0.1"] == "FAIL" and "stack match" not in lv,
                          "a server that does not answer fails, and the stack read is skipped")
print("%d failure(s)" % fails); sys.exit(1 if fails else 0)
