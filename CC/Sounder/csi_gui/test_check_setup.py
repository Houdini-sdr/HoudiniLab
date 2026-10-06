# check_setup.py against a fake host: a stand-in SoapySDRUtil, a listening
# socket for each radio's server, a process table of its own (--proc-root) with
# processes named `sounder`, and a stand-in SoapySDR module that serves each
# radio's hardware info. The host's own processes and plugin environment never
# enter it, so it passes in an operator's shell mid-run. Stdlib only; run from
# csi_gui/ (ctest does).
import json, os, shutil, socket, subprocess, sys, tempfile, threading, time
for k in ("SOAPY_SDR_ROOT",):  # the loader's own; the check sets it from --soapy-root
    os.environ.pop(k, None)
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
info_file = os.path.join(root, "info.json"); egress_file = os.path.join(root, "egress.json")
open(os.path.join(fake, "SoapySDR.py"), "w").write(
    "import json\nclass Device:\n"
    "    def __init__(self, a):\n"
    "        import os\n"
    "        assert (os.environ.get('SOAPY_SDR_PLUGIN_PATH', '').endswith('modules0.8-3') or\n"
    "                (os.environ.get('SOAPY_SDR_ROOT') and os.environ.get('SOAPY_SDR_PLUGIN_PATH') == '')), 'no plugin path'\n"
    "        assert int(a.get('timeout', '0')) >= 1000000, 'no timeout: the plugin default is 300 ms'\n"
    "        self.ip = a['remote'].split('//')[1].split(':')[0]\n"
    "        slow = os.path.join(%r, 'slow_' + self.ip)\n"
    "        if os.path.exists(slow):\n"
    "            n = int(open(slow).read())\n"
    "            if n > 0:\n"
    "                open(slow, 'w').write(str(n - 1))\n"
    "                raise RuntimeError('SoapyRPCUnpacker::recv() TIMEOUT')\n"
    "    def getHardwareInfo(self): return json.load(open(%r))[self.ip]\n"
    "    def readSetting(self, k):\n"
    "        if k == 'CLOCK_ADJ': return json.load(open(%r))[self.ip]\n"
    "        assert k == 'EGRESS_STATUS', k\n"
    "        v = json.load(open(%r)).get(self.ip)\n"
    "        if v is None: raise RuntimeError('unknown key')\n"
    "        return v\n"
    "    @staticmethod\n"
    "    def unmake(d): open(%r, 'a').write(d.ip + '\\n')\n"
    # as the real binding: close() flags the object, __del__ closes; a direct
    # unmake is not flagged, so __del__ unmakes the freed device again
    "    def close(self):\n"
    "        try: getattr(self, '__closed__')\n"
    "        except AttributeError: Device.unmake(self)\n"
    "        setattr(self, '__closed__', True)\n"
    "    def __del__(self): self.close()\n"
    % (root, info_file, os.path.join(root, "clock.json"), egress_file, os.path.join(root, "unmade")))
HEALTHY = "drop=p0:0,p1:0,p2:0,p3:0;stall_seen=0,stall_evt=0;marked=0"
json.dump({"127.0.0.1": HEALTHY, "127.0.0.2": HEALTHY}, open(egress_file, "w"))
clock_file = os.path.join(root, "clock.json")
def clock_adj(dac, cal=408):
    return "holdover=1 man_dac=%d rb_dac=%d pll1_locked=1 ref=calibrated cal_dac=%d offset=%d" % (dac, dac, cal, dac - cal)
json.dump({"127.0.0.1": clock_adj(404, 404), "127.0.0.2": clock_adj(408)}, open(clock_file, "w"))
same = {k: "v1" for k in ("fpga_version", "fpga_commit", "fpga_board", "device_version",
                          "device_build", "host_version", "host_build")}
json.dump({"127.0.0.1": same, "127.0.0.2": same}, open(info_file, "w"))

env = dict(os.environ, PYTHONPATH=fake)
for k in ("SOAPY_SDR_PLUGIN_PATH", "LD_LIBRARY_PATH", "VIRTUAL_ENV"):  # the checker must set them itself
    env.pop(k, None)
open(os.path.join(sd, "files", "topo-other.json"), "w").write(
    '{"BaseStations": {"BS0": {"sdr": ["127.0.0.9"]}}, "Clients": {"sdr": ["127.0.0.8"]}}')
json.dump({"serial_file": "files/topo-other.json"}, open(os.path.join(sd, "files", "houdini-other.json"), "w"))
fproc = os.path.join(root, "proc"); os.makedirs(fproc)
def fake_sounder(pid, argv, cwd):
    """A process named `sounder` in the test's process table, as /proc shows one."""
    d = os.path.join(fproc, str(pid)); os.makedirs(d)
    open(os.path.join(d, "comm"), "w").write("sounder\n")
    open(os.path.join(d, "cmdline"), "wb").write(b"\0".join(a.encode() for a in argv) + b"\0")
    os.symlink(cwd, os.path.join(d, "cwd"))
    return d
def run(*extra, conf="files/houdini-x.json"):
    out = subprocess.run([sys.executable, os.path.abspath("check_setup.py"), "--sounder-dir", sd,
                          "--venv", venv, "--examples", ex, "--conf", conf, "--json", "--proc-root", fproc]
                         + list(extra),
                         env=env, stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=120)
    rep = json.loads(out.stdout)
    return out.returncode, rep, {r["what"]: r["level"] for r in rep["results"]}

rc, rep, lv = run()
check(rc == 0 and rep["ok"], "a ready host passes (rc 0)")
check(lv.get("stack match") == "PASS" and lv.get("server 127.0.0.2") == "PASS", "full form: servers answer and stacks match")
check(sorted(open(os.path.join(root, "unmade")).read().split()) == ["127.0.0.1", "127.0.0.2"],
      "each radio opened for its stack is closed exactly once, not dropped at exit (fails on a direct Device.unmake: the binding's __del__ repeats it)")
check(all(l != "WARN" for l in lv.values()), "no warnings on a ready host: %s" % lv)
# Fails under: never reading CLOCK_ADJ (both then FAIL 'read back empty').
check(all(r["level"] == "INFO" and "offset 0" in r["detail"] for r in rep["results"]
          if r["what"] in ("clock 127.0.0.1", "clock 127.0.0.2"))
      and lv.get("clock 127.0.0.1") == "INFO" and lv.get("clock 127.0.0.2") == "INFO",
      "full form: each radio's clock steering offset is read and reported (0 on both)")
rc, rep, lv = run("--quick")
check(rc == 0 and "stack match" not in lv, "--quick does not open the radios")
# A node left steered: a WARN naming it, with the release as the fix; the run
# can still go (rc 0). Fails under: dropping the offset check (an INFO).
json.dump({"127.0.0.1": clock_adj(404, 404), "127.0.0.2": clock_adj(411)}, open(clock_file, "w"))
rc, rep, lv = run()
fix = [r for r in rep["results"] if r["what"] == "clock 127.0.0.2"][0]
check(rc == 0 and lv.get("clock 127.0.0.2") == "WARN" and "+3" in fix["detail"] and "release" in fix["fix"]
      and lv.get("clock 127.0.0.1") == "INFO", "a node left steered +3 counts is a WARN naming it, with the release")
json.dump({"127.0.0.1": clock_adj(404, 404), "127.0.0.2": "holdover=0 man_dac=0 rb_dac=0 pll1_locked=1 ref=internal "
           "cal_dac=none offset=none"}, open(clock_file, "w"))
rc, rep, lv = run()
check(rc == 0 and lv.get("clock 127.0.0.2") == "INFO", "a node on ref=internal has no offset to report (INFO)")
# Fails under: reporting a calibrated node out of its hold as INFO (the old rule).
json.dump({"127.0.0.1": clock_adj(404, 404), "127.0.0.2": "holdover=0 man_dac=408 rb_dac=433 pll1_locked=1 "
           "ref=calibrated cal_dac=408 offset=none"}, open(clock_file, "w"))
rc, rep, lv = run()
fix = [r for r in rep["results"] if r["what"] == "clock 127.0.0.2"][0]
check(rc == 0 and lv.get("clock 127.0.0.2") == "WARN" and "release" in fix["fix"],
      "a calibrated node whose hold is not in force is a WARN, with the release")
json.dump({"127.0.0.1": clock_adj(404, 404), "127.0.0.2": clock_adj(408)}, open(clock_file, "w"))

# Each broken piece, one at a time, is a FAIL (or the stated WARN) under its own name.
json.dump({"127.0.0.1": same, "127.0.0.2": dict(same, fpga_commit="v2")}, open(info_file, "w"))
rc, rep, lv = run(); check(rc == 1 and lv["stack match"] == "FAIL", "a node on a different gateware fails 'stack match'")
check("fpga_commit" in [r for r in rep["results"] if r["what"] == "stack match"][0]["detail"], "and names the differing key")
json.dump({"127.0.0.1": same, "127.0.0.2": same}, open(info_file, "w"))
# The egress stall (a node wedged when the host's data port bounced). Each
# assertion names the mutation that breaks it.
rc, rep, lv = run()
check(lv.get("egress 127.0.0.1") == "PASS" and lv.get("egress 127.0.0.2") == "PASS",
      "healthy egress passes on each node (breaks if check_egress is not called)")
json.dump({"127.0.0.1": HEALTHY, "127.0.0.2": "drop=p0:255,p1:0,p2:0,p3:0;stall_seen=1,stall_evt=255;marked=0"},
          open(egress_file, "w"))
rc, rep, lv = run()
check(rc == 1 and lv["egress 127.0.0.2"] == "FAIL" and lv["egress 127.0.0.1"] == "PASS",
      "a sticky stall fails that node only (breaks if the stall branch is a WARN or keyed to the wrong node)")
json.dump({"127.0.0.1": HEALTHY, "127.0.0.2": "drop=p0:17,p1:0,p2:0,p3:0;stall_seen=0,stall_evt=0;marked=0"},
          open(egress_file, "w"))
rc, rep, lv = run()
check(rc == 0 and lv["egress 127.0.0.2"] == "PASS", "drops without the stall bit pass (breaks if any nonzero count fails)")
# Saturated at 255 (only an egress reset clears them): a WARN naming the port,
# since the run's link health is then blind to new drops; not a FAIL.
json.dump({"127.0.0.1": HEALTHY, "127.0.0.2": "drop=p0:255,p1:0,p2:0,p3:0;stall_seen=0,stall_evt=0;marked=0"},
          open(egress_file, "w"))
rc, rep, lv = run()
det = [r["detail"] for r in rep["results"] if r["what"] == "egress 127.0.0.2"]
check(rc == 0 and lv["egress 127.0.0.2"] == "WARN" and det and "drop p0" in det[0] and lv["egress 127.0.0.1"] == "PASS",
      "saturated egress drop counters are a WARN naming the port (mutation: only the stall bit read)")
# The plugin from the --soapy-root prefix: the venv without a Houdini module
# (the demo rig's, since the release prefixes carry it) and the prefix with one.
vmod = os.path.join(venv, "lib", "SoapySDR", "modules0.8-3", "libHoudiniSDRSupport.so")
rel = os.path.join(root, "rel"); os.makedirs(os.path.join(rel, "lib", "SoapySDR", "modules0.8-3"))
open(os.path.join(rel, "lib", "SoapySDR", "modules0.8-3", "libHoudiniSDRSupport.so"), "w").close()
os.rename(vmod, vmod + ".off")
rc, rep, lv = run("--quick", "--soapy-root", rel)
check(lv.get("plugin") == "PASS", "the plugin is found in the --soapy-root prefix when the venv has none "
      "(mutation: only the venv's module dir searched)")
rc, rep, lv = run("--quick")
det = [r["fix"] for r in rep["results"] if r["what"] == "plugin"]
check(rc == 1 and lv.get("plugin") == "FAIL" and det and "--soapy-root" in det[0],
      "no module in the venv and no --soapy-root fails, naming the option (mutation: the old advice)")
# The full check under --soapy-root with the venv's module still absent
# (the rig's layout): the radios are read through that prefix, and a steered
# node's release line carries it.
json.dump({"127.0.0.1": clock_adj(404, 404), "127.0.0.2": clock_adj(411)}, open(clock_file, "w"))
rc, rep, lv = run("--soapy-root", rel)
os.rename(vmod + ".off", vmod)
fix = [r["fix"] for r in rep["results"] if r["what"] == "clock 127.0.0.2"]
check(lv.get("stack match") == "PASS" and fix and fix[0].startswith("Release it before the run: SOAPY_SDR_ROOT=%s SOAPY_SDR_PLUGIN_PATH= python3" % rel),
      "under --soapy-root the full check reads the radios and the printed release carries the prefix "
      "(mutation: a bare python3 line, which finds no Houdini module on the rig): %s" % (fix[:1],))
json.dump({"127.0.0.1": clock_adj(404, 404), "127.0.0.2": clock_adj(408)}, open(clock_file, "w"))
# A host plugin that is not the radios' device build: a WARN per node.
json.dump({"127.0.0.1": dict(same, host_build="v0"), "127.0.0.2": dict(same, host_build="v0")}, open(info_file, "w"))
json.dump({"127.0.0.1": HEALTHY, "127.0.0.2": HEALTHY}, open(egress_file, "w"))
rc, rep, lv = run()
check(rc == 0 and lv.get("stack match") == "PASS" and lv.get("plugin build 127.0.0.1") == "WARN"
      and lv.get("plugin build 127.0.0.2") == "WARN",
      "a host plugin that is not the radios' device build is a WARN per node, not a failure (mutation: builds not "
      "compared): %s" % lv)
# Both nodes' host plugins a different release than their device modules: the
# nodes agree (stack match), and each one FAILs the driver's release lockstep.
json.dump({"127.0.0.1": dict(same, host_version="v2"), "127.0.0.2": dict(same, host_version="v2")}, open(info_file, "w"))
rc, rep, lv = run()
check(rc == 1 and lv.get("stack match") == "PASS" and lv.get("lockstep 127.0.0.1") == "FAIL"
      and lv.get("lockstep 127.0.0.2") == "FAIL",
      "a host release that is not the radio's device release FAILs each node (mutation: the lockstep check "
      "removed, the nodes agreeing passes): %s" % lv)
json.dump({"127.0.0.1": same, "127.0.0.2": same}, open(info_file, "w"))
# A slow first open (DEMO_VERIFICATION 9.83: 3.34 s against the 3 s timeout):
# one timeout is tried again and passes; one that persists names the slow
# open, not the venv.
slow = os.path.join(root, "slow_127.0.0.2")
open(slow, "w").write("1"); rc, rep, lv = run()
check(rc == 0 and lv.get("stack match") == "PASS",
      "one timed-out hardware-info read is tried again and the check passes (mutation: no retry): %s" % lv)
open(slow, "w").write("9"); rc, rep, lv = run()
fix = [r["fix"] for r in rep["results"] if r["what"] == "stack 127.0.0.2"]
check(rc == 1 and lv.get("stack 127.0.0.2") == "FAIL" and fix and "run this check again" in fix[0]
      and "venv" not in fix[0],
      "a timeout that persists FAILs with the slow-open advice, not the venv's (mutation: the old advice): %s" % fix)
os.remove(slow)
# The device's single marked=N count.
json.dump({"127.0.0.1": "drop=p0:0,p1:0,p2:0,p3:0;stall_seen=0,stall_evt=0;marked=1",
           "127.0.0.2": "drop=p0:0,p1:0,p2:0,p3:0;stall_seen=0,stall_evt=0;marked=255"}, open(egress_file, "w"))
rc, rep, lv = run()
det = [r["detail"] for r in rep["results"] if r["what"] == "egress 127.0.0.2"]
check(rc == 0 and lv["egress 127.0.0.1"] == "PASS" and lv["egress 127.0.0.2"] == "WARN" and det and "marked" in det[0],
      "a saturated single marked=N counter is a WARN, an unsaturated one passes (mutation: only per-port items read)")
json.dump({"127.0.0.1": "drop=p0:0,p1:0,p2:0,p3:0;stall_seen=0,stall_evt=0;marked=0",
           "127.0.0.2": "drop=p0:3,p1:2,p2:0,p3:255;stall_seen=1,stall_evt=7;marked=5"}, open(egress_file, "w"))
rc, rep, lv = run()
check(rc == 1 and lv["egress 127.0.0.1"] == "PASS" and lv["egress 127.0.0.2"] == "FAIL",
      "the device's own strings (its lifecycle test's golden lines): clean passes, a recorded stall fails (mutation: "
      "stall_seen not read)")
json.dump({"127.0.0.1": HEALTHY}, open(egress_file, "w"))
rc, rep, lv = run()
check(rc == 1 and lv.get("stack 127.0.0.2") == "FAIL" and "egress 127.0.0.2" not in lv and lv["egress 127.0.0.1"] == "PASS",
      "an EGRESS_STATUS read that throws fails that node's stack read (mutation: the old fallback, a WARN): %s" % lv)
json.dump({"127.0.0.1": HEALTHY, "127.0.0.2": HEALTHY}, open(egress_file, "w"))
json.dump({"127.0.0.1": same, "127.0.0.2": {k: v for k, v in same.items() if k != "host_version"}}, open(info_file, "w"))
rc, rep, lv = run()
det = [r["detail"] for r in rep["results"] if r["what"] == "stack 127.0.0.2"]
check(rc == 1 and lv.get("stack 127.0.0.2") == "FAIL" and det and "host_version" in det[0] and "stack match" not in lv
      and "lockstep 127.0.0.2" not in lv,
      "a hardware info that lacks a stack key FAILs that node, naming the key (mutation: an absent key tolerated): %s" % lv)
json.dump({"127.0.0.1": same, "127.0.0.2": same}, open(info_file, "w"))
rc, rep, lv = run(conf="files/houdini-bad.json"); check(rc == 1 and lv["config"] == "FAIL", "a config that is not JSON fails 'config'")
rc, rep, lv = run(conf="files/none.json"); check(rc == 1 and lv["config"] == "FAIL", "a missing config fails 'config'")
os.utime(os.path.join(sd, "a.cc"), None); os.utime(exe, (time.time() - 60, time.time() - 60))
rc, rep, lv = run("--quick"); check(rc == 0 and lv["build"] == "WARN", "a source newer than the binary is a WARN, not a FAIL")
os.utime(exe, None)
os.makedirs(os.path.join(sd, "sync")); open(os.path.join(sd, "sync", "det.cc"), "w").close()
os.utime(exe, (time.time() - 60, time.time() - 60))
os.utime(os.path.join(sd, "a.cc"), (time.time() - 120, time.time() - 120))  # only sync/det.cc is newer
rc, rep, lv = run("--quick"); check(lv["build"] == "WARN" and "sync" in [r for r in rep["results"] if r["what"] == "build"][0]["detail"],
                                    "a newer source in a subdirectory (sync/) is seen too")
os.utime(exe, None)
cfg = json.load(open(os.path.join(sd, "files", "houdini-x.json")))
json.dump(dict(cfg, remote_port=int(cfg["remote_port"])), open(os.path.join(sd, "files", "houdini-int.json"), "w"))
rc, rep, lv = run("--quick", conf="files/houdini-int.json")
check(rc == 1 and lv["config"] == "FAIL", "a numeric remote_port fails 'config' (the sounder would throw)")
json.dump(dict(cfg, remote_port="abc"), open(os.path.join(sd, "files", "houdini-abc.json"), "w"))
rc, rep, lv = run("--quick", conf="files/houdini-abc.json")
check(rc == 1 and lv["config"] == "FAIL", "a non-numeric remote_port is a FAIL, not a traceback")
open(os.path.join(sd, "files", "topo-list.json"), "w").write('["127.0.0.1"]')
json.dump(dict(cfg, serial_file="files/topo-list.json"), open(os.path.join(sd, "files", "houdini-tl.json"), "w"))
rc, rep, lv = run("--quick", conf="files/houdini-tl.json")
check(rc == 1 and lv.get("topology") == "FAIL", "a topology of the wrong shape is a FAIL, not a traceback")
for bad, what in (({"BaseStations": {"cell0": {"sdr": ["127.0.0.1"]}}, "Clients": {"sdr": ["127.0.0.2"]}}, "a cell not named BS0"),
                  ({"BaseStations": {"BS0": {"sdr": ["127.0.0.1"]}}, "Clients": ["127.0.0.2"]}, "a bare client list")):
    json.dump(bad, open(os.path.join(sd, "files", "topo-bad.json"), "w"))
    json.dump(dict(cfg, serial_file="files/topo-bad.json"), open(os.path.join(sd, "files", "houdini-tb.json"), "w"))
    rc, rep, lv = run("--quick", conf="files/houdini-tb.json")
    check(rc == 1 and lv.get("topology") == "FAIL", "%s (the sounder would not open it) fails 'topology'" % what)
os.rename(exe, exe + ".x"); rc, rep, lv = run("--quick"); check(rc == 1 and lv["build"] == "FAIL", "no binary fails 'build'"); os.rename(exe + ".x", exe)
open(util, "w").write("#!/bin/sh\necho 'Available factories... remote'\n")
rc, rep, lv = run("--quick"); check(rc == 1 and lv["plugin"] == "FAIL", "SoapySDR not loading the Houdini module fails 'plugin'")
open(util, "w").write("#!/bin/sh\necho 'Available factories... houdinisdr, remote'\n")
rc, rep, lv = run("--quick", "--examples", root)
check(rc == 0 and lv["teardown"] == "WARN", "missing host examples is a WARN (mutation: --examples ignored)")
# A sounder run from the sounder directory with a --conf_file, as the real one is.
held = fake_sounder(4242, ["./build/sounder", "--conf_file", "files/houdini-x.json"], sd)
rc, rep, lv = run("--quick"); check(rc == 1 and lv["radios free"] == "FAIL", "a sounder on the same radios fails 'radios free'")
fix = [r for r in rep["results"] if r["what"] == "radios free"][0]["fix"]
check("kill -INT 4242" in fix and "rig_release_holders" not in fix,
      "its fix names that pid, not the tool that kills every sounder and dashboard")
before = open(os.path.join(root, "unmade")).read()
rc, rep, lv = run()
check(rc == 1 and "stack match" not in lv and lv.get("stack") == "INFO"
      and open(os.path.join(root, "unmade")).read() == before,
      "full form with a sounder on these radios opens no radio (mutation: read the stacks whenever the servers answer)")
shutil.rmtree(held)
other = fake_sounder(4243, ["./build/sounder", "--conf_file=files/houdini-other.json"], sd)
rc, rep, lv = run(); check(rc == 0 and lv["radios free"] == "PASS" and lv.get("stack match") == "PASS",
                           "a sounder on other radios does not block, and the stacks are read (mutation: any sounder blocks)")
shutil.rmtree(other)
# Its --topology overrides its config's serial_file, read as gflags reads it:
# either spelling, the last occurrence winning, nothing after a bare '--'.
for argv in (["--conf_file=files/houdini-other.json", "--topology", "files/topo.json"],
             ["-conf_file", "files/houdini-other.json", "-topology=files/topo.json"],
             ["--conf_file=files/houdini-other.json", "--topology", "files/topo-other.json",
              "--topology=files/topo.json"],
             ["--conf_file=files/houdini-x.json", "--", "--topology=files/topo-other.json"]):
    over = fake_sounder(4245, ["./build/sounder"] + argv, sd)
    rc, rep, lv = run("--quick")
    check(rc == 1 and lv["radios free"] == "FAIL",
          "a sounder whose topology names these radios holds them: %s (mutation: the override ignored, read in "
          "one spelling only, the first occurrence taken, or read past '--')" % " ".join(argv))
    shutil.rmtree(over)
unknown = fake_sounder(4244, ["./build/sounder"], sd)  # no --conf_file: its radios cannot be read
before = open(os.path.join(root, "unmade")).read()
rc, rep, lv = run()
check(rc == 0 and lv["radios free"] == "WARN" and "stack match" not in lv
      and open(os.path.join(root, "unmade")).read() == before,
      "a sounder whose radios cannot be read is a WARN, and the full form opens no radio then either "
      "(mutation: skip the stack read only when held)")
shutil.rmtree(unknown)
srv.shutdown(socket.SHUT_RDWR); srv.close()
rc, rep, lv = run(); check(rc == 1 and lv["server 127.0.0.1"] == "FAIL" and "stack match" not in lv,
                          "a server that does not answer fails, and the stack read is skipped")
# plugin_env, the environment the dashboard's Start runs the sounder in, takes
# the --soapy-root prefix exactly as run_rung.sh does.
import check_setup
orig_root = os.environ.get("SOAPY_SDR_ROOT")
e0 = check_setup.plugin_env("/v")
e1 = check_setup.plugin_env("/v", "/slots")
check(e0["SOAPY_SDR_PLUGIN_PATH"] == "/v/lib/SoapySDR/modules0.8-3" and e0.get("SOAPY_SDR_ROOT") == orig_root,
      "without a root the venv's plugin loads (mutation: the root applied always)")
check(e1.get("SOAPY_SDR_ROOT") == "/slots" and e1.get("SOAPY_SDR_PLUGIN_PATH") == "",
      "with a root that prefix loads, not the venv's (mutation: the root ignored, so the dashboard runs a slots "
      "config on the default plugin; or the venv's module path left set, which SoapySDR searches too)")
check(e1.get("PATH") == os.environ.get("PATH"),
      "the operator's PATH rides along (mutation: the environment built from scratch)")
# The teardown's radios, by hand: --node, else --topology, else the topology
# the config names, else a refusal (a default topology names one bench only).
import teardown_framer as tf
cfx, topo = os.path.join(sd, "files", "houdini-x.json"), os.path.join(sd, "files", "topo.json")
check(tf.resolve_nodes(None, None, cfx, sd) == (["127.0.0.1", "127.0.0.2"], None),
      "--conf tears down the radios its serial_file names, relative to the checkout (mutation: ignore --conf, "
      "or resolve serial_file against the cwd)")
nodes, err = tf.resolve_nodes(None, None, None, sd)
check(nodes == [] and err and "--conf" in err,
      "with no --node, --topology or --conf the teardown refuses (mutation: fall back to a default topology)")
check(tf.resolve_nodes(["10.9.9.9"], topo, cfx, sd) == (["10.9.9.9"], None)
      and tf.resolve_nodes(None, topo, os.path.join(sd, "files", "houdini-bad.json"), sd)[0] == ["127.0.0.1", "127.0.0.2"],
      "--node wins over --topology and --conf, and --topology over --conf (mutation: --conf read first)")
nodes, err = tf.resolve_nodes(None, None, os.path.join(sd, "files", "houdini-bad.json"), sd)
check(nodes == [] and "cannot read the config" in err, "an unreadable config is refused, not a traceback")
import shutil; shutil.rmtree(root, ignore_errors=True)  # no temp dir left per run
print("%d failure(s)" % fails); sys.exit(1 if fails else 0)
