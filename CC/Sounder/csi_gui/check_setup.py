#!/usr/bin/env python3
"""Check that this host is ready to run the demo, and say how to fix what is not.

Run it before the first start on a new host, and whenever a start fails:

    python3 csi_gui/check_setup.py --conf files/houdini-dualband.json
    python3 csi_gui/check_setup.py --conf <config> --quick   # skip opening the radios

Each line is PASS, WARN or FAIL, and every WARN or FAIL says what to do. The
exit status is non-zero when anything FAILs. The dashboard (`--control`) runs
the quick form before every Start and the full form from its Check button.

What it checks, in order:

  1. the config parses, and its topology file names a base station and a client;
  2. the sounder binary exists (and is not older than its sources);
  3. the host plugin: the venv, the Houdini module, and SoapySDR loading it;
  4. the SoapyHoudiniSDR host examples the framer teardown imports;
  5. no other sounder on this host holds these radios (one on other radios is named);
  6. each radio's server answers on the config's remote port;
  7. (full form only) each radio's stack: gateware, firmware and plugin
     versions, which must agree between the nodes; each radio's data egress,
     which must not have stalled (EGRESS_STATUS stall_seen); and each radio's
     clock steering offset (CLOCK_ADJ), which should be 0.

Checks 1 to 6 touch no radio. Check 7 opens each radio and reads its hardware
info, EGRESS_STATUS and CLOCK_ADJ, as the sounder does at startup, and changes
nothing. Do not run the full
form against radios someone else is using.

Run it with the venv's python (or with the venv activated): check 7 imports
SoapySDR from there.
"""
import argparse
import glob
import json
import os
import re
import shlex
import socket
import subprocess
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_SOUNDER = os.path.dirname(_HERE)
sys.path.insert(0, _HERE)
from teardown_framer import DEFAULT_EXAMPLES, roles_from_topology  # noqa: E402  one reader, not two

# The keys every node in one run must agree on: the sounder's own list
# (include/node_version.h, kMustMatch), so the two cannot disagree about a bench.
MUST_MATCH = ("fpga_version", "fpga_commit", "fpga_board", "device_version",
              "device_build", "host_version", "host_build")


class Report:
    def __init__(self):
        self.lines = []

    def add(self, level, what, detail="", fix=""):
        self.lines.append({"level": level, "what": what, "detail": detail, "fix": fix})

    def failed(self):
        return any(r["level"] == "FAIL" for r in self.lines)


def check_config(rep, sd, conf, topology=None):
    """Returns (config dict, [base station ips], [client ips]) or (None, [], [])."""
    path = conf if os.path.isabs(conf) else os.path.join(sd, conf)
    try:
        with open(path, encoding="utf-8") as f:
            cfg = json.load(f)
    except OSError as e:
        rep.add("FAIL", "config", "%s: %s" % (conf, e.strerror),
                "Give --conf a config under %s/files/ (paths are relative to the sounder directory)." % sd)
        return None, [], []
    except ValueError as e:
        rep.add("FAIL", "config", "%s is not valid JSON: %s" % (conf, e),
                "Fix the JSON syntax at the line and column shown.")
        return None, [], []
    topo = topology or cfg.get("serial_file")
    if not topo:
        rep.add("FAIL", "config", "%s names no topology (serial_file)" % conf,
                "Add \"serial_file\": \"files/topology-<name>.json\" to the config.")
        return cfg, [], []
    tpath = topo if os.path.isabs(topo) else os.path.join(sd, topo)
    try:
        with open(tpath, encoding="utf-8") as f:
            t = json.load(f)
        bs, ue = roles_from_topology(t)
    except (OSError, ValueError, AttributeError, TypeError) as e:  # not JSON, or not the topology shape
        rep.add("FAIL", "topology", "%s: %s" % (topo, e),
                "Create %s with your radios' addresses (walkthrough section 2.6)." % topo)
        return cfg, [], []
    # The sounder reads the cells as BS0, BS1, ... in order and the clients as
    # {"sdr": [...]} (config.cc), stricter than the teardown's tolerant reader:
    # another key is silently never opened, and a bare client list throws.
    cells = t.get("BaseStations")
    if not isinstance(cells, dict) or set(cells) != {"BS%d" % i for i in range(len(cells))} \
            or not isinstance(t.get("Clients"), dict):
        rep.add("FAIL", "topology", "%s is not in the sounder's shape" % topo,
                "Use {\"BaseStations\": {\"BS0\": {\"sdr\": [\"<bs-ip>\"]}}, "
                "\"Clients\": {\"sdr\": [\"<ue-ip>\"]}} (walkthrough section 2.6).")
        return cfg, [], []
    if not bs or not ue:
        rep.add("FAIL", "topology", "%s lists base station %s, client %s" % (topo, bs or "none", ue or "none"),
                "The demo needs one base station and one client address in %s." % topo)
        return cfg, bs, ue
    desc = cfg.get("_description", "")
    rep.add("PASS", "config", "%s%s; base station %s, client %s"
            % (conf, (" (" + desc + ")") if desc else "", ", ".join(bs), ", ".join(ue)))
    return cfg, bs, ue


def check_build(rep, sd):
    exe = os.path.join(sd, "build", "sounder")
    if not os.access(exe, os.X_OK):
        rep.add("FAIL", "build", "%s not found" % exe,
                "Build it: cd %s && cmake -B build -DCMAKE_BUILD_TYPE=Release "
                "-DSoapySDR_DIR=<prefix>/share/cmake/SoapySDR && cmake --build build -j "
                "(walkthrough section 2.5)." % sd)
        return
    srcs = []
    for dirpath, dirs, files in os.walk(sd):
        # the build tree, the vendored FFT library and the radio-free tests are not the sounder
        dirs[:] = [d for d in dirs if d not in ("build", "mufft", "tests", "csi_gui", ".git")]
        srcs += [os.path.join(dirpath, f) for f in files if f.endswith((".cc", ".cpp", ".h", ".hpp"))]
    newest = max(srcs, key=os.path.getmtime) if srcs else None
    if newest and os.path.getmtime(newest) > os.path.getmtime(exe):
        rep.add("WARN", "build", "%s is newer than build/sounder" % os.path.relpath(newest, sd),
                "Rebuild (cmake --build build -j) unless you meant to run the older binary.")
    else:
        rep.add("PASS", "build", "build/sounder present and up to date")


def plugin_dir(venv):
    """Where the venv's SoapySDR looks for modules (its ABI version, 0.8-3)."""
    return os.path.join(venv, "lib", "SoapySDR", "modules0.8-3")


def plugin_env(venv, root=None):
    """The environment that loads the Houdini plugin; csi_server.py runs the sounder in it.

    `root` (--soapy-root) is the release's host-plugin prefix (built with the
    radios' device build; the venv then carries no Houdini module): SoapySDR
    then searches only that prefix (SOAPY_SDR_ROOT, the loader's own variable,
    with its plugin path emptied), exactly as tests/demo-verify/run_rung.sh
    does, so the dashboard's Check and Start run the stack a scripted run
    validated.
    """
    env = dict(os.environ, LD_LIBRARY_PATH=os.path.join(venv, "lib"), SOAPY_SDR_PLUGIN_PATH=plugin_dir(venv))
    if root:
        env.update(SOAPY_SDR_ROOT=root, SOAPY_SDR_PLUGIN_PATH="")
    return env


def check_plugin(rep, venv, root=None):
    # The Houdini module comes from the --soapy-root prefix when one is given
    # (plugin_env loads it from there), else from the venv's own module dir.
    moddir = plugin_dir(root or venv)
    if not os.path.isdir(venv):
        rep.add("FAIL", "plugin", "venv %s not found" % venv,
                "Pass the SoapySDR venv as --venv (walkthrough section 2.4); the Houdini plugin's own prefix goes "
                "in --soapy-root, not here.")
        return
    mods = [m for m in glob.glob(os.path.join(moddir, "*.so")) if "houdini" in os.path.basename(m).lower()]
    if not mods:
        rep.add("FAIL", "plugin", "no Houdini module in %s%s" % (moddir, " (--soapy-root)" if root else ""),
                "Pass --soapy-root <the host-plugin prefix of the radios' release> before the check and the "
                "dashboard (walkthrough section 2.4; the demo rig's is in DEMO_BENCH_RUNBOOK A3).")
        return
    util = os.path.join(venv, "bin", "SoapySDRUtil")
    env = plugin_env(venv, root)
    try:
        out = subprocess.run([util, "--info"], env=env, timeout=30, stdout=subprocess.PIPE,
                             stderr=subprocess.STDOUT).stdout.decode("utf-8", "replace")
    except (OSError, subprocess.TimeoutExpired) as e:
        rep.add("FAIL", "plugin", "SoapySDRUtil --info did not run (%s)" % e,
                "Check the SoapySDR install in %s." % venv)
        return
    fac = [l for l in out.splitlines() if l.startswith("Available factories")]
    if not fac or "houdinisdr" not in fac[0]:
        rep.add("FAIL", "plugin", "SoapySDR does not load the Houdini module (%s)"
                % (fac[0] if fac else "no factory list"),
                "Run the SoapySDRUtil --info line from walkthrough section 2.4 and read the module error.")
        return
    rep.add("PASS", "plugin", "%s loads (%s)" % (os.path.basename(mods[0]), fac[0].split("...")[-1].strip()))


def check_examples(rep, ex):
    # `ex`: where the teardown will look (--examples)
    if os.path.isfile(os.path.join(ex, "houdini_setup.py")):
        rep.add("PASS", "teardown", "houdini_setup found in %s" % ex)
    else:
        rep.add("WARN", "teardown", "houdini_setup.py not in %s" % ex,
                "The framer teardown before each start needs it: pass --examples "
                "<path-to-SoapyHoudiniSDR>/host/examples (walkthrough section 2.4).")


PROC = "/proc"  # the process table read below; --proc-root points the test at its own
HWINFO_ATTEMPTS = 3  # tries of a radio's hardware-info read that timed out


def running_sounders():
    """Local processes named `sounder`. By /proc comm, so this never matches itself."""
    found = []
    for e in os.listdir(PROC):
        if not e.isdigit():
            continue
        try:
            with open(os.path.join(PROC, e, "comm")) as f:
                if f.read().strip() == "sounder":
                    found.append(int(e))
        except OSError:
            pass
    return found


def gflag(argv, name):
    """The value gflags gives flag `name` on this command line: '-name' and
    '--name' alike, '=value' or the next word, the last one winning, nothing
    after a bare '--'."""
    val, i = None, 1
    while i < len(argv):
        a = argv[i]
        if a == "--":
            break
        a = "-" + a if a.startswith("-") and not a.startswith("--") else a
        if a == "--" + name and i + 1 < len(argv):
            val, i = argv[i + 1], i + 2
            continue
        if a.startswith("--" + name + "="):
            val = a.split("=", 1)[1]
        i += 1
    return val


def radios_of(pid):
    """The radio addresses a running sounder uses, from its --conf_file and its
    working directory, or None when that cannot be read (another user's process)."""
    try:
        with open(os.path.join(PROC, str(pid), "cmdline"), "rb") as f:
            argv = f.read().decode("utf-8", "replace").split("\0")
        cwd = os.readlink(os.path.join(PROC, str(pid), "cwd"))
        conf, topo = gflag(argv, "conf_file"), gflag(argv, "topology")
        if not topo:  # the sounder's --topology overrides its config's serial_file
            with open(os.path.join(cwd, conf), encoding="utf-8") as f:
                topo = json.load(f)["serial_file"]
        with open(os.path.join(cwd, topo), encoding="utf-8") as f:
            bs, ue = roles_from_topology(json.load(f))
        return set(bs + ue)
    except (OSError, ValueError, KeyError, TypeError):
        return None


def check_no_sounder(rep, nodes):
    """A sounder on the same radios holds them; one on other radios (a shared
    host, another bench) does not block this run. True when a sounder holds
    these radios or might (its radios could not be read): the full form must
    not open them then, or it disturbs that run."""
    held, unknown, other = [], [], []
    for pid in running_sounders():
        r = radios_of(pid)
        if r is None:
            unknown.append(pid)
        elif r & set(nodes):
            held.append(pid)
        else:
            other.append(pid)
    if held:
        rep.add("FAIL", "radios free", "a sounder on this host is using these radios (pid %s)"
                % ", ".join(map(str, held)),
                # Name the pids: tools/rig_release_holders.py would also kill this
                # dashboard and every sounder on the host, including other benches'.
                "Stop the run that started it, or end it with: kill -INT %s (the sounder's own stop; a plain kill "
                "leaves its clock steered)" % " ".join(map(str, held)))
    elif unknown:
        rep.add("WARN", "radios free", "a sounder is running (pid %s) and its radios could not be read"
                % ", ".join(map(str, unknown)),
                "If it uses these radios, stop it first: the start will fail with the radios held.")
    else:
        rep.add("PASS", "radios free", "no other sounder on these radios"
                + (" (pid %s runs on other radios)" % ", ".join(map(str, other)) if other else ""))
    return bool(held or unknown)


def check_servers(rep, nodes, port):
    # A bare connect, so the quick form opens no radio. The server logs one
    # "handlerLoop() FAIL: recv(header)" per probe (a connection that sends no
    # RPC header); known and harmless.
    ok = []
    for ip in nodes:
        try:
            with socket.create_connection((ip, int(port)), timeout=3):
                pass
            rep.add("PASS", "server %s" % ip, "answers on port %s" % port)
            ok.append(ip)
        except OSError as e:
            rep.add("FAIL", "server %s" % ip, "no answer on port %s (%s)" % (port, e.strerror or e),
                    "Check the radio is powered and on the network (ping %s), and that its "
                    "SoapySDRServer is running (ssh to it: systemctl status SoapySDRServer)." % ip)
    return ok


def hwinfo(ip, port):
    """Read one radio's hardware info exactly as the sounder opens it (RadioHoudini.cc)."""
    import SoapySDR
    # The timeout as the sounder passes it (include/Radio.h). Without one the
    # plugin's default is 300 ms, which the first discovery in a process sleeps
    # in full by SoapyRemote's design, and the read then failed on the rig
    # although the radio was fine.
    sdr = SoapySDR.Device({"driver": "houdinisdr", "remote": "tcp://%s:%s" % (ip, port),
                           "remote:driver": "houdinisdr-device", "remote:type": "houdinisdr",
                           "timeout": "3000000"})
    try:
        # A read that throws fails this node's stack read (check_versions).
        info = dict(sdr.getHardwareInfo())
        info["egress_status"] = sdr.readSetting("EGRESS_STATUS")
        info["_clock_adj"] = str(sdr.readSetting("CLOCK_ADJ"))
        return info
    finally:
        # A clean close: a connection dropped at process exit leaves a
        # "handlerLoop() FAIL: recv(header)" in each radio's server journal.
        # close(), not Device.unmake(sdr): only close() flags the object, so the
        # binding's __del__ does not unmake the freed device a second time (in
        # one process that corrupted the heap on the next open: a bus error).
        sdr.close()


def check_egress(rep, ip, raw):
    # The stall bit is sticky (REGISTERS.md, EGRESS_STALL_WD): once the egress
    # merge sat in a frame without progress, as when the host's data port went
    # down under it, the radio sends nothing more over a link that is up, until
    # its gateware is reloaded. The per-port drop counts alone are not that.
    m = re.search(r"stall_seen=(\d+)", raw)
    if m is None:
        rep.add("FAIL", "egress %s" % ip, "EGRESS_STATUS has no stall_seen field (%s)" % (raw or "empty")[:120],
                "Run the check again; if it persists, the radio does not run the plugin's release (ask whoever "
                "maintains the boards).")
    elif int(m.group(1)):
        rep.add("FAIL", "egress %s" % ip, "the radio's data egress has stalled (%s); it will send no samples although its link is up" % raw,
                "Reload the radio's gateware (PL) or reboot the radio. A bounce of this host's data port (a host reboot, a cable pull) causes it.")
    else:
        # The per-port drop counters and the one marked-frame counter saturate
        # at 255 and only a node boot or a PL reload clears them: saturated,
        # the run's link health cannot see a new egress drop.
        drop = re.search(r"drop=([^;]*)", raw)
        marked = re.search(r"marked=(\d+)", raw)
        full = ["drop %s" % port for port, v in re.findall(r"(p\d+):(\d+)", drop.group(1) if drop else "")
                if int(v) >= 255]
        if marked and int(marked.group(1)) >= 255:
            full.append("marked")
        if full:
            rep.add("WARN", "egress %s" % ip, "no data-path stall recorded, but the egress counters %s are saturated "
                    "at 255, so a new egress drop in the run goes unseen" % ", ".join(full),
                    "The run is not blocked. A reboot of the radio, or a reload of its gateware (PL), clears them; ask "
                    "whoever maintains the boards, and say which run or test drove the drops.")
        else:
            rep.add("PASS", "egress %s" % ip, "no data-path stall recorded")


def release_cmd(ip, port, root=None):
    """The shell line that releases a node's clock into its calibrated hold, in
    the plugin environment this check ran with (the --soapy-root prefix)."""
    pre = "SOAPY_SDR_ROOT=%s SOAPY_SDR_PLUGIN_PATH= " % shlex.quote(root) if root else ""
    return (pre + "python3 -c \"import SoapySDR as S; d = S.Device({'driver': 'houdinisdr', 'remote': "
            "'tcp://%s:%s', 'remote:driver': 'houdinisdr-device', 'remote:type': 'houdinisdr', 'timeout': "
            "'3000000'}); d.writeSetting('CLOCK_ADJ', 'release'); d.close()\"" % (ip, port))


def check_clock(rep, ip, port, st, root=None):
    """A radio's CLOCK_ADJ state. A node left steered (a steering run that did
    not release, or a steering script) runs every later run off its
    calibration point, and a run with steering off never reads it."""
    f = dict(kv.split("=", 1) for kv in st.split() if "=" in kv)
    off = f.get("offset", "")
    if not st:
        rep.add("FAIL", "clock %s" % ip, "CLOCK_ADJ read back empty",
                "Run the check again; if it persists, ask whoever maintains the boards.")
    elif f.get("ref") == "calibrated" and not off.lstrip("-").isdigit():
        # Calibrated but out of its hold: PLL1 is tracking, so the tick is not at
        # the calibrated frequency (the device warns at make() too).
        rep.add("WARN", "clock %s" % ip, "ref=calibrated but the hold is not in force (CLOCK_ADJ %s)" % st,
                "Release it back into the calibrated hold before the run: " + release_cmd(ip, port, root))
    elif not off.lstrip("-").isdigit():
        rep.add("INFO", "clock %s" % ip, "ref=%s: not held at a calibration code, no steering offset"
                % f.get("ref", "?"))
    elif int(off) != 0:
        rep.add("WARN", "clock %s" % ip, "left steered %+d counts from its calibration code (CLOCK_ADJ %s)"
                % (int(off), st),
                "Release it before the run: " + release_cmd(ip, port, root))
    else:
        rep.add("INFO", "clock %s" % ip, "ref=%s, at its calibration code %s (offset 0)"
                % (f.get("ref", "?"), f.get("cal_dac", "?")))


def check_versions(rep, sd, nodes, port, env, root=None):
    infos = {}
    for ip in nodes:
        # A child process with a timeout: a radio that accepts the connection but
        # never answers must not hang the check. A node's first opens after its
        # boot can take 3.3 s against the 3 s open timeout (DEMO_VERIFICATION
        # 9.83), so a timeout is tried again, as the dashboard's radio opens are.
        for attempt in range(1, HWINFO_ATTEMPTS + 1):
            err = None
            try:
                out = subprocess.run([sys.executable, os.path.abspath(__file__), "--hwinfo", ip, str(port)],
                                     cwd=sd, env=env, timeout=60, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
                if out.returncode != 0:
                    raise RuntimeError((out.stderr.decode("utf-8", "replace").strip().splitlines() or ["?"])[-1])
                infos[ip] = json.loads(out.stdout)
            except (subprocess.TimeoutExpired, RuntimeError, ValueError) as e:
                err = str(e)
            if err is None or "TIMEOUT" not in err:
                break
        if err is None:
            continue
        if "TIMEOUT" in err:
            fix = ("The radio did not answer within the open timeout in %d tries. A node's first opens after its "
                   "boot can be that slow: run this check again. If it persists, its SoapySDRServer is stuck "
                   "(restart it with the rig booked)." % HWINFO_ATTEMPTS)
        elif "No module named" in err:
            fix = "Run this check with the venv's python (--venv, and its bin/python3)."
        else:
            fix = "If the radio is held by another run, stop that run first."
        rep.add("FAIL", "stack %s" % ip, "could not read the radio's hardware info (%s)" % err[:160], fix)
    for ip in list(infos):
        missing = [k for k in MUST_MATCH if k not in infos[ip]]
        if missing:
            rep.add("FAIL", "stack %s" % ip, "the radio's hardware info lacks %s: not a SoapyHoudiniSDR 0.4.0 stack"
                    % ", ".join(missing), "Run with the release's host prefix (--soapy-root) against radios on "
                    "that release (ask whoever maintains the boards).")
            del infos[ip]
    if not infos:
        return
    for ip, info in infos.items():
        rep.add("INFO", "stack %s" % ip, " ".join("%s=%s" % (k, info[k]) for k in MUST_MATCH))
        # The driver's release lockstep: one release on the host plugin and the
        # radio's device module.
        hv, dv = info["host_version"], info["device_version"]
        if hv != dv:
            rep.add("FAIL", "lockstep %s" % ip, "the host plugin is release %s and the radio's device module %s"
                    % (hv, dv), "Run with the host prefix of the radio's release, or deploy the plugin's release "
                    "to the radio (ask whoever maintains the boards).")
        check_egress(rep, ip, info["egress_status"])
        check_clock(rep, ip, port, info["_clock_adj"], root)
    if len(infos) < 2:
        return
    diff = [k for k in MUST_MATCH if len({i[k] for i in infos.values()}) > 1]
    if diff:
        rep.add("FAIL", "stack match", "the nodes differ in %s" % ", ".join(diff),
                "Put both radios on the same blessed stack (ask whoever maintains the boards).")
    else:
        rep.add("PASS", "stack match", "both nodes on the same gateware, firmware and plugin")
    # A release pairs the host plugin with the radios' device build (lockstep);
    # a mismatch is a stale or wrong --soapy-root.
    for ip, info in sorted(infos.items()):
        hb, db = info["host_build"], info["device_build"]
        if hb != db:
            rep.add("WARN", "plugin build %s" % ip, "the host plugin (host_build %s) is not this radio's device "
                    "build (%s)" % (hb, db),
                    "Pass --soapy-root with the prefix built with the radios' release (the demo rig's is in "
                    "DEMO_BENCH_RUNBOOK A3).")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--conf", default="files/houdini-dualband.json",
                    help="the config you will run, relative to the sounder directory")
    ap.add_argument("--sounder-dir", default=_SOUNDER)
    ap.add_argument("--venv", default=os.environ.get("VIRTUAL_ENV") or os.path.expanduser("~/houdini_test"),
                    help="the SoapySDR venv (default: the active one, else %(default)s); the Houdini plugin comes "
                         "from --soapy-root's prefix when one is given")
    ap.add_argument("--soapy-root", default=None, metavar="DIR",
                    help="the release's host-plugin prefix (the venv carries no Houdini module)")
    ap.add_argument("--examples", default=DEFAULT_EXAMPLES, metavar="DIR",
                    help="the SoapyHoudiniSDR host examples the framer teardown imports (default %(default)s)")
    ap.add_argument("--topology", default=None, metavar="FILE",
                    help="the topology file the run will use, overriding the config's serial_file "
                         "(the sounder's --topology)")
    ap.add_argument("--quick", action="store_true", help="skip check 7 (does not open the radios)")
    ap.add_argument("--json", action="store_true", help="print the report as JSON (for the dashboard)")
    ap.add_argument("--hwinfo", nargs=2, metavar=("IP", "PORT"), help=argparse.SUPPRESS)
    ap.add_argument("--proc-root", default="/proc", help=argparse.SUPPRESS)  # the test's own process table
    args = ap.parse_args()
    global PROC
    PROC = args.proc_root

    if args.hwinfo:  # the child of check_versions
        print(json.dumps(hwinfo(*args.hwinfo)))
        return 0

    sd = os.path.abspath(args.sounder_dir)
    rep = Report()
    cfg, bs, ue = check_config(rep, sd, args.conf, args.topology)
    check_build(rep, sd)
    check_plugin(rep, args.venv, args.soapy_root)
    check_examples(rep, args.examples)
    nodes = list(dict.fromkeys(bs + ue))
    held = check_no_sounder(rep, nodes)
    port = (cfg or {}).get("remote_port", "55132")  # config.cc's default
    if not isinstance(port, str) or not port.isdigit():
        # config.cc reads it as a string and throws on a number
        rep.add("FAIL", "config", "remote_port is %r, not a quoted port number" % (port,),
                "Write it as a quoted number in the config, e.g. \"remote_port\": \"55132\"")
        nodes = []  # no port to probe
    up = check_servers(rep, nodes, port)
    if args.quick:
        pass
    elif held:
        rep.add("INFO", "stack", "skipped: a sounder holds these radios, and opening them would disturb its run")
    elif up == nodes and nodes:
        check_versions(rep, sd, nodes, port, plugin_env(args.venv, args.soapy_root), args.soapy_root)
    else:
        rep.add("INFO", "stack", "skipped: not every radio's server answers")

    if args.json:
        print(json.dumps({"ok": not rep.failed(), "quick": args.quick, "conf": args.conf,
                          "results": rep.lines}))
    else:
        for r in rep.lines:
            print("%-4s  %-18s %s" % (r["level"], r["what"], r["detail"]))
            if r["fix"]:
                print("      %-18s -> %s" % ("", r["fix"]))
        print("\n%s" % ("NOT READY: fix each FAIL above, then run this again." if rep.failed()
                        else "Ready." + ("" if not args.quick else " (quick form: the radios' stacks were not read)")))
    return 1 if rep.failed() else 0


if __name__ == "__main__":
    sys.exit(main())
