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
  5. no other sounder is running on this host (it would hold the radios);
  6. each radio's server answers on the config's remote port;
  7. (full form only) each radio's stack: gateware, firmware, plugin and protocol
     versions, which must agree between the nodes.

Checks 1 to 6 touch no radio. Check 7 opens each radio and reads its hardware
info, as the sounder does at startup, and changes nothing. Do not run the full
form against radios someone else is using.

Run it with the venv's python (or with the venv activated): check 7 imports
SoapySDR from there.
"""
import argparse
import glob
import json
import os
import socket
import subprocess
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_SOUNDER = os.path.dirname(_HERE)
sys.path.insert(0, _HERE)
from teardown_framer import roles_from_topology  # noqa: E402  one reader, not two

# The keys every node in one run must agree on: the sounder's own list
# (include/node_version.h, kMustMatch), so the two cannot disagree about a bench.
MUST_MATCH = ("fpga_version", "fpga_commit", "fpga_board", "device_version",
              "device_build", "host_version", "host_build", "proto_version")


class Report:
    def __init__(self):
        self.lines = []

    def add(self, level, what, detail="", fix=""):
        self.lines.append({"level": level, "what": what, "detail": detail, "fix": fix})

    def failed(self):
        return any(r["level"] == "FAIL" for r in self.lines)


def check_config(rep, sd, conf):
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
    topo = cfg.get("serial_file")
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
                "Create %s with your radios' addresses (walkthrough section 3)." % topo)
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
                "Build it: cd %s && cmake -B build -DCMAKE_BUILD_TYPE=Release && cmake --build build -j "
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


def plugin_env(venv):
    """The environment that loads the Houdini plugin: the sounder's (csi_server.py)."""
    return dict(os.environ, LD_LIBRARY_PATH=os.path.join(venv, "lib"),
                SOAPY_SDR_PLUGIN_PATH=os.path.join(venv, "lib", "SoapySDR", "modules0.8-3"))


def check_plugin(rep, venv):
    moddir = os.path.join(venv, "lib", "SoapySDR", "modules0.8-3")
    if not os.path.isdir(venv):
        rep.add("FAIL", "plugin", "venv %s not found" % venv,
                "Install the SoapyHoudiniSDR host (walkthrough section 2.4) and pass its prefix as --venv.")
        return
    mods = [m for m in glob.glob(os.path.join(moddir, "*.so")) if "houdini" in os.path.basename(m).lower()]
    if not mods:
        rep.add("FAIL", "plugin", "no Houdini module in %s" % moddir,
                "Install the SoapyHoudiniSDR host plugin into this venv (walkthrough section 2.4).")
        return
    util = os.path.join(venv, "bin", "SoapySDRUtil")
    env = plugin_env(venv)
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


def check_examples(rep):
    ex = os.environ.get("HOUDINI_EXAMPLES", os.path.expanduser("~/repos/SoapyHoudiniSDR/host/examples"))
    if os.path.isfile(os.path.join(ex, "houdini_setup.py")):
        rep.add("PASS", "teardown", "houdini_setup found in %s" % ex)
    else:
        rep.add("WARN", "teardown", "houdini_setup.py not in %s" % ex,
                "The framer teardown before each start needs it: export HOUDINI_EXAMPLES="
                "<path-to-SoapyHoudiniSDR>/host/examples (walkthrough section 2.4).")


def running_sounders():
    """Local processes named `sounder`. By /proc comm, so this never matches itself."""
    found = []
    for e in os.listdir("/proc"):
        if not e.isdigit():
            continue
        try:
            with open("/proc/%s/comm" % e) as f:
                if f.read().strip() == "sounder":
                    found.append(int(e))
        except OSError:
            pass
    return found


def radios_of(pid):
    """The radio addresses a running sounder uses, from its --conf_file and its
    working directory, or None when that cannot be read (another user's process)."""
    try:
        with open("/proc/%d/cmdline" % pid, "rb") as f:
            argv = f.read().decode("utf-8", "replace").split("\0")
        cwd = os.readlink("/proc/%d/cwd" % pid)
        conf = None
        for i, a in enumerate(argv):
            if a == "--conf_file" and i + 1 < len(argv):
                conf = argv[i + 1]
            elif a.startswith("--conf_file="):
                conf = a.split("=", 1)[1]
        with open(os.path.join(cwd, conf), encoding="utf-8") as f:
            topo = json.load(f)["serial_file"]
        with open(os.path.join(cwd, topo), encoding="utf-8") as f:
            bs, ue = roles_from_topology(json.load(f))
        return set(bs + ue)
    except (OSError, ValueError, KeyError, TypeError):
        return None


def check_no_sounder(rep, nodes):
    """A sounder on the same radios holds them; one on other radios (a shared
    host, another bench) does not block this run."""
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
                "Stop the run that started it, or end it with: kill %s" % " ".join(map(str, held)))
    elif unknown:
        rep.add("WARN", "radios free", "a sounder is running (pid %s) and its radios could not be read"
                % ", ".join(map(str, unknown)),
                "If it uses these radios, stop it first: the start will fail with the radios held.")
    else:
        rep.add("PASS", "radios free", "no other sounder on these radios"
                + (" (pid %s runs on other radios)" % ", ".join(map(str, other)) if other else ""))


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
        return dict(sdr.getHardwareInfo())
    finally:
        # A clean close: a connection dropped at process exit leaves a
        # "handlerLoop() FAIL: recv(header)" in each radio's server journal.
        SoapySDR.Device.unmake(sdr)


def check_versions(rep, sd, nodes, port, env):
    infos = {}
    for ip in nodes:
        # A child process with a timeout: a radio that accepts the connection but
        # never answers must not hang the check.
        try:
            out = subprocess.run([sys.executable, os.path.abspath(__file__), "--hwinfo", ip, str(port)],
                                 cwd=sd, env=env, timeout=60, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            if out.returncode != 0:
                raise RuntimeError((out.stderr.decode("utf-8", "replace").strip().splitlines() or ["?"])[-1])
            infos[ip] = json.loads(out.stdout)
        except (subprocess.TimeoutExpired, RuntimeError, ValueError) as e:
            rep.add("FAIL", "stack %s" % ip, "could not read the radio's hardware info (%s)" % str(e)[:160],
                    "Run this check with the venv's python. If the radio is held by another "
                    "run, stop that run first.")
    if not infos:
        return
    for ip, info in infos.items():
        rep.add("INFO", "stack %s" % ip, " ".join("%s=%s" % (k, info.get(k, "<absent>")) for k in MUST_MATCH))
    if len(infos) < 2:
        return
    diff = [k for k in MUST_MATCH if len({i.get(k, "<absent>") for i in infos.values()}) > 1]
    if diff:
        rep.add("FAIL", "stack match", "the nodes differ in %s" % ", ".join(diff),
                "Put both radios on the same blessed stack (ask whoever maintains the boards).")
    else:
        rep.add("PASS", "stack match", "both nodes on the same gateware, firmware, plugin and protocol")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--conf", default="files/houdini-dualband.json",
                    help="the config you will run, relative to the sounder directory")
    ap.add_argument("--sounder-dir", default=_SOUNDER)
    ap.add_argument("--venv", default=os.environ.get("VIRTUAL_ENV") or os.path.expanduser("~/houdini_test"),
                    help="the SoapyHoudiniSDR host prefix (default: $VIRTUAL_ENV, else %(default)s)")
    ap.add_argument("--quick", action="store_true", help="skip check 7 (does not open the radios)")
    ap.add_argument("--json", action="store_true", help="print the report as JSON (for the dashboard)")
    ap.add_argument("--hwinfo", nargs=2, metavar=("IP", "PORT"), help=argparse.SUPPRESS)
    args = ap.parse_args()

    if args.hwinfo:  # the child of check_versions
        print(json.dumps(hwinfo(*args.hwinfo)))
        return 0

    sd = os.path.abspath(args.sounder_dir)
    rep = Report()
    cfg, bs, ue = check_config(rep, sd, args.conf)
    check_build(rep, sd)
    check_plugin(rep, args.venv)
    check_examples(rep)
    nodes = list(dict.fromkeys(bs + ue))
    check_no_sounder(rep, nodes)
    port = (cfg or {}).get("remote_port", "55132")  # config.cc's default
    if not isinstance(port, str) or not port.isdigit():
        # config.cc reads it as a string and throws on a number
        rep.add("FAIL", "config", "remote_port is %r, not a quoted port number" % (port,),
                "Write it as a quoted number in the config, e.g. \"remote_port\": \"55132\"")
        nodes = []  # no port to probe
    up = check_servers(rep, nodes, port)
    if not args.quick and up == nodes and nodes:
        check_versions(rep, sd, nodes, port, plugin_env(args.venv))
    elif not args.quick:
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
