# Every key a shipped sounder config sets is one the sounder reads. config.cc
# reads the top level with json.value(key, default), so a misspelt key (say
# "bs_rx_slot") is silently ignored and the run takes the default. The sync
# block is exempt: SyncConfig already refuses an unknown key there
# (sync_config.cc). Also exempt: "_" notes, the dashboard_* keys csi_gui
# reads, and the topology files, whose keys are names. A key is known when a
# sounder source names it as a string literal. Run from CC/Sounder (ctest does).
# Mutation: rename a key in a demo config (bs_rx_slots -> bs_rx_slot); this fails.
import glob, json, os, re, sys

src = ""
for f in (glob.glob("*.cc") + glob.glob("sync/*.cc") + glob.glob("include/*.h")
          + glob.glob("include/*/*.h")):
    with open(f, errors="replace") as fh:
        src += fh.read()
known = set(re.findall(r'"([A-Za-z_][A-Za-z0-9_]*)"', src))
# the dashboard's own keys, as csi_gui names them (dashboard_mag_top)
gui = ""
for f in glob.glob("csi_gui/*.py"):
    with open(f, errors="replace") as fh:
        gui += fh.read()
dashboard = set(re.findall(r'"(dashboard_[A-Za-z0-9_]*)"', gui))


def unknown(o, path=""):
    out = []
    if isinstance(o, dict):
        for k, v in o.items():
            if k.startswith("_") or (path == "" and (k == "sync" or k in dashboard)):
                continue
            if k not in known:
                out.append(path + k)
            out += unknown(v, path + k + ".")
    elif isinstance(o, list):
        for v in o:
            out += unknown(v, path)
    return out


fails = 0
confs = [f for f in sorted(glob.glob("files/*.json")) if not os.path.basename(f).startswith("topology-")]
for f in confs:
    with open(f) as fh:
        bad = unknown(json.load(fh))
    print(("FAIL %s sets keys the sounder never reads: %s" % (f, ", ".join(bad))) if bad else ("PASS " + f))
    fails += bool(bad)
if not confs or len(known) < 100:
    print("FAIL no configs, or the key scan found almost nothing (run from CC/Sounder)")
    fails += 1
print("%d failure(s)" % fails)
sys.exit(1 if fails else 0)
