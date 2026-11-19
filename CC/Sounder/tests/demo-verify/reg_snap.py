#!/usr/bin/env python3
"""The packet gate's fault counts on each node, read with no stream open (before
and after a run; compare as differences):
reg_snap.py <label> <node address> [<node address> ...] [--port 55132] [--soapy-root DIR]

Each RX channel's bank through RX_BANK_STATUS: aborts (the timed starts the
bus late-aborted: must not move), hwm (the FIFO's high-water mark) and gated
(stays as it was for a continuous arm), with the fail-stop state beside them.
The two counts are 64-bit, counted from the gateware load and never cleared,
so a run's count is the difference of its two readings. TDD_STAT (state,
epoch_late, gates_held, edge_late) is read through its setting too. Every read
is a setting, which the device resolves on its own register map: no register
offset is written here, since an offset from another map reads some other
word without an error. The egress stall count is check_setup's (EGRESS_STATUS).
Opened as check_setup opens a node, and closed cleanly. DEMO_VERIFICATION.md
9.70 (SM1) is this tool's before/after reading of fpga 1.34, taken when it
read the registers raw.
"""
import argparse
import os

ap = argparse.ArgumentParser(usage=__doc__.split("\n")[2])
ap.add_argument("label")
ap.add_argument("nodes", nargs="+")
ap.add_argument("--port", default="55132", help="the radio server's port (config.cc's default)")
ap.add_argument("--soapy-root", default=None, metavar="DIR",
                help="the release's host-plugin prefix (the venv carries no Houdini module)")
args = ap.parse_args()
# The Houdini plugin lives in the --soapy-root prefix: SoapySDR searches that
# root (the loader's own SOAPY_SDR_ROOT), the venv's plugin path emptied.
if args.soapy_root:
    os.environ.update(SOAPY_SDR_ROOT=args.soapy_root, SOAPY_SDR_PLUGIN_PATH="")
import SoapySDR  # noqa: E402  after the plugin environment above
for ip in args.nodes:
    sdr = SoapySDR.Device({"driver": "houdinisdr", "remote": "tcp://%s:%s" % (ip, args.port),
                           "remote:driver": "houdinisdr-device", "remote:type": "houdinisdr",
                           "timeout": "3000000"})
    try:
        raw = sdr.readSetting("RX_BANK_STATUS")
        banks = []
        for group in raw.split(";"):
            ch, _, body = group.partition(":")
            f = dict(t.split("=", 1) for t in body.split(",") if "=" in t)
            missing = [k for k in ("aborts", "hwm", "gated", "halted", "stop") if k not in f]
            if not ch.startswith("ch") or missing:
                raise SystemExit("%s %s: RX_BANK_STATUS is not the per-channel form this tool reads (%r)"
                                 % (args.label, ip, raw))
            banks.append("rx%s abort=%s hwm=%s gated=%s halted=%s stop=%s"
                         % (ch[2:], f["aborts"], f["hwm"], f["gated"], f["halted"], f["stop"]))
        st = dict(t.split("=", 1) for t in sdr.readSetting("TDD_STAT").split() if "=" in t)
        print("%s %s: %s | TDD_STAT state %s, epoch_late %s, gates_held %s, edge_late %s"
              % (args.label, ip, "; ".join(banks), st.get("state"), st.get("epoch_late"), st.get("gates_held"),
                 st.get("edge_late")))
    finally:
        sdr.close()
