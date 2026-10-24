#!/usr/bin/env python3
"""The fpga 1.34 packet gate's fault registers on each node, read with no stream
open (before and after a run; compare as deltas):
reg_snap.py <label> <node address> [<node address> ...] [--port 55132] [--soapy-root DIR]

RFCORE (0xA007_0000): RX bank k at 0x1000 + 0x80*k, ABORT_COUNT +0x18 (must
not move; survives RX_CLEAR), FIFO_HWM +0x14, GATED_DROPS +0x38 (stays 0 for a
continuous arm); the bank offsets are the same in register map v3. TDD_STAT
(state, epoch_late, gates_held, edge_late) is read through its setting, which
the device plugin resolves on its own register map (v3 moved the register from
0xAC to 0x31C, and the old offset on a v3 bitstream reads 0, which decodes as
idle). The egress stall watchdog (FHCORE +0x28) is outside the
driver's FHCORE register window; check_setup's EGRESS_STATUS covers it. Opened
as check_setup opens a node, and closed cleanly. DEMO_VERIFICATION.md 9.70
(SM1) is this tool's before/after reading of fpga 1.34.
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
        r = lambda core, off: int(sdr.readRegister(core, off))
        banks = []
        for k in range(4):
            b = 0x1000 + 0x80 * k
            banks.append("rx%d abort=%d hwm=%d gated=%d" % (k, r("RFCORE", b + 0x18), r("RFCORE", b + 0x14),
                                                          r("RFCORE", b + 0x38)))
        st = dict(t.split("=", 1) for t in sdr.readSetting("TDD_STAT").split() if "=" in t)
        print("%s %s: %s | TDD_STAT state %s, epoch_late %s, gates_held %s, edge_late %s"
              % (args.label, ip, "; ".join(banks), st.get("state"), st.get("epoch_late"), st.get("gates_held"),
                 st.get("edge_late")))
    finally:
        sdr.close()
