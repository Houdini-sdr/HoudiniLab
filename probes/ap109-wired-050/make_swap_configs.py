#!/usr/bin/env python3
"""The swapped-role config copies of DEMO_VERIFICATION 9.84: each named config
with only `serial_file` (the swapped topology), `_label` and, for the demo
config, `_description` changed. Run from CC/Sounder on the rig host; writes
into <outdir> next to the swapped topology.

usage: make_swap_configs.py <outdir>   (9.84: ~/app_archive/swap)"""
import json
import os
import sys

out = os.path.abspath(sys.argv[1])
topo = os.path.join(out, "topology-houdini-dualband-swap.json")
for src, dst, label in (
    ("files/houdini-dualband-xw-steer-slots.json", "houdini-dualband-xw-steer-slots-swap.json",
     "Demo, cabled, roles swapped"),
    ("files/houdini-dualband-steer.json", "houdini-dualband-steer-swap.json",
     "Fallback 48 MHz, cabled, roles swapped"),
):
    c = json.load(open(src))
    c["serial_file"] = topo
    c["_label"] = label
    if dst.startswith("houdini-dualband-xw-steer-slots"):
        c["_description"] = ("THE DEMO, cabled, with the BS on .21 and the UE on .22 (the bench as wired): "
                             + c["_description"].split(": ", 1)[1])
    json.dump(c, open(os.path.join(out, dst), "w"), indent=1)
    print(dst)
