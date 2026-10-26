# check_setup's stack checks (the release floor, the lockstep, the egress and clock lines) on .21 and .23, the two
# nodes the demo topology does not pair (.22 is out), run from the rig worktree's CC/Sounder with the venv's python:
# python3 stack_both.py <host prefix>. The two boards differ, so the stack match FAILs by design.
import os, sys
sys.path.insert(0, "csi_gui")
import check_setup as cs
root = sys.argv[1]
rep = cs.Report()
cs.check_versions(rep, os.getcwd(), ["168.6.244.21", "168.6.244.23"], 55132,
                  cs.plugin_env(os.environ.get("VIRTUAL_ENV"), root), root)
for r in rep.lines:
    print(r["level"], r["what"], r["detail"], ("| " + r["fix"]) if r["level"] == "FAIL" and r["fix"] else "")
