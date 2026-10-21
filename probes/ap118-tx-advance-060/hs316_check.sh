#!/bin/bash
# AP-118's HS-316 re-check on the rig host: three separate bring-ups of the
# demo config at tx_advance 198, each one edge_run.sh (the BS frame dumps,
# slot_edges.py, the logged seat), with edge_run.sh's stack string replaced by
# the one check_setup read. Run as a file, detached; the output is the log.
# usage: hs316_check.sh "<stack ids>"
set -u
stack=$1
sed "s|^STACK=.*|STACK=\"$stack\"|" /tmp/edge_run.sh > /tmp/edge_run_145.sh || exit 1
grep -q "^STACK=\"$stack\"" /tmp/edge_run_145.sh || { echo "stack not set"; exit 1; }
for t in H198a H198b H198c; do
  bash /tmp/edge_run_145.sh "$t" files/houdini-dualband-xw-steer-slots.json 150 \
    "the demo config, tx_advance 198 (AP-118, the HS-316 re-check), a separate bring-up, BS frame dumps, the bench as cabled (BS .21), rig build de414432" < /dev/null 2>&1
done
echo "######## hs316 check done $(date -u +%T)"
