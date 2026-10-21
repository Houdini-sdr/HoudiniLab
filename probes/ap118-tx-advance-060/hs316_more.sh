#!/bin/bash
# AP-118's HS-316 re-check, at matched carrier: one 600 s run for the settled
# seat, then three more 150 s bring-ups on warm nodes; /tmp/edge_run_145.sh
# as hs316_check.sh left it (the 1.45 stack string).
set -u
grep -q '^STACK="fpga 1.45 36935fdb' /tmp/edge_run_145.sh || { echo "stack not set"; exit 1; }
F="the demo config, tx_advance 198 (AP-118, the HS-316 re-check at matched carrier), BS frame dumps, the bench as cabled (BS .21), rig build de414432"
bash /tmp/edge_run_145.sh W198a files/houdini-dualband-xw-steer-slots.json 600 "$F, 600 s for the settled seat" < /dev/null 2>&1
for t in H198d H198e H198f; do
  bash /tmp/edge_run_145.sh "$t" files/houdini-dualband-xw-steer-slots.json 150 "$F, a separate bring-up on warm nodes" < /dev/null 2>&1
done
echo "######## hs316 more done $(date -u +%T)"
