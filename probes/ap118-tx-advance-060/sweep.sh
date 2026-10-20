#!/bin/bash
# The tx_advance response on the rig host: one edge_run.sh per value, each on a
# copy of the demo config with only tx_advance changed (in /tmp/txadv, so the
# repo and the rig worktree stay as shipped). 198 is the calibrated value, so
# the seat should read about 198 - v; past -30 the BS warns at the slot edge
# and its placed start pins at the cut; past +32 the slot cut takes the tail.
# usage: sweep.sh <config> <tag prefix> <value> [<value> ...]
set -u
conf=$1 pre=$2; shift 2
D=/tmp/txadv; mkdir -p "$D"
cd ~/repos/HoudiniLab-rxwin/CC/Sounder || exit 1
for v in "$@"; do
  c=$D/$(basename "$conf" .json)-tx$v.json
  ~/houdini_test/bin/python3 - "$conf" "$c" "$v" <<'EOF'
import json, sys
src, dst, v = sys.argv[1], sys.argv[2], int(sys.argv[3])
c = json.load(open(src))
c["tx_advance"] = [v]
c["_label"] = "tx_advance sweep %d" % v
json.dump(c, open(dst, "w"), indent=1)
EOF
  bash /tmp/edge_run.sh "$pre$v" "$c" 150 "tx_advance sweep: $v, a copy of $conf (AP-118), rig build de414432" < /dev/null 2>&1 |
    grep -E "^######## |^/ant|head at slot|PILOT LOST|no UE burst|beacon snr|pilot_grid_off \{|frames/s|VERDICT|WARN|^P over|^U over"
done
echo "######## sweep done $(date -u +%T)"
