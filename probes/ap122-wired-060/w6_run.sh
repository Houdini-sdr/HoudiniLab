#!/bin/bash
# One demo run on the rig host on the 0.6.0 stack, with a core check 70 s in.
# usage: w6_run.sh <tag> <secs> <config> <ue ssh> <filters>
set -u
cd ~/repos/HoudiniLab-rxwin/CC/Sounder || exit 1
PREFIX=$HOME/houdini_hil
STACK="fpga 1.44 4912c735, device 0.6.0 1684af6f, host 0.6.0 a8f0f184; HoudiniLab develop code + the lab topology"
tag=$1 secs=$2 conf=$3 ue=$4 filt=$5
echo "######## $tag $(date -u +%T) $conf"
( sleep 70; date -u +"pcc %T"; python3 tests/demo-verify/pacer_core_check.py --cores 16,18,19 --ports enp1s0f0np0,enp1s0f1np1; echo "pcc rc=$?" ) > /tmp/pcc_$tag.txt 2>&1 &
pcc=$!
bash tests/demo-verify/demo_run.sh --core-map main=16 --tx-cpu-affinity 18,19 --ue-ssh "$ue" "$tag" "$conf" "$secs" "$PREFIX" "$filt; $STACK" < /dev/null
echo "demo_run rc=$?"
wait $pcc
cat /tmp/pcc_$tag.txt; rm -f /tmp/pcc_$tag.txt
echo "######## done $(date -u +%T)"
