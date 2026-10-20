#!/bin/bash
# The tx_advance stability run on the rig host: the repo's demo config (198)
# through demo_run.sh --topology for <secs>, with a pacer_core_check 70 s in.
# usage: long_run.sh <tag> <secs> <filters>
set -u
cd ~/repos/HoudiniLab-rxwin/CC/Sounder || exit 1
source ~/houdini_test/bin/activate || exit 1
tag=$1 secs=$2 filt=$3
STACK="fpga 1.44 4912c735, device 0.6.0 1684af6f, host 0.6.0 a8f0f184"
echo "######## $tag $(date -u +%T)"
( sleep 70; date -u +"pcc %T"; python3 tests/demo-verify/pacer_core_check.py --cores 16,18,19 --ports enp1s0f0np0,enp1s0f1np1; echo "pcc rc=$?" ) > /tmp/pcc_$tag.txt 2>&1 &
pcc=$!
bash tests/demo-verify/demo_run.sh --topology files/topology-houdini-dualband-swap.json --core-map main=16 \
  --tx-cpu-affinity 18,19 --ue-ssh houdini@168.6.244.22 "$tag" files/houdini-dualband-xw-steer-slots.json "$secs" \
  "$HOME/houdini_hil" "$filt; $STACK" < /dev/null
echo "demo_run rc=$?"
wait $pcc
cat /tmp/pcc_$tag.txt; rm -f /tmp/pcc_$tag.txt
echo "######## done $(date -u +%T)"
