#!/bin/bash
# Three 120 s runs on the rig host, one after another, each with a core check
# 70 s in. Runs on burton (fed over ssh with bash -s).
set -u
cd ~/repos/HoudiniLab-rxwin/CC/Sounder || exit 1
SW=$HOME/app_archive/swap
PREFIX=$HOME/houdini_hil
STACK="fpga 1.38 b963949b, device and host 0.5.0 4bb091c3; HoudiniLab develop code + the lab topology"
run() {  # tag config cores-to-check filters
  local tag=$1 conf=$2 cores=$3 filt=$4
  echo "######## $tag $(date -u +%T) $conf"
  ( sleep 70; date -u +"pcc %T"; python3 tests/demo-verify/pacer_core_check.py --cores "$cores" --ports enp1s0f0np0,enp1s0f1np1; echo "pcc rc=$?" ) > /tmp/pcc_$tag.txt 2>&1 &
  local pcc=$!
  bash tests/demo-verify/demo_run.sh --core-map main=16 --tx-cpu-affinity 18,19 --ue-ssh "$5" "$tag" "$conf" 120 "$PREFIX" "$filt; $STACK" < /dev/null
  echo "demo_run rc=$?"
  wait $pcc
  cat /tmp/pcc_$tag.txt; rm -f /tmp/pcc_$tag.txt
}
run W5R1 files/houdini-dualband-r1.json 16,18,19 "R1 sub-6 only, 256 FFT, 1024-sample packets, the repo topology (BS .22, UE .21) over the sub-6 cables" houdini@168.6.244.21
run W5S3 "$SW/houdini-dualband-xw-steer-slots-swap.json" 16,18,19 "the demo config, roles swapped (BS .21, UE .22), a fresh bring-up for the pilot seat" houdini@168.6.244.22
echo "######## all done $(date -u +%T)"
