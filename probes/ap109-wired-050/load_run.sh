#!/bin/bash
# One 180 s demo run (roles swapped, main 16, pacers 18,19, as W5S2) with two
# marked CPU bursts on the non-isolated cores 0-14: at +60 s one busy process
# for 10 s, at +120 s fifteen for 10 s. Runs on burton (fed over ssh, bash -s).
# Usage: bash -s <tag> [core_map]
set -u
TAG=${1:?tag}; CM=${2:-main=16}
cd ~/repos/HoudiniLab-rxwin/CC/Sounder || exit 1
SW=$HOME/app_archive/swap
OUT=/tmp/load_$TAG.txt
burst() {  # n procs, secs
  local pids="" i
  date -u +"burst $1x start %T.%N" | cut -c1-30
  for i in $(seq 1 "$1"); do
    timeout "$2" taskset -c 0-14 python3 -I -c 'while True: pass' &
    pids="$pids $!"
  done
  wait $pids
  date -u +"burst $1x end   %T.%N" | cut -c1-30
}
date -u +"launch %T"
bash tests/demo-verify/demo_run.sh --core-map "$CM" --tx-cpu-affinity 18,19 --ue-ssh houdini@168.6.244.22 "$TAG" \
  "$SW/houdini-dualband-xw-steer-slots-swap.json" 180 "$HOME/houdini_hil" \
  "host-load test: 1 busy process at +60 s and 15 at +120 s, 10 s each, on cores 0-14; core map $CM; roles swapped (BS .21, UE .22); fpga 1.38 b963949b, device and host 0.5.0 4bb091c3" > "$OUT" 2>&1 < /dev/null &
DR=$!
sleep 60; burst 1 10
sleep 50; burst 15 10
wait $DR; echo "demo_run rc=$?"
cat "$OUT"; rm -f "$OUT"
