#!/bin/bash
# One demo-config run on the rig host with the BS frame dumps (--bs_dump_frame)
# and the MER every 15 s, then the run's verdict, slot_edges.py over the dumps
# and the logged pilot_grid_off of each dumped frame (the analyzer's check).
# The bench as cabled: the swapped topology, main on 16, the pacers on 18,19.
# usage: edge_run.sh <tag> <config> <secs> <filters>
set -u
cd ~/repos/HoudiniLab-rxwin/CC/Sounder || exit 1
source ~/houdini_test/bin/activate || exit 1
tag=$1 conf=$2 secs=$3 filt=$4
STACK="fpga 1.44 4912c735, device 0.6.0 1684af6f, host 0.6.0 a8f0f184"
LM=/tmp/lm_$tag; rm -rf "$LM"; mkdir -p "$LM"
setsid nohup python3 tests/demo-verify/mer_sampler.py "$LM/mer.txt" 15 $((secs - 60)) 40 > /dev/null 2>&1 < /dev/null &
mer=$!
echo "######## $tag $(date -u +%T) $conf"
bash tests/demo-verify/fstage_run.sh --sounder-dir "$PWD" --health-s 5 --filters "$filt; $STACK" \
  --soapy-root "$HOME/houdini_hil" --sounder-arg=--tx_host_status --sounder-arg=--core_map=main=16 \
  --sounder-arg=--tx_cpu_affinity=18,19 --sounder-arg=--topology=files/topology-houdini-dualband-swap.json \
  "--sounder-arg=--bs_dump_frame=$LM" DEMO "$tag" "$conf" "$secs" < /dev/null 2>&1 | grep -E "launched|done|failed"
kill "$mer" 2> /dev/null
R=$(ls -d ap79_runs/DEMO/"${tag}"_* | tail -1)
L=$R/$(basename "$R").log
mkdir -p "$R/bsframes" && cp "$LM"/* "$R/bsframes/"
echo "== MER"; grep -o "/ant/0/cns MER [0-9.]* dB\|/ant/1/cns MER [0-9.]* dB" "$R/bsframes/mer.txt" | awk '{print $1, $3}' | sort | awk '{a[$1]=a[$1]" "$2} END {for (k in a) print k":"a[k]}'
python3 tests/demo-verify/run_summary.py "$R" | sed -n '/== warning counts/,$p' | grep -vE "^  (pilot failed|stray|window untrusted|CSI view|rotation)"
echo "== slot edges"; python3 /tmp/slot_edges.py "$R/bsframes"
echo "== the logged seat of each dumped frame"
for f in $(grep -h "^frame " "$R"/bsframes/bsframe_*.txt | awk '{print $2}'); do
  echo "frame $f logged $(grep -m1 "HOUDINI_BS_RX: frame=$f " "$L" | grep -o "pilot_grid_off=[-0-9]*")"
done
echo "######## done $(date -u +%T) $R"
