#!/bin/bash
# usage: [CONF=<config>] [TOPO=<topology>|TOPO=] [UE=<ue address>] run_wd.sh <tag> <secs> "<filters>": the demo config
# on the two nodes on Build 3, by default as cabled (.21 BS, .22 UE; TOPO= runs the config's own topology, .22 BS, as the
# X-band chain): check_setup, the link record (LINK_CAUSE then cleared) and the packet gate before, demo_run.sh with the
# pacer cores read 75 s in, the link record and the gate after; logs in ~/demo_b3/<tag>
TAG=$1; SECS=$2; FILT=$3; D=~/demo_b3; L=$D/$TAG; S=~/repos/HoudiniLab-rxwin/CC/Sounder
ROOT=/home/houdini/houdini_sh571; EX=/home/houdini/repos/SoapyHoudiniSDR-sh571/host/examples
TOPO=${TOPO-files/topology-houdini-dualband-swap.json}; CONF=${CONF:-files/houdini-dualband-xw-steer-slots.json}
UE=${UE:-168.6.244.22}; TOPOARG=(); [ -n "$TOPO" ] && TOPOARG=(--topology "$TOPO")
NODES="168.6.244.21 168.6.244.22"
mkdir -p $L && cd $S && source ~/houdini_test/bin/activate || exit 1
snap() {
  SOAPY_SDR_ROOT=$ROOT SOAPY_SDR_PLUGIN_PATH= timeout 90 python3 $D/link_snap.py $1 $NODES $2 > $L/link_$1.txt 2>&1
  timeout 90 python3 tests/demo-verify/reg_snap.py $1 $NODES --soapy-root $ROOT > $L/gate_$1.txt 2>&1
}
echo "start $(date -u +%FT%TZ)"
timeout 300 python3 csi_gui/check_setup.py --soapy-root $ROOT --examples $EX "${TOPOARG[@]}" --conf $CONF \
  > $L/check_setup.txt 2>&1
rc=$?; echo "check_setup rc=$rc"
[ $rc = 0 ] || { echo "check_setup failed: no run"; exit 1; }
snap before --clear-cause
(sleep 75; python3 tests/demo-verify/pacer_core_check.py --cores 16,18,19 --ports enp1s0f0np0,enp1s0f1np1 \
   > $L/pacer_core_check.txt 2>&1; echo "rc=$?" >> $L/pacer_core_check.txt) &
PC=$!
bash tests/demo-verify/demo_run.sh "${TOPOARG[@]}" --core-map main=16 --ue-ssh houdini@$UE \
  $TAG $CONF $SECS $ROOT "$FILT" > $L/demo_run.txt 2>&1 < /dev/null
echo "demo_run rc=$?"
wait $PC
snap after
pgrep -a -x sounder || echo "no sounder running"
echo "end $(date -u +%FT%TZ)"
