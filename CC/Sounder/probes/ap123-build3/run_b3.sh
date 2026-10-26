#!/bin/bash
# usage: run_b3.sh: DEMO_VERIFICATION 9.88's checks again on Build 3, on .21 alone (.22 is out): the stack read, part 2's
# bring-up, part 2's and then part 1's dashboard teardown, a second bring-up, the stack read again; logs in ~/ap123_b3
D=~/ap123_b3; S=~/repos/HoudiniLab-rxwin/CC/Sounder; P1=$S/ap79_runs/AP123/p1/CC/Sounder
EX=/home/houdini/repos/SoapyHoudiniSDR-sh571/host/examples; ROOT=/home/houdini/houdini_sh571
cd $D && source ~/houdini_test/bin/activate || exit 1
stack() { SOAPY_SDR_ROOT=$ROOT SOAPY_SDR_PLUGIN_PATH= timeout 60 python3 $D/stack21.py > $D/stack21_$1.txt 2>&1; }
teardown() {
  (cd "$1" && echo "start $(date -u +%FT%TZ)" && timeout 90 python3 csi_gui/teardown_framer.py --node 168.6.244.21 $3 \
     --examples $EX --soapy-root $ROOT; echo "rc=$?") > $D/$2.log 2>&1
}
stack before
$D/bring_up.sh P2a 60 < /dev/null
teardown $S P2_teardown ""
teardown $P1 P1_teardown "--passes 1"
$D/bring_up.sh P2b 45 < /dev/null
stack after
pgrep -a sounder || echo "no sounder running"
