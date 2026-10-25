#!/bin/bash
# usage: bring_up.sh <tag> <secs>: one --bs_only sounder bring-up on .21 (BS by the swap topology), Build 2 host prefix
TAG=$1; SECS=$2
cd ~/repos/HoudiniLab-rxwin/CC/Sounder || exit 1
source ~/houdini_test/bin/activate || exit 1
export LD_LIBRARY_PATH=$VIRTUAL_ENV/lib SOAPY_SDR_ROOT=/home/houdini/houdini_sh571 SOAPY_SDR_PLUGIN_PATH=
echo "start $(date -u +%FT%T.%3NZ) head $(git rev-parse --short HEAD)" > ~/ap123_pre/$TAG.log
timeout -s INT "$SECS" ./build/sounder --view --conf_file files/houdini-dualband.json --topology=files/topology-houdini-dualband-swap.json --bs_only --core_map=main=16 --max_frame=2000000000 --dump_dir=$HOME/ap123_pre >> ~/ap123_pre/$TAG.log 2>&1 < /dev/null
rc=$?
echo "end $(date -u +%FT%T.%3NZ) rc=$rc" >> ~/ap123_pre/$TAG.log
echo "rc=$rc"
