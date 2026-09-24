#!/bin/bash
# usage: run_rung.sh <tag> <conf> <secs> [health_s]
# Rig tool (AP-79): one sounder run in view mode with the receive-only
# dashboard, CPU and thread sampling. Paths are the AP-79 rig host's: the
# worktree ~/repos/HoudiniLab-ap79 and the venv ~/houdini_test (SOUNDER_DIR and
# VENV override them). The sounder's pid goes to ap79_runs/<tag>.pid.
cd "${SOUNDER_DIR:-$HOME/repos/HoudiniLab-ap79/CC/Sounder}" || exit 1  # SOUNDER_DIR: another worktree's build
source "${VENV:-$HOME/houdini_test}/bin/activate" || exit 1
export LD_LIBRARY_PATH=$VIRTUAL_ENV/lib SOAPY_SDR_PLUGIN_PATH=$VIRTUAL_ENV/lib/SoapySDR/modules0.8-3
export HOUDINI_MAX_FRAME=2000000000 HOUDINI_UE_TX_DEBUG=1
mkdir -p ap79_runs; export HOUDINI_DUMP_DIR=$PWD/ap79_runs
if [ -n "$4" ]; then export HOUDINI_LINK_HEALTH_S=$4 HOUDINI_CFO_LOG_EVERY=100; else unset HOUDINI_LINK_HEALTH_S; fi
T=$(date +%H%M%S); echo "$1_$T" > "ap79_runs/$1.current"
setsid nohup timeout -s INT $(( $3 + 15 )) python3 csi_gui/csi_server.py --conf "$2" > "ap79_runs/$1_csi_$T.log" 2>&1 < /dev/null &
sleep 2
setsid nohup timeout -s INT "$3" ./build/sounder --view --conf_file "$2" > "ap79_runs/$1_$T.log" 2>&1 < /dev/null &
TP=$!  # timeout's pid (setsid and nohup exec in place): the sounder is its child
# THIS run's sounder, not whichever sounder on the host has the lowest pid
SP=
for _ in 1 2 3 4 5 6 7 8 9 10; do
    SP=$(pgrep -x sounder -P "$TP") && break
    sleep 0.5
done
echo "$SP" > "ap79_runs/$1.pid"
export S_TIME_FORMAT=ISO
setsid nohup timeout $(( $3 + 5 )) mpstat -P ALL 5 > "ap79_runs/$1_cpu_$T.log" 2>&1 < /dev/null &
[ -n "$SP" ] && setsid nohup timeout $(( $3 + 5 )) pidstat -t -p "$SP" 5 > "ap79_runs/$1_threads_$T.log" 2>&1 < /dev/null &
echo "launched $1_$T at $(date -u +%H:%M:%SZ) (sounder pid ${SP:-none found})"
