#!/bin/bash
# usage: run_rung.sh [options] <tag> <conf> <secs>
#   --soapy-root DIR   the release's host-plugin prefix (the venv carries no Houdini module)
#   --venv DIR         the SoapySDR venv (default ~/houdini_test)
#   --sounder-dir DIR  the checkout to run (default: the one this script is in)
#   --health-s N       the link-health period, s (the sounder's --link_health_s; default its own)
#   --record FILE      the dashboard records every datagram to FILE (the canned-data fallback)
#   --sounder-arg ARG  one more sounder flag, repeatable (its run options: ./build/sounder --helpon=main)
# Rig tool (AP-79): one sounder run in view mode with the receive-only
# dashboard, CPU and thread sampling. The sounder's pid goes to
# ap79_runs/<tag>.pid.
ROOT=; VENV=$HOME/houdini_test; SD=$(cd "$(dirname "$0")/../.." && pwd); HEALTH=; REC=; SARGS=()
while [ $# -gt 0 ]; do
  case $1 in
    --soapy-root) ROOT=$2; shift 2;;
    --venv) VENV=$2; shift 2;;
    --sounder-dir) SD=$2; shift 2;;
    --health-s) HEALTH=$2; shift 2;;
    --record) REC=$2; shift 2;;
    --sounder-arg) SARGS+=("$2"); shift 2;;
    --sounder-arg=*) SARGS+=("${1#--sounder-arg=}"); shift;;
    -*) echo "run_rung.sh: unknown option $1 (the options are at the head of this file)" >&2; exit 2;;
    *) break;;
  esac
done
[ $# -eq 3 ] || { echo "usage: run_rung.sh [options] <tag> <conf> <secs>" >&2; exit 2; }
TAG=$1; CONF=$2; SECS=$3
cd "$SD" || exit 1
rm -f "ap79_runs/$TAG.current" "ap79_runs/$TAG.pid" "ap79_runs/$TAG.csipid"  # a launch that fails below leaves no stale record to reuse
source "$VENV/bin/activate" || exit 1
export LD_LIBRARY_PATH=$VIRTUAL_ENV/lib SOAPY_SDR_PLUGIN_PATH=$VIRTUAL_ENV/lib/SoapySDR/modules0.8-3
# --soapy-root: SoapySDR then searches only that prefix's modules (the
# loader's own SOAPY_SDR_ROOT, with the plugin path emptied).
if [ -n "$ROOT" ]; then export SOAPY_SDR_ROOT=$ROOT SOAPY_SDR_PLUGIN_PATH=; fi
ls "${ROOT:-$VIRTUAL_ENV}"/lib/SoapySDR/modules*/libHoudiniSDRSupport.so >/dev/null 2>&1 ||
  { echo "no Houdini host plugin under ${ROOT:-the venv $VIRTUAL_ENV}: pass --soapy-root <the release's host-plugin prefix>"; exit 1; }
mkdir -p ap79_runs
SFLAGS=(--max_frame=2000000000 --ue_tx_debug "--dump_dir=$PWD/ap79_runs")
[ -n "$HEALTH" ] && SFLAGS+=("--link_health_s=$HEALTH")
CFLAGS=(); [ -n "$REC" ] && CFLAGS+=(--record "$REC")
T=$(date +%H%M%S); echo "${TAG}_$T" > "ap79_runs/$TAG.current" || exit 1  # before anything is launched
setsid nohup timeout -s INT $(( SECS + 15 )) python3 csi_gui/csi_server.py --conf "$CONF" "${CFLAGS[@]}" > "ap79_runs/${TAG}_csi_$T.log" 2>&1 < /dev/null &
echo $! > "ap79_runs/$TAG.csipid"  # the dashboard's timeout wrapper: fstage_run stops it when the sounder is done
sleep 2
setsid nohup timeout -s INT "$SECS" ./build/sounder --view --conf_file "$CONF" "${SFLAGS[@]}" "${SARGS[@]}" > "ap79_runs/${TAG}_$T.log" 2>&1 < /dev/null &
TP=$!  # timeout's pid (setsid and nohup exec in place): the sounder is its child.
# That holds in a shell without job control (bash run_rung.sh, as fstage_run and
# the runbook call it); under set -m setsid forks and TP is its exited parent.
# THIS run's sounder, not whichever sounder on the host has the lowest pid
SP=
for _ in 1 2 3 4 5 6 7 8 9 10; do
    SP=$(pgrep -x sounder -P "$TP") && break
    sleep 0.5
done
echo "$SP" > "ap79_runs/$TAG.pid"
export S_TIME_FORMAT=ISO
setsid nohup timeout $(( SECS + 5 )) mpstat -P ALL 5 > "ap79_runs/${TAG}_cpu_$T.log" 2>&1 < /dev/null &
[ -n "$SP" ] && setsid nohup timeout $(( SECS + 5 )) pidstat -t -p "$SP" 5 > "ap79_runs/${TAG}_threads_$T.log" 2>&1 < /dev/null &
echo "launched ${TAG}_$T at $(date -u +%H:%M:%SZ) (sounder pid ${SP:-none found})"
