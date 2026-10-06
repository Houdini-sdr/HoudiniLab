#!/bin/bash
# usage: demo_run.sh [options] <tag> <config> <secs> <host-plugin prefix> ["<filters>"]
# One evidence run of the demo head on the rig host, with the samplers the
# records cite: fstage_run.sh from this checkout, with freeze_watch.py,
# mer_sampler.py (every 15 s) and spc_sample.py (three spectra) alongside,
# all filed into the run directory, then run_summary.py's verdict (the exit
# status is the verdict's) and demo_report.py's TX and constellation lines.
#   <host-plugin prefix>  the release's host-plugin prefix, built with the
#                radios' device build (the venv carries no Houdini module)
#   <filters>    the filter state, noted in the run's stage.txt
# Options:
#   --core-map M, --tx-cpu-affinity L  the sounder's thread placement and
#                pacer cpus, as DEMO_BENCH_RUNBOOK.md A4 (defaults main=15 and
#                18,19, the reference rig's isolated cores; pacer_core_check.py
#                confirms or corrects them for yours)
#   --stage S    the ap79_runs subdirectory (default DEMO)
#   --record F   record the dashboard stream to this new file (a fallback)
#   --ue-ssh U   user@<ue-address>: count the UE's RFDC interrupts over the
#                run (the DAC FIFO storm watch); unset skips it
#   --venv DIR   the SoapySDR venv (default ~/houdini_test)
set -u
CM=main=15; TA=18,19; STAGE=DEMO; REC=; UE_SSH=; VENV=$HOME/houdini_test
while [ $# -gt 0 ]; do
  case $1 in
    --core-map) CM=$2; shift 2;;
    --tx-cpu-affinity) TA=$2; shift 2;;
    --stage) STAGE=$2; shift 2;;
    --record) REC=$2; shift 2;;
    --ue-ssh) UE_SSH=$2; shift 2;;
    --venv) VENV=$2; shift 2;;
    -*) echo "demo_run.sh: unknown option $1 (the options are at the head of this file)" >&2; exit 2;;
    *) break;;
  esac
done
[ $# -ge 4 ] || { echo "usage: demo_run.sh [options] <tag> <config> <secs> <host-plugin prefix> [\"<filters>\"]" >&2; exit 2; }
TAG=$1; CONF=$2; SECS=$3; PLUG=$4; FILT=${5:-}
HERE=$(cd "$(dirname "$0")" && pwd)
D=$(cd "$HERE/../.." && pwd)
cd "$D" || exit 1
source "$VENV/bin/activate" || exit 1
pgrep -x sounder >/dev/null && { echo "a sounder is running on this host; stop it first"; exit 1; }
ls "$PLUG"/lib/SoapySDR/modules*/libHoudiniSDRSupport.so >/dev/null 2>&1 || { echo "no Houdini host plugin under $PLUG"; exit 1; }
[ -f "$CONF" ] || { echo "no config $CONF (relative to $D)"; exit 1; }
[ "$SECS" -gt 90 ] || { echo "secs must be over 90 (the samplers start 40 s in)"; exit 1; }
S=$(mktemp -d /tmp/demo_run_XXXX)
setsid nohup python3 "$HERE/freeze_watch.py" --run-dir "$D/ap79_runs" --tag "$TAG" > "$S/freeze.txt" 2>&1 < /dev/null &
SAMPLERS=$!
# The MER every 15 s from 40 s in, the last about 20 s before the run ends.
setsid nohup python3 "$HERE/mer_sampler.py" "$S/mer.txt" 15 $((SECS - 60)) 40 > /dev/null 2>&1 < /dev/null &
SAMPLERS="$SAMPLERS $!"
setsid nohup python3 "$HERE/spc_sample.py" "$S/spc.txt" 60,$((SECS / 2)),$((SECS - 40)) > "$S/spc.err" 2>&1 < /dev/null &
SAMPLERS="$SAMPLERS $!"
RUN=(--sounder-dir "$D" --health-s 5 --filters "$FILT" --soapy-root "$PLUG" --venv "$VENV"
     --sounder-arg=--tx_host_status "--sounder-arg=--core_map=$CM" "--sounder-arg=--tx_cpu_affinity=$TA")
[ -n "$REC" ] && RUN+=(--record "$REC")
irq() {
  [ -n "$UE_SSH" ] || return 0
  ssh -o ConnectTimeout=5 -o BatchMode=yes "$UE_SSH" 'grep -iE "rfdc|rf_data|usp_rf|xrfdc" /proc/interrupts' 2>/dev/null |
    awk '{n=0; for(i=2;i<=NF;i++) if($i ~ /^[0-9]+$/) n+=$i; else break; print $1, n, $NF}'
}
IRQ0=$(irq)
date -u +"start %H:%M:%S"
bash "$HERE/fstage_run.sh" "${RUN[@]}" "$STAGE" "$TAG" "$CONF" "$SECS" 2>&1 | grep -E "launched|done|failed|Radios Not Found|TIMEOUT"
sleep 5
IRQ1=$(irq)
R=$(cat "ap79_runs/$TAG.current" 2>/dev/null); O=$D/ap79_runs/$STAGE/$R
# A run that filed nothing leaves no samplers behind: stop them by their own pids.
[ -n "$R" ] && [ -d "$O" ] || { echo "no run directory filed"; kill $SAMPLERS 2>/dev/null; rm -rf "$S"; exit 1; }
cp "$S/mer.txt" "$O/mer.txt" 2>/dev/null; cp "$S/freeze.txt" "$O/freeze_watch.txt" 2>/dev/null
cat "$S/spc.txt" "$S/spc.err" > "$O/spectrum.txt" 2>/dev/null
echo "run dir: $O"
echo "== MER (15 s samples)"; cut -c1-160 "$O/mer.txt"
echo "== freeze watch"; tail -1 "$O/freeze_watch.txt"
echo "== spectrum"; cut -c1-200 "$O/spectrum.txt"
echo "== TX and constellation"; python3 "$HERE/demo_report.py" "$O" 2>&1 | grep -E "TX totals|TX status events|CNS" | cut -c1-200
if [ -n "$UE_SSH" ]; then
  echo "== UE RFDC interrupts over the run"
  paste <(echo "$IRQ0") <(echo "$IRQ1") | awk '{printf "  %s %s -> %s (+%d)\n", $1, $2, $5, $5-$2}'
fi
rm -rf "$S"
python3 "$HERE/run_summary.py" "$O"
