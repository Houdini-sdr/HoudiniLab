#!/bin/bash
# usage: demo_run.sh <tag> <config> <secs> [default|slots] ["<filters>"]
# One evidence run of the demo head on the rig host, with the samplers the
# records cite: fstage_run.sh from this checkout, with freeze_watch.py,
# mer_sampler.py (every 15 s) and spc_sample.py (three spectra) alongside,
# all filed into the run directory, then run_summary.py's verdict (the exit
# status is the verdict's) and demo_report.py's TX and constellation lines.
#   slots        run under the slots host plugin (HOUDINI_SLOTS_ROOT, default
#                ~/houdini_slots), for the slots configs; default: the venv's
#   <filters>    the filter state, noted in the run's stage.txt
# Environment, all optional:
#   HOUDINI_CORE_MAP, HOUDINI_TX_CPU_AFFINITY  as DEMO_BENCH_RUNBOOK.md A4
#                (the defaults below are the reference rig's isolated cores;
#                pacer_core_check.py confirms or corrects them for yours)
#   STAGE        the ap79_runs subdirectory (default DEMO)
#   DEMO_RECORD  record the dashboard stream to this new file (a fallback)
#   UE_SSH       user@<ue-address>: count the UE's RFDC interrupts over the
#                run (the DAC FIFO storm watch); unset skips it
set -u
TAG=$1; CONF=$2; SECS=$3; PLUG=${4:-default}; FILT=${5:-}
HERE=$(cd "$(dirname "$0")" && pwd)
D=$(cd "$HERE/../.." && pwd)
cd "$D" || exit 1
source "${VENV:-$HOME/houdini_test}/bin/activate" || exit 1
unset HOUDINI_TX_STREAM_ARGS HOUDINI_CLOCK_STEER HOUDINI_CSI_RECORD HOUDINI_SOAPY_ROOT
[ -n "${DEMO_RECORD:-}" ] && export HOUDINI_CSI_RECORD=$DEMO_RECORD
pgrep -x sounder >/dev/null && { echo "a sounder is running on this host; stop it first"; exit 1; }
[ -f "$CONF" ] || { echo "no config $CONF (relative to $D)"; exit 1; }
[ "$SECS" -gt 90 ] || { echo "secs must be over 90 (the samplers start 40 s in)"; exit 1; }
S=$(mktemp -d /tmp/demo_run_XXXX)
setsid nohup python3 "$HERE/freeze_watch.py" --run-dir "$D/ap79_runs" --tag "$TAG" > "$S/freeze.txt" 2>&1 < /dev/null &
setsid nohup python3 "$HERE/mer_sampler.py" "$S/mer.txt" 15 $((SECS - 30)) 40 > /dev/null 2>&1 < /dev/null &
setsid nohup python3 "$HERE/spc_sample.py" "$S/spc.txt" 60,$((SECS / 2)),$((SECS - 40)) > "$S/spc.err" 2>&1 < /dev/null &
[ "$PLUG" = slots ] && export HOUDINI_SOAPY_ROOT=${HOUDINI_SLOTS_ROOT:-$HOME/houdini_slots}
export SOUNDER_DIR=$D HEALTH_S=5 HOUDINI_TX_HOST_STATUS=1 FILTERS="$FILT"
export HOUDINI_CORE_MAP=${HOUDINI_CORE_MAP:-main=15} HOUDINI_TX_CPU_AFFINITY=${HOUDINI_TX_CPU_AFFINITY:-18,19}
irq() {
  [ -n "${UE_SSH:-}" ] || return 0
  ssh -o ConnectTimeout=5 -o BatchMode=yes "$UE_SSH" 'grep -iE "rfdc|rf_data|usp_rf|xrfdc" /proc/interrupts' 2>/dev/null |
    awk '{n=0; for(i=2;i<=NF;i++) if($i ~ /^[0-9]+$/) n+=$i; else break; print $1, n, $NF}'
}
IRQ0=$(irq)
date -u +"start %H:%M:%S"
bash "$HERE/fstage_run.sh" "${STAGE:-DEMO}" "$TAG" "$CONF" "$SECS" 2>&1 | grep -E "launched|done|failed|Radios Not Found|TIMEOUT"
sleep 5
IRQ1=$(irq)
R=$(cat "ap79_runs/$TAG.current" 2>/dev/null); O=$D/ap79_runs/${STAGE:-DEMO}/$R
[ -n "$R" ] && [ -d "$O" ] || { echo "no run directory filed"; exit 1; }
cp "$S/mer.txt" "$O/mer.txt" 2>/dev/null; cp "$S/freeze.txt" "$O/freeze_watch.txt" 2>/dev/null
cat "$S/spc.txt" "$S/spc.err" > "$O/spectrum.txt" 2>/dev/null
echo "run dir: $O"
echo "== MER (15 s samples)"; cut -c1-160 "$O/mer.txt"
echo "== freeze watch"; tail -1 "$O/freeze_watch.txt"
echo "== spectrum"; cut -c1-200 "$O/spectrum.txt"
echo "== TX and constellation"; python3 "$HERE/demo_report.py" "$O" 2>&1 | grep -E "TX totals|TX status events|CNS" | cut -c1-200
if [ -n "${UE_SSH:-}" ]; then
  echo "== UE RFDC interrupts over the run"
  paste <(echo "$IRQ0") <(echo "$IRQ1") | awk '{printf "  %s %s -> %s (+%d)\n", $1, $2, $5, $5-$2}'
fi
rm -rf "$S"
python3 "$HERE/run_summary.py" "$O"
