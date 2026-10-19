#!/bin/bash
# usage: demo_dashboard.sh [options] [csi_server.py flags ...]
# The live CSI dashboard for the dual-band demo on the rig host, in control
# mode (DEMO_BENCH_RUNBOOK.md A4): the page offers the labelled configs and
# the replays; pick "Demo, cabled" and press Start. Ctrl+C stops the dashboard
# and any sounder it started. Each Start writes <log dir>/sounder_<UTC>.log;
# summarise one with tests/demo-verify/run_summary.py <log>.
# Options (the defaults are the reference rig's, runbook A1 to A4; a relative
# path is relative to CC/Sounder, as the configs' are):
#   --soapy-root DIR       the host-plugin prefix of the radios' release
#                          (default ~/houdini_hil)
#   --topology F           the topology every run uses (default
#                          files/topology-houdini-dualband-swap.json, the bench as cabled)
#   --no-topology          each config's own serial_file instead (the X-band
#                          chain's plan roles, runbook A2b)
#   --core-map M, --tx-cpu-affinity L
#                          the sounder's thread placement and pacer cpus
#                          (default main=16 and 18,19; pacer_core_check.py
#                          confirms or corrects them after each boot of the host)
#   --log-dir DIR          (default ~/demo_logs)
#   --replay-dir DIR       the recordings the page offers (default ~/demo_rec/replays)
#   --port N               the page's port (default 8080)
#   --venv DIR             the SoapySDR venv (default ~/houdini_test)
# Any other flag is passed to csi_server.py.
set -u
ROOT=$HOME/houdini_hil; TOPO=files/topology-houdini-dualband-swap.json; CM=main=16; TA=18,19
LOGS=$HOME/demo_logs; REPLAYS=$HOME/demo_rec/replays; PORT=8080; VENV=$HOME/houdini_test; EXTRA=()
while [ $# -gt 0 ]; do
  opt=$1; val=; shift
  case $opt in
    --no-topology) TOPO=; continue;;
    --soapy-root=*|--topology=*|--core-map=*|--tx-cpu-affinity=*|--log-dir=*|--replay-dir=*|--port=*|--venv=*)
      val=${opt#*=}; opt=${opt%%=*};;
    --soapy-root|--topology|--core-map|--tx-cpu-affinity|--log-dir|--replay-dir|--port|--venv)
      [ $# -gt 0 ] || { echo "demo_dashboard.sh: $opt needs a value (the options are at the head of this file)" >&2; exit 2; }
      val=$1; shift;;
    *) EXTRA+=("$opt"); continue;;
  esac
  case $opt in
    --soapy-root) ROOT=$val;; --topology) TOPO=$val;; --core-map) CM=$val;; --tx-cpu-affinity) TA=$val;;
    --log-dir) LOGS=$val;; --replay-dir) REPLAYS=$val;; --port) PORT=$val;; --venv) VENV=$val;;
  esac
done
D=$(cd "$(dirname "$0")/../.." && pwd)
cd "$D" || exit 1
source "$VENV/bin/activate" || exit 1
ls "$ROOT"/lib/SoapySDR/modules*/libHoudiniSDRSupport.so > /dev/null 2>&1 || { echo "no Houdini host plugin under $ROOT" >&2; exit 1; }
[ -z "$TOPO" ] || [ -f "$TOPO" ] || { echo "no topology $TOPO (relative to $D)" >&2; exit 1; }
[ -x build/sounder ] || { echo "no build/sounder in $D: build it first" >&2; exit 1; }
if ss -ltn | grep -q ":$PORT "; then
  echo "port $PORT is in use: another dashboard runs" >&2
  ss -ltnp 2>/dev/null | grep ":$PORT " >&2
  exit 1
fi
pgrep -x sounder > /dev/null && { echo "a sounder is running on this host (pid $(pgrep -x sounder | tr '\n' ' ')); stop it first" >&2; exit 1; }
TOPOARG=()
[ -n "$TOPO" ] && TOPOARG=(--topology "$TOPO")
REPLAY=()
for r in "xband_chain_bench_60s.rec=Replay, X-band bench" \
         "xband_ota_60s.rec=Replay, X-band over the air" \
         "xband_ota_blockage_30s.rec=Replay, over the air with blockage" \
         "xband_ota_noisy_sub6_60s.rec=Replay, over the air, noisy sub-6"; do
  [ -f "$REPLAYS/${r%%=*}" ] && REPLAY+=(--replay "$REPLAYS/$r")
done
mkdir -p "$LOGS"
echo "dashboard: http://$(hostname -I | awk '{print $1}'):$PORT/ (or ssh -L $PORT:localhost:$PORT $(whoami)@<this host>, then http://localhost:$PORT/)"
echo "pick \"Demo, cabled\" and press Start; Ctrl+C here stops everything"
exec python3 csi_gui/csi_server.py --control --http-host 0.0.0.0 --http-port "$PORT" --configs labelled \
  --soapy-root "$ROOT" "${TOPOARG[@]}" \
  "--sounder-arg=--core_map=$CM" "--sounder-arg=--tx_cpu_affinity=$TA" --sounder-arg=--tx_host_status \
  "${REPLAY[@]}" --log-dir "$LOGS" --conf files/houdini-dualband-xw-steer-slots.json "${EXTRA[@]}"
