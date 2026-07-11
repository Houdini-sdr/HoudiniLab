#!/bin/bash
# rx_drop_probe.sh — measure kernel UDP drops around one rx-recorder run.
#
# The AP-3 single-socket experiment harness (docs/RX_MAX_RATE.md): snapshots
# the kernel UDP counters (netstat -su) before/after a capture, so each run
# reports exactly how many datagrams the kernel delivered vs dropped
# (RcvbufErrors = socket-buffer-full, the measured wall at max rate).
#
# Usage:
#   rx_drop_probe.sh <conf.json> [--worker-cpus LIST] [--app-cpu N]
#                    [--storepath DIR] [--duration SEC]
#
#   --worker-cpus  pin the plugin's recvmmsg worker (stream kwarg
#                  cpu_affinity) — the prime suspect experiment: put it on
#                  a fast core (rig B: cores 15-19 are the ~4 GHz ones)
#   --app-cpu      taskset the rx-recorder process (capture thread) itself
#   --duration     override duration_sec in the config for this run
#
# Run from CC/Sounder (expects ./build/rx-recorder). Root not required.

set -euo pipefail

CONF=${1:?usage: rx_drop_probe.sh conf.json [--worker-cpus LIST] [--app-cpu N]}
shift
WORKER_CPUS=""
APP_CPU=""
STOREPATH="/tmp/rx-probe"
DURATION=""
while [ $# -gt 0 ]; do
  case "$1" in
    --worker-cpus) WORKER_CPUS="$2"; shift 2 ;;
    --app-cpu)     APP_CPU="$2";     shift 2 ;;
    --storepath)   STOREPATH="$2";   shift 2 ;;
    --duration)    DURATION="$2";    shift 2 ;;
    *) echo "unknown arg: $1" >&2; exit 2 ;;
  esac
done

udp_stat() {  # $1 = "received" | "buffer"
  netstat -su | awk -v want="$1" '
    /^Udp:/    { in_udp = 1; next }
    /^[A-Za-z]/{ in_udp = 0 }
    in_udp && want == "received" && /packets received/       { print $1; exit }
    in_udp && want == "buffer"   && /receive buffer errors/  { print $1; exit }'
}

RUN_CONF="$CONF"
if [ -n "$WORKER_CPUS" ] || [ -n "$DURATION" ]; then
  RUN_CONF=$(mktemp /tmp/rx-probe-conf.XXXX.json)
  python3 - "$CONF" "$RUN_CONF" "$WORKER_CPUS" "$DURATION" <<'EOF'
import json, sys
src, dst, cpus, dur = sys.argv[1:5]
conf = json.load(open(src))
if cpus:
    conf.setdefault("stream", {})["cpu_affinity"] = cpus
if dur:
    conf["duration_sec"] = float(dur)
json.dump(conf, open(dst, "w"), indent=2)
EOF
fi

RX=(./build/rx-recorder -conf_file "$RUN_CONF" -storepath "$STOREPATH")
[ -n "$APP_CPU" ] && RX=(taskset -c "$APP_CPU" "${RX[@]}")

echo "== rx_drop_probe: conf=$RUN_CONF worker_cpus='${WORKER_CPUS:-default}'" \
     "app_cpu='${APP_CPU:-default}'"
RECV0=$(udp_stat received); DROP0=$(udp_stat buffer)

"${RX[@]}" 2>&1 | grep -E "Capture plan|Recorded slots|Stream gaps|Backward|resync|Untrusted|Overflows|exact|widened" || true

RECV1=$(udp_stat received); DROP1=$(udp_stat buffer)
DRECV=$((RECV1 - RECV0)); DDROP=$((DROP1 - DROP0))
TOTAL=$((DRECV + DDROP))
echo "== kernel UDP delta: delivered=$DRECV dropped(RcvbufErrors)=$DDROP"
if [ "$TOTAL" -gt 0 ]; then
  echo "== kernel drop rate: $(python3 -c "print(f'{100.0*$DDROP/$TOTAL:.2f}')") %"
fi
[ "$RUN_CONF" != "$CONF" ] && rm -f "$RUN_CONF"
