#!/bin/bash
# usage: fstage_run.sh [options] <stage F0..F4b> <rung tag> <conf> <secs>
#   --filters STATE    the filter state, noted in the run's stage.txt
#   --csi-dump N       the constellation dump's skip (the sounder's --csi_dump; default 60)
#   --health-s N       the link-health period, s (default 5)
#   --sounder-dir DIR  the checkout to run (default: the one this script is in)
#   --soapy-root DIR, --venv DIR, --record FILE, --sounder-arg ARG: passed to run_rung.sh
# One filter-staging run (Houdini-Streaming docs/DEMO_FREQUENCY_PLAN.md 6.1b): run_rung.sh with the
# measurement dumps on, then every per-run artefact moved into
# ap79_runs/<stage>/<tag>_<T>/ so runs never overwrite. Rig tool. Analyse
# with run_summary.py or fstage_report.py.
set -u
HERE=$(cd "$(dirname "$0")" && pwd)
FILT=; CSID=60; HEALTH=5; SD=$(cd "$(dirname "$0")/../.." && pwd); PASS=()
while [ $# -gt 0 ]; do
  case $1 in
    --filters) FILT=$2; shift 2;;
    --csi-dump) CSID=$2; shift 2;;
    --health-s) HEALTH=$2; shift 2;;
    --sounder-dir) SD=$2; shift 2;;
    --soapy-root|--venv|--record|--sounder-arg) PASS+=("$1" "$2"); shift 2;;
    --sounder-arg=*) PASS+=("$1"); shift;;
    -*) echo "fstage_run.sh: unknown option $1 (the options are at the head of this file)" >&2; exit 2;;
    *) break;;
  esac
done
[ $# -eq 4 ] || { echo "usage: fstage_run.sh [options] <stage> <tag> <conf> <secs>" >&2; exit 2; }
ST=$1; TAG=$2; CONF=$3; SECS=$4
cd "$SD" || exit 1
D=ap79_runs
rm -f "$D"/cns_dump*.bin "$D/beacon_ram.bin" "$D/gold.bin"
# State records already here belong to a run this script did not launch (a
# direct run_rung.sh): set them aside, or the filing below takes them as this
# run's own.
mkdir -p "$D/unfiled" && mv "$D"/rfdc_*.txt "$D"/modev_*.txt "$D/unfiled/" 2>/dev/null
RW=$(mktemp -d /tmp/rw_XXXX)
DUMPS=("--sounder-arg=--csi_dump=$CSID" --sounder-arg=--bs_rx_debug --sounder-arg=--dump_beacon
       "--sounder-arg=--dump_resync_win=$RW")
# SYN retransmits on this host around the run: an open that stalls about 1 s
# before connecting (cause unmeasured; a lost SYN is one candidate) races the
# A/B build's 1 s device timeout and reads as "Radios Not Found".
SYN0=$(nstat -az TcpExtTCPSynRetrans 2>/dev/null | awk '/SynRetrans/{print $2}')
# A launch that failed wrote no record: stop here rather than file into the
# previous run's directory. The checkout is passed as the absolute path
# already entered (run_rung.sh cds again).
bash "$HERE/run_rung.sh" --sounder-dir "$PWD" --health-s "$HEALTH" "${DUMPS[@]}" "${PASS[@]}" "$TAG" "$CONF" "$SECS" ||
  { rmdir "$RW"; echo "run_rung.sh failed; nothing filed"; exit 1; }
# Wait for THIS run's sounder (run_rung.sh recorded its pid), not any sounder on the host.
SP=$(cat "$D/$TAG.pid")
while [ -n "$SP" ] && [ "$(cat "/proc/$SP/comm" 2>/dev/null)" = sounder ]; do sleep 5; done; sleep 3
# A sounder that exits early (a radio that would not open) leaves this run's
# dashboard up until its own timeout, holding the ports the next run needs.
CP=$(cat "$D/$TAG.csipid" 2>/dev/null)
[ -n "$CP" ] && [ "$(cat "/proc/$CP/comm" 2>/dev/null)" = timeout ] && kill -INT "$CP" 2>/dev/null
R=$(cat "$D/$TAG.current"); T=${R#"${TAG}"_}
O=$D/$ST/$R; mkdir -p "$O"
mv "$D/$R.log" "$D/${TAG}_csi_$T.log" "$D/${TAG}_cpu_$T.log" "$D/${TAG}_threads_$T.log" "$O/" 2>/dev/null
mv "$D"/cns_dump*.bin "$D/beacon_ram.bin" "$O/" 2>/dev/null
# The per-node RFDC state records (RFDC_SNAPSHOT: the MTS and tile state,
# written at stream start and at the end of the run under the sounder's --dump_dir):
# a run that moves between sessions (a pilot seat, a level) is traced from them.
mv "$D"/rfdc_*.txt "$O/" 2>/dev/null
mv "$D"/modev_*.txt "$O/" 2>/dev/null  # each node's mode-V bring-up record
mv "$RW" "$O/resync"
SYN1=$(nstat -az TcpExtTCPSynRetrans 2>/dev/null | awk '/SynRetrans/{print $2}')
echo "stage $ST filters: ${FILT:-unset}" > "$O/stage.txt"
echo "TcpExtTCPSynRetrans before ${SYN0:-?} after ${SYN1:-?}" >> "$O/stage.txt"
echo "done $O $(date -u +%H:%M:%S)"
