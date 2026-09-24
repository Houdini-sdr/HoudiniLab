#!/bin/bash
# usage: [FILTERS=<state>] fstage_run.sh <stage F0..F4b> <rung tag> <conf> <secs>
# One filter-staging run (DEMO_FREQUENCY_PLAN 6.1b): run_rung.sh with the
# measurement dumps on, then every per-run artefact moved into
# ap79_runs/<stage>/<tag>_<T>/ so runs never overwrite. Rig tool: the paths are
# the AP-79 rig host's (see run_rung.sh). Analyse with fstage_report.py.
set -u
HERE=$(cd "$(dirname "$0")" && pwd)
ST=$1; TAG=$2; CONF=$3; SECS=$4
cd "${SOUNDER_DIR:-$HOME/repos/HoudiniLab-ap79/CC/Sounder}" || exit 1  # SOUNDER_DIR: another worktree's build
D=ap79_runs
rm -f $D/cns_dump*.bin $D/beacon_ram.bin $D/gold.bin
RW=$(mktemp -d /tmp/rw_XXXX)
export HOUDINI_CSI_DUMP=60 HOUDINI_BS_RX_DEBUG=1 HOUDINI_DUMP_BEACON=1 HOUDINI_DUMP_RESYNC_WIN=$RW
# SYN retransmits on this host around the run: an open that stalls about 1 s
# before connecting (cause unmeasured; a lost SYN is one candidate) races the
# A/B build's 1 s device timeout and reads as "Radios Not Found".
SYN0=$(nstat -az TcpExtTCPSynRetrans 2>/dev/null | awk '/SynRetrans/{print $2}')
bash "$HERE/run_rung.sh" $TAG $CONF $SECS 5
while pgrep -x sounder >/dev/null; do sleep 5; done; sleep 3
R=$(cat $D/$TAG.current); T=${R#${TAG}_}
O=$D/$ST/$R; mkdir -p $O
mv $D/$R.log $D/${TAG}_csi_$T.log $D/${TAG}_cpu_$T.log $D/${TAG}_threads_$T.log $O/ 2>/dev/null
mv $D/cns_dump*.bin $D/beacon_ram.bin $O/ 2>/dev/null
mv $RW $O/resync
SYN1=$(nstat -az TcpExtTCPSynRetrans 2>/dev/null | awk '/SynRetrans/{print $2}')
echo "stage $ST filters: ${FILTERS:-unset}" > $O/stage.txt
echo "TcpExtTCPSynRetrans before ${SYN0:-?} after ${SYN1:-?}" >> $O/stage.txt
echo "done $O $(date -u +%H:%M:%S)"
