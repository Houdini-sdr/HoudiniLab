#!/bin/bash
# Pad-campaign driver: back-to-back capture runs with the BS RX / UE TX debug
# instruments on, each into its own log directory, for the per-frame dumps the
# DEMO_VERIFICATION.md rows cite.
#
#   PAD_RUNS="6 7"      which run indices to take        (default "4 5")
#   HOUDINI_MAX_FRAME   frames per run                   (default 2500)
#   CONF                config file, relative to Sounder (default houdini-ul.json)
#   SOUNDER_DIR         checkout to run from             (default: derived from
#                       this script's own location)
#   VENV                Soapy/venv prefix                (default ~/houdini_test)
#   PAD_FORCE=1         allow overwriting a non-empty logs/padN
#
# Exits non-zero if ANY run fails, so a caller cannot read a green exit over a
# campaign where every run died.
#
# Bench specifics (which machine, which addresses) live in DEMO_BENCH_RUNBOOK.md.
set -u

SOUNDER_DIR="${SOUNDER_DIR:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)}"
VENV="${VENV:-$HOME/houdini_test}"
CONF="${CONF:-files/houdini-ul.json}"
FRAMES="${HOUDINI_MAX_FRAME:-2500}"
RUNS="${PAD_RUNS:-4 5}"
FORCE="${PAD_FORCE:-0}"

cd "$SOUNDER_DIR" || { echo "no such directory: $SOUNDER_DIR" >&2; exit 1; }
[ -x ./build/sounder ] || { echo "no sounder binary in $SOUNDER_DIR/build" >&2; exit 1; }

# Same environment every other launcher in this repo uses. Without the plugin
# path every radio open dies with Device::make() "no match"; without the venv
# lib path the binary resolves the wrong libSoapySDR.
# shellcheck disable=SC1091
[ -f "$VENV/bin/activate" ] && . "$VENV/bin/activate"
export SOAPY_SDR_PLUGIN_PATH="${SOAPY_SDR_PLUGIN_PATH:-$VENV/lib/SoapySDR/modules0.8-3}"
# PREPEND, do not default-if-unset. `activate` never touches this variable, so
# on any rig where a Vivado/CUDA/module profile already set it, `:-` would leave
# $VENV/lib out entirely and the sounder would link the system libSoapySDR --
# producing evidence gathered against a different Soapy stack.
export LD_LIBRARY_PATH="$VENV/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"

# Refuse to silently destroy evidence a ledger row may cite.
for i in $RUNS; do
  d="logs/pad$i"
  if [ "$FORCE" != "1" ] && [ -d "$d" ] && [ -n "$(ls -A "$d" 2>/dev/null)" ]; then
    echo "refusing to overwrite non-empty $d (set PAD_FORCE=1 to replace)" >&2
    exit 1
  fi
done

# The topology the chosen CONF actually points at, with the shipped default as
# a fallback (teardown_framer.py's own default is the same file).
TOPO=$(python3 -c "import json,sys
try:
    print(json.load(open(sys.argv[1])).get('serial_file') or 'files/topology-houdini.json')
except Exception:
    print('files/topology-houdini.json')" "$CONF" 2>/dev/null || echo files/topology-houdini.json)

echo "campaign: runs [$RUNS] x $FRAMES frames, conf $CONF, topology $TOPO, from $SOUNDER_DIR"
rc_all=0
for i in $RUNS; do
  d="logs/pad$i"
  mkdir -p "$d" && rm -f "$d"/*
  # Release any framer a previous run left armed, exactly as the demo launcher
  # does; without it the next radio open fails with "a stream is open".
  # Tear down the topology THIS CONF names, not a hardcoded one: CONF is an
  # advertised knob and the tree ships several topologies, so a hardcoded file
  # would issue device-touching framer releases against the wrong bench while
  # the radios actually opened kept their stale state.
  for attempt in 1 2; do
    timeout 90 python3 csi_gui/teardown_framer.py --topology "$TOPO" \
        > "$d/teardown.log" 2>&1
    sleep 5
    env HOUDINI_MAX_FRAME="$FRAMES" HOUDINI_BS_RX_DEBUG=1 HOUDINI_UE_TX_DEBUG=1 \
        HOUDINI_BS_DUMP_FRAME="$d" HOUDINI_CSI_R_DEBUG=1 \
        ./build/sounder --view --conf_file "$CONF" > "$d/run.log" 2>&1
    rc=$?
    # The cold start is known-flaky (the demo launcher retries for the same
    # reason), so one attempt failing must not condemn a whole campaign.
    [ "$rc" -eq 0 ] && break
    echo "run $i attempt $attempt rc=$rc, retrying"
  done
  echo "run $i rc=$rc"
  [ "$rc" -eq 0 ] || rc_all=1
  sleep 3
done
echo "CAMPAIGN-DONE rc=$rc_all"
exit "$rc_all"
