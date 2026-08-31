#!/bin/bash
# Pad-campaign driver: back-to-back capture runs with the BS RX / UE TX debug
# instruments on, each into its own log directory, for the per-frame dumps the
# DEMO_VERIFICATION.md rows cite.
#
# Runs are numbered so a campaign can be extended without clobbering earlier
# evidence. Every knob has a default and an override:
#
#   PAD_RUNS="6 7"      which run indices to take        (default "4 5")
#   HOUDINI_MAX_FRAME   frames per run                   (default 2500)
#   CONF                config file, relative to Sounder (default houdini-ul.json)
#   SOUNDER_DIR         checkout to run from             (default: derived from
#                       this script's own location, so it follows the checkout
#                       it is committed in)
#
# Bench specifics (which machine, which addresses) live in DEMO_BENCH_RUNBOOK.md.
set -u

SOUNDER_DIR="${SOUNDER_DIR:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)}"
VENV="${VENV:-$HOME/houdini_test}"
PLUGIN_PATH="${SOAPY_SDR_PLUGIN_PATH:-$VENV/lib/SoapySDR/modules0.8-3}"
CONF="${CONF:-files/houdini-ul.json}"
FRAMES="${HOUDINI_MAX_FRAME:-2500}"
RUNS="${PAD_RUNS:-4 5}"

cd "$SOUNDER_DIR" || { echo "no such directory: $SOUNDER_DIR" >&2; exit 1; }
[ -x ./build/sounder ] || { echo "no sounder binary in $SOUNDER_DIR/build" >&2; exit 1; }

echo "campaign: runs [$RUNS] x $FRAMES frames, conf $CONF, from $SOUNDER_DIR"

for i in $RUNS; do
  d="logs/pad$i"
  mkdir -p "$d" && rm -f "$d"/*
  env SOAPY_SDR_PLUGIN_PATH="$PLUGIN_PATH" \
      HOUDINI_MAX_FRAME="$FRAMES" HOUDINI_BS_RX_DEBUG=1 HOUDINI_UE_TX_DEBUG=1 \
      HOUDINI_BS_DUMP_FRAME="$d" HOUDINI_CSI_R_DEBUG=1 \
      ./build/sounder --view --conf_file "$CONF" > "$d/run.log" 2>&1
  echo "run $i rc=$?"
  sleep 3
done
echo CAMPAIGN-DONE
