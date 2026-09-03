#!/bin/bash
# Interleaved beacon-shape campaign THROUGH THE SOUNDER: one binary, one
# config, only `beacon_type` varied, each shape run for a fixed wall time per
# round with the ORDER ROTATED between rounds. Written for AP-66 (the NR
# acquisition architecture, `nr_pss`) and kept general.
#
# Interleaved because a non-interleaved comparison on this bench produced a
# confident 2.8x conclusion that had to be retracted (DEMO_VERIFICATION 8.100):
# the clock pair wanders ~0.06 ppm within a session, so back-to-back legs of
# ONE shape measure the clock, not the shape.
#
#   SHAPES="legacy nr nr_pss"   which shapes                    (default)
#   ROUNDS=3                    rounds; round k starts at shape k (Latin square)
#   RUN_S=60                    wall seconds per run
#   CONF=files/houdini-ul.json  base config; a per-shape copy is written beside it
#   OUT=logs/shape_<date>       log directory
#   ATTEMPTS=3                  launch attempts per run before giving up (1 for
#                               a level sweep where "no lock" IS the result)
#   SOUNDER_DIR / VENV          as run_pad_campaign.sh
#
# Output: $OUT/<shape>_r<k>.log per run, $OUT/campaign.log, and a gate_summary
# over all logs at the end. Exit code is non-zero if any run failed to START
# (a run ending on the wall clock is the normal outcome, not a failure).
set -u

SOUNDER_DIR="${SOUNDER_DIR:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)}"
VENV="${VENV:-$HOME/houdini_test}"
CONF="${CONF:-files/houdini-ul.json}"
SHAPES="${SHAPES:-legacy nr nr_pss}"
ROUNDS="${ROUNDS:-3}"
RUN_S="${RUN_S:-60}"
OUT="${OUT:-logs/shape_$(date +%Y%m%d-%H%M)}"

cd "$SOUNDER_DIR" || { echo "no such directory: $SOUNDER_DIR" >&2; exit 1; }
[ -x ./build/sounder ] || { echo "no sounder binary in $SOUNDER_DIR/build" >&2; exit 1; }
# shellcheck disable=SC1091
[ -f "$VENV/bin/activate" ] && . "$VENV/bin/activate"
export SOAPY_SDR_PLUGIN_PATH="${SOAPY_SDR_PLUGIN_PATH:-$VENV/lib/SoapySDR/modules0.8-3}"
export LD_LIBRARY_PATH="$VENV/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
# Run until the wall clock says stop, never until max_frame.
export HOUDINI_MAX_FRAME="${HOUDINI_MAX_FRAME:-2000000000}"

mkdir -p "$OUT"
TOPO=$(python3 -c "import json,sys
try:
    print(json.load(open(sys.argv[1])).get('serial_file') or 'files/topology-houdini.json')
except Exception:
    print('files/topology-houdini.json')" "$CONF" 2>/dev/null || echo files/topology-houdini.json)

# One config per shape, differing from the base in beacon_type ONLY. Written
# beside the base so every relative path in it resolves identically.
read -r -a shape_arr <<< "$SHAPES"
declare -A conf_of
for s in "${shape_arr[@]}"; do
  c="${CONF%.json}-$s.json"
  python3 - "$CONF" "$c" "$s" <<'PY'
import json, sys
src, dst, shape = sys.argv[1:4]
d = json.load(open(src))
d["beacon_type"] = shape
json.dump(d, open(dst, "w"), indent=4)
PY
  conf_of[$s]="$c"
done

log="$OUT/campaign.log"
echo "campaign: shapes [$SHAPES] x $ROUNDS rounds x ${RUN_S}s, conf $CONF, topology $TOPO, out $OUT" | tee "$log"
echo "binary: $(ls -la --time-style=+%F_%T build/sounder | awk '{print $6, $7}')  git: $(git -C "$SOUNDER_DIR" log --oneline -1 2>/dev/null)" | tee -a "$log"
rc_all=0
n=${#shape_arr[@]}
for ((k = 0; k < ROUNDS; ++k)); do
  for ((j = 0; j < n; ++j)); do
    s="${shape_arr[$(( (k + j) % n ))]}"
    f="$OUT/${s}_r$((k + 1)).log"
    echo "[$(date +%T)] round $((k + 1)) shape $s -> $f" | tee -a "$log"
    started=0
    for attempt in $(seq 1 "${ATTEMPTS:-3}"); do
      # Release anything a previous run left holding a board, then the framer.
      python3 tools/rig_release_holders.py >> "$log" 2>&1 || true
      # Named so that the campaign's own "*_r*.log" glob (and gate_summary's
      # caller) never sweeps it in as a run: the first campaign did exactly
      # that and the aggregate refused to run over nine empty "runs".
      timeout 90 python3 csi_gui/teardown_framer.py --topology "$TOPO" > "$OUT/td-${s}-$((k + 1)).txt" 2>&1
      sleep 5
      # SIGTERM at the wall clock is the demo launcher's own shutdown path, so
      # streams close the way they do in the demo; a hard kill 15 s later is
      # the backstop. rc 124 (timed out) is therefore the SUCCESS code here.
      timeout -k 15 "$RUN_S" ./build/sounder --view --conf_file "${conf_of[$s]}" > "$f" 2>&1
      rc=$?
      if [ "$rc" -eq 124 ] || grep -q "lock CONFIRMED" "$f"; then started=1; break; fi
      echo "  attempt $attempt rc=$rc (no lock, run ended early), retrying" | tee -a "$log"
      cp "$f" "$f.attempt$attempt"
    done
    echo "  rc=$rc started=$started accepts=$(grep -c 'beacon alive on the anchored grid' "$f")" | tee -a "$log"
    [ "$started" -eq 1 ] || rc_all=1
    sleep 3
  done
done
python3 tools/rig_release_holders.py >> "$log" 2>&1 || true
echo "=== gate_summary ===" | tee -a "$log"
python3 tests/demo-verify/gate_summary.py --json "$OUT/gate.json" "$OUT"/*_r*.log 2>&1 | tee -a "$log"
for s in "${shape_arr[@]}"; do rm -f "${conf_of[$s]}"; done
echo "CAMPAIGN-DONE rc=$rc_all" | tee -a "$log"
exit "$rc_all"
