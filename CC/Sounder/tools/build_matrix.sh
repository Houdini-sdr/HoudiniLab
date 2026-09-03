#!/usr/bin/env bash
# Build the sounder for every RADIO_TYPE this host can build, out of tree, so
# a change that breaks a compile for a platform we cannot run is caught the
# day it lands (baseline assessment B1; docs/RADIO_PLATFORM_SEAM.md S0).
#
#   tools/build_matrix.sh [out-dir]      (default: /tmp/sounder-matrix)
#
# A type whose dependency is missing (PURE_UHD without libuhd) is reported as
# SKIPPED, never as passed. FORCE_BUILD_PATH=OFF keeps every matrix build's
# archive out of the source tree's build/ (the two would otherwise race).
set -u
SOUNDER_DIR="$(cd "$(dirname "$0")/.." && pwd)"
OUT="${1:-/tmp/sounder-matrix}"
mkdir -p "$OUT"
rc_all=0
printf '%-12s %-8s %s\n' "RADIO_TYPE" "result" "detail"
for t in SOAPY_IRIS SOAPY_UHD PURE_UHD; do
  b="$OUT/$t"
  log="$OUT/$t.log"
  if ! cmake -S "$SOUNDER_DIR" -B "$b" -DRADIO_TYPE="$t" -DFORCE_BUILD_PATH=OFF \
       -DSOUNDER_BUILD_TESTS=OFF > "$log" 2>&1; then
    if grep -qiE "Could not find a package configuration file provided by \"UHD\"|Could NOT find UHD|UHD.*not found" "$log"; then
      printf '%-12s %-8s %s\n' "$t" "SKIPPED" "libuhd not found on this host ($log)"
    else
      printf '%-12s %-8s %s\n' "$t" "FAIL" "configure failed ($log)"
      rc_all=1
    fi
    continue
  fi
  if cmake --build "$b" --target sounder -j"$(nproc)" >> "$log" 2>&1; then
    w=$(grep -c "warning:" "$log" || true)
    printf '%-12s %-8s %s\n' "$t" "PASS" "sounder built, $w warning(s) ($log)"
  else
    printf '%-12s %-8s %s\n' "$t" "FAIL" "build failed ($log)"
    rc_all=1
  fi
done
exit "$rc_all"
