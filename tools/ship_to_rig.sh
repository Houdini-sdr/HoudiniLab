#!/bin/bash
# usage: tools/ship_to_rig.sh <user@rig-host> <rig worktree> [<branch>]
#
# Ship <branch> (default: the current branch) from this checkout into an existing
# worktree of the same repository on the rig host, build it there with the host's
# own toolchain, and run ctest. Run it from the lane's checkout. It fails closed at
# every step, each one a trap already paid for:
#
#   - Before anything is copied it refuses while a sounder runs on the rig host,
#     while the host is busy (1 min load average over LOAD_MAX, default 2: another
#     lane's hardware tests measure host timing, and a build disturbs them), and
#     while the rig worktree has uncommitted changes (a config edited on the rig,
#     say a re-derived tx_advance, would be silently reverted by the reset).
#   - It bundles <rig HEAD>..<branch> (the rig worktree's HEAD must be an ancestor
#     of <branch>), copies the bundle, and fetches INSIDE the rig worktree:
#     FETCH_HEAD is per worktree, so a fetch in the main checkout leaves this
#     worktree on the old commit and the build silently rebuilds it.
#   - It relinks CC/Sounder/mufft after the reset when the reset left it an empty
#     directory (`ln -sfn` onto a directory would nest the link), pointing at the
#     rig repository's main checkout; a populated mufft is left alone.
#   - It configures and builds with the venv sourced (VENV, default ~/houdini_test)
#     and a fresh build directory's cmake pointed at the venv's SoapySDR
#     (SoapySDR_DIR; without it the configure fails), never under `set -e`
#     (sourcing the venv's activate under it aborts the shell). A failed configure
#     leaves no cache behind, so the next ship configures again.
#   - It gates on make's own exit status: ctest after a failed build runs the OLD
#     binaries and passes.
#   - With CHECK_STRING set, it requires the new sounder binary to carry that
#     string (pick one only the new code emits; no single quotes).
#   - Any failure after the reset puts the worktree back on its previous commit and
#     rebuilds it, so the checkout's sources and its binary never disagree.
#   - The rig host's aarch64 toolchain reaches fewer headers transitively than an
#     x86 development machine: a C++ change is not done until it built here.
#
# Environment: LOAD_MAX (default 2), VENV (default ~/houdini_test on the rig),
# CHECK_STRING (optional), JOBS (default 8). Exit 0 only when every step passed.
set -u

relink_mufft() {  # in the worktree root
  local M=CC/Sounder/mufft MAIN
  MAIN=$(cd "$(git rev-parse --git-common-dir)/.." && pwd)
  if [ ! -L "$M" ] && [ -z "$(ls -A "$M" 2>/dev/null)" ]; then
    rmdir "$M" 2>/dev/null
    ln -s "$MAIN/$M" "$M" || return 1
  fi
  [ -e "$M/" ]
}

configure_if_needed() {  # in CC/Sounder, the venv active
  [ -f build/CMakeCache.txt ] && return 0
  mkdir -p build
  local SDR_DIR=""
  [ -d "$VIRTUAL_ENV/share/cmake/SoapySDR" ] && SDR_DIR="-DSoapySDR_DIR=$VIRTUAL_ENV/share/cmake/SoapySDR"
  # shellcheck disable=SC2086
  (cd build && cmake .. -DCMAKE_BUILD_TYPE=Release $SDR_DIR > cmake.log 2>&1) && return 0
  rm -f build/CMakeCache.txt  # a failed configure leaves no cache, so the next run configures again
  return 1
}

build_and_test() {  # in the worktree root, the venv active; 0 only when all passed
  cd CC/Sounder || return 1
  configure_if_needed || { echo "FAIL: cmake (build/cmake.log)"; cd ../..; return 1; }
  (cd build && nice -n 10 make -j"${JOBS:-8}" > make.log 2>&1)
  local RC=$?
  if [ "$RC" -ne 0 ]; then
    echo "FAIL: make exited $RC"; grep -E "error|Error" build/make.log | head -5; cd ../..; return 1
  fi
  if [ -n "${CHECK_STRING:-}" ] && ! strings build/sounder | grep -qF -- "$CHECK_STRING"; then
    echo "FAIL: the new sounder binary does not carry '$CHECK_STRING'"; cd ../..; return 1
  fi
  (cd build && nice -n 10 ctest -j4 > ctest.log 2>&1)
  RC=$?
  tail -3 build/ctest.log
  cd ../..
  [ "$RC" -eq 0 ] || { echo "FAIL: ctest exited $RC (CC/Sounder/build/ctest.log)"; return 1; }
}

if [ "${1:-}" = "--preflight" ]; then
  # The rig-host checks that must pass before anything is copied.
  WT=$2
  cd "$WT" || { echo "FAIL: no worktree $WT"; exit 1; }
  if pgrep -x sounder >/dev/null; then echo "FAIL: a sounder is running on this host"; exit 1; fi
  LOAD=$(cut -d' ' -f1 /proc/loadavg)
  if awk -v l="$LOAD" -v m="${LOAD_MAX:-2}" 'BEGIN { exit !(l > m) }'; then
    echo "FAIL: the host is busy (load $LOAD over ${LOAD_MAX:-2}); another lane may be measuring"; exit 1
  fi
  DIRTY=$(git status --porcelain --untracked-files=no | grep -v " CC/Sounder/mufft$")
  if [ -n "$DIRTY" ]; then
    echo "FAIL: the rig worktree has uncommitted changes; commit, stash or discard them first:"; echo "$DIRTY" | head -10
    exit 1
  fi
  exit 0
fi

if [ "${1:-}" = "--remote" ]; then
  # The rig-host half, run by the local half below.
  WT=$2; BUNDLE=$3; REF=$4
  cd "$WT" || { echo "FAIL: no worktree $WT"; rm -f "$BUNDLE"; exit 1; }
  # The preflight ran seconds ago; a sounder started since would lose its binary.
  if pgrep -x sounder >/dev/null; then echo "FAIL: a sounder started on this host after the preflight; nothing changed"; rm -f "$BUNDLE"; exit 1; fi
  OLD=$(git rev-parse HEAD)
  rollback() {
    [ "$(git rev-parse HEAD)" = "$OLD" ] && return
    echo "rolling the worktree back to ${OLD:0:12} and rebuilding it"
    # shellcheck disable=SC1090
    [ -n "${VIRTUAL_ENV:-}" ] || source "${VENV:-$HOME/houdini_test}/bin/activate"
    git reset -q --hard "$OLD" && relink_mufft &&
      (cd CC/Sounder && configure_if_needed && cd build && nice -n 10 make -j"${JOBS:-8}" > make.log 2>&1) &&
      echo "rolled back: sources and binary at ${OLD:0:12}" || echo "ROLLBACK INCOMPLETE: check $WT by hand"
  }
  if [ -n "$BUNDLE" ]; then
    git fetch -q "$BUNDLE" "$REF" || { echo "FAIL: fetch from the bundle"; rm -f "$BUNDLE"; exit 1; }
    rm -f "$BUNDLE"
    git reset -q --hard FETCH_HEAD || { echo "FAIL: reset to the fetched commit"; rollback; exit 1; }
  fi
  relink_mufft || { echo "FAIL: CC/Sounder/mufft does not resolve"; rollback; exit 1; }
  echo "rig worktree at $(git log --oneline -1 | cut -c1-72)"
  # shellcheck disable=SC1090
  source "${VENV:-$HOME/houdini_test}/bin/activate" || { echo "FAIL: no venv ${VENV:-$HOME/houdini_test}"; rollback; exit 1; }
  if ! build_and_test; then rollback; exit 1; fi
  echo "OK: built and tested at $(git log --oneline -1 | cut -c1-12)"
  exit 0
fi

[ $# -ge 2 ] || { sed -n '2,3p' "$0"; exit 2; }
RIG=$1; WT=$2; BR=${3:-$(git rev-parse --abbrev-ref HEAD)}
case "${CHECK_STRING:-}" in *"'"*) echo "FAIL: CHECK_STRING must not contain a single quote"; exit 2;; esac
HEAD_LOCAL=$(git rev-parse --verify "$BR^{commit}") || { echo "FAIL: no branch $BR here"; exit 1; }
RIG_HEAD=$(ssh -o BatchMode=yes "$RIG" "git -C '$WT' rev-parse HEAD") || { echo "FAIL: cannot read $WT on $RIG"; exit 1; }
REMOTE_SELF=/tmp/ship_to_rig_$$.sh
scp -q "$0" "$RIG:$REMOTE_SELF" || { echo "FAIL: copy this script to $RIG"; exit 1; }
ENVS="LOAD_MAX='${LOAD_MAX:-2}' VENV='${VENV:-}' CHECK_STRING='${CHECK_STRING:-}' JOBS='${JOBS:-8}'"
ssh -o BatchMode=yes "$RIG" "$ENVS bash $REMOTE_SELF --preflight '$WT'" || { ssh -o BatchMode=yes "$RIG" "rm -f $REMOTE_SELF"; exit 1; }
BUNDLE_REMOTE=""
if [ "$RIG_HEAD" != "$HEAD_LOCAL" ]; then
  git merge-base --is-ancestor "$RIG_HEAD" "$HEAD_LOCAL" ||
    { echo "FAIL: the rig worktree's HEAD ${RIG_HEAD:0:12} is not an ancestor of $BR (${HEAD_LOCAL:0:12}); move it first"
      ssh -o BatchMode=yes "$RIG" "rm -f $REMOTE_SELF"; exit 1; }
  B=$(mktemp /tmp/ship_to_rig_XXXX.bundle)
  git bundle create -q "$B" "$RIG_HEAD..$BR" ||
    { echo "FAIL: bundle $RIG_HEAD..$BR"; rm -f "$B"; ssh -o BatchMode=yes "$RIG" "rm -f $REMOTE_SELF"; exit 1; }
  BUNDLE_REMOTE=/tmp/ship_to_rig_$$.bundle
  scp -q "$B" "$RIG:$BUNDLE_REMOTE" ||
    { echo "FAIL: copy the bundle"; rm -f "$B"; ssh -o BatchMode=yes "$RIG" "rm -f $REMOTE_SELF $BUNDLE_REMOTE"; exit 1; }
  rm -f "$B"
  echo "shipping $BR ${RIG_HEAD:0:12}..${HEAD_LOCAL:0:12} to $RIG:$WT"
else
  echo "$RIG:$WT is already at ${HEAD_LOCAL:0:12}; building and testing it"
fi
ssh -o BatchMode=yes "$RIG" "$ENVS bash $REMOTE_SELF --remote '$WT' '$BUNDLE_REMOTE' '$BR'; rc=\$?; rm -f $REMOTE_SELF; exit \$rc"
