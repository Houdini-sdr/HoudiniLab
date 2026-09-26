#!/bin/bash
# usage: tools/ship_to_rig.sh <user@rig-host> <rig worktree> [<branch>]
#
# Ship <branch> (default: the current branch) from this checkout into an existing
# worktree of the same repository on the rig host, build it there with the host's
# own toolchain, and run ctest. Run it from the lane's checkout. It fails closed at
# every step, each one a trap already paid for:
#
#   - It refuses while a sounder runs on the rig host, and while the host is busy
#     (1 min load average over LOAD_MAX, default 2): another lane's hardware tests
#     measure host timing, and a build disturbs them.
#   - It bundles <rig HEAD>..<branch> (the rig worktree's HEAD must be an ancestor
#     of <branch>), copies the bundle, and fetches INSIDE the rig worktree:
#     FETCH_HEAD is per worktree, so a fetch in the main checkout leaves this
#     worktree on the old commit and the build silently rebuilds it.
#   - It relinks CC/Sounder/mufft after the reset (the reset restores the empty
#     submodule directory; `ln -sfn` onto that directory would nest the link),
#     pointing at the rig repository's main checkout.
#   - It configures and builds with the venv sourced (VENV, default ~/houdini_test;
#     a fresh worktree's cmake does not find SoapySDR without it), never under
#     `set -e` (sourcing the venv's activate under it aborts the shell).
#   - It gates on make's own exit status: ctest after a failed build runs the OLD
#     binaries and passes.
#   - With CHECK_STRING set, it requires the new sounder binary to carry that
#     string (pick one only the new code emits).
#   - The rig host's aarch64 toolchain reaches fewer headers transitively than an
#     x86 development machine: a C++ change is not done until it built here.
#
# Environment: LOAD_MAX (default 2), VENV (default ~/houdini_test on the rig),
# CHECK_STRING (optional), JOBS (default 8). Exit 0 only when every step passed.
set -u

if [ "${1:-}" = "--remote" ]; then
  # The rig-host half, run by the local half below.
  WT=$2; BUNDLE=$3; REF=$4
  cd "$WT" || { echo "FAIL: no worktree $WT"; exit 1; }
  if pgrep -x sounder >/dev/null; then echo "FAIL: a sounder is running on this host"; exit 1; fi
  LOAD=$(cut -d' ' -f1 /proc/loadavg)
  if awk -v l="$LOAD" -v m="${LOAD_MAX:-2}" 'BEGIN { exit !(l > m) }'; then
    echo "FAIL: the host is busy (load $LOAD over ${LOAD_MAX:-2}); another lane may be measuring"; exit 1
  fi
  if [ -n "$BUNDLE" ]; then
    git fetch -q "$BUNDLE" "$REF" || { echo "FAIL: fetch from the bundle"; exit 1; }
    git reset -q --hard FETCH_HEAD || { echo "FAIL: reset to the fetched commit"; exit 1; }
    rm -f "$BUNDLE"
  fi
  MAIN=$(cd "$(git rev-parse --git-common-dir)/.." && pwd)
  M=CC/Sounder/mufft
  if [ ! -L "$M" ]; then
    rmdir "$M" 2>/dev/null
    ln -s "$MAIN/$M" "$M" || { echo "FAIL: relink $M"; exit 1; }
  fi
  [ -e "$M/" ] || { echo "FAIL: $M does not resolve ($MAIN/$M)"; exit 1; }
  echo "rig worktree at $(git log --oneline -1 | cut -c1-72)"
  # shellcheck disable=SC1090
  source "${VENV:-$HOME/houdini_test}/bin/activate" || { echo "FAIL: no venv ${VENV:-$HOME/houdini_test}"; exit 1; }
  cd CC/Sounder || exit 1
  if [ ! -f build/CMakeCache.txt ]; then
    mkdir -p build
    (cd build && cmake .. -DCMAKE_BUILD_TYPE=Release > cmake.log 2>&1) || { echo "FAIL: cmake (build/cmake.log)"; exit 1; }
  fi
  (cd build && nice -n 10 make -j"${JOBS:-8}" > make.log 2>&1)
  RC=$?
  if [ "$RC" -ne 0 ]; then
    echo "FAIL: make exited $RC"; grep -E "error|Error" build/make.log | head -5; exit 1
  fi
  if [ -n "${CHECK_STRING:-}" ] && ! strings build/sounder | grep -qF -- "$CHECK_STRING"; then
    echo "FAIL: the new sounder binary does not carry '$CHECK_STRING'"; exit 1
  fi
  (cd build && nice -n 10 ctest -j4 > ctest.log 2>&1)
  RC=$?
  tail -3 build/ctest.log
  [ "$RC" -eq 0 ] || { echo "FAIL: ctest exited $RC (build/ctest.log)"; exit 1; }
  echo "OK: built and tested at $(git log --oneline -1 | cut -c1-12)"
  exit 0
fi

[ $# -ge 2 ] || { sed -n '2,3p' "$0"; exit 2; }
RIG=$1; WT=$2; BR=${3:-$(git rev-parse --abbrev-ref HEAD)}
HEAD_LOCAL=$(git rev-parse --verify "$BR^{commit}") || { echo "FAIL: no branch $BR here"; exit 1; }
RIG_HEAD=$(ssh -o BatchMode=yes "$RIG" "git -C '$WT' rev-parse HEAD") || { echo "FAIL: cannot read $WT on $RIG"; exit 1; }
REMOTE_SELF=/tmp/ship_to_rig_$$.sh
scp -q "$0" "$RIG:$REMOTE_SELF" || { echo "FAIL: copy this script to $RIG"; exit 1; }
BUNDLE_REMOTE=""
if [ "$RIG_HEAD" != "$HEAD_LOCAL" ]; then
  git merge-base --is-ancestor "$RIG_HEAD" "$HEAD_LOCAL" ||
    { echo "FAIL: the rig worktree's HEAD ${RIG_HEAD:0:12} is not an ancestor of $BR (${HEAD_LOCAL:0:12}); move it first"; exit 1; }
  B=$(mktemp /tmp/ship_to_rig_XXXX.bundle)
  git bundle create -q "$B" "$RIG_HEAD..$BR" || { echo "FAIL: bundle $RIG_HEAD..$BR"; rm -f "$B"; exit 1; }
  BUNDLE_REMOTE=/tmp/ship_to_rig_$$.bundle
  scp -q "$B" "$RIG:$BUNDLE_REMOTE" || { echo "FAIL: copy the bundle"; rm -f "$B"; exit 1; }
  rm -f "$B"
  echo "shipping $BR ${RIG_HEAD:0:12}..${HEAD_LOCAL:0:12} to $RIG:$WT"
else
  echo "$RIG:$WT is already at ${HEAD_LOCAL:0:12}; building and testing it"
fi
ssh -o BatchMode=yes "$RIG" "LOAD_MAX='${LOAD_MAX:-2}' VENV='${VENV:-}' CHECK_STRING='${CHECK_STRING:-}' JOBS='${JOBS:-8}' bash $REMOTE_SELF --remote '$WT' '$BUNDLE_REMOTE' '$BR'; rc=\$?; rm -f $REMOTE_SELF; exit \$rc"
