#!/usr/bin/env bash
# Fresh-clone check for a second machine. Run this on the 6.1.0-rc.26 box.
#
# This is the only test of the install path and of Isaac Sim 6.1 compatibility: everything else has been
# verified on one machine, against one Isaac version, from a working tree that was never installed from
# scratch. It deliberately clones into a temporary directory rather than using an existing checkout, so a
# missing file or an untracked dependency shows up instead of being silently satisfied.
#
# Nothing here is destructive: it clones to a temp directory, installs with `pip install --user`, and
# removes nothing. It does not touch any existing clone.
#
# IMPORTANT: this tests whatever is on the remote branch you name. Push first, or the run will
# faithfully verify code that is not the code you are shipping.
#
# Usage:
#   ./scripts/fresh_clone_check.sh                               # origin, branch main
#   ./scripts/fresh_clone_check.sh git@github.com:JerichoGroup/isaac_core_6.git dev
#   ./scripts/fresh_clone_check.sh /path/to/local/repo dev        # clone a local path instead
set -uo pipefail

REMOTE="${1:-git@github.com:JerichoGroup/isaac_core_6.git}"
BRANCH="${2:-main}"
WORK="$(mktemp -d /tmp/isaac_core_fresh_XXXXXX)"
REPORT="${WORK}/report.txt"

step() { printf '\n=== %s\n' "$1" | tee -a "$REPORT"; }
record() { printf '%s\n' "$1" | tee -a "$REPORT"; }

step "machine and versions"
record "host:        $(hostname)"
record "os:          $(. /etc/os-release && echo "$PRETTY_NAME")"
record "kernel:      $(uname -r)"
record "gpu:         $(nvidia-smi --query-gpu=name --format=csv,noheader 2>/dev/null || echo 'nvidia-smi unavailable')"
record "driver:      $(nvidia-smi --query-gpu=driver_version --format=csv,noheader 2>/dev/null || echo unknown)"
record "gpu free:    $(nvidia-smi --query-gpu=memory.free --format=csv,noheader 2>/dev/null || echo unknown)"
record "python3:     $(python3 --version 2>&1)"
record "isaac:       $(ls -d "$HOME"/isaacsim 2>/dev/null || echo 'not at ~/isaacsim')"
if [ -f "$HOME/isaacsim/VERSION" ]; then
  record "isaac ver:   $(cat "$HOME/isaacsim/VERSION")"
fi
record "ros distro:  ${ROS_DISTRO:-not sourced}"

step "clone"
if git clone --depth 1 --branch "$BRANCH" "$REMOTE" "${WORK}/isaac_core_6" >>"$REPORT" 2>&1; then
  record "clone: OK from ${REMOTE} branch ${BRANCH}"
else
  record "clone: FAILED from ${REMOTE} branch ${BRANCH} -- see report"
  record "REPORT AT: ${REPORT}"
  exit 1
fi
cd "${WORK}/isaac_core_6" || exit 1
record "commit:  $(git rev-parse --short HEAD)"
record "subject: $(git log -1 --pretty=%s)"

# Confirm the clone actually contains the work under test. Verifying a stale branch is worse than not
# verifying at all, because it produces a green report for code nobody is shipping.
step "is this the code we mean to test?"
MISSING=0
for f in src/isaac_core/sim/segmentation.py src/isaac_core/contracts/zoom.py \
         src/isaac_core/assets/layers/segmentation/layer.toml scripts/eyes_on_check.py; do
  if [ -e "$f" ]; then record "present: $f"; else record "MISSING: $f"; MISSING=1; fi
done
if [ -e src/isaac_core/sidecar ]; then
  record "MISSING-REMOVAL: src/isaac_core/sidecar still exists, so this branch predates its deletion"
  MISSING=1
else
  record "confirmed: sidecar has been removed"
fi
if [ "$MISSING" -ne 0 ]; then
  record ""
  record "STOP: this branch is not the one to test. Push the branch with the V3 work and rerun."
  record "REPORT AT: ${REPORT}"
  exit 1
fi

step "setup.sh"
# The whole point of this check: a fresh machine running the documented installer.
if ./scripts/setup.sh >>"$REPORT" 2>&1; then
  record "setup.sh: exit 0"
else
  record "setup.sh: NON-ZERO EXIT -- the interesting part of this whole check; see report"
fi

step "doctor"
if isaac-core doctor 2>&1 | tee -a "$REPORT" | tail -25; then
  record "doctor: ran"
else
  record "doctor: FAILED TO RUN (is ~/.local/bin on PATH in this shell?)"
fi

step "unit tests"
if python3 -m pytest -q 2>&1 | tail -5 | tee -a "$REPORT"; then
  record "unit tests: ran"
else
  record "unit tests: reported failures -- see the tail above"
fi

step "dry run, no Isaac launched"
isaac-core run --dry-run 2>&1 | tail -20 | tee -a "$REPORT"

step "first flight, headless, 90 second cap"
# Headless on purpose: this is about whether it composes on 6.1, not about looking at it.
timeout -k 10 90 isaac-core run --set sim.headless true --set sim.control_plane.port 8765 \
  >"${WORK}/first_flight.log" 2>&1 &
SIM_PID=$!
sleep 45
if python3 - <<'PY' 2>&1 | tee -a "$REPORT"
import socket
try:
    with socket.create_connection(("127.0.0.1", 8765), timeout=5.0):
        print("control plane: ANSWERING")
except OSError as exc:
    print(f"control plane: NOT ANSWERING ({exc})")
PY
then :; fi
grep -iE '✓|✗|composed|skipped' "${WORK}/first_flight.log" | head -15 | tee -a "$REPORT"
grep -icE '\[error\]|\[fatal\]' "${WORK}/first_flight.log" | sed 's/^/error lines in sim log: /' | tee -a "$REPORT"
# Isaac ignores SIGTERM, so kill the group hard.
kill -9 -- "-$(ps -o pgid= "$SIM_PID" 2>/dev/null | tr -d ' ')" 2>/dev/null || kill -9 "$SIM_PID" 2>/dev/null
wait "$SIM_PID" 2>/dev/null

step "system tests, if you have time (about 3 minutes)"
record "SKIPPED BY DEFAULT. To include them, run:"
record "  cd ${WORK}/isaac_core_6 && python3 -m pytest tests/system -q --system"

step "done"
record "Full report:      ${REPORT}"
record "Simulator log:    ${WORK}/first_flight.log"
record "Clone kept at:    ${WORK}/isaac_core_6  (delete when finished)"
printf '\n%s\n' "Send back: the report file above, plus the simulator log if anything failed."
