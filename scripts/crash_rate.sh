#!/usr/bin/env bash
# Measure the startup segfault rate.
#
# Runs the simulator N times, each for a fixed window, and reports how many died with a
# segfault. Every run is killed before the next starts, and stale /tmp/carb.* debris is
# cleared first, because crash leftovers demonstrably feed further crashes.
#
# Usage: crash_rate.sh <trials> <label>
set -uo pipefail

TRIALS="${1:-10}"
LABEL="${2:-run}"
CONFIG=/tmp/cfg_run.toml
ISAAC="${ISAACSIM_PATH:-$HOME/isaacsim}/python.sh"
WINDOW="${3:-45}"

kill_sims() {
  python3 - <<'PY' >/dev/null 2>&1
import os, signal, time
from pathlib import Path
me = {os.getpid(), os.getppid()}

def find():
    out = []
    for entry in Path("/proc").iterdir():
        if not entry.name.isdigit() or int(entry.name) in me:
            continue
        try:
            argv = (entry / "cmdline").read_bytes().split(b"\0")
        except OSError:
            continue
        if not argv or not argv[0]:
            continue
        joined = b" ".join(argv).decode(errors="replace")
        if "telemetry" in joined:
            continue
        if "isaac_core.sim" in joined or "send_test_pose" in joined:
            out.append(int(entry.name))
    return out

for sig in (signal.SIGTERM, signal.SIGKILL):
    pids = find()
    if not pids:
        break
    for pid in pids:
        try:
            os.kill(pid, sig)
        except ProcessLookupError:
            pass
    time.sleep(4)
PY
}

kill_sims
rm -rf /tmp/carb.* 2>/dev/null

crashes=0
ran=0
for trial in $(seq 1 "$TRIALS"); do
  log="/tmp/cr_${LABEL}_${trial}.log"
  timeout "$WINDOW" "$ISAAC" -m isaac_core.sim --config "$CONFIG" > "$log" 2>&1
  code=$?
  kill_sims

  seg=$(grep -cE 'Segmentation fault|exit code 139' "$log")
  started=$(grep -c 'simulation running' "$log")
  ran=$((ran + 1))
  if [ "$seg" != "0" ]; then
    crashes=$((crashes + 1))
    status="SEGFAULT"
  elif [ "$code" = "124" ]; then
    status="ok"
  else
    status="other(exit=$code)"
  fi
  printf "  %-6s trial %2s  reached_play=%s  %s\n" "$LABEL" "$trial" "$started" "$status"
done

echo "  ---- $LABEL: $crashes segfaults in $ran runs"
