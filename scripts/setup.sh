#!/usr/bin/env bash
# Setup script for isaac_core_6.
#
# Installs Python dependencies into the user's site-packages, installs the
# package into Isaac Sim's bundled interpreter, links extensions, and runs
# the doctor diagnostic.
#
# Usage:
#   scripts/setup.sh [--isaac-path /path/to/isaacsim]

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

# --- Resolve Isaac Sim path ---
# Uses the same logic as install.py: explicit argument, then probe known locations.
# NEVER trusts $ISAACSIM_PATH blindly -- it is often stale.

ISAAC_PATH=""

# Accept --isaac-path argument
while [[ $# -gt 0 ]]; do
    case "$1" in
        --isaac-path)
            ISAAC_PATH="$2"
            shift 2
            ;;
        *)
            echo "ERROR: Unknown argument: $1" >&2
            echo "Usage: scripts/setup.sh [--isaac-path /path/to/isaacsim]" >&2
            exit 1
            ;;
    esac
done

validate_isaac_dir() {
    local dir="$1"
    [[ -d "$dir" ]] && \
    [[ -f "$dir/python.sh" ]] && \
    [[ -f "$dir/isaac-sim.sh" ]] && \
    [[ -f "$dir/VERSION" ]]
}

if [[ -n "$ISAAC_PATH" ]]; then
    if ! validate_isaac_dir "$ISAAC_PATH"; then
        echo "ERROR: Specified Isaac path is not a valid install: $ISAAC_PATH" >&2
        echo "  A valid install contains: python.sh, isaac-sim.sh, VERSION" >&2
        exit 1
    fi
else
    # Probe known locations (do NOT trust $ISAACSIM_PATH without validation)
    CANDIDATES=(
        "/home/ofer/isaacsim"
        "/opt/isaacsim"
        "/opt/nvidia/isaac-sim"
        "/isaac-sim"
    )

    # Check $ISAACSIM_PATH only if it validates
    if [[ -n "${ISAACSIM_PATH:-}" ]] && validate_isaac_dir "$ISAACSIM_PATH"; then
        ISAAC_PATH="$ISAACSIM_PATH"
    else
        for candidate in "${CANDIDATES[@]}"; do
            if validate_isaac_dir "$candidate"; then
                ISAAC_PATH="$candidate"
                break
            fi
        done
    fi

    if [[ -z "$ISAAC_PATH" ]]; then
        echo "ERROR: Cannot find a valid Isaac Sim installation." >&2
        echo "  Tried: ${CANDIDATES[*]}" >&2
        echo "  A valid install contains: python.sh, isaac-sim.sh, VERSION" >&2
        echo "  Fix: pass --isaac-path /path/to/isaacsim" >&2
        exit 1
    fi
fi

echo "=== isaac_core_6 setup ==="
echo "Repository: $REPO_ROOT"
echo "Isaac Sim:  $ISAAC_PATH ($(cat "$ISAAC_PATH/VERSION"))"
echo

# --- 1. Install into user's Python ---
echo "[1/4] Installing requirements into user site-packages..."
pip install --user -r "$REPO_ROOT/requirements.txt"
echo

# --- 2. Install the package in editable mode ---
echo "[2/4] Installing isaac-core in editable mode..."
pip install --user -e "$REPO_ROOT"
echo

# --- 3. Install into Isaac's bundled Python ---
echo "[3/4] Installing isaac-core into Isaac's Python..."
"$ISAAC_PATH/python.sh" -m pip install -e "$REPO_ROOT"
echo

# --- 4. Link extensions ---
echo "[4/4] Linking extensions..."
"$SCRIPT_DIR/link_extensions.sh" --isaac-path "$ISAAC_PATH"
echo

# --- Run doctor ---
echo "=== Running isaac-core doctor ==="
isaac-core doctor || true
echo
echo "Setup complete."
