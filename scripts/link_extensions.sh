#!/usr/bin/env bash
# Symlink isaac_core_ogn.* extensions into Isaac Sim's extsUser directory.
#
# Idempotent: safe to re-run. Replaces existing symlinks but refuses to clobber
# a real directory. Resolves the Isaac path via the same validation logic as
# install.py -- never trusts $ISAACSIM_PATH blindly.
#
# Usage:
#   scripts/link_extensions.sh [--isaac-path /path/to/isaacsim]

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
EXTENSIONS_DIR="$REPO_ROOT/extensions"

# --- Resolve Isaac Sim path ---
ISAAC_PATH=""

while [[ $# -gt 0 ]]; do
    case "$1" in
        --isaac-path)
            ISAAC_PATH="$2"
            shift 2
            ;;
        *)
            echo "ERROR: Unknown argument: $1" >&2
            echo "Usage: scripts/link_extensions.sh [--isaac-path /path/to/isaacsim]" >&2
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

if [[ -z "$ISAAC_PATH" ]]; then
    # Probe known locations
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
        echo "  Fix: pass --isaac-path /path/to/isaacsim" >&2
        exit 1
    fi
else
    if ! validate_isaac_dir "$ISAAC_PATH"; then
        echo "ERROR: Specified Isaac path is not a valid install: $ISAAC_PATH" >&2
        echo "  A valid install contains: python.sh, isaac-sim.sh, VERSION" >&2
        exit 1
    fi
fi

EXTS_USER_DIR="$ISAAC_PATH/extsUser"

echo "Linking extensions into: $EXTS_USER_DIR"

# Ensure extsUser exists
mkdir -p "$EXTS_USER_DIR"

# Link each isaac_core_ogn.* extension
linked=0
for ext_dir in "$EXTENSIONS_DIR"/isaac_core_ogn.*; do
    if [[ ! -d "$ext_dir" ]]; then
        continue
    fi

    ext_name="$(basename "$ext_dir")"
    link_target="$EXTS_USER_DIR/$ext_name"

    if [[ -L "$link_target" ]]; then
        # Replace existing symlink
        echo "  Replacing symlink: $ext_name"
        rm "$link_target"
    elif [[ -d "$link_target" ]]; then
        # Refuse to clobber a real directory
        echo "  SKIP: $link_target is a real directory (not a symlink). Remove it manually if intended." >&2
        continue
    elif [[ -e "$link_target" ]]; then
        # Something else exists -- refuse
        echo "  SKIP: $link_target exists and is not a symlink. Remove it manually." >&2
        continue
    fi

    ln -s "$ext_dir" "$link_target"
    echo "  Linked: $ext_name -> $ext_dir"
    linked=$((linked + 1))
done

echo "Done. $linked extension(s) linked."
