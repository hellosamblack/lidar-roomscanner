#!/usr/bin/env bash
# ==============================================================================
# host/tools/worktree_setup.sh
#
# Automated bootstrap and sanity check for newly created git worktrees
# in lidar-roomscanner (Orca ADE, Codex, Claude fleet workers, etc.).
#
# What it does:
# 1. Identifies current worktree and main repository roots.
# 2. Creates the required symlinks:
#    - captures/ -> <main>/captures (test datasets)
#    - host/transform/build/ -> <main>/host/transform/build (native C transform library)
#    - (Optional) firmware/vendor/lwip -> <main>/firmware/vendor/lwip (if missing)
# 3. Verifies the Python venv (host/.venv) and PYTHONPATH configuration.
# 4. Runs a fast pytest smoke test to ensure zero unexpected skips.
# ==============================================================================

set -euo pipefail

SKIP_TESTS=0
SETUP_FIRMWARE=0

while [[ $# -gt 0 ]]; do
    case "$1" in
        --skip-tests)
            SKIP_TESTS=1
            shift
            ;;
        --firmware)
            SETUP_FIRMWARE=1
            shift
            ;;
        -h|--help)
            echo "Usage: $0 [--skip-tests] [--firmware]"
            echo "  --skip-tests  Skip running pytest sanity tests"
            echo "  --firmware    Link firmware vendor submodules (e.g. lwip)"
            exit 0
            ;;
        *)
            echo "Unknown argument: $1" >&2
            exit 1
            ;;
    esac
done

# Resolve repo roots
CURRENT_ROOT="$(git rev-parse --show-toplevel 2>/dev/null || pwd)"
GIT_COMMON_DIR="$(git rev-parse --git-common-dir 2>/dev/null || echo "$CURRENT_ROOT/.git")"
MAIN_REPO_ROOT="$(cd "$GIT_COMMON_DIR/.." && pwd)"

echo "=== lidar-roomscanner worktree setup ==="
echo "Worktree root : $CURRENT_ROOT"
echo "Main repo root: $MAIN_REPO_ROOT"

# If running inside the main checkout itself, just verify
IS_MAIN_CHECKOUT=0
if [[ "$CURRENT_ROOT" == "$MAIN_REPO_ROOT" ]]; then
    IS_MAIN_CHECKOUT=1
    echo "Note: Running in main repository checkout."
fi

# 1. Link captures
if [[ $IS_MAIN_CHECKOUT -eq 0 ]]; then
    if [[ ! -e "$CURRENT_ROOT/captures" && -d "$MAIN_REPO_ROOT/captures" ]]; then
        echo "[1/3] Creating symlink: captures -> $MAIN_REPO_ROOT/captures"
        ln -s "$MAIN_REPO_ROOT/captures" "$CURRENT_ROOT/captures"
    elif [[ -e "$CURRENT_ROOT/captures" ]]; then
        echo "[1/3] captures/ already present."
    else
        echo "[1/3] Warning: $MAIN_REPO_ROOT/captures not found."
    fi
else
    echo "[1/3] captures/ check: $(ls "$CURRENT_ROOT/captures" 2>/dev/null | wc -l) files in main."
fi

# 2. Link host/transform/build
if [[ $IS_MAIN_CHECKOUT -eq 0 ]]; then
    mkdir -p "$CURRENT_ROOT/host/transform"
    if [[ ! -e "$CURRENT_ROOT/host/transform/build" && -d "$MAIN_REPO_ROOT/host/transform/build" ]]; then
        echo "[2/3] Creating symlink: host/transform/build -> $MAIN_REPO_ROOT/host/transform/build"
        ln -s "$MAIN_REPO_ROOT/host/transform/build" "$CURRENT_ROOT/host/transform/build"
    elif [[ -e "$CURRENT_ROOT/host/transform/build" ]]; then
        echo "[2/3] host/transform/build already present."
    else
        echo "[2/3] Warning: $MAIN_REPO_ROOT/host/transform/build not found. Native library might not be built."
    fi
else
    echo "[2/3] host/transform/build present."
fi

# Optional: Firmware submodules
if [[ $SETUP_FIRMWARE -eq 1 && $IS_MAIN_CHECKOUT -eq 0 ]]; then
    if [[ -d "$MAIN_REPO_ROOT/firmware/vendor/lwip" && (! -d "$CURRENT_ROOT/firmware/vendor/lwip" || -z "$(ls -A "$CURRENT_ROOT/firmware/vendor/lwip" 2>/dev/null)") ]]; then
        echo "[+] Linking firmware/vendor/lwip..."
        mkdir -p "$CURRENT_ROOT/firmware/vendor"
        rm -rf "$CURRENT_ROOT/firmware/vendor/lwip"
        ln -s "$MAIN_REPO_ROOT/firmware/vendor/lwip" "$CURRENT_ROOT/firmware/vendor/lwip"
    fi
fi

# 3. Python venv & environment check
PYTHON_BIN="$MAIN_REPO_ROOT/host/.venv/bin/python"
if [[ ! -x "$PYTHON_BIN" ]]; then
    # Fallback to local worktree venv or system python if venv not in main
    if [[ -x "$CURRENT_ROOT/host/.venv/bin/python" ]]; then
        PYTHON_BIN="$CURRENT_ROOT/host/.venv/bin/python"
    elif command -v python3 >/dev/null 2>&1; then
        PYTHON_BIN="$(command -v python3)"
    fi
fi

echo "[3/3] Python executable: $PYTHON_BIN"

export PYTHONPATH="$CURRENT_ROOT/host/src:$CURRENT_ROOT/host${PYTHONPATH:+:$PYTHONPATH}"
export ROOMSCAN_TRANSFORM_DLL="$MAIN_REPO_ROOT/host/transform/build/libroomscan_transform.so"

if [[ $SKIP_TESTS -eq 0 ]]; then
    echo "--- Running fast smoke tests ---"
    (
        cd "$CURRENT_ROOT/host"
        "$PYTHON_BIN" -m pytest tests/test_protocol.py tests/test_native.py -q --tb=short
    )
    echo "--- Smoke tests passed! ---"
fi

echo "Worktree setup complete."
