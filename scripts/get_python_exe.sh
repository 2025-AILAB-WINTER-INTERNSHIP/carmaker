#!/bin/bash
# =============================================================================
# scripts/get_python_exe.sh
# Centralized Python Interpreter Detector (Single Source of Truth)
# Refactored for Enterprise Stability: Validates file existence and execution bits.
# =============================================================================

VENV_PATH="/workspace/install/.venv"
VENV_CFG="${VENV_PATH}/pyvenv.cfg"
DEFAULT_SYS_PYTHON="/usr/bin/python3"
SYS_EXE="${SYS_PYTHON_EXE:-$DEFAULT_SYS_PYTHON}"

# Logic:
# 1. If venv exists AND has 'include-system-site-packages = true' AND the binary is executable
#    -> Use venv python (Hybrid mode: system + venv)
# 2. Otherwise, use system python (Stable/Isolated mode)

if [ -f "$VENV_CFG" ]; then
    # Use awk for robust parsing: handle spaces, case-insensitivity, and extract value
    INCLUDE_SYSTEM=$(awk -F'[[:space:]]*=[[:space:]]*' 'tolower($1) == "include-system-site-packages" {print tolower($2)}' "$VENV_CFG" | xargs)

    if [ "$INCLUDE_SYSTEM" = "true" ] && [ -x "${VENV_PATH}/bin/python3" ]; then
        echo "${VENV_PATH}/bin/python3"
        exit 0
    fi
fi

# Fallback to system python if executable, or ultimate default
if [ -x "$SYS_EXE" ]; then
    echo "$SYS_EXE"
else
    echo "$DEFAULT_SYS_PYTHON"
fi
