#!/bin/bash
# =============================================================================
# scripts/check_deps.sh
# Verifies build artifacts for missing runtime shared library dependencies
#
# This script is a DEVELOPMENT/BUILD-TIME tool only.
# It is NOT included in production runtime images (dev-runtime, ros-runtime).
#
# Scans the target directory for ELF files and shared objects, using ldd to
# identify missing dependencies.
# =============================================================================

set -eo pipefail

TARGET_DIR=${1:-/workspace/install}
MISSING_COUNT=0

# Load logging utility
SOURCE_LOG="/docker_dev/scripts/utils_logging.sh"
[ ! -f "$SOURCE_LOG" ] && SOURCE_LOG="$(dirname "${BASH_SOURCE[0]}")/utils_logging.sh"
[ -f "$SOURCE_LOG" ] && source "$SOURCE_LOG"
LOG_PREFIX="[Sanity Check]"

log_info "Scanning for missing dependencies in $TARGET_DIR..."

if [ ! -d "$TARGET_DIR" ]; then
    log_error "Target directory '$TARGET_DIR' does not exist."
    log_info "Make sure you have built your project (e.g., 'cb' or 'make install') before checking dependencies."
    exit 1
fi

# Find executable files and shared libraries (.so) - Batch processed for speed
while IFS= read -r ELF_FILE; do
    MISSING=$(ldd "$ELF_FILE" 2>/dev/null | grep "not found" || true)
    if [ -n "$MISSING" ]; then
        log_error "Missing dependencies for: $ELF_FILE\n$(echo "$MISSING" | sed 's/^/  /')"
        MISSING_COUNT=$((MISSING_COUNT + 1))
    fi
done < <(find "$TARGET_DIR" -type f \( -executable -o -name "*.so*" \) ! -name "*.py" -exec file {} + 2>/dev/null | grep -E 'ELF.*(executable|shared object)' | cut -d: -f1)

# --- Python Integrity Check (SSOT Integration) -------------------------------
function check_python_integrity() {
    local script_dir="$(dirname "${BASH_SOURCE[0]}")"
    local get_py_script="${script_dir}/get_python_exe.sh"

    if [ ! -f "$get_py_script" ]; then
        log_warn "Python detector script not found. Skipping Python integrity check."
        return 0
    fi

    local active_py=$(bash "$get_py_script")
    log_info "Verifying Python integrity for: $active_py"

    # 1. Basic Interpreter execution test
    if ! "$active_py" --version &>/dev/null; then
        log_error "Python interpreter is not executable: $active_py"
        return 1
    fi

    # 2. ROS Distro-specific binding test
    case "${ROS_DISTRO}" in
        noetic)
            if ! "$active_py" -c "import rospy" &>/dev/null; then
                log_error "ROS 1 (Noetic) bindings not found in $active_py. Check your venv configuration."
                return 1
            fi
            ;;
        humble|foxy|galactic)
            if ! "$active_py" -c "import rclpy" &>/dev/null; then
                log_error "ROS 2 (${ROS_DISTRO}) bindings not found in $active_py. Check your venv configuration."
                return 1
            fi
            ;;
        *)
            log_warn "Unknown ROS_DISTRO '${ROS_DISTRO}'. Skipping specific binding check."
            ;;
    esac

    log_ok "Python ROS bindings verified successfully."
}

check_python_integrity || MISSING_COUNT=$((MISSING_COUNT + 1))

if [ $MISSING_COUNT -gt 0 ]; then
    log_error "$MISSING_COUNT files have missing dependencies."
    log_info "Please add missing packages to dependencies/apt.txt (with # runtime comment)."
    exit 1
else
    log_ok "All runtime dependencies are satisfied."
    exit 0
fi
