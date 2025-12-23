#!/usr/bin/env bash
# Build script for easy_manipulation_deployment workspace
# Supports: ROS 2 Humble on Ubuntu 22.04
# Usage examples (sanitation always runs first):
#   ./scripts/fix_and_build.sh
#   SANITIZE_LEVEL=build-only ./scripts/fix_and_build.sh --clean
#   SANITIZE_LEVEL=pkg SANITIZE_PKG=my_package ./scripts/fix_and_build.sh --packages my_package
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
STATE_DIR="${STATE_DIR:-$REPO_ROOT/.build_state}"

source "$SCRIPT_DIR/lib/common.sh"
source "$SCRIPT_DIR/lib/environment.sh"
source "$SCRIPT_DIR/lib/dependencies.sh"
source "$SCRIPT_DIR/lib/patches.sh"
source "$SCRIPT_DIR/lib/build.sh"

detect_workspace() {
    local repo_parent
    repo_parent="$(dirname "$REPO_ROOT")"

    if [[ -n ${WS:-} ]]; then
        WORKSPACE_ROOT="$(cd "$WS" && pwd)"
    elif [[ $(basename "$repo_parent") == "src" ]]; then
        WORKSPACE_ROOT="$(cd "$repo_parent/.." && pwd)"
    else
        WORKSPACE_ROOT="$HOME/workcell_ws"
    fi

    SRC_DIR="$WORKSPACE_ROOT/src"
}

source_ros() {
    log_info "Using ROS_DISTRO=$ROS_DISTRO"
    unset COLCON_PREFIX_PATH AMENT_PREFIX_PATH CMAKE_PREFIX_PATH
    set +u
    : "${AMENT_TRACE_SETUP_FILES:=}"
    # shellcheck source=/dev/null
    source "/opt/ros/$ROS_DISTRO/setup.bash"
    set -u
}

validate_workspace_root() {
    if [[ -z ${WORKSPACE_ROOT:-} ]]; then
        die "Workspace root could not be determined"
    fi

    case "$WORKSPACE_ROOT" in
        "/"|"$HOME"|""|".")
            die "Unsafe workspace root: '$WORKSPACE_ROOT'"
            ;;
    esac

    [[ -d "$WORKSPACE_ROOT" ]] || die "Workspace root does not exist: $WORKSPACE_ROOT"
    [[ -d "$WORKSPACE_ROOT/src" ]] || die "Workspace src directory missing: $WORKSPACE_ROOT/src"
}

sanitize_workspace() {
    local level="${1:-${SANITIZE_LEVEL:-full}}"
    local pkg="${2:-${SANITIZE_PKG:-}}"

    [[ -n "$level" ]] || level="full"

    local targets=()
    case "$level" in
        full)
            targets=("$WORKSPACE_ROOT/build" "$WORKSPACE_ROOT/install" "$WORKSPACE_ROOT/log")
            ;;
        build-only)
            targets=("$WORKSPACE_ROOT/build" "$WORKSPACE_ROOT/log")
            ;;
        pkg)
            [[ -n "$pkg" ]] || die "SANITIZE_PKG must be set when SANITIZE_LEVEL=pkg"
            targets=("$WORKSPACE_ROOT/build/$pkg" "$WORKSPACE_ROOT/log")
            log_warn "Package-level sanitation cannot remove merged install artifacts; consider SANITIZE_LEVEL=full if issues persist"
            ;;
        *)
            die "Unknown sanitize level: $level"
            ;;
    esac

    log_info "=== Sanitizing workspace: $WORKSPACE_ROOT (level=$level) ==="
    for path in "${targets[@]}"; do
        [[ -e "$path" ]] || continue
        rm -rf "$path"
    done
    log_info "=== Sanitize complete ==="
}

usage() {
    cat <<'USAGE'
Usage: ./fix_and_build.sh [OPTIONS]

Options:
    -h, --help          Show this help message
    -c, --clean         Clean build artifacts and re-apply patches
    -j, --jobs N        Number of parallel jobs (default: nproc)
    -v, --verbose       Verbose output
    --skip-deps         Skip dependency installation
    --skip-patches      Skip patch application
    --packages PKG...   Build only specified packages
    --lightweight       Disable OMPL, Descartes, Trajopt planners
USAGE
}

main() {
    local CLEAN=0
    local JOBS
    JOBS="$(nproc 2>/dev/null || echo 1)"
    local VERBOSE=0
    local SKIP_DEPS=0
    local SKIP_PATCHES=0
    local LIGHTWEIGHT=0
    local SHOW_HELP=0
    PACKAGES=()

    detect_workspace
    validate_workspace_root
    sanitize_workspace "${SANITIZE_LEVEL:-}" "${SANITIZE_PKG:-}"

    while [[ $# -gt 0 ]]; do
        case "$1" in
            -h|--help)
                SHOW_HELP=1
                ;;
            -c|--clean)
                CLEAN=1
                ;;
            -j|--jobs)
                shift
                [[ $# -gt 0 ]] || die "--jobs requires a value"
                JOBS="$1"
                ;;
            -v|--verbose)
                VERBOSE=1
                ;;
            --skip-deps)
                SKIP_DEPS=1
                ;;
            --skip-patches)
                SKIP_PATCHES=1
                ;;
            --packages)
                shift
                while [[ $# -gt 0 && ${1:0:1} != '-' ]]; do
                    PACKAGES+=("$1")
                    shift
                done
                continue
                ;;
            --lightweight)
                LIGHTWEIGHT=1
                ;;
            *)
                die "Unknown option: $1"
                ;;
        esac
        shift
    done

    [[ $VERBOSE -eq 1 ]] && set -x

    if [[ $SHOW_HELP -eq 1 ]]; then
        usage
        return 0
    fi

    log_info "Starting build process"

    setup_environment

    if [[ $CLEAN -eq 1 ]]; then
        revert_patches || true
        clean_build_artifacts
        rm -rf "$STATE_DIR"
    fi

    ensure_dir "$STATE_DIR"

    install_dependencies
    apply_patches
    build_workspace

    log_success "Build completed successfully"
}

main "$@"
