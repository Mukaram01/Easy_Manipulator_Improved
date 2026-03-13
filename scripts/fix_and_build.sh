#!/usr/bin/env bash
# Build script for easy_manipulation_deployment workspace
# Supports: ROS 2 Humble on Ubuntu 22.04
# Usage examples (sanitation always runs first):
#   ./scripts/fix_and_build.sh
#   ./scripts/fix_and_build.sh --clean
#   ./scripts/fix_and_build.sh --packages my_package
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
    local repo_parent search_dir candidate

    repo_parent="$(dirname "$REPO_ROOT")"

    if [[ -n ${WS:-} ]]; then
        WORKSPACE_ROOT="$(cd "$WS" && pwd)"
    elif [[ $(basename "$repo_parent") == "src" ]]; then
        WORKSPACE_ROOT="$(cd "$repo_parent/.." && pwd)"
    fi

    if [[ -z ${WORKSPACE_ROOT:-} ]]; then
        search_dir="$(pwd)"
        while [[ "$search_dir" != "/" ]]; do
            if [[ -d "$search_dir/src" || -d "$search_dir/build" ]]; then
                candidate="$(cd "$search_dir" && pwd)"
                if [[ "$REPO_ROOT" == "$candidate/src/"* ]]; then
                    WORKSPACE_ROOT="$candidate"
                    break
                fi
                WORKSPACE_ROOT="${WORKSPACE_ROOT:-$candidate}"
            fi
            search_dir="$(dirname "$search_dir")"
        done
    fi

    if [[ -z ${WORKSPACE_ROOT:-} ]]; then
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
    log_info "=== Sanitizing workspace: $WORKSPACE_ROOT (level=full) ==="
    rm -rf "$WORKSPACE_ROOT/build" "$WORKSPACE_ROOT/install" "$WORKSPACE_ROOT/log"
    log_info "=== Sanitize complete ==="
}

apply_ros_industrial_patches() {
    local target_repo="$WORKSPACE_ROOT/src/ros_industrial_cmake_boilerplate"
    local patch_failed=0
    local patch

    if [[ ! -d "$target_repo" ]]; then
        log_warn "ros_industrial_cmake_boilerplate repo not found at $target_repo; skipping patch application"
        return
    fi

    local cfg_path="$target_repo/cmake/cmake_tools.cmake"
    if [[ ! -f "$cfg_path" ]]; then
        log_warn "ros_industrial_cmake_boilerplate config helper missing at $cfg_path; skipping targeted patches"
        return
    fi

    if ! grep -q 'install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/${extra_config}' "$cfg_path" >/dev/null 2>&1; then
        log_info "ros_industrial_cmake_boilerplate already contains CFG_EXTRAS fix; skipping targeted patches"
        return
    fi

    log_info "Applying ros_industrial_cmake_boilerplate patches from $SCRIPT_DIR/patches"

    shopt -s nullglob
    for patch in "$SCRIPT_DIR/patches/"*-ricb-*.patch; do
        [[ -f "$patch" ]] || continue

        if git -C "$target_repo" apply --reverse --check "$patch" >/dev/null 2>&1; then
            log_info "Patch $(basename "$patch") already applied to ros_industrial_cmake_boilerplate"
            continue
        fi

        log_info "Applying patch $(basename "$patch") to ros_industrial_cmake_boilerplate"
        if git -C "$target_repo" apply --whitespace=nowarn "$patch" >/dev/null 2>&1; then
            continue
        fi

        log_warn "Standard git apply failed; attempting 3-way merge for $(basename "$patch")"
        if git -C "$target_repo" apply --3way --whitespace=nowarn "$patch" >/dev/null 2>&1; then
            continue
        fi

        log_warn "3-way merge failed; attempting patch(1) with fuzz for $(basename "$patch")"
        if ! (cd "$target_repo" && patch -p1 -N --fuzz=5 --ignore-whitespace <"$patch"); then
            log_error "Failed to apply $(basename "$patch") to ros_industrial_cmake_boilerplate"
            patch_failed=1
        fi
    done
    shopt -u nullglob

    if grep -q 'install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/${extra_config}' "$cfg_path" >/dev/null 2>&1; then
        die "CFG_EXTRAS bug still present in ros_industrial_cmake_boilerplate after patch attempts"
    fi

    if [[ $patch_failed -eq 1 ]]; then
        log_warn "One or more ros_industrial_cmake_boilerplate patches failed to apply cleanly, but the CFG_EXTRAS fix is present"
    fi
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
    cd "$WORKSPACE_ROOT"
    sanitize_workspace

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

    # Apply targeted ros_industrial_cmake_boilerplate patches as early as possible
    # so patch validation can run even on hosts without a sourced ROS environment.
    apply_ros_industrial_patches

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
