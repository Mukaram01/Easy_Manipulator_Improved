#!/usr/bin/env bash
# Build script for easy_manipulation_deployment workspace
# Supports: ROS 2 Humble on Ubuntu 22.04
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
STATE_DIR="${STATE_DIR:-$REPO_ROOT/.build_state}"

source "$SCRIPT_DIR/lib/common.sh"
source "$SCRIPT_DIR/lib/environment.sh"
source "$SCRIPT_DIR/lib/dependencies.sh"
source "$SCRIPT_DIR/lib/patches.sh"
source "$SCRIPT_DIR/lib/build.sh"

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
    PACKAGES=()

    while [[ $# -gt 0 ]]; do
        case "$1" in
            -h|--help)
                usage
                return 0
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
