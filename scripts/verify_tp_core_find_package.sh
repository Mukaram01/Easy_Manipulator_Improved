#!/usr/bin/env bash
# Verify that tesseract_time_parameterization_core can be found via CMake and trigger a rebuild of tesseract_task_composer.
#
# Expected usage:
#   Run after the workspace has been built and installed to the default install/ prefix (or provide --workspace to override).
#   This script checks for the generated CMake package files, performs a minimal CMake configure with CMAKE_PREFIX_PATH
#   pointing to the workspace install, and then runs `colcon build --packages-select tesseract_task_composer --cmake-clean-cache`.
#   The script is intended for CI and exits non-zero on any failure.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
INSTALL_PREFIX="$WORKSPACE_DIR/install"

usage() {
    cat <<USAGE
Usage: ${0##*/} [--workspace PATH]

Options:
  --workspace PATH   Workspace root containing the install/ directory (default: repo root)
  -h, --help         Show this help message

USAGE
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --workspace)
            shift
            [[ $# -gt 0 ]] || { echo "--workspace requires a value" >&2; exit 1; }
            WORKSPACE_DIR="$(cd "$1" && pwd)"
            INSTALL_PREFIX="$WORKSPACE_DIR/install"
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            echo "Unknown option: $1" >&2
            usage
            exit 1
            ;;
    esac
    shift
done

CONFIG_DIR="$INSTALL_PREFIX/lib/cmake/tesseract_time_parameterization_core"
CONFIG_FOUND=0
for FILE in \
    "$CONFIG_DIR/tesseract_time_parameterization_coreConfig.cmake" \
    "$CONFIG_DIR/tesseract_time_parameterization_core-config.cmake"; do
    if [[ -f "$FILE" ]]; then
        CONFIG_FOUND=1
        break
    fi
done

if [[ $CONFIG_FOUND -ne 1 ]]; then
    echo "Expected tesseract_time_parameterization_core CMake package config not found in $CONFIG_DIR" >&2
    exit 2
fi

TMP_DIR="$(mktemp -d)"
trap 'rm -rf "$TMP_DIR"' EXIT

cat >"$TMP_DIR/CMakeLists.txt" <<'CMAKE'
cmake_minimum_required(VERSION 3.10)
find_package(tesseract_time_parameterization_core REQUIRED)
CMAKE

cmake -S "$TMP_DIR" -B "$TMP_DIR/build" -DCMAKE_PREFIX_PATH="$INSTALL_PREFIX"

pushd "$WORKSPACE_DIR" >/dev/null
colcon build --packages-select tesseract_task_composer --cmake-clean-cache
popd >/dev/null
