#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'USAGE'
Usage: ensure_taskflow_cmake_package.sh [--export]

Create a lightweight Taskflow CMake package config for header-only source
checkouts imported into a ROS workspace. With --export, print shell exports for
Taskflow_DIR and CMAKE_PREFIX_PATH.
USAGE
}

MODE="create"
if [[ ${1:-} == "--export" ]]; then
  MODE="export"
elif [[ ${1:-} == "-h" || ${1:-} == "--help" ]]; then
  usage
  exit 0
elif [[ $# -gt 0 ]]; then
  usage >&2
  exit 1
fi

WORKSPACE_ROOT="${WS:-$(pwd)}"
if [[ ! -d "$WORKSPACE_ROOT/src" && -d "$WORKSPACE_ROOT/../src" ]]; then
  WORKSPACE_ROOT="$(cd "$WORKSPACE_ROOT/.." && pwd)"
fi
WORKSPACE_ROOT="$(cd "$WORKSPACE_ROOT" && pwd)"
TASKFLOW_SRC="${TASKFLOW_SOURCE_DIR:-$WORKSPACE_ROOT/src/taskflow}"

system_config=""
for candidate in \
  /usr/lib/cmake/Taskflow/TaskflowConfig.cmake \
  /usr/local/lib/cmake/Taskflow/TaskflowConfig.cmake \
  /usr/lib/x86_64-linux-gnu/cmake/Taskflow/TaskflowConfig.cmake; do
  if [[ -f "$candidate" ]]; then
    system_config="$candidate"
    break
  fi
done

if [[ -n "$system_config" ]]; then
  taskflow_dir="$(dirname "$system_config")"
else
  if [[ ! -f "$TASKFLOW_SRC/taskflow/taskflow.hpp" ]]; then
    echo "Taskflow headers not found at $TASKFLOW_SRC; import dependencies/emd_epd_ws.repos first." >&2
    exit 1
  fi

  taskflow_dir="$WORKSPACE_ROOT/.cmake-packages/taskflow/lib/cmake/Taskflow"
  mkdir -p "$taskflow_dir"

  cat > "$taskflow_dir/TaskflowConfig.cmake" <<CFG
set(Taskflow_FOUND TRUE)
set(Taskflow_VERSION 0)
set(Taskflow_INCLUDE_DIRS "$TASKFLOW_SRC")

if(NOT TARGET Taskflow::Taskflow)
  add_library(Taskflow::Taskflow INTERFACE IMPORTED)
  set_target_properties(Taskflow::Taskflow PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "$TASKFLOW_SRC"
  )
endif()
CFG

  cat > "$taskflow_dir/TaskflowConfigVersion.cmake" <<'CFG'
set(PACKAGE_VERSION "0")
set(PACKAGE_VERSION_COMPATIBLE TRUE)
set(PACKAGE_VERSION_EXACT FALSE)
CFG
fi

if [[ "$MODE" == "export" ]]; then
  prefix="$(cd "$taskflow_dir/../../.." && pwd)"
  printf 'export Taskflow_DIR=%q\n' "$taskflow_dir"
  printf 'export CMAKE_PREFIX_PATH=%q\n' "$prefix${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
else
  echo "Taskflow CMake package available at $taskflow_dir"
fi
