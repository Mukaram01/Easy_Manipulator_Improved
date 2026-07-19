#!/usr/bin/env bash
set -euo pipefail

WORKSPACE_ROOT="${WORKSPACE_ROOT:-$(pwd)}"
SRC_DIR="${SRC_DIR:-$WORKSPACE_ROOT/src}"

if [[ ! -d "$SRC_DIR" ]]; then
  echo "Workspace src directory not found: $SRC_DIR" >&2
  exit 1
fi
if ! command -v colcon >/dev/null 2>&1; then
  echo "colcon is required for workspace validation" >&2
  exit 1
fi

REPO_DIR="$SRC_DIR/easy_manipulation_deployment"
if [[ ! -d "$REPO_DIR" ]]; then
  echo "Missing canonical repository layout: $REPO_DIR" >&2
  exit 1
fi
if [[ ! -d "$REPO_DIR/assets" ]]; then
  echo "Missing canonical layout/content: $REPO_DIR/assets" >&2
  exit 1
fi
if [[ ! -f "$REPO_DIR/scenes/ur5_2f_test/package.xml" ]]; then
  echo "Missing canonical layout/content: $REPO_DIR/scenes/ur5_2f_test/package.xml" >&2
  exit 1
fi
if [[ ! -f "$REPO_DIR/workcell_builder/package.xml" ]]; then
  echo "Missing canonical layout/content: $REPO_DIR/workcell_builder/package.xml" >&2
  exit 1
fi

mapfile -t package_rows < <(colcon list --base-paths "$SRC_DIR" 2>/dev/null || true)
if [[ ${#package_rows[@]} -eq 0 ]]; then
  echo "No packages discovered under $SRC_DIR." >&2
  exit 1
fi

declare -A seen=()
declare -A present=()
for row in "${package_rows[@]}"; do
  name="${row%% *}"
  path="${row#* }"
  if [[ -n "${seen[$name]:-}" ]]; then
    echo "Duplicate package discovered: $name" >&2
    echo "  - ${seen[$name]}" >&2
    echo "  - $path" >&2
    exit 1
  fi
  seen[$name]="$path"
  present[$name]=1
done

required=(easy_manipulation_deployment workcell_builder)
for pkg in "${required[@]}"; do
  [[ -n "${present[$pkg]:-}" ]] || { echo "Missing required package: $pkg" >&2; exit 1; }
done

echo "Workspace validation passed: canonical content exists, packages are unique, and required packages are discoverable exactly once."
