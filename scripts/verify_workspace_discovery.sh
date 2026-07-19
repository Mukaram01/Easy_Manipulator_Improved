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

CANONICAL_REPO="$SRC_DIR/easy_manipulation_deployment"

canonical_layout_valid=false
if [[ -d "$CANONICAL_REPO" \
  && -d "$CANONICAL_REPO/assets" \
  && -d "$CANONICAL_REPO/scenes" \
  && -f "$CANONICAL_REPO/workcell_builder/package.xml" \
  && -f "$CANONICAL_REPO/scenes/ur5_2f_test/package.xml" ]]; then
  canonical_layout_valid=true
fi

legacy_layout_valid=false
if [[ -d "$SRC_DIR/assets" \
  && -d "$SRC_DIR/scenes" \
  && -f "$SRC_DIR/scenes/ur5_2f_test/package.xml" ]]; then
  legacy_layout_valid=true
fi

if [[ "$canonical_layout_valid" == true ]]; then
  layout_description="canonical repository layout at $CANONICAL_REPO"
elif [[ "$legacy_layout_valid" == true ]]; then
  layout_description="legacy workspace aliases at $SRC_DIR/assets and $SRC_DIR/scenes"
else
  echo "Missing workspace layout: expected canonical repo at $CANONICAL_REPO with assets/ and scenes/, or legacy aliases at $SRC_DIR/assets and $SRC_DIR/scenes." >&2
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

echo "Workspace validation passed: $layout_description is present, packages are unique, and required packages are discoverable exactly once."
