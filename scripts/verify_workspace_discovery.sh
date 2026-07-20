#!/usr/bin/env bash
set -euo pipefail

WORKSPACE_ROOT="${WORKSPACE_ROOT:-$(pwd)}"
SRC_DIR="${SRC_DIR:-$WORKSPACE_ROOT/src}"
CANONICAL_REPO="$SRC_DIR/easy_manipulation_deployment"
REQUIRED_PACKAGES=(workcell_builder ur5_2f_test workbench_description)

if [[ ! -d "$SRC_DIR" ]]; then
  echo "Workspace src directory not found: $SRC_DIR" >&2
  exit 1
fi
if ! command -v colcon >/dev/null 2>&1; then
  echo "colcon is required for workspace validation" >&2
  exit 1
fi

colcon_output=""
if ! colcon_output="$(colcon list --base-paths "$SRC_DIR" 2>&1)"; then
  echo "colcon package discovery failed under $SRC_DIR:" >&2
  printf '%s\n' "$colcon_output" >&2
  exit 1
fi

mapfile -t package_rows < <(printf '%s\n' "$colcon_output" | sed '/^[[:space:]]*$/d')
if [[ ${#package_rows[@]} -eq 0 ]]; then
  echo "No packages discovered by: colcon list --base-paths $SRC_DIR" >&2
  exit 1
fi

declare -A seen=()
declare -A present=()
for row in "${package_rows[@]}"; do
  read -r name path _extra <<<"$row"
  if [[ -z "${name:-}" || -z "${path:-}" ]]; then
    echo "Invalid colcon list row: $row" >&2
    exit 1
  fi
  if [[ -n "${seen[$name]:-}" ]]; then
    echo "Duplicate package discovered: $name" >&2
    echo "  - ${seen[$name]}" >&2
    echo "  - $path" >&2
    exit 1
  fi
  if [[ ! -f "$path/package.xml" ]]; then
    echo "Discovered package path is missing package.xml: $name -> $path/package.xml" >&2
    exit 1
  fi
  seen[$name]="$path"
  present[$name]=1
done

for pkg in "${REQUIRED_PACKAGES[@]}"; do
  if [[ -z "${present[$pkg]:-}" ]]; then
    echo "Missing required package from colcon discovery: $pkg" >&2
    echo "Command: colcon list --base-paths $SRC_DIR" >&2
    exit 1
  fi
done

repo_real="$(realpath "$CANONICAL_REPO")"
workcell_builder_path="${seen[workcell_builder]}"
workcell_builder_real="$(realpath "$workcell_builder_path")"
if [[ ! -f "$workcell_builder_real/package.xml" ]]; then
  echo "Discovered workcell_builder path is missing package.xml: $workcell_builder_real/package.xml" >&2
  exit 1
fi
case "$workcell_builder_real" in
  "$repo_real"|"$repo_real"/*) ;;
  *)
    echo "Discovered workcell_builder path is outside easy_manipulation_deployment checkout: $workcell_builder_real" >&2
    echo "Expected within: $repo_real" >&2
    echo "Command: colcon list --base-paths $SRC_DIR" >&2
    exit 1
    ;;
esac

echo "Workspace validation passed: colcon discovered required packages exactly once, workcell_builder resolves inside easy_manipulation_deployment, and package names are unique."
