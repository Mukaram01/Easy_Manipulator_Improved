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

mapfile -t packages < <(colcon list --base-paths "$SRC_DIR" --names-only 2>/dev/null || true)
if [[ ${#packages[@]} -eq 0 ]]; then
  echo "No packages discovered under $SRC_DIR. Run vcs import + fix_workspace_layout first." >&2
  exit 1
fi

declare -A present=()
for pkg in "${packages[@]}"; do
  present["$pkg"]=1
done

required=(
  tesseract_motion_planners
  tesseract_rosutils
  trajopt
  trajopt_common
  trajopt_sco
  trajopt_ifopt
  trajopt_sqp
)

missing=()
for pkg in "${required[@]}"; do
  if [[ -z "${present[$pkg]:-}" ]]; then
    missing+=("$pkg")
  fi
done

if [[ ${#missing[@]} -gt 0 ]]; then
  echo "Workspace validation failed: required packages are missing from colcon discovery:" >&2
  printf '  - %s\n' "${missing[@]}" >&2
  echo "Remediation: source /opt/ros/humble/setup.bash, run vcs import --recursive --skip-existing, then run scripts/fix_workspace_layout.sh and retry." >&2
  exit 1
fi

echo "Workspace validation passed: required Tesseract/TrajOpt packages are discoverable."
