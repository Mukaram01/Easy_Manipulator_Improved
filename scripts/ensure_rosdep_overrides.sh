#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
OVERRIDES_FILE="${REPO_ROOT}/scripts/rosdep_overrides.yaml"
ROSDEP_SOURCE_LIST="/etc/ros/rosdep/sources.list.d/10-easy-manipulator-overrides.list"
ROSDEP_SOURCE_ENTRY="yaml file://${OVERRIDES_FILE}"
VALIDATION_KEY="${1:-cereal}"

if [[ ! -f "$OVERRIDES_FILE" ]]; then
  echo "Rosdep overrides file is missing: $OVERRIDES_FILE" >&2
  exit 1
fi

run_as_root() {
  if [[ $EUID -eq 0 ]]; then
    "$@"
  elif command -v sudo >/dev/null 2>&1; then
    sudo "$@"
  else
    echo "Rosdep override registration needs root access to write $ROSDEP_SOURCE_LIST." >&2
    echo "Run as root or install the source manually before rosdep install:" >&2
    echo "  echo '$ROSDEP_SOURCE_ENTRY' | sudo tee $ROSDEP_SOURCE_LIST >/dev/null" >&2
    echo "  rosdep update" >&2
    exit 1
  fi
}

needs_update=0
if [[ ! -f "$ROSDEP_SOURCE_LIST" ]]; then
  needs_update=1
elif ! grep -Fxq "$ROSDEP_SOURCE_ENTRY" "$ROSDEP_SOURCE_LIST"; then
  needs_update=1
fi

if [[ $needs_update -eq 1 ]]; then
  echo "Registering rosdep overrides from $OVERRIDES_FILE"
  run_as_root mkdir -p "$(dirname "$ROSDEP_SOURCE_LIST")"
  printf '%s\n' "$ROSDEP_SOURCE_ENTRY" | run_as_root tee "$ROSDEP_SOURCE_LIST" >/dev/null
else
  echo "Rosdep overrides already registered in $ROSDEP_SOURCE_LIST"
fi

rosdep update

echo "Validating rosdep override resolution for '$VALIDATION_KEY'"
rosdep resolve "$VALIDATION_KEY"
