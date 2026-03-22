#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

if [[ -n ${WORKSPACE_ROOT:-} ]]; then
  WORKSPACE_ROOT="$(cd "$WORKSPACE_ROOT" && pwd)"
elif [[ -n ${WS:-} ]]; then
  WORKSPACE_ROOT="$(cd "$WS" && pwd)"
else
  repo_parent="$(dirname "$REPO_DIR")"
  if [[ $(basename "$repo_parent") == "src" ]]; then
    WORKSPACE_ROOT="$(cd "$repo_parent/.." && pwd)"
  else
    WORKSPACE_ROOT="$(pwd)"
  fi
fi

SRC_DIR="$WORKSPACE_ROOT/src"
REQUIRED_PACKAGES=(
  ur5_moveit_config
  robotiq_85_moveit_config
  ur_description
  robotiq_85_description
  workbench_description
)

path_exists() {
  local path="$1"
  [ -e "$path" ] || [ -L "$path" ]
}

resolve_package() {
  local pkg="$1"
  python3 - "$pkg" <<'PY'
import sys
try:
    from ament_index_python.packages import get_package_share_directory
except Exception:
    sys.exit(2)
pkg = sys.argv[1]
try:
    print(get_package_share_directory(pkg))
except Exception:
    sys.exit(1)
PY
}

missing=()
index_fail=()

for pkg in "${REQUIRED_PACKAGES[@]}"; do
  if ! path_exists "$SRC_DIR/$pkg"; then
    missing+=("$pkg")
    continue
  fi

  if ! resolved_path="$(resolve_package "$pkg")"; then
    case $? in
      1)
        index_fail+=("$pkg")
        ;;
      2)
        echo "ament_index_python is unavailable in the current shell. Source /opt/ros/<distro>/setup.bash and your workspace install/setup.bash before running this validator." >&2
        exit 2
        ;;
    esac
  else
    printf 'OK: %-24s -> %s\n' "$pkg" "$resolved_path"
  fi
done

if [ ${#missing[@]} -eq 0 ] && [ ${#index_fail[@]} -eq 0 ]; then
  echo
  echo "Workspace asset validation passed."
  echo "Required asset packages are exposed in src/ and resolvable through ament_index."
  exit 0
fi

echo >&2
echo "Workspace asset validation failed." >&2
if [ ${#missing[@]} -gt 0 ]; then
  printf 'Missing src/ entries: %s\n' "$(printf '%s\n' "${missing[@]}" | paste -sd ', ' -)" >&2
fi
if [ ${#index_fail[@]} -gt 0 ]; then
  printf 'Not resolvable through ament_index: %s\n' "$(printf '%s\n' "${index_fail[@]}" | paste -sd ', ' -)" >&2
fi

cat >&2 <<EOF

Remediation:
  1. cd ${WORKSPACE_ROOT}
  2. ./src/easy_manipulation_deployment/scripts/fix_workspace_layout.sh
  3. colcon build --symlink-install --parallel-workers 2
  4. source install/setup.bash
  5. rerun ${SCRIPT_DIR}/validate_workspace_assets.sh

If the workspace uses this repository as a source checkout, those asset packages stay hidden until fix_workspace_layout.sh exposes them from assets/ into src/.
EOF

exit 1
