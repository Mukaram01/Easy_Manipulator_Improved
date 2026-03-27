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
  table_description
  ur5_moveit_config
  robotiq_85_moveit_config
  single_suction_moveit_config
  ur_description
  robotiq_85_description
  single_suction_description
  workbench_description
)
REPOSITORY_PACKAGES=(
  table_description
  ur5_moveit_config
  robotiq_85_moveit_config
  single_suction_moveit_config
  robotiq_85_description
  single_suction_description
  workbench_description
)
EXTERNAL_PACKAGES=(
  ur_description
)
SUCTION_PACKAGES=(
  single_suction_description
  single_suction_moveit_config
)

package_in_list() {
  local pkg="$1"
  shift
  local candidate

  for candidate in "$@"; do
    if [ "$candidate" = "$pkg" ]; then
      return 0
    fi
  done

  return 1
}

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

join_by_comma() {
  if [ $# -eq 0 ]; then
    return
  fi
  local first=1
  local item
  for item in "$@"; do
    if [ $first -eq 1 ]; then
      printf '%s' "$item"
      first=0
    else
      printf ', %s' "$item"
    fi
  done
  printf '\n'
}

if [ ${#missing[@]} -eq 0 ] && [ ${#index_fail[@]} -eq 0 ]; then
  echo
  echo "Workspace asset validation passed."
  echo "Required repository asset packages are exposed in src/, and required external dependencies resolve through ament_index."
  exit 0
fi

echo >&2
echo "Workspace asset validation failed." >&2
if [ ${#missing[@]} -gt 0 ]; then
  printf 'Missing src/ entries: %s\n' "$(join_by_comma "${missing[@]}")" >&2
fi
if [ ${#index_fail[@]} -gt 0 ]; then
  printf 'Not resolvable through ament_index: %s\n' "$(join_by_comma "${index_fail[@]}")" >&2
fi

suction_missing=()
suction_index_fail=()
for pkg in "${missing[@]}"; do
  if package_in_list "$pkg" "${SUCTION_PACKAGES[@]}"; then
    suction_missing+=("$pkg")
  fi
done
for pkg in "${index_fail[@]}"; do
  if package_in_list "$pkg" "${SUCTION_PACKAGES[@]}"; then
    suction_index_fail+=("$pkg")
  fi
done

if [ ${#suction_missing[@]} -gt 0 ]; then
  printf 'Suction packages missing from src/: %s\n' "$(join_by_comma "${suction_missing[@]}")" >&2
fi
if [ ${#suction_index_fail[@]} -gt 0 ]; then
  printf 'Suction packages not resolvable through ament_index: %s\n' "$(join_by_comma "${suction_index_fail[@]}")" >&2
fi

repo_missing=()
repo_index_fail=()
external_missing=()
external_index_fail=()

for pkg in "${missing[@]}"; do
  if package_in_list "$pkg" "${REPOSITORY_PACKAGES[@]}"; then
    repo_missing+=("$pkg")
  elif package_in_list "$pkg" "${EXTERNAL_PACKAGES[@]}"; then
    external_missing+=("$pkg")
  fi
done

for pkg in "${index_fail[@]}"; do
  if package_in_list "$pkg" "${REPOSITORY_PACKAGES[@]}"; then
    repo_index_fail+=("$pkg")
  elif package_in_list "$pkg" "${EXTERNAL_PACKAGES[@]}"; then
    external_index_fail+=("$pkg")
  fi
done

cat >&2 <<EOF

Remediation:
EOF

if [ ${#repo_missing[@]} -gt 0 ] || [ ${#repo_index_fail[@]} -gt 0 ]; then
  cat >&2 <<EOF
  Repository asset packages: $(join_by_comma "${repo_missing[@]}" "${repo_index_fail[@]}")
  (including suction assets: single_suction_description, single_suction_moveit_config)
    1. cd ${WORKSPACE_ROOT}
    2. ./src/easy_manipulation_deployment/scripts/fix_workspace_layout.sh
    3. colcon build --symlink-install --parallel-workers 2
    4. source install/setup.bash
    5. rerun ${SCRIPT_DIR}/validate_workspace_assets.sh

EOF
fi

if [ ${#external_missing[@]} -gt 0 ] || [ ${#external_index_fail[@]} -gt 0 ]; then
  cat >&2 <<EOF
  External ROS dependency: $(join_by_comma "${external_missing[@]}" "${external_index_fail[@]}")
    - Install the required system package documented in README.md step 1, e.g. sudo apt install -y ros-humble-ur-description
    - Rebuild/source the workspace if needed, then rerun ${SCRIPT_DIR}/validate_workspace_assets.sh

EOF
fi

if [ ${#repo_missing[@]} -gt 0 ] || [ ${#repo_index_fail[@]} -gt 0 ]; then
  cat >&2 <<EOF
If the workspace uses this repository as a source checkout, those repository asset packages (including suction packages) stay hidden until fix_workspace_layout.sh exposes them from assets/ into src/.
EOF
fi

exit 1
