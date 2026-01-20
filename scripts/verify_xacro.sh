#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd "${script_dir}/.." && pwd)"

workspace_root=""
if [[ -n "${WORKSPACE_ROOT:-}" ]]; then
  workspace_root="${WORKSPACE_ROOT}"
elif [[ -n "${WS:-}" ]]; then
  workspace_root="${WS}"
else
  search_dir="${repo_root}"
  while [[ -n "${search_dir}" && "${search_dir}" != "/" ]]; do
    if [[ -f "${search_dir}/install/setup.bash" ]]; then
      workspace_root="${search_dir}"
      break
    fi
    search_dir="$(dirname "${search_dir}")"
  done

  if [[ -z "${workspace_root}" ]]; then
    if [[ "${repo_root}" == *"/src/"* ]]; then
      workspace_root="${repo_root%%/src/*}"
    elif [[ "${repo_root}" == */src ]]; then
      workspace_root="${repo_root%/src}"
    fi
  fi
fi

if [[ -z "${workspace_root}" ]]; then
  echo "Error: Unable to determine WORKSPACE_ROOT." >&2
  echo "Resolved repo root: ${repo_root}" >&2
  exit 1
fi

ros_setup="/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
workspace_setup="${workspace_root}/install/setup.bash"

if [[ ! -f "${ros_setup}" ]]; then
  echo "Error: ROS setup file not found at ${ros_setup}." >&2
  echo "Resolved workspace root: ${workspace_root}" >&2
  exit 1
fi

if [[ ! -f "${workspace_setup}" ]]; then
  echo "Error: Workspace setup file not found at ${workspace_setup}." >&2
  echo "Resolved workspace root: ${workspace_root}" >&2
  exit 1
fi

source "${ros_setup}"
source "${workspace_setup}"

search_paths=(
  "${workspace_root}/src/scenes"
  "${repo_root}/scenes"
)

shopt -s nullglob
xacro_files=(
  "${search_paths[0]}"/*/urdf/scene.urdf.xacro
  "${search_paths[1]}"/*/urdf/scene.urdf.xacro
)
shopt -u nullglob

if (( ${#xacro_files[@]} == 0 )); then
  echo "Error: No scene.urdf.xacro files found." >&2
  echo "Resolved workspace root: ${workspace_root}" >&2
  echo "Search paths:" >&2
  for path in "${search_paths[@]}"; do
    echo "  - ${path}" >&2
  done
  exit 1
fi

for xacro_path in "${xacro_files[@]}"; do
  error_log="$(mktemp)"
  if ! ros2 run xacro xacro --inorder "${xacro_path}" > /tmp/xacro_check.urdf 2> "${error_log}"; then
    echo "Error: Xacro check failed for ${xacro_path}." >&2
    cat "${error_log}" >&2
    rm -f "${error_log}"
    exit 1
  fi
  rm -f "${error_log}"
  echo "OK: ${xacro_path}"
done

echo "All xacro files verified successfully."
