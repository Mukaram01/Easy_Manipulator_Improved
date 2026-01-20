#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="${WORKSPACE_ROOT:-$(cd "${script_dir}/.." && pwd)}"
ros_distro="${ROS_DISTRO:-humble}"
ros_setup="/opt/ros/${ros_distro}/setup.bash"

if [[ ! -f "${ros_setup}" ]]; then
  echo "Error: ROS 2 setup not found at ${ros_setup}." >&2
  exit 1
fi

if [[ ! -f "${WORKSPACE_ROOT}/install/setup.bash" ]]; then
  echo "Error: Workspace overlay setup not found at ${WORKSPACE_ROOT}/install/setup.bash." >&2
  echo "Build the workspace and try again." >&2
  exit 1
fi

source "${ros_setup}"
source "${WORKSPACE_ROOT}/install/setup.bash"

shopt -s nullglob
xacro_files=("${WORKSPACE_ROOT}"/scenes/*/urdf/scene.urdf.xacro)
shopt -u nullglob

if (( ${#xacro_files[@]} == 0 )); then
  echo "Error: No scene.urdf.xacro files found under ${WORKSPACE_ROOT}/scenes." >&2
  exit 1
fi

for xacro_path in "${xacro_files[@]}"; do
  scene_dir="$(dirname "$(dirname "${xacro_path}")")"
  scene_name="$(basename "${scene_dir}")"
  output_path="/tmp/${scene_name}.urdf"
  error_log="$(mktemp)"
  if ! xacro --inorder "${xacro_path}" > "${output_path}" 2> "${error_log}"; then
    echo "Error: Failed to process xacro ${xacro_path}." >&2
    cat "${error_log}" >&2
    rm -f "${error_log}"
    exit 1
  fi
  rm -f "${error_log}"
  echo "Generated ${output_path} from ${xacro_path}"
done
