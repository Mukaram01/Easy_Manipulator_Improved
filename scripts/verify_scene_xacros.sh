#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd "${script_dir}/.." && pwd)"

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "Error: ROS 2 Humble setup not found at /opt/ros/humble/setup.bash." >&2
  exit 1
fi

if [[ ! -f "${repo_root}/install/setup.bash" ]]; then
  echo "Error: Workspace overlay setup not found at ${repo_root}/install/setup.bash." >&2
  echo "Build the workspace and try again." >&2
  exit 1
fi

source /opt/ros/humble/setup.bash
source "${repo_root}/install/setup.bash"

shopt -s nullglob
xacro_files=("${repo_root}"/scenes/*/urdf/scene.urdf.xacro)
shopt -u nullglob

if (( ${#xacro_files[@]} == 0 )); then
  echo "Error: No scene.urdf.xacro files found under ${repo_root}/scenes." >&2
  exit 1
fi

for xacro_path in "${xacro_files[@]}"; do
  scene_dir="$(dirname "$(dirname "${xacro_path}")")"
  scene_name="$(basename "${scene_dir}")"
  output_path="/tmp/${scene_name}.urdf"
  if ! xacro "${xacro_path}" > "${output_path}"; then
    echo "Error: Failed to process xacro ${xacro_path}." >&2
    exit 1
  fi
  echo "Generated ${output_path} from ${xacro_path}"
done
