#!/usr/bin/env bash
set -euo pipefail

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "Error: /opt/ros/humble/setup.bash not found." >&2
  exit 1
fi

if [[ ! -f ./install/setup.bash ]]; then
  echo "Error: ./install/setup.bash not found. Run this from the workspace root after building." >&2
  exit 1
fi

source /opt/ros/humble/setup.bash
source ./install/setup.bash

shopt -s nullglob
xacro_files=(scenes/*/urdf/scene.urdf.xacro)
shopt -u nullglob

if (( ${#xacro_files[@]} == 0 )); then
  echo "Error: No scene.urdf.xacro files found under scenes/." >&2
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
