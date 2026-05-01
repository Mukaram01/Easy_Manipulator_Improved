#!/usr/bin/env bash
set -euo pipefail

mkdir -p logs/colcon_builds
log_file="logs/colcon_builds/colcon_build_$(date -u +%Y%m%dT%H%M%SZ).log"

build_cmd=(
  colcon build --symlink-install
  --packages-skip tesseract_rviz
  --allow-overriding ruckig tesseract_monitoring tesseract_msgs tesseract_rosutils
  --metas colcon/quiet_third_party_warnings.meta
)

echo "Running developer build command:"
printf '  %q' "${build_cmd[@]}"
echo

echo "Build log: ${log_file}"
set +e
"${build_cmd[@]}" 2>&1 | tee "${log_file}"
build_rc=${PIPESTATUS[0]}
set -e

if [[ ${build_rc} -ne 0 ]]; then
  echo "Build failed with exit code ${build_rc}."
  exit "${build_rc}"
fi

scripts/check_colcon_build_warnings.py --log "${log_file}" --fail-on-owned
