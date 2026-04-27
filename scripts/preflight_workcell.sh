#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
REPORT_PATH="${REPO_ROOT}/docs/manuals/latest_scene_validation_report.md"

printf '\n=== Easy Manipulation Workcell Preflight ===\n'
printf 'Repository: %s\n' "${REPO_ROOT}"
printf 'Timestamp (UTC): %s\n\n' "$(date -u +"%Y-%m-%dT%H:%M:%SZ")"

if command -v ros2 >/dev/null 2>&1; then
  echo "ROS check: ros2 command found."
else
  echo "ROS check: ros2 command NOT found (validation may SKIP discoverability checks)."
fi

if [[ -n "${AMENT_PREFIX_PATH:-}" ]]; then
  echo "ROS check: AMENT_PREFIX_PATH is set."
else
  echo "ROS check: AMENT_PREFIX_PATH is not set (source ROS/workspace setup before launch preflight)."
fi

echo
"${SCRIPT_DIR}/check_all_scenes.sh"
"${SCRIPT_DIR}/generate_scene_validation_report.py"

echo
printf 'Preflight report written to: %s\n' "${REPORT_PATH}"
