#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
REPORT_PATH="${REPO_ROOT}/docs/manuals/latest_scene_validation_report.md"
SELF_TEST_REPORT_PATH="${REPO_ROOT}/docs/manuals/latest_scene_self_test_report.md"
TASK_RECIPE_REPORT_PATH="${REPO_ROOT}/docs/manuals/latest_task_recipe_report.md"
TASK_RECIPE_DRY_RUN_REPORT_PATH="${REPO_ROOT}/docs/manuals/latest_task_recipe_dry_run_report.md"
TASK_EXECUTION_PLAN_REPORT_PATH="${REPO_ROOT}/docs/manuals/latest_task_execution_plan_report.md"
SMOKE_REPORT_PATH="${REPO_ROOT}/docs/manuals/latest_smoke_launch_report.md"
WITH_SMOKE=false

while (( $# > 0 )); do
  case "$1" in
    --with-smoke)
      WITH_SMOKE=true
      ;;
    -h|--help)
      cat <<USAGE
Usage: ./scripts/preflight_workcell.sh [--with-smoke]

  --with-smoke   Run launch smoke tests and generate smoke report after validation.
USAGE
      exit 0
      ;;
    *)
      echo "Unknown option: $1" >&2
      exit 2
      ;;
  esac
  shift
done

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
"${SCRIPT_DIR}/check_scene_self_tests.sh"
"${SCRIPT_DIR}/generate_scene_self_test_report.py"
"${SCRIPT_DIR}/check_task_recipes.sh"
"${SCRIPT_DIR}/generate_task_recipe_report.py"
"${SCRIPT_DIR}/check_task_recipe_dry_runs.sh"
"${SCRIPT_DIR}/generate_task_recipe_dry_run_report.py"
"${SCRIPT_DIR}/check_task_execution_plans.sh"
"${SCRIPT_DIR}/generate_task_execution_plan_report.py"

echo
printf 'Preflight report written to: %s\n' "${REPORT_PATH}"
printf 'Self-test report written to: %s\n' "${SELF_TEST_REPORT_PATH}"
printf 'Task recipe report written to: %s\n' "${TASK_RECIPE_REPORT_PATH}"
printf 'Task recipe dry-run report written to: %s\n' "${TASK_RECIPE_DRY_RUN_REPORT_PATH}"
printf 'Task execution plan report written to: %s\n' "${TASK_EXECUTION_PLAN_REPORT_PATH}"

if [[ "${WITH_SMOKE}" == true ]]; then
  echo
  echo "=== Optional launch smoke preflight ==="
  "${SCRIPT_DIR}/smoke_launch_scenes.sh"
  "${SCRIPT_DIR}/generate_smoke_launch_report.py"
  echo
  printf 'Smoke launch report written to: %s\n' "${SMOKE_REPORT_PATH}"
fi
