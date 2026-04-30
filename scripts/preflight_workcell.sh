#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
REPORT_PATH="${REPO_ROOT}/docs/manuals/latest_scene_validation_report.md"
SELF_TEST_REPORT_PATH="${REPO_ROOT}/docs/manuals/latest_scene_self_test_report.md"
TASK_RECIPE_REPORT_PATH="${REPO_ROOT}/docs/manuals/latest_task_recipe_report.md"
TASK_RECIPE_DRY_RUN_REPORT_PATH="${REPO_ROOT}/docs/manuals/latest_task_recipe_dry_run_report.md"
TASK_EXECUTION_PLAN_REPORT_PATH="${REPO_ROOT}/docs/manuals/latest_task_execution_plan_report.md"
WORKCELL_BUNDLE_DIR="${REPO_ROOT}/dist/workcell_bundles"
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
echo "=== Scene creation readiness helpers (offline, non-invasive) ==="
if python3 "${SCRIPT_DIR}/report_workcell_builder_paths.py"; then
  echo "Scene creation path report: PASS"
else
  echo "Scene creation path report: WARN (report tool unavailable)"
fi

if [[ -d "${REPO_ROOT}/scenes" ]]; then
  if python3 "${SCRIPT_DIR}/check_scene_readiness.py" --workcell-root "${REPO_ROOT}"; then
    echo "Scene readiness check: PASS/WARN"
  else
    echo "Scene readiness check: WARN (readiness failures detected)"
  fi
else
  echo "Scene readiness check: SKIP (no scenes directory at ${REPO_ROOT}/scenes)"
fi

LAYOUT_FIXTURE="${REPO_ROOT}/tests/fixtures/environment_layouts/ur5_table_bins_existing_assets.layout.yaml"
if [[ -f "${LAYOUT_FIXTURE}" ]]; then
  if python3 "${SCRIPT_DIR}/environment_layout_to_scene_checklist.py" "${LAYOUT_FIXTURE}" >/dev/null; then
    echo "Environment layout checklist bridge: PASS"
  else
    echo "Environment layout checklist bridge: WARN (could not generate checklist)"
  fi
else
  echo "Environment layout checklist bridge: SKIP (fixture not found)"
fi

echo
echo "INFO: To capture live EPD detections manually, run:"
echo "python3 ${SCRIPT_DIR}/capture_epd_detected_objects.py --topic /easy_perception_deployment/epd_localize_output --output /tmp/mvp1/live_detected_objects.yaml --scene-package ur5_2f_test --timeout 10 --min-objects 1 --once"
LIVE_DETECTIONS="/tmp/mvp1/live_detected_objects.yaml"
if [[ -f "${LIVE_DETECTIONS}" ]]; then
  set +e
  LIVE_VALIDATE_OUTPUT="$(python3 "${SCRIPT_DIR}/validate_detected_objects.py" "${LIVE_DETECTIONS}" --json)"
  LIVE_VALIDATE_EXIT=$?
  set -e
  echo "${LIVE_VALIDATE_OUTPUT}"
  if [[ ${LIVE_VALIDATE_EXIT} -eq 0 ]]; then
    echo "Live EPD detected_objects snapshot validation: PASS/WARN"
  else
    echo "Live EPD detected_objects snapshot validation: WARN (validation failed)"
  fi
else
  echo "INFO: Live EPD detected_objects snapshot not found at ${LIVE_DETECTIONS}; skipping optional validation."
fi

echo
"${SCRIPT_DIR}/check_all_scenes.sh"
"${SCRIPT_DIR}/check_capability_contracts.sh"
"${SCRIPT_DIR}/check_cell_capability_integration.sh"
"${SCRIPT_DIR}/check_environment_layouts.sh"
"${SCRIPT_DIR}/generate_scene_validation_report.py"
"${SCRIPT_DIR}/check_scene_self_tests.sh"
"${SCRIPT_DIR}/generate_scene_self_test_report.py"
"${SCRIPT_DIR}/check_task_recipes.sh"
"${SCRIPT_DIR}/generate_task_recipe_report.py"
"${SCRIPT_DIR}/check_task_recipe_dry_runs.sh"
"${SCRIPT_DIR}/generate_task_recipe_dry_run_report.py"
"${SCRIPT_DIR}/check_task_execution_plans.sh"
"${SCRIPT_DIR}/check_emd_grasp_bridge.sh"
"${SCRIPT_DIR}/check_detected_objects_pipeline.sh"
set +e
MVP1_OUTPUT="$(python3 "${SCRIPT_DIR}/run_generated_cell_acceptance.py" \
  --scene-package ur5_robotiq_generated_cell \
  --task-recipe "${REPO_ROOT}/tests/fixtures/task_recipes/mvp1_generated_cell_colour_sorting.yaml" \
  --detected-objects "${REPO_ROOT}/tests/fixtures/detected_objects/mvp1_colour_sorting_with_fallback.yaml" \
  --output-dir "${REPO_ROOT}/reports/generated_cell_acceptance" --json)"
MVP1_EXIT=$?
set -e
echo "${MVP1_OUTPUT}"
if [[ ${MVP1_EXIT} -eq 0 ]]; then
  if grep -q '"status": "WARN"' <<<"${MVP1_OUTPUT}"; then
    echo "Generated cell MVP-1 acceptance: WARN"
  else
    echo "Generated cell MVP-1 acceptance: PASS"
  fi
else
  echo "Generated cell MVP-1 acceptance: FAIL"
fi

MVP1_BRIDGE_PAYLOAD="/tmp/mvp1/emd_grasp_bridge_payload.json"
if [[ -f "${MVP1_BRIDGE_PAYLOAD}" ]]; then
  set +e
  REPLAY_OUTPUT="$(python3 "${SCRIPT_DIR}/replay_emd_bridge_payload.py" --payload "${MVP1_BRIDGE_PAYLOAD}" --scene-package ur5_2f_test --dry-run)"
  REPLAY_EXIT=$?
  set -e
  echo "${REPLAY_OUTPUT}"
  if [[ ${REPLAY_EXIT} -eq 0 ]]; then
    echo "MVP-1 bridge replay dry-run: PASS/WARN"
  else
    echo "MVP-1 bridge replay dry-run: WARN (validation failed)"
  fi
else
  echo "INFO: Skipping MVP-1 bridge replay dry-run (/tmp/mvp1/emd_grasp_bridge_payload.json not found)."
fi
"${SCRIPT_DIR}/generate_task_execution_plan_report.py"
"${SCRIPT_DIR}/check_workcell_bundles.sh"
"${SCRIPT_DIR}/check_generated_workcells.sh"
"${SCRIPT_DIR}/check_workcell_projects.sh"
"${SCRIPT_DIR}/check_workcell_dashboard.sh"
set +e
CELL_WIZARD_OUTPUT="$(${SCRIPT_DIR}/check_cell_definition_wizard.sh)"
CELL_WIZARD_EXIT=$?
set -e
echo "${CELL_WIZARD_OUTPUT}"
if [[ ${CELL_WIZARD_EXIT} -eq 0 ]]; then
  if grep -q "Cell definition wizard checks: WARN" <<<"${CELL_WIZARD_OUTPUT}"; then
    echo "Cell definition wizard checks: WARN"
  else
    echo "Cell definition wizard checks: PASS"
  fi
else
  echo "Cell definition wizard checks: FAIL"
  exit ${CELL_WIZARD_EXIT}
fi
set +e
CELL_DEF_OUTPUT="$(${SCRIPT_DIR}/check_cell_definitions.sh)"
CELL_DEF_EXIT=$?
set -e
echo "${CELL_DEF_OUTPUT}"
if [[ ${CELL_DEF_EXIT} -eq 0 ]]; then
  if grep -q "Cell definition checks: WARN" <<<"${CELL_DEF_OUTPUT}"; then
    echo "Cell definition checks: WARN"
  else
    echo "Cell definition checks: PASS"
  fi
else
  echo "Cell definition checks: FAIL"
  exit ${CELL_DEF_EXIT}
fi

set +e
WORKCELL_TEMPLATE_OUTPUT="$(${SCRIPT_DIR}/check_workcell_builder_templates.sh)"
WORKCELL_TEMPLATE_EXIT=$?
set -e
echo "${WORKCELL_TEMPLATE_OUTPUT}"

if [[ ${WORKCELL_TEMPLATE_EXIT} -eq 0 ]]; then
  if grep -q "Workcell builder template check: WARN" <<<"${WORKCELL_TEMPLATE_OUTPUT}"; then
    echo "Workcell builder template check: WARN"
  else
    echo "Workcell builder template check: PASS"
  fi
else
  echo "Workcell builder template check: FAIL"
  exit ${WORKCELL_TEMPLATE_EXIT}
fi

echo
printf 'Preflight report written to: %s\n' "${REPORT_PATH}"
printf 'Self-test report written to: %s\n' "${SELF_TEST_REPORT_PATH}"
printf 'Task recipe report written to: %s\n' "${TASK_RECIPE_REPORT_PATH}"
printf 'Task recipe dry-run report written to: %s\n' "${TASK_RECIPE_DRY_RUN_REPORT_PATH}"
printf 'Task execution plan report written to: %s\n' "${TASK_EXECUTION_PLAN_REPORT_PATH}"
printf 'Workcell bundles written to: %s\n' "${WORKCELL_BUNDLE_DIR}"

if [[ "${WITH_SMOKE}" == true ]]; then
  echo
  echo "=== Optional launch smoke preflight ==="
  "${SCRIPT_DIR}/smoke_launch_scenes.sh"
  "${SCRIPT_DIR}/generate_smoke_launch_report.py"
  echo
  printf 'Smoke launch report written to: %s\n' "${SMOKE_REPORT_PATH}"
fi
