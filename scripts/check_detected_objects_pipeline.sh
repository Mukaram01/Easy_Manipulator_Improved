#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
FIXTURES="${REPO_ROOT}/tests/fixtures/detected_objects"
TASK_FIXTURES="${REPO_ROOT}/tests/fixtures/task_recipes"
VALIDATOR="${SCRIPT_DIR}/validate_detected_objects.py"
ADAPTER="${SCRIPT_DIR}/run_task_recipe_adapter.py"
PIPELINE="${SCRIPT_DIR}/run_perception_task_pipeline.py"

pass=0
warn=0
fail=0

run_expect_status() {
  local fixture="$1"
  local expected="$2"
  local strict_flag="${3:-}"
  set +e
  local out
  out="$(python3 "${VALIDATOR}" "${FIXTURES}/${fixture}" --json ${strict_flag} 2>&1)"
  local rc=$?
  set -e
  local status
  status="$(python3 -c 'import json,sys; print(json.loads(sys.stdin.read()).get("status",""))' <<<"${out}")"
  if [[ "${status}" == "${expected}" ]]; then
    echo "PASS validate ${fixture} => ${status}"
    ((pass+=1))
  elif [[ "${expected}" == "WARN" && "${status}" == "WARN" ]]; then
    echo "WARN validate ${fixture} => ${status}"
    ((warn+=1))
  else
    echo "FAIL validate ${fixture}: expected=${expected} got=${status} rc=${rc}" >&2
    echo "${out}" >&2
    ((fail+=1))
  fi
}

run_expect_status "valid_epd_single_box.yaml" "PASS"
run_expect_status "valid_epd_colour_sorting.yaml" "PASS"
run_expect_status "valid_epd_garbage_sorting.yaml" "WARN"
run_expect_status "missing_dimensions_warn.yaml" "WARN"
run_expect_status "missing_pose_fail.yaml" "FAIL"
run_expect_status "missing_dimensions_warn.yaml" "FAIL" "--strict"

if python3 "${ADAPTER}" --task-recipe "${TASK_FIXTURES}/valid_sort_by_colour.yaml" --objects "${FIXTURES}/valid_epd_colour_sorting.yaml" --json >/dev/null; then
  echo "PASS adapter detected_objects/v1"
  ((pass+=1))
else
  echo "FAIL adapter detected_objects/v1" >&2
  ((fail+=1))
fi

if python3 "${PIPELINE}" --task-recipe "${TASK_FIXTURES}/valid_sort_by_colour.yaml" --detected-objects "${FIXTURES}/valid_epd_colour_sorting.yaml" --output-dir "${REPO_ROOT}/reports/runtime_pipeline" >/dev/null; then
  echo "PASS offline perception pipeline"
  ((pass+=1))
else
  echo "FAIL offline perception pipeline" >&2
  ((fail+=1))
fi

echo "Summary: PASS=${pass} WARN=${warn} FAIL=${fail}"
[[ ${fail} -eq 0 ]]
