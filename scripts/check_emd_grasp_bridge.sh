#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
BRIDGE="${SCRIPT_DIR}/convert_runtime_plan_to_emd_grasp.py"
FIXTURES="${REPO_ROOT}/tests/fixtures/emd_grasp_bridge"

if [[ ! -x "${BRIDGE}" ]]; then
  echo "FAIL: missing bridge converter ${BRIDGE}" >&2
  exit 2
fi

run_case() {
  local name="$1"
  local expected_status="$2"
  shift 2
  local output
  set +e
  output="$(python3 "${BRIDGE}" --runtime-plan "${FIXTURES}/${name}" --json "$@" 2>&1)"
  local cmd_exit=$?
  set -e
  local status
  status="$(python3 -c 'import json,sys
text=sys.stdin.read()
start=text.find("{")
print(json.loads(text[start:]).get("status",""))' <<<"${output}")"
  if [[ "${status}" == "${expected_status}" ]]; then
    echo "PASS fixture=${name} status=${status} cmd_exit=${cmd_exit}"
    return 0
  fi
  echo "FAIL fixture=${name} expected=${expected_status} got=${status} cmd_exit=${cmd_exit}" >&2
  echo "${output}" >&2
  return 1
}


pass_count=0
warn_count=0
fail_count=0

if run_case "valid_single_box_runtime_plan.json" "PASS"; then ((pass_count+=1)); else ((fail_count+=1)); fi
if run_case "valid_colour_sort_multi_runtime_plan.json" "WARN"; then ((warn_count+=1)); else ((fail_count+=1)); fi
if run_case "missing_dimensions_warn_runtime_plan.json" "WARN"; then ((warn_count+=1)); else ((fail_count+=1)); fi
if run_case "missing_pose_fail_runtime_plan.json" "FAIL"; then ((pass_count+=1)); else ((fail_count+=1)); fi
if run_case "synthesized_grasp_pose_warn_runtime_plan.json" "WARN"; then ((warn_count+=1)); else ((fail_count+=1)); fi
if run_case "rejected_unknown_skip_runtime_plan.json" "WARN"; then ((warn_count+=1)); else ((fail_count+=1)); fi
if run_case "shape_mapping_cases_runtime_plan.json" "WARN"; then ((warn_count+=1)); else ((fail_count+=1)); fi

if run_case "missing_dimensions_warn_runtime_plan.json" "FAIL" --strict; then ((pass_count+=1)); else ((fail_count+=1)); fi

echo "Summary: PASS=${pass_count} WARN=${warn_count} FAIL=${fail_count}"
[[ ${fail_count} -eq 0 ]]
