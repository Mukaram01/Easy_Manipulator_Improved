#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
GENERATOR="${SCRIPT_DIR}/generate_task_execution_plan.py"
REPORTER="${SCRIPT_DIR}/generate_task_execution_plan_report.py"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
OUTPUT_DIR="${REPO_ROOT}/docs/manuals/generated_execution_plans"

if [[ ! -x "${GENERATOR}" ]]; then
  echo "ERROR: Missing task execution plan generator at ${GENERATOR}" >&2
  exit 2
fi

if [[ ! -x "${REPORTER}" ]]; then
  echo "ERROR: Missing task execution plan reporter at ${REPORTER}" >&2
  exit 2
fi

set +e
CHECK_OUTPUT="$(${GENERATOR} --check "$@")"
GEN_EXIT=$?
set -e

if [[ ${GEN_EXIT} -ne 0 ]]; then
  echo "ERROR: Task execution plan generation command failed." >&2
  echo "${CHECK_OUTPUT}" >&2
  exit ${GEN_EXIT}
fi

echo "${CHECK_OUTPUT}"

if [[ ! -d "${OUTPUT_DIR}" ]]; then
  mkdir -p "${OUTPUT_DIR}"
fi

while IFS= read -r line; do
  [[ -z "${line}" ]] && continue
  case "${line}" in
    PASS*)
      scene="$(awk '{print $2}' <<<"${line}")"
      steps_field="$(awk '{print $3}' <<<"${line}")"
      md_field="$(awk '{print $4}' <<<"${line}")"
      json_field="$(awk '{print $5}' <<<"${line}")"

      steps="${steps_field#steps=}"
      md_path="${md_field#md=}"
      json_path="${json_field#json=}"

      if [[ "${steps}" -lt 1 ]]; then
        echo "ERROR: PASS scene ${scene} generated no steps." >&2
        exit 1
      fi
      if [[ ! -f "${md_path}" ]]; then
        echo "ERROR: PASS scene ${scene} missing markdown output ${md_path}." >&2
        exit 1
      fi
      if [[ ! -f "${json_path}" ]]; then
        echo "ERROR: PASS scene ${scene} missing JSON output ${json_path}." >&2
        exit 1
      fi
      ;;
    WARN*|FAIL*|SKIP*|Summary:*)
      ;;
    *)
      ;;
  esac
done <<< "${CHECK_OUTPUT}"

${REPORTER} "$@"

echo "Execution plan checks complete."
