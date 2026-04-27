#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
EXPORTER="${SCRIPT_DIR}/export_workcell_bundle.py"
INSPECTOR="${SCRIPT_DIR}/inspect_workcell_bundle.py"
OUTPUT_DIR="${REPO_ROOT}/dist/workcell_bundles"
PLAN_DIR="${REPO_ROOT}/docs/manuals/generated_execution_plans"

if [[ ! -x "${EXPORTER}" ]]; then
  echo "FAIL: missing exporter script at ${EXPORTER}" >&2
  exit 2
fi

if [[ ! -x "${INSPECTOR}" ]]; then
  echo "FAIL: missing inspector script at ${INSPECTOR}" >&2
  exit 2
fi

set +e
EXPORT_OUTPUT="$(${EXPORTER} --force "$@")"
EXPORT_EXIT=$?
set -e

if [[ ${EXPORT_EXIT} -ne 0 ]]; then
  echo "FAIL: bundle export failed." >&2
  echo "${EXPORT_OUTPUT}" >&2
  exit ${EXPORT_EXIT}
fi

echo "${EXPORT_OUTPUT}"

if (( $# > 0 )); then
  SCENES=("$@")
else
  mapfile -t SCENES < <(
    find "${REPO_ROOT}/scenes" -mindepth 2 -maxdepth 2 \
      \( -name "scene_manifest.yaml" -o -name "workcell.yaml" \) \
      -printf '%h\n' | xargs -n1 basename | sort -u
  )
fi

SUMMARY_FAIL=0
SUMMARY_WARN=0
SUMMARY_PASS=0
SUMMARY_SKIP=0

for scene in "${SCENES[@]}"; do
  bundle_path="${OUTPUT_DIR}/${scene}"
  if [[ ! -d "${bundle_path}" ]]; then
    echo "FAIL ${scene}: bundle folder missing (${bundle_path})"
    SUMMARY_FAIL=$((SUMMARY_FAIL + 1))
    continue
  fi

  set +e
  INSPECT_OUTPUT="$(${INSPECTOR} "${bundle_path}")"
  INSPECT_EXIT=$?
  set -e

  echo "${INSPECT_OUTPUT}"

  if [[ ${INSPECT_EXIT} -ne 0 ]]; then
    echo "FAIL ${scene}: generated bundle cannot be inspected"
    SUMMARY_FAIL=$((SUMMARY_FAIL + 1))
    continue
  fi

  if grep -q '^WARN ' <<<"${INSPECT_OUTPUT}"; then
    SUMMARY_WARN=$((SUMMARY_WARN + 1))
  elif grep -q '^PASS ' <<<"${INSPECT_OUTPUT}"; then
    SUMMARY_PASS=$((SUMMARY_PASS + 1))
  else
    SUMMARY_SKIP=$((SUMMARY_SKIP + 1))
  fi

  dry_status="$(python3 - <<'PY' "${bundle_path}/validation_summary.md"
import re
import sys
from pathlib import Path
text = Path(sys.argv[1]).read_text(encoding='utf-8')
match = re.search(r"Dry-run status: \*\*(\w+)\*\*", text)
print(match.group(1) if match else "UNKNOWN")
PY
)"

  if [[ "${dry_status}" == "PASS" ]]; then
    if [[ ! -f "${bundle_path}/execution_plan.md" || ! -f "${bundle_path}/execution_plan.json" ]]; then
      echo "FAIL ${scene}: dry-run PASS but execution_plan files missing"
      SUMMARY_FAIL=$((SUMMARY_FAIL + 1))
    fi
  fi

done

echo "Summary: PASS=${SUMMARY_PASS} WARN=${SUMMARY_WARN} FAIL=${SUMMARY_FAIL} SKIP=${SUMMARY_SKIP}"

if [[ ${SUMMARY_FAIL} -ne 0 ]]; then
  exit 1
fi

exit 0
