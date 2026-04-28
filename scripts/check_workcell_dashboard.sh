#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
GENERATOR="${SCRIPT_DIR}/create_workcell_project.py"
DASHBOARD_GEN="${SCRIPT_DIR}/generate_workcell_dashboard.py"
TMP_ROOT="${REPO_ROOT}/dist/.tmp_workcell_dashboard_checks"
OUT_ROOT="${TMP_ROOT}/projects"

fixtures=(
  "pick_place:tests/fixtures/cell_definition_pick_place.yaml:ur5_2f_demo_cell"
  "sort_by_colour:tests/fixtures/cell_definition_sort_by_colour.yaml:ur5_colour_sort_cell"
)

pass_count=0
warn_count=0
fail_count=0

rm -rf "${TMP_ROOT}"
mkdir -p "${OUT_ROOT}"

require_section() {
  local file="$1"
  local needle="$2"
  if ! grep -q "${needle}" "${file}"; then
    return 1
  fi
  return 0
}

for entry in "${fixtures[@]}"; do
  name="${entry%%:*}"
  rest="${entry#*:}"
  fixture_rel="${rest%%:*}"
  cell_id="${entry##*:}"
  fixture="${REPO_ROOT}/${fixture_rel}"

  echo "--- dashboard fixture: ${name} ---"
  set +e
  gen_out="$(python3 "${GENERATOR}" --cell-definition "${fixture}" --output-dir "${OUT_ROOT}" --force 2>&1)"
  gen_rc=$?
  set -e
  echo "${gen_out}"
  if [[ ${gen_rc} -ne 0 ]]; then
    echo "FAIL ${name}: project generation failed"
    ((fail_count+=1))
    continue
  fi

  project_dir="${OUT_ROOT}/${cell_id}"
  manifest="${project_dir}/project_manifest.json"
  dashboard="${project_dir}/dashboard/index.html"

  set +e
  dash_out="$(python3 "${DASHBOARD_GEN}" --project-dir "${project_dir}" --manifest "${manifest}" --quiet 2>&1)"
  dash_rc=$?
  set -e
  if [[ ${dash_rc} -ne 0 ]]; then
    echo "WARN ${name}: dashboard script returned non-zero"
    echo "${dash_out}"
    ((warn_count+=1))
  fi

  if [[ ! -f "${dashboard}" ]]; then
    echo "FAIL ${name}: missing dashboard/index.html"
    ((fail_count+=1))
    continue
  fi

  sections=(
    "Workcell Project Dashboard"
    "Project Summary"
    "Cell Overview"
    "Task Overview"
    "Generated Artifacts"
    "Operator Next Steps"
  )

  missing=0
  for section in "${sections[@]}"; do
    if ! require_section "${dashboard}" "${section}"; then
      echo "FAIL ${name}: missing section '${section}'"
      missing=1
    fi
  done

  if [[ ${missing} -eq 1 ]]; then
    ((fail_count+=1))
    continue
  fi

  echo "PASS ${name}: dashboard generated with required sections"
  ((pass_count+=1))
done

echo
if [[ ${fail_count} -gt 0 ]]; then
  echo "Workcell dashboard checks: FAIL (pass=${pass_count} warn=${warn_count} fail=${fail_count})"
  exit 1
fi
if [[ ${warn_count} -gt 0 ]]; then
  echo "Workcell dashboard checks: WARN (pass=${pass_count} warn=${warn_count} fail=0)"
  exit 0
fi

echo "Workcell dashboard checks: PASS (pass=${pass_count} warn=0 fail=0)"
exit 0
