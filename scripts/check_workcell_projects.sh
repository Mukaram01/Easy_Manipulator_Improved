#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
GENERATOR="${SCRIPT_DIR}/create_workcell_project.py"
VALIDATOR="${SCRIPT_DIR}/validate_scene_contract.py"
TMP_ROOT="${REPO_ROOT}/dist/.tmp_workcell_project_checks"
OUT_ROOT="${TMP_ROOT}/projects"

fixtures=(
  "pick_place:tests/fixtures/cell_definition_pick_place.yaml:ur5_2f_demo_cell"
  "sort_by_colour:tests/fixtures/cell_definition_sort_by_colour.yaml:ur5_colour_sort_cell"
  "sort_by_shape:tests/fixtures/cell_definition_sort_by_shape.yaml:ur5_shape_sort_cell"
  "garbage_sorting:tests/fixtures/cell_definition_garbage_sorting.yaml:ur5_garbage_sort_cell"
)

pass_count=0
warn_count=0
fail_count=0

rm -rf "${TMP_ROOT}"
mkdir -p "${OUT_ROOT}"

for entry in "${fixtures[@]}"; do
  name="${entry%%:*}"
  rest="${entry#*:}"
  fixture_rel="${rest%%:*}"
  cell_id="${entry##*:}"
  fixture="${REPO_ROOT}/${fixture_rel}"

  echo "--- project fixture: ${name} ---"
  set +e
  output="$(python3 "${GENERATOR}" --cell-definition "${fixture}" --output-dir "${OUT_ROOT}" --force 2>&1)"
  rc=$?
  set -e
  echo "${output}"
  if [[ ${rc} -ne 0 ]]; then
    echo "FAIL ${name}: generator command failed"
    ((fail_count+=1))
    continue
  fi

  project_dir="${OUT_ROOT}/${cell_id}"
  manifest="${project_dir}/project_manifest.json"
  cell_def="${project_dir}/cell_definition.yaml"

  if [[ ! -f "${manifest}" || ! -f "${cell_def}" ]]; then
    echo "FAIL ${name}: missing project_manifest.json or cell_definition.yaml"
    ((fail_count+=1))
    continue
  fi

  pkg_name="$(python3 - <<'PY' "${manifest}"
import json,sys
obj=json.load(open(sys.argv[1],encoding='utf-8'))
print(obj.get('generated_package_name',''))
PY
)"
  pkg_dir="${project_dir}/generated_workcell/${pkg_name}"
  scene_manifest="${pkg_dir}/scene_manifest.yaml"

  if [[ -z "${pkg_name}" || ! -d "${pkg_dir}" || ! -f "${scene_manifest}" ]]; then
    echo "FAIL ${name}: missing generated package folder or scene_manifest.yaml"
    ((fail_count+=1))
    continue
  fi

  set +e
  val_output="$(python3 "${VALIDATOR}" "${scene_manifest}" 2>&1)"
  val_rc=$?
  set -e
  if [[ ${val_rc} -ne 0 ]]; then
    echo "FAIL ${name}: direct manifest validation failed"
    echo "${val_output}"
    ((fail_count+=1))
    continue
  fi

  if [[ ! -f "${project_dir}/reports/validation_summary.md" || ! -f "${project_dir}/reports/task_recipe_dry_run.md" ]]; then
    echo "FAIL ${name}: missing reports"
    ((fail_count+=1))
    continue
  fi

  if [[ -d "${project_dir}/commissioning_bundle" && ! -f "${project_dir}/commissioning_bundle/bundle_manifest.json" ]]; then
    echo "WARN ${name}: commissioning bundle exists but bundle_manifest.json missing"
    ((warn_count+=1))
    continue
  fi

  echo "PASS ${name}: project generated and validated"
  ((pass_count+=1))
done

if [[ -x "${SCRIPT_DIR}/create_cell_definition_wizard.py" ]]; then
  echo "--- project template: wizard sort_by_colour ---"
  set +e
  tpl_output="$(python3 "${GENERATOR}" \
    --template sort_by_colour \
    --cell-name "Template Colour" \
    --cell-id template_colour \
    --robot ur5 \
    --end-effector robotiq_2f \
    --camera realsense_d435i \
    --output-dir "${OUT_ROOT}" \
    --force 2>&1)"
  tpl_rc=$?
  set -e
  echo "${tpl_output}"
  if [[ ${tpl_rc} -ne 0 ]]; then
    echo "WARN template sort_by_colour: generator returned non-zero"
    ((warn_count+=1))
  else
    echo "PASS template sort_by_colour"
    ((pass_count+=1))
  fi
fi

echo
if [[ ${fail_count} -gt 0 ]]; then
  echo "Workcell project checks: FAIL (pass=${pass_count} warn=${warn_count} fail=${fail_count})"
  exit 1
fi
if [[ ${warn_count} -gt 0 ]]; then
  echo "Workcell project checks: WARN (pass=${pass_count} warn=${warn_count} fail=0)"
  exit 0
fi

echo "Workcell project checks: PASS (pass=${pass_count} warn=0 fail=0)"
exit 0
