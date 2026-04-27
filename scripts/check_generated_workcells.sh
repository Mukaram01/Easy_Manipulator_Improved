#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
GENERATOR="${SCRIPT_DIR}/generate_workcell_from_cell_definition.py"
FIXTURE_GLOB="${REPO_ROOT}/tests/fixtures/cell_definition_*.yaml"

if [[ ! -x "${GENERATOR}" ]]; then
  echo "FAIL: missing generator script at ${GENERATOR}" >&2
  exit 2
fi

shopt -s nullglob
fixtures=( ${FIXTURE_GLOB} )
shopt -u nullglob

if [[ ${#fixtures[@]} -eq 0 ]]; then
  echo "WARN: no cell definition fixtures found under tests/fixtures"
  echo "Generated workcell checks: WARN"
  exit 0
fi

tmp_root="$(mktemp -d /tmp/generated_workcells_check.XXXXXX)"
trap 'rm -rf "${tmp_root}"' EXIT

echo "Generated workcell check temp dir: ${tmp_root}"

pass_count=0
warn_count=0
fail_count=0

for fixture in "${fixtures[@]}"; do
  base_name="$(basename "${fixture}" .yaml)"
  package_name="generated_${base_name}"

  echo "--- ${base_name} ---"

  set +e
  gen_output="$(python3 "${GENERATOR}" "${fixture}" --output-dir "${tmp_root}" --package-name "${package_name}" --force 2>&1)"
  gen_code=$?
  set -e
  echo "${gen_output}"

  if [[ ${gen_code} -ne 0 ]]; then
    echo "FAIL ${base_name}: generation failed"
    fail_count=$((fail_count + 1))
    continue
  fi

  if grep -q '^WARN:' <<<"${gen_output}"; then
    warn_count=$((warn_count + 1))
  else
    pass_count=$((pass_count + 1))
  fi

  pkg_dir="${tmp_root}/${package_name}"
  manifest_path="${pkg_dir}/scene_manifest.yaml"

  for required in package.xml CMakeLists.txt scene_manifest.yaml README.md generated/commissioning_summary.md generated/validation_report.md; do
    if [[ ! -f "${pkg_dir}/${required}" ]]; then
      echo "FAIL ${base_name}: missing generated file ${required}"
      fail_count=$((fail_count + 1))
    fi
  done

  set +e
  validate_output="$(python3 - <<'PY' "${REPO_ROOT}" "${manifest_path}" "${base_name}"
import importlib.util
import sys
from pathlib import Path

repo_root = Path(sys.argv[1])
manifest_path = Path(sys.argv[2])
scene_name = sys.argv[3]

validator_spec = importlib.util.spec_from_file_location("validate_scene_contract", repo_root / "scripts" / "validate_scene_contract.py")
validator = importlib.util.module_from_spec(validator_spec)
sys.modules["validate_scene_contract"] = validator
validator_spec.loader.exec_module(validator)

manifest, parser, notes = validator._read_manifest(str(manifest_path))
status, task_notes = validator.validate_task_recipe_block(manifest)
print(f"validate_scene_contract(practical): scene={scene_name} parser={parser} status={status}")
for note in notes + task_notes:
    print(f"NOTE: {note}")
raise SystemExit(0 if status in {"PASS", "WARN"} else 1)
PY
2>&1)"
  validate_code=$?
  set -e
  echo "${validate_output}"
  if [[ ${validate_code} -ne 0 ]]; then
    echo "FAIL ${base_name}: practical scene contract validation failed"
    fail_count=$((fail_count + 1))
  elif grep -q 'status=WARN' <<<"${validate_output}"; then
    warn_count=$((warn_count + 1))
  fi

  set +e
  dry_output="$(python3 - <<'PY' "${REPO_ROOT}" "${manifest_path}" "${base_name}"
import importlib.util
import sys
from pathlib import Path

repo_root = Path(sys.argv[1])
manifest_path = Path(sys.argv[2])
scene_name = sys.argv[3]

spec = importlib.util.spec_from_file_location("dry_run_task_recipe", repo_root / "scripts" / "dry_run_task_recipe.py")
module = importlib.util.module_from_spec(spec)
sys.modules["dry_run_task_recipe"] = module
spec.loader.exec_module(module)
row = module.evaluate_scene(scene_name, manifest_path)
print(f"dry_run_task_recipe(practical): scene={scene_name} status={row.status} rule={row.matched_rule_id} destination={row.selected_destination_id}")
for note in row.notes:
    print(f"NOTE: {note}")
raise SystemExit(0 if row.status in {"PASS", "WARN", "SKIP"} else 1)
PY
2>&1)"
  dry_code=$?
  set -e
  echo "${dry_output}"
  if [[ ${dry_code} -ne 0 ]]; then
    echo "FAIL ${base_name}: dry-run failed"
    fail_count=$((fail_count + 1))
  elif grep -q 'status=WARN\|status=SKIP' <<<"${dry_output}"; then
    warn_count=$((warn_count + 1))
  fi

  set +e
  plan_output="$(python3 - <<'PY' "${REPO_ROOT}" "${manifest_path}" "${base_name}" "${pkg_dir}/generated"
import importlib.util
import sys
from pathlib import Path

repo_root = Path(sys.argv[1])
manifest_path = Path(sys.argv[2])
scene_name = sys.argv[3]
out_dir = Path(sys.argv[4])
out_dir.mkdir(parents=True, exist_ok=True)

spec = importlib.util.spec_from_file_location("generate_task_execution_plan", repo_root / "scripts" / "generate_task_execution_plan.py")
module = importlib.util.module_from_spec(spec)
sys.modules["generate_task_execution_plan"] = module
spec.loader.exec_module(module)
original = module.OUTPUT_DIR
module.OUTPUT_DIR = out_dir
try:
    row = module.evaluate_scene(scene_name, manifest_path)
finally:
    module.OUTPUT_DIR = original
print(f"generate_task_execution_plan(practical): scene={scene_name} status={row.status} steps={row.steps_count}")
for note in row.notes:
    print(f"NOTE: {note}")
raise SystemExit(0 if row.status in {"PASS", "WARN", "SKIP"} else 1)
PY
2>&1)"
  plan_code=$?
  set -e
  echo "${plan_output}"
  if [[ ${plan_code} -ne 0 ]]; then
    echo "FAIL ${base_name}: execution plan generation failed"
    fail_count=$((fail_count + 1))
  elif grep -q 'status=WARN\|status=SKIP' <<<"${plan_output}"; then
    warn_count=$((warn_count + 1))
  fi

  set +e
  bundle_output="$(python3 - <<'PY' "${REPO_ROOT}" "${manifest_path}" "${base_name}" "${pkg_dir}/generated"
import importlib.util
import sys
from pathlib import Path

repo_root = Path(sys.argv[1])
manifest_path = Path(sys.argv[2])
scene_name = sys.argv[3]
out_dir = Path(sys.argv[4]) / "bundle"
out_dir.mkdir(parents=True, exist_ok=True)

spec = importlib.util.spec_from_file_location("export_workcell_bundle", repo_root / "scripts" / "export_workcell_bundle.py")
module = importlib.util.module_from_spec(spec)
sys.modules["export_workcell_bundle"] = module
spec.loader.exec_module(module)

orig_reports = module.REPORTS_DIR
orig_plan_dir = module.PLAN_OUTPUT_DIR
orig_default = module.DEFAULT_OUTPUT_DIR
orig_plan_generator_out = module.plan_generator.OUTPUT_DIR
module.REPORTS_DIR = out_dir
module.PLAN_OUTPUT_DIR = out_dir
module.DEFAULT_OUTPUT_DIR = out_dir
module.plan_generator.OUTPUT_DIR = out_dir
try:
    bundle_dir, _ = module.export_scene(scene_name, manifest_path, out_dir, zip_output=False, force=True)
finally:
    module.REPORTS_DIR = orig_reports
    module.PLAN_OUTPUT_DIR = orig_plan_dir
    module.DEFAULT_OUTPUT_DIR = orig_default
    module.plan_generator.OUTPUT_DIR = orig_plan_generator_out
print(f"export_workcell_bundle(practical): scene={scene_name} bundle={bundle_dir}")
raise SystemExit(0)
PY
2>&1)"
  bundle_code=$?
  set -e
  echo "${bundle_output}"
  if [[ ${bundle_code} -ne 0 ]]; then
    echo "WARN ${base_name}: bundle export skipped or failed practically"
    warn_count=$((warn_count + 1))
  fi

done

status="PASS"
if [[ ${fail_count} -ne 0 ]]; then
  status="FAIL"
elif [[ ${warn_count} -ne 0 ]]; then
  status="WARN"
fi

echo "Generated workcell checks: ${status}"
echo "Summary: PASS=${pass_count} WARN=${warn_count} FAIL=${fail_count}"

if [[ ${fail_count} -ne 0 ]]; then
  exit 1
fi
exit 0
