#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
VALIDATOR="${SCRIPT_DIR}/validate_cell_definition.py"
GENERATOR="${SCRIPT_DIR}/generate_scene_from_cell_definition.py"

status="PASS"
warn_count=0
fail_count=0

if [[ ! -x "${VALIDATOR}" || ! -x "${GENERATOR}" ]]; then
  echo "Cell definition checks: FAIL"
  echo "FAIL: Required tools are missing or not executable." >&2
  exit 2
fi

mapfile -t fixtures < <(find "${REPO_ROOT}/tests/fixtures" -maxdepth 1 -type f -name 'cell_definition_*.yaml' | sort)
if [[ ${#fixtures[@]} -eq 0 ]]; then
  echo "Cell definition checks: WARN"
  echo "WARN: No cell definition fixtures found under tests/fixtures." >&2
  exit 0
fi

tmp_root="$(mktemp -d /tmp/emd_cell_defs.XXXXXX)"
trap 'rm -rf "${tmp_root}"' EXIT

echo "Running cell definition checks in ${tmp_root}"
for fixture in "${fixtures[@]}"; do
  name="$(basename "${fixture}" .yaml)"
  output_dir="${tmp_root}/${name}"

  echo "--- ${name} ---"
  set +e
  validator_output="$(python3 "${VALIDATOR}" "${fixture}" 2>&1)"
  validator_code=$?
  set -e
  echo "${validator_output}"
  if [[ ${validator_code} -ne 0 ]]; then
    status="FAIL"
    ((fail_count+=1))
    continue
  fi
  if grep -q "RESULT: WARN" <<<"${validator_output}"; then
    [[ "${status}" == "PASS" ]] && status="WARN"
    ((warn_count+=1))
  fi

  python3 "${GENERATOR}" "${fixture}" --output-dir "${output_dir}"

  for generated in scene_manifest.preview.yaml task_recipe.preview.yaml commissioning_summary.md; do
    if [[ ! -f "${output_dir}/${generated}" ]]; then
      echo "FAIL: Missing generated artifact ${output_dir}/${generated}"
      status="FAIL"
      ((fail_count+=1))
    fi
  done

  set +e
  scene_check_output="$(python3 -c "import importlib.util,sys; from pathlib import Path; repo=Path(sys.argv[1]); manifest_path=Path(sys.argv[2]); spec=importlib.util.spec_from_file_location('validate_scene_contract', repo / 'scripts' / 'validate_scene_contract.py'); module=importlib.util.module_from_spec(spec); sys.modules['validate_scene_contract']=module; spec.loader.exec_module(module); manifest, parser, notes = module._read_manifest(str(manifest_path)); status, rule_notes = module.validate_task_recipe_block(manifest); print(f'Scene preview check parser={parser} status={status}'); [print(f'NOTE: {n}') for n in (notes + rule_notes)]; raise SystemExit(0 if status in {'PASS','WARN'} else 1)" "${REPO_ROOT}" "${output_dir}/scene_manifest.preview.yaml" 2>&1)"
  scene_check_code=$?
  set -e
  echo "${scene_check_output}"
  if [[ ${scene_check_code} -ne 0 ]]; then
    status="FAIL"
    ((fail_count+=1))
  elif grep -q "status=WARN" <<<"${scene_check_output}"; then
    [[ "${status}" == "PASS" ]] && status="WARN"
    ((warn_count+=1))
  fi
done

echo "Cell definition checks: ${status}"
echo "Summary: WARN=${warn_count} FAIL=${fail_count}"

if [[ "${status}" == "FAIL" ]]; then
  exit 1
fi
exit 0
