#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WIZARD="${SCRIPT_DIR}/create_cell_definition_wizard.py"
VALIDATOR="${SCRIPT_DIR}/validate_cell_definition.py"
WORKCELL_GEN="${SCRIPT_DIR}/generate_workcell_from_cell_definition.py"

status="PASS"
warn_count=0
fail_count=0

tmp_root="$(mktemp -d /tmp/emd_cell_wizard.XXXXXX)"
trap 'rm -rf "${tmp_root}"' EXIT

templates=(pick_place sort_by_colour sort_by_shape garbage_sorting)

for template in "${templates[@]}"; do
  cell_id="wizard_${template}"
  output="${tmp_root}/${cell_id}.yaml"
  echo "--- wizard ${template} ---"
  if ! python3 "${WIZARD}" \
    --template "${template}" \
    --cell-name "Wizard ${template}" \
    --cell-id "${cell_id}" \
    --robot ur5 \
    --end-effector robotiq_2f \
    --camera realsense_d435i \
    --output "${output}" \
    --force; then
    echo "FAIL: wizard generation failed for ${template}"
    status="FAIL"
    ((fail_count+=1))
    continue
  fi

  if ! python3 "${VALIDATOR}" "${output}"; then
    echo "FAIL: generated yaml validation failed for ${template}"
    status="FAIL"
    ((fail_count+=1))
  fi

done

if [[ "${status}" != "FAIL" ]]; then
  pkg_out="${tmp_root}/generated"
  if ! python3 "${WIZARD}" \
    --template sort_by_shape \
    --cell-name "Wizard shape workcell" \
    --cell-id wizard_shape_workcell \
    --robot ur5 \
    --end-effector robotiq_2f \
    --camera realsense_d435i \
    --output "${tmp_root}/wizard_shape_workcell.yaml" \
    --generate-workcell \
    --workcell-output-dir "${pkg_out}" \
    --package-name generated_wizard_shape_workcell \
    --force; then
    echo "WARN: wizard generate-workcell step failed"
    [[ "${status}" == "PASS" ]] && status="WARN"
    ((warn_count+=1))
  elif [[ ! -d "${pkg_out}/generated_wizard_shape_workcell" ]]; then
    echo "WARN: expected generated package folder missing"
    [[ "${status}" == "PASS" ]] && status="WARN"
    ((warn_count+=1))
  fi
fi

echo "Cell definition wizard checks: ${status}"
echo "Summary: WARN=${warn_count} FAIL=${fail_count}"

if [[ "${status}" == "FAIL" ]]; then
  exit 1
fi
exit 0
