#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
CAP_DIR="${REPO_ROOT}/tests/fixtures/capabilities"
FIX_DIR="${REPO_ROOT}/tests/fixtures"
GEN_SCRIPT="${REPO_ROOT}/scripts/generate_scene_from_cell_definition.py"
VAL_SCRIPT="${REPO_ROOT}/scripts/validate_cell_definition.py"

pass=0
warn=0
fail=0

echo "== Cell capability integration checks =="

if "${REPO_ROOT}/scripts/check_capability_contracts.sh"; then
  ((pass+=1))
else
  echo "FAIL: capability contracts invalid"
  ((fail+=1))
fi

valid_fixtures=(
  "cell_definition_pick_place_with_capabilities.yaml"
  "cell_definition_colour_sorting_with_capabilities.yaml"
  "cell_definition_delta_conveyor_sorting_with_capabilities.yaml"
)

for name in "${valid_fixtures[@]}"; do
  if python3 "${VAL_SCRIPT}" "${FIX_DIR}/${name}" --capabilities-dir "${CAP_DIR}" >/tmp/cap_val.out 2>&1; then
    ((pass+=1))
  else
    echo "FAIL: ${name} failed validation"
    cat /tmp/cap_val.out
    ((fail+=1))
  fi
  if grep -q "RESULT: WARN" /tmp/cap_val.out; then
    ((warn+=1))
  fi

done

if python3 "${VAL_SCRIPT}" "${FIX_DIR}/cell_definition_unknown_capability_warn.yaml" --capabilities-dir "${CAP_DIR}" >/tmp/cap_warn.out 2>&1; then
  if grep -q "RESULT: WARN" /tmp/cap_warn.out; then
    ((pass+=1))
    ((warn+=1))
  else
    echo "FAIL: unknown capability fixture did not emit WARN"
    ((fail+=1))
  fi
else
  echo "FAIL: unknown capability fixture unexpectedly failed"
  ((fail+=1))
fi

if python3 "${VAL_SCRIPT}" "${FIX_DIR}/cell_definition_unknown_capability_warn.yaml" --strict --capabilities-dir "${CAP_DIR}" >/tmp/cap_strict.out 2>&1; then
  echo "FAIL: strict mode should fail unknown capability fixture"
  ((fail+=1))
else
  ((pass+=1))
fi

if python3 "${VAL_SCRIPT}" "${FIX_DIR}/cell_definition_incompatible_capability_fail.yaml" --capabilities-dir "${CAP_DIR}" >/tmp/cap_mismatch.out 2>&1; then
  echo "FAIL: incompatible capability fixture should fail"
  ((fail+=1))
else
  ((pass+=1))
fi

for name in "${valid_fixtures[@]}"; do
  outdir="$(mktemp -d)"
  python3 "${GEN_SCRIPT}" "${FIX_DIR}/${name}" --output-dir "${outdir}" >/tmp/cap_gen.out 2>&1 || {
    echo "FAIL: generation failed for ${name}"
    cat /tmp/cap_gen.out
    ((fail+=1))
    continue
  }
  if grep -q "capabilities:" "${outdir}/scene_manifest.preview.yaml"; then
    ((pass+=1))
  else
    echo "FAIL: generated manifest missing capabilities block for ${name}"
    ((fail+=1))
  fi
  rm -rf "${outdir}"
done

status="PASS"
if (( fail > 0 )); then
  status="FAIL"
elif (( warn > 0 )); then
  status="WARN"
fi

echo "Cell capability integration checks: ${status}"
echo "SUMMARY: PASS=${pass} WARN=${warn} FAIL=${fail}"
[[ "${status}" != "FAIL" ]]
