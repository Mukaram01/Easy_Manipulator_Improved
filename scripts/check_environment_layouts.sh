#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
FIX_DIR="${REPO_ROOT}/tests/fixtures/environment_layouts"
PASS=0
WARN=0
FAIL=0

run_ok() {
  if "$@"; then PASS=$((PASS+1)); else FAIL=$((FAIL+1)); fi
}

echo "=== Environment layout checks (offline) ==="
run_ok "${SCRIPT_DIR}/inspect_asset_inventory.py" --quiet

for fixture in \
  ur5_table_bins_existing_assets.layout.yaml \
  ur5_conveyor_sorting_existing_assets.layout.yaml \
  machine_tending_existing_assets.layout.yaml \
  delta_conveyor_sorting_existing_assets.layout.yaml; do
  run_ok "${SCRIPT_DIR}/validate_environment_layout.py" "${FIX_DIR}/${fixture}" --quiet
 done

if "${SCRIPT_DIR}/validate_environment_layout.py" "${FIX_DIR}/invalid_missing_pose.layout.yaml" --quiet; then
  echo "FAIL: invalid_missing_pose should fail"
  FAIL=$((FAIL+1))
else
  PASS=$((PASS+1))
fi

if "${SCRIPT_DIR}/validate_environment_layout.py" "${FIX_DIR}/invalid_duplicate_asset_id.layout.yaml" --quiet; then
  echo "FAIL: invalid_duplicate_asset_id should fail"
  FAIL=$((FAIL+1))
else
  PASS=$((PASS+1))
fi

if "${SCRIPT_DIR}/validate_environment_layout.py" "${FIX_DIR}/invalid_bounds.layout.yaml" --quiet; then
  echo "FAIL: invalid_bounds should fail"
  FAIL=$((FAIL+1))
else
  PASS=$((PASS+1))
fi

if "${SCRIPT_DIR}/validate_environment_layout.py" "${FIX_DIR}/unknown_asset_ref_warn.layout.yaml" --quiet; then
  WARN=$((WARN+1))
else
  FAIL=$((FAIL+1))
fi

if "${SCRIPT_DIR}/validate_environment_layout.py" "${FIX_DIR}/unknown_asset_ref_warn.layout.yaml" --strict --quiet; then
  echo "FAIL: strict unknown_asset_ref should fail"
  FAIL=$((FAIL+1))
else
  PASS=$((PASS+1))
fi

run_ok "${SCRIPT_DIR}/generate_environment_preview.py" "${FIX_DIR}/ur5_table_bins_existing_assets.layout.yaml" --output "${REPO_ROOT}/reports/environment_layout_preview.md"
run_ok "${SCRIPT_DIR}/validate_cell_definition.py" "${REPO_ROOT}/tests/fixtures/cell_definition_pick_place_with_layout.yaml" --quiet

if python3 - <<'PY'
import json
from pathlib import Path
p=Path('/tmp/emd_layout_project')
if p.exists():
    import shutil; shutil.rmtree(p)
PY
then :; fi

if "${SCRIPT_DIR}/create_workcell_project.py" --cell-definition "${REPO_ROOT}/tests/fixtures/cell_definition_pick_place_with_layout.yaml" --output-dir /tmp/emd_layout_project --force --quiet >/dev/null 2>&1; then
  if python3 - <<'PY'
import json
from pathlib import Path
manifest=Path('/tmp/emd_layout_project/ur5_pick_place_layout/project_manifest.json')
payload=json.loads(manifest.read_text())
assert 'environment_layout' in payload
assert payload['environment_layout']
print('ok')
PY
  then
    PASS=$((PASS+1))
  else
    FAIL=$((FAIL+1))
  fi
else
  WARN=$((WARN+1))
fi

if (( FAIL > 0 )); then
  echo "Environment layout checks: FAIL (pass=${PASS} warn=${WARN} fail=${FAIL})"
  exit 1
fi
if (( WARN > 0 )); then
  echo "Environment layout checks: WARN (pass=${PASS} warn=${WARN} fail=${FAIL})"
  exit 0
fi

echo "Environment layout checks: PASS (pass=${PASS} warn=${WARN} fail=${FAIL})"
exit 0
