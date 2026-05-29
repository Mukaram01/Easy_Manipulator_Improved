#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
VALIDATOR="${SCRIPT_DIR}/validate_scene_contract.py"
CATALOG="${REPO_ROOT}/scenes/supported_scenes.yaml"

if [[ ! -x "${VALIDATOR}" ]]; then
  echo "ERROR: Missing validator script at ${VALIDATOR}" >&2
  exit 2
fi

if [[ ! -f "${CATALOG}" ]]; then
  echo "ERROR: Missing supported-scene catalog at ${CATALOG}" >&2
  exit 2
fi

mapfile -t SCENES < <(PYTHONPATH="${SCRIPT_DIR}:${PYTHONPATH:-}" python3 - "${CATALOG}" <<'PY'
from pathlib import Path
import sys
from supported_scene_catalog import load_supported_scene_catalog

catalog_path = Path(sys.argv[1])
_, entries, errors = load_supported_scene_catalog(catalog_path)
if errors:
    for error in errors:
        print(f"catalog error: {error}", file=sys.stderr)
    raise SystemExit(2)
for entry in entries:
    if entry.enabled and entry.support_level != "experimental":
        print(entry.scene_name)
PY
)

failures=0
installed_count=0

for scene in "${SCENES[@]}"; do
  echo "=== ${scene} ==="
  set +e
  output="$("${VALIDATOR}" "${scene}" 2>&1)"
  status=$?
  set -e

  printf '%s\n' "${output}"

  case "${status}" in
    0)
      if printf "%s\n" "${output}" | grep -q "RESULT: WARN"; then
        echo "SUMMARY: WARN (${scene})"
      else
        echo "SUMMARY: PASS (${scene})"
      fi
      installed_count=$((installed_count + 1))
      ;;
    3)
      echo "SUMMARY: SKIP (${scene}) package not installed/discoverable"
      ;;
    *)
      echo "SUMMARY: FAIL (${scene})"
      failures=$((failures + 1))
      installed_count=$((installed_count + 1))
      ;;
  esac
  echo
 done

if (( failures > 0 )); then
  echo "Scene contract checks failed for ${failures} installed scene(s)."
  exit 1
fi

if (( installed_count == 0 )); then
  echo "No catalog scenes were installed. Nothing to validate."
else
  echo "All installed catalog scenes passed scene contract validation."
fi

exit 0
