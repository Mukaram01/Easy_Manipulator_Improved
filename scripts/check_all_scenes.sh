#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VALIDATOR="${SCRIPT_DIR}/validate_scene_contract.py"

SCENES=(
  ur5_2f_test
  ur5_3f_test
  ur5_airpick4_test
  suction_test
)

if [[ ! -x "${VALIDATOR}" ]]; then
  echo "ERROR: Missing validator script at ${VALIDATOR}" >&2
  exit 2
fi

failures=0
installed_count=0

for scene in "${SCENES[@]}"; do
  echo "=== ${scene} ==="
  set +e
  output="$(${VALIDATOR} "${scene}" 2>&1)"
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
  echo "No listed scenes were installed. Nothing to validate."
else
  echo "All installed scenes passed scene contract validation."
fi

exit 0
