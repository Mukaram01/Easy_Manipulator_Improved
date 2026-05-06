#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
VALIDATOR="${SCRIPT_DIR}/validate_capability_contracts.py"
FIXTURE_DIR="${REPO_ROOT}/tests/fixtures/capabilities"
CATALOG_DIR="${REPO_ROOT}/catalog/capabilities"

if [[ ! -x "${VALIDATOR}" ]]; then
  echo "Capability contract checks: FAIL"
  echo "Missing validator: ${VALIDATOR}" >&2
  exit 2
fi

for DIR in "${FIXTURE_DIR}" "${CATALOG_DIR}"; do
  if [[ ! -d "${DIR}" ]]; then
    echo "Capability contract checks: FAIL"
    echo "Missing capability directory: ${DIR}" >&2
    exit 2
  fi

  echo "Validating ${DIR}"
  set +e
  OUTPUT="$(${VALIDATOR} "${DIR}" 2>&1)"
  STATUS=$?
  set -e

  printf '%s\n' "${OUTPUT}"

  if [[ ${STATUS} -ne 0 ]]; then
    echo "Capability contract checks: FAIL"
    exit ${STATUS}
  fi

  if grep -q "\[WARN\]" <<<"${OUTPUT}"; then
    FINAL_STATUS="WARN"
  fi
done

if [[ "${FINAL_STATUS:-PASS}" == "WARN" ]]; then
  echo "Capability contract checks: WARN"
else
  echo "Capability contract checks: PASS"
fi
