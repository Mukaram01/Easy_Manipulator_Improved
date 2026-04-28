#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
VALIDATOR="${SCRIPT_DIR}/validate_capability_contracts.py"
FIXTURE_DIR="${REPO_ROOT}/tests/fixtures/capabilities"

if [[ ! -x "${VALIDATOR}" ]]; then
  echo "Capability contract checks: FAIL"
  echo "Missing validator: ${VALIDATOR}" >&2
  exit 2
fi

if [[ ! -d "${FIXTURE_DIR}" ]]; then
  echo "Capability contract checks: FAIL"
  echo "Missing fixture directory: ${FIXTURE_DIR}" >&2
  exit 2
fi

set +e
OUTPUT="$(${VALIDATOR} "${FIXTURE_DIR}" 2>&1)"
STATUS=$?
set -e

printf '%s\n' "${OUTPUT}"

if [[ ${STATUS} -ne 0 ]]; then
  echo "Capability contract checks: FAIL"
  exit ${STATUS}
fi

if grep -q "\[WARN\]" <<<"${OUTPUT}"; then
  echo "Capability contract checks: WARN"
else
  echo "Capability contract checks: PASS"
fi
