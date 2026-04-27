#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPORTER="${SCRIPT_DIR}/generate_scene_self_test_report.py"

if [[ ! -x "${REPORTER}" ]]; then
  echo "ERROR: Missing self-test reporter at ${REPORTER}" >&2
  exit 2
fi

"${REPORTER}" --check "$@"
