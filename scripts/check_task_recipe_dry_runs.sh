#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DRY_RUNNER="${SCRIPT_DIR}/dry_run_task_recipe.py"

if [[ ! -x "${DRY_RUNNER}" ]]; then
  echo "ERROR: Missing task recipe dry-run tool at ${DRY_RUNNER}" >&2
  exit 2
fi

"${DRY_RUNNER}" --check "$@"
