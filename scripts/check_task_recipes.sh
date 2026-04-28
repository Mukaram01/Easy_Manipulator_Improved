#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
VALIDATOR="${SCRIPT_DIR}/validate_task_recipe.py"

if [[ ! -x "${VALIDATOR}" ]]; then
  echo "ERROR: Missing validator at ${VALIDATOR}" >&2
  exit 2
fi

pass_count=0
warn_count=0
fail_count=0

run_expect_pass() {
  local path="$1"
  local output
  set +e
  output=$(python3 "${VALIDATOR}" "$path" 2>&1)
  local rc=$?
  set -e
  if [[ $rc -eq 0 ]]; then
    if grep -q "RESULT: WARN\|\"result\": \"WARN\"" <<<"$output"; then
      warn_count=$((warn_count + 1))
      echo "WARN (allowed): $path"
    else
      pass_count=$((pass_count + 1))
      echo "PASS: $path"
    fi
  else
    fail_count=$((fail_count + 1))
    echo "FAIL: unexpected validation failure for $path"
    echo "$output"
  fi
}

run_expect_fail() {
  local path="$1"
  local output
  set +e
  output=$(python3 "${VALIDATOR}" "$path" 2>&1)
  local rc=$?
  set -e
  if [[ $rc -ne 0 ]]; then
    pass_count=$((pass_count + 1))
    echo "PASS (expected failure): $path"
  else
    fail_count=$((fail_count + 1))
    echo "FAIL: expected failure but got pass for $path"
    echo "$output"
  fi
}

FIXTURE_DIR="${REPO_ROOT}/tests/fixtures/task_recipes"
if [[ -d "${FIXTURE_DIR}" ]]; then
  while IFS= read -r file; do
    base="$(basename "$file")"
    case "$base" in
      fail_*) run_expect_fail "$file" ;;
      *) run_expect_pass "$file" ;;
    esac
  done < <(find "${FIXTURE_DIR}" -maxdepth 1 -type f \( -name '*.yaml' -o -name '*.yml' \) | sort)
else
  echo "SKIP: ${FIXTURE_DIR} not found"
fi

for optional_dir in "${REPO_ROOT}/examples" "${REPO_ROOT}/docs"; do
  if [[ -d "$optional_dir" ]]; then
    while IFS= read -r file; do
      run_expect_pass "$file"
    done < <(find "$optional_dir" -type f \( -name '*task_recipe*.yaml' -o -name '*task_recipe*.yml' \) | sort)
  fi
done

echo "Task recipe checks: PASS=${pass_count} WARN=${warn_count} FAIL=${fail_count}"
[[ ${fail_count} -eq 0 ]]
