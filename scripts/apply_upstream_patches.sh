#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
PATCH_FILE="${REPO_ROOT}/patches/tesseract_qt_qtads_target_fix.patch"

find_workspace_root() {
  local dir

  if [[ -d "${PWD}/src/tesseract_qt" ]]; then
    echo "${PWD}"
    return 0
  fi

  dir="${PWD}"
  while [[ "${dir}" != "/" ]]; do
    if [[ -d "${dir}/src/tesseract_qt" ]]; then
      echo "${dir}"
      return 0
    fi
    dir="$(dirname "${dir}")"
  done

  if [[ -d "${REPO_ROOT}/../tesseract_qt" ]]; then
    echo "$(cd "${REPO_ROOT}/../.." && pwd)"
    return 0
  fi

  return 1
}

if [[ ! -f "${PATCH_FILE}" ]]; then
  echo "ERROR: Patch file not found: ${PATCH_FILE}" >&2
  exit 1
fi

WORKSPACE_ROOT="$(find_workspace_root || true)"
if [[ -z "${WORKSPACE_ROOT}" ]]; then
  echo "ERROR: Could not find workspace root containing src/tesseract_qt." >&2
  echo "Run this script from your workspace root (for example: ~/emd_epd_ws)." >&2
  exit 1
fi

TESSERACT_QT_DIR="${WORKSPACE_ROOT}/src/tesseract_qt"
if [[ ! -d "${TESSERACT_QT_DIR}" ]]; then
  echo "ERROR: Missing expected directory: ${TESSERACT_QT_DIR}" >&2
  exit 1
fi

echo "Workspace root: ${WORKSPACE_ROOT}"
echo "Applying patch: ${PATCH_FILE}"

if git -C "${TESSERACT_QT_DIR}" apply --check "${PATCH_FILE}" >/dev/null 2>&1; then
  git -C "${TESSERACT_QT_DIR}" apply "${PATCH_FILE}"
  echo "SUCCESS: Applied tesseract_qt Qt ADS target fix patch."
elif git -C "${TESSERACT_QT_DIR}" apply --reverse --check "${PATCH_FILE}" >/dev/null 2>&1; then
  echo "INFO: Patch already applied; nothing to do."
else
  echo "ERROR: Patch could not be applied cleanly." >&2
  echo "Inspect local changes in ${TESSERACT_QT_DIR} and apply manually if required." >&2
  exit 1
fi
