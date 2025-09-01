#!/usr/bin/env bash
set -euo pipefail

SRC_DIR="$HOME/workcell_ws/src"
BACKUP_DIR="$HOME/trajopt_DISABLED_BACKUP"

if [ ! -d "$SRC_DIR" ]; then
  exit 0
fi

# Move old trajopt directory to backup if it exists
if [ -d "${SRC_DIR}/trajopt" ] && [ ! -L "${SRC_DIR}/trajopt" ]; then
  mkdir -p "${BACKUP_DIR}"
  mv "${SRC_DIR}/trajopt"/* "${BACKUP_DIR}"/ 2>/dev/null || true
  rmdir "${SRC_DIR}/trajopt" 2>/dev/null || true
fi

# Ensure symlink for trajopt_sco pointing to nested package
if [ ! -e "${SRC_DIR}/trajopt_sco" ]; then
  ln -s "${SRC_DIR}/easy_manipulation_deployment/trajopt/trajopt_sco" "${SRC_DIR}/trajopt_sco"
fi

# Link trajopt_common from backup if available
if [ -d "${BACKUP_DIR}/trajopt_common" ] && [ ! -e "${SRC_DIR}/trajopt_common" ]; then
  ln -s "${BACKUP_DIR}/trajopt_common" "${SRC_DIR}/trajopt_common"
fi

# Link trajopt package from backup if available
if [ -d "${BACKUP_DIR}/trajopt" ] && [ ! -e "${SRC_DIR}/trajopt" ]; then
  ln -s "${BACKUP_DIR}/trajopt" "${SRC_DIR}/trajopt"
fi
