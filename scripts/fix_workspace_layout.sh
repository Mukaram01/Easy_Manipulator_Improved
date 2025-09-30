#!/usr/bin/env bash
set -euo pipefail

WS=${WS:-$HOME/workcell_ws}
SRC_DIR="$WS/src"
BACKUP_DIR="$WS/trajopt_DISABLED_BACKUP"
ORIG_DIR="$BACKUP_DIR/original"

if [ ! -d "$SRC_DIR" ]; then
  exit 0
fi

# Move old trajopt directory to backup if it exists. The previous implementation
# only moved non-hidden files which left behind the .git directory and prevented
# the symlinks below from being created on subsequent runs. This ensured the
# trajopt CMake package was never visible to colcon, which later caused build
# failures when tesseract_motion_planners attempted to find the trajopt package.
if [ -d "${SRC_DIR}/trajopt" ] && [ ! -L "${SRC_DIR}/trajopt" ]; then
  mkdir -p "${BACKUP_DIR}"
  if [ ! -d "${ORIG_DIR}" ]; then
    mv "${SRC_DIR}/trajopt" "${ORIG_DIR}"
  else
    rm -rf "${SRC_DIR}/trajopt"
  fi
fi

link_from_backup() {
  local pkg="$1"
  local target=""
  for candidate in "${BACKUP_DIR}/${pkg}" "${ORIG_DIR}/${pkg}"; do
    if [ -d "${candidate}" ]; then
      target="${candidate}"
      break
    fi
  done

  if [ -n "${target}" ] && [ ! -e "${SRC_DIR}/${pkg}" ]; then
    ln -s "${target}" "${SRC_DIR}/${pkg}"
  fi
}

# Ensure symlink for trajopt_sco pointing to nested package
if [ ! -e "${SRC_DIR}/trajopt_sco" ]; then
  ln -s "${SRC_DIR}/easy_manipulation_deployment/trajopt/trajopt_sco" "${SRC_DIR}/trajopt_sco"
fi

# Link trajopt-related packages from the backup if available
link_from_backup trajopt_common
link_from_backup trajopt
