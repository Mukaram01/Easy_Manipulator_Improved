#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

if [[ -n ${WORKSPACE_ROOT:-} ]]; then
  WORKSPACE_ROOT="$(cd "$WORKSPACE_ROOT" && pwd)"
elif [[ -n ${WS:-} ]]; then
  WORKSPACE_ROOT="$(cd "$WS" && pwd)"
else
  repo_parent="$(dirname "$REPO_DIR")"
  if [[ $(basename "$repo_parent") == "src" ]]; then
    WORKSPACE_ROOT="$(cd "$repo_parent/.." && pwd)"
  else
    WORKSPACE_ROOT="$HOME/workcell_ws"
  fi
fi

WS="${WS:-$WORKSPACE_ROOT}"
SRC_DIR="$WORKSPACE_ROOT/src"
BACKUP_DIR="$WORKSPACE_ROOT/trajopt_DISABLED_BACKUP"
ORIG_DIR="$BACKUP_DIR/original"

if [ ! -d "$SRC_DIR" ]; then
  exit 0
fi

# Install core system dependencies (Boost graph/program_options/serialization and
# TinyXML2) up front so users who only run this script still avoid
# trajopt_common configure errors such as "Could NOT find Boost (missing:
# Boost_INCLUDE_DIR graph)" during subsequent colcon builds.
"${SCRIPT_DIR}/install_system_deps.sh"

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

# Ensure the backup tree is ignored by colcon so cached packages are not
# rediscovered during future builds. colcon stops traversing a directory tree
# as soon as it encounters a COLCON_IGNORE (and, for compatibility with older
# tools, AMENT_IGNORE) marker, which keeps the archived packages hidden while
# still retaining them for manual inspection if needed.
if [ -d "${BACKUP_DIR}" ]; then
  for marker in COLCON_IGNORE AMENT_IGNORE; do
    if [ ! -e "${BACKUP_DIR}/${marker}" ]; then
      touch "${BACKUP_DIR}/${marker}"
    fi
  done
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

# Ensure trajopt_common declares all dependencies it links against. Some
# upstream snapshots omit find_package() calls for tinyxml2, Boost graph, and
# the KDL state solver, which results in CMake configure errors such as missing
# tinyxml2::tinyxml2 or tesseract::tesseract_state_solver_kdl targets.
TRAJOPT_COMMON_CMAKE=$(find "${SRC_DIR}" -path "*/trajopt_common/CMakeLists.txt" -print -quit 2>/dev/null || true)
if [[ -n "${TRAJOPT_COMMON_CMAKE}" && -f "${TRAJOPT_COMMON_CMAKE}" ]]; then
  if [[ "${TRAJOPT_COMMON_CMAKE}" == "${REPO_DIR}"/* ]]; then
    echo "Skipping modification of tracked file ${TRAJOPT_COMMON_CMAKE}"
  else
    echo "Ensuring required find_package() entries exist in ${TRAJOPT_COMMON_CMAKE}"
    TRAJOPT_COMMON_CMAKE="${TRAJOPT_COMMON_CMAKE}" python3 - <<'PY'
import os
from pathlib import Path

cmake = Path(os.environ["TRAJOPT_COMMON_CMAKE"])
lines = cmake.read_text().splitlines()


def ensure_find_package(statement: str) -> None:
    if any(statement in line for line in lines):
        return
    try:
        insert_at = next(i for i, line in enumerate(lines) if line.strip().startswith("find_package"))
    except StopIteration:
        try:
            insert_at = next(i for i, line in enumerate(lines) if line.strip().startswith("project")) + 1
        except StopIteration:
            insert_at = 0
    lines.insert(insert_at, statement)


ensure_find_package("find_package(tinyxml2 CONFIG REQUIRED)")
ensure_find_package("find_package(Boost COMPONENTS graph REQUIRED)")
ensure_find_package("find_package(tesseract_state_solver COMPONENTS kdl REQUIRED)")

cmake.write_text("\n".join(lines) + "\n")
PY
  fi
fi

# Ensure tesseract_command_language can locate the helper macros exported by
# tesseract_common. Some environments fail to load the CONFIG_EXTRAS provided by
# tesseract_common automatically, which leaves tesseract_variables() undefined
# and causes CMake to abort early. Inject a small fallback before the first call
# to tesseract_variables() so the build can continue. Only apply this to copies
# of the overlay within the workspace, not the repository checkout itself.
TCL_CMAKE=""
while IFS= read -r candidate; do
  if [[ "${candidate}" == "${REPO_DIR}"/* ]]; then
    echo "Skipping modification of tracked file ${candidate}"
    continue
  fi

  TCL_CMAKE="${candidate}"
  break
done < <(find "${SRC_DIR}" -path "*/tesseract_command_language/CMakeLists.txt" -print 2>/dev/null || true)

if [[ -n "${TCL_CMAKE}" && -f "${TCL_CMAKE}" ]]; then
  echo "Adding tesseract_common fallback include to ${TCL_CMAKE}"
  TCL_CMAKE="${TCL_CMAKE}" python3 - <<'PY'
import os
from pathlib import Path

cmake = Path(os.environ["TCL_CMAKE"])
lines = cmake.read_text().splitlines()

begin_marker = "# BEGIN tesseract_common fallback"
end_marker = "# END tesseract_common fallback"
sentinel = "  message(FATAL_ERROR \"tesseract_variables() is unavailable; ensure tesseract_common is discoverable\")"


def find_index(predicate):
    for idx, line in enumerate(lines):
        if predicate(line):
            return idx
    return None


def remove_existing_block():
    """Strip any previously injected guard to avoid malformed nesting."""

    if begin_marker in lines:
        start = lines.index(begin_marker)
        try:
            end = lines.index(end_marker, start) + 1
        except ValueError:
            end = start
            while end < len(lines) and lines[end].strip():
                end += 1
            if end < len(lines):
                end += 1
        del lines[start:end]
        return

    sentinel_idx = find_index(lambda l: "tesseract_variables() is unavailable" in l)
    if sentinel_idx is not None:
        start = sentinel_idx
        while start > 0 and lines[start - 1].strip():
            start -= 1
        end = sentinel_idx + 1
        while end < len(lines) and lines[end].strip():
            end += 1
        if end < len(lines):
            end += 1
        del lines[start:end]


guard = [
    begin_marker,
    "if(NOT DEFINED tesseract_common_DIR)",
    "  find_package(tesseract_common QUIET CONFIG)",
    "endif()",
    "if(NOT COMMAND tesseract_variables)",
    "  if(DEFINED tesseract_common_DIR AND EXISTS \"${tesseract_common_DIR}/tesseract_common_module_path.cmake\")",
    "    include(\"${tesseract_common_DIR}/tesseract_common_module_path.cmake\")",
    "  endif()",
    "endif()",
    "if(NOT COMMAND tesseract_variables)",
    "  find_path(_tesseract_common_src cmake/tesseract_macros.cmake",
    "    HINTS",
    "      \"${CMAKE_CURRENT_LIST_DIR}/../tesseract_common\"",
    "      \"${CMAKE_CURRENT_LIST_DIR}/../../tesseract_common\"",
    "      \"${CMAKE_CURRENT_LIST_DIR}/../../tesseract/tesseract_common\"",
    "      \"${CMAKE_CURRENT_LIST_DIR}/../../../tesseract/tesseract_common\"",
    "      \"${CMAKE_CURRENT_LIST_DIR}/../../easy_manipulation_deployment/tesseract/tesseract_common\"",
    "      \"${CMAKE_CURRENT_LIST_DIR}/../../../easy_manipulation_deployment/tesseract/tesseract_common\"",
    "    PATH_SUFFIXES share/tesseract_common",
    "  )",
    "  if(NOT _tesseract_common_src STREQUAL \"_tesseract_common_src-NOTFOUND\")",
    "    list(APPEND CMAKE_MODULE_PATH \"${_tesseract_common_src}/cmake\")",
    "    if(EXISTS \"${_tesseract_common_src}/cmake/tesseract_macros.cmake\")",
    "      include(\"${_tesseract_common_src}/cmake/tesseract_macros.cmake\")",
    "    endif()",
    "  endif()",
    "endif()",
    "if(NOT COMMAND tesseract_variables)",
    sentinel,
    "endif()",
    end_marker,
    "",
]


remove_existing_block()

insert_at = find_index(lambda l: "tesseract_variables()" in l)
if insert_at is not None:
    lines[insert_at:insert_at] = guard


cmake.write_text("\n".join(lines) + "\n")
PY
fi

# Link trajopt-related packages from the backup if available
link_from_backup trajopt_common
link_from_backup trajopt

# Generate a minimal CMake package configuration for
# tesseract_motion_planners_core and tesseract_motion_planners_simple when the
# upstream build does not install one. Downstream packages such as
# tesseract_task_composer rely on these configs to discover the imported targets
# even in merged colcon installs.
ensure_motion_planners_component_config() {
  local component="$1"
  local install_prefix="${WS}/install"
  local package_prefix="${install_prefix}/tesseract_motion_planners"
  local merge_layout=0

  if [[ -f "${install_prefix}/.colcon_install_layout" ]] &&
     grep -q "merged" "${install_prefix}/.colcon_install_layout"; then
    merge_layout=1
  elif [[ -d "${install_prefix}/lib/cmake/tesseract_motion_planners_${component}" ]]; then
    merge_layout=1
  fi

  if (( merge_layout )); then
    package_prefix="${install_prefix}"
  fi

  if [[ ! -d "${package_prefix}" ]]; then
    return
  fi

  local config_dir="${package_prefix}/lib/cmake/tesseract_motion_planners_${component}"
  local config_file="${config_dir}/tesseract_motion_planners_${component}Config.cmake"
  local targets_file="${config_dir}/tesseract_motion_planners_${component}Targets.cmake"

  local needs_regeneration=0
  if [[ ! -f "${config_file}" || ! -f "${targets_file}" ]]; then
    needs_regeneration=1
  elif ! grep -Eq "tesseract_motion_planners_${component}" "${config_file}" "${targets_file}"; then
    needs_regeneration=1
  fi

  if (( !needs_regeneration )); then
    return
  fi

  local lib=""
  local candidate
  for candidate in \
    "${package_prefix}/lib/libtesseract_motion_planners_${component}.so" \
    "${package_prefix}/lib/libtesseract_motion_planners_${component}.dylib" \
    "${package_prefix}/lib/tesseract_motion_planners_${component}.lib" \
    "${package_prefix}/bin/tesseract_motion_planners_${component}.dll"; do
    if [[ -f "${candidate}" ]]; then
      lib="${candidate}"
      break
    fi
  done

  if [[ -z "${lib}" ]]; then
    echo "tesseract_motion_planners_${component} library not found under ${package_prefix}; skipping config synthesis" >&2
    return
  fi

  local include_dir="${package_prefix}/include"
  if [[ ! -d "${include_dir}" ]]; then
    include_dir="${package_prefix}/include/tesseract_motion_planners"
  fi

  mkdir -p "${config_dir}"

  cat >"${targets_file}" <<EOF
if(NOT TARGET tesseract_motion_planners_${component})
  add_library(tesseract_motion_planners_${component} SHARED IMPORTED)
  set_target_properties(tesseract_motion_planners_${component} PROPERTIES
    IMPORTED_LOCATION "${lib}"
    INTERFACE_INCLUDE_DIRECTORIES "${include_dir}"
EOF
  if [[ "${component}" == "simple" ]]; then
    cat >>"${targets_file}" <<'EOF'
    INTERFACE_LINK_LIBRARIES "tesseract::tesseract_motion_planners_core"
EOF
  fi
  cat >>"${targets_file}" <<'EOF'
  )
endif()
if(NOT TARGET tesseract::tesseract_motion_planners_${component})
  add_library(tesseract::tesseract_motion_planners_${component} ALIAS tesseract_motion_planners_${component})
endif()
EOF

  cat >"${config_file}" <<EOF
# Generated by fix_workspace_layout.sh to provide a minimal CMake package for
# tesseract_motion_planners_${component} when the upstream build does not install one.
include(CMakeFindDependencyMacro)
EOF

  if [[ "${component}" == "core" ]]; then
    cat >>"${config_file}" <<'EOF'
find_dependency(Eigen3)
find_dependency(console_bridge)
find_dependency(tesseract_common)
find_dependency(tesseract_environment)
find_dependency(tesseract_command_language)
find_dependency(Boost)
EOF
  else
    cat >>"${config_file}" <<'EOF'
find_dependency(tesseract_motion_planners_core)
find_dependency(tesseract_environment)
find_dependency(tesseract_command_language)
find_dependency(cereal)
find_dependency(Boost)
EOF
  fi

  cat >>"${config_file}" <<EOF
include("\${CMAKE_CURRENT_LIST_DIR}/tesseract_motion_planners_${component}Targets.cmake")
if(NOT TARGET tesseract::tesseract_motion_planners_${component})
  add_library(tesseract::tesseract_motion_planners_${component} ALIAS tesseract_motion_planners_${component})
endif()
set(tesseract_motion_planners_${component}_FOUND TRUE)
EOF

  local package_xml="${package_prefix}/share/tesseract_motion_planners/package.xml"
  if [[ ! -f "${package_xml}" && -f "${REPO_DIR}/tesseract_planning/tesseract_motion_planners/package.xml" ]]; then
    package_xml="${REPO_DIR}/tesseract_planning/tesseract_motion_planners/package.xml"
  fi

  local version="0.0.0"
  if [[ -f "${package_xml}" ]]; then
    version="$(PACKAGE_XML="${package_xml}" python3 - <<'PY'
import os
import xml.etree.ElementTree as ET

path = os.environ["PACKAGE_XML"]
try:
    root = ET.parse(path).getroot()
except Exception:
    print("0.0.0")
else:
    text = root.findtext("version")
    print(text.strip() if text else "0.0.0")
PY
)"
    version="${version:-0.0.0}"
  fi

  local version_file="${config_dir}/tesseract_motion_planners_${component}ConfigVersion.cmake"
  local version_script
  version_script=$(mktemp)
  cat >"${version_script}" <<'EOF'
include(CMakePackageConfigHelpers)
write_basic_package_version_file("${OUTPUT_FILE}" VERSION "${PACKAGE_VERSION}" COMPATIBILITY SameMajorVersion)
EOF
  cmake -DPACKAGE_VERSION="${version}" -DOUTPUT_FILE="${version_file}" -P "${version_script}"
  rm -f "${version_script}"

  echo "Synthesized tesseract_motion_planners_${component} package configuration at ${config_dir}"
}

# Remove duplicate packages that collide with the overlays bundled inside the
# easy_manipulation_deployment repository. Keeping the patched overlays and
# discarding the external checkouts prevents rosdep from aborting with runtime
# errors about duplicate package names.
if command -v colcon >/dev/null 2>&1; then
  declare -A pkg_keep=()
  declare -A pkg_removed=()
  while read -r name path _; do
    [ -n "${name}" ] || continue
    if [[ -v pkg_keep[$name] ]]; then
      keep="${pkg_keep[$name]}"
      drop="${path}"
      if [[ "$path" == */easy_manipulation_deployment/* && "$keep" != */easy_manipulation_deployment/* ]]; then
        drop="${keep}"
        keep="${path}"
      fi
      if [ "${drop}" != "${keep}" ] && [ -e "${drop}" ]; then
        echo "Removing duplicate package '$name' from ${drop} (keeping ${keep})"
        rm -rf "${drop}"
        pkg_removed[$name]=1
      fi
      pkg_keep[$name]="${keep}"
    else
      pkg_keep[$name]="${path}"
    fi
  done < <(colcon list --base-paths "${SRC_DIR}")

  if [ ${#pkg_removed[@]} -gt 0 ]; then
    for pkg in "${!pkg_removed[@]}"; do
      for artifact in "${WS}/build/${pkg}" "${WS}/log/${pkg}" "${WS}/log/latest_build/${pkg}"; do
        if [ -e "${artifact}" ]; then
          echo "Removing stale build artifact ${artifact}"
          rm -rf "${artifact}"
        fi
      done
    done
  fi
fi

ensure_motion_planners_component_config core
ensure_motion_planners_component_config simple
