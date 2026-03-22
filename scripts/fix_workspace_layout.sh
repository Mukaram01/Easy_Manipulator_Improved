#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

source "${SCRIPT_DIR}/lib/motion_planner_configs.sh"

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
EXPOSED_PACKAGES=()

if [ ! -d "$SRC_DIR" ]; then
  exit 0
fi

path_exists() {
  local path="$1"
  [ -e "$path" ] || [ -L "$path" ]
}

path_type() {
  local path="$1"

  if [ -L "$path" ]; then
    echo "symlink"
  elif [ -d "$path" ]; then
    echo "directory"
  elif [ -f "$path" ]; then
    echo "file"
  elif [ -e "$path" ]; then
    echo "other"
  else
    echo "missing"
  fi
}

resolved_path_or_unresolved() {
  local path="$1"

  if path_exists "$path"; then
    readlink -f "$path" 2>/dev/null || echo "<unresolved>"
  else
    echo "<missing>"
  fi
}

path_details() {
  local path="$1"
  local listing resolved

  listing="$(ls -ld -- "$path" 2>/dev/null || echo "<ls unavailable>")"
  resolved="$(resolved_path_or_unresolved "$path")"
  printf '%s (type=%s, resolved=%s)' "$listing" "$(path_type "$path")" "$resolved"
}

report_invalid_workspace_entry() {
  local path="$1"
  local expected_target="$2"

  echo "Invalid workspace entry at ${path}: $(path_details "$path"). Expected a directory or symlink resolving to $(readlink -f "$expected_target")." >&2
}

symlink_repo_package() {
  local pkg_name="$1"
  local pkg_path="$2"
  local dest="$SRC_DIR/$pkg_name"
  local expected_resolved actual_resolved

  if [ ! -d "$pkg_path" ]; then
    return
  fi

  expected_resolved="$(readlink -f "$pkg_path")"

  if [ -L "$dest" ]; then
    actual_resolved="$(resolved_path_or_unresolved "$dest")"
    if [ "$actual_resolved" = "$expected_resolved" ]; then
      return
    fi

    report_invalid_workspace_entry "$dest" "$pkg_path"
    echo "Replacing incorrect symlink at ${dest} with the expected target." >&2
    rm -f "$dest"
  elif [ -d "$dest" ]; then
    actual_resolved="$(resolved_path_or_unresolved "$dest")"
    if [ "$actual_resolved" = "$expected_resolved" ]; then
      return
    fi

    report_invalid_workspace_entry "$dest" "$pkg_path"
    echo "Refusing to replace directory ${dest}. Remove or relocate it, then rerun scripts/fix_workspace_layout.sh." >&2
    return 1
  elif path_exists "$dest"; then
    report_invalid_workspace_entry "$dest" "$pkg_path"
    echo "Replacing invalid non-directory collision at ${dest} with the expected symlink." >&2
    rm -rf "$dest"
  fi

  echo "Linking repository package ${pkg_name} -> ${pkg_path}"
  ln -s "$pkg_path" "$dest"
  EXPOSED_PACKAGES+=("$pkg_name")
}

expose_repo_packages() {
  local pkg_xml pkg_dir pkg_name

  # The repository keeps many robot/environment asset packages under assets/
  # with top-level ignore markers to avoid duplicate discovery in fresh clones.
  # When the repo is used directly inside a workspace, rosdep/colcon still need
  # concrete package entries at src/<package_name>. Create symlinks for those
  # hidden packages so dependencies such as workbench_description,
  # ur5_moveit_config, and robotiq_85_moveit_config resolve without requiring
  # users to manually move directories around the workspace.
  while IFS= read -r pkg_xml; do
    pkg_dir="$(dirname "$pkg_xml")"
    pkg_name="$(
      PKG_XML="$pkg_xml" python3 - <<'PY'
import os
import xml.etree.ElementTree as ET

pkg_xml = os.environ["PKG_XML"]
print((ET.parse(pkg_xml).getroot().findtext("name") or "").strip())
PY
    )"
    [ -n "$pkg_name" ] || continue
    symlink_repo_package "$pkg_name" "$pkg_dir"
  done < <(find "$REPO_DIR/assets" -name package.xml -print 2>/dev/null | sort)

  # The workcell_builder ROS package already lives underneath this repository.
  # When the repository itself is checked out inside workspace/src, rosdep and
  # colcon discover that nested package recursively, so creating an additional
  # src/workcell_builder symlink introduces a duplicate package name
  # ("workcell_builder" vs
  # "easy_manipulation_deployment/workcell_builder/workcell_builder"). Only
  # export the convenience symlink when the repository is *not* already under
  # the active workspace src tree.
  local repo_exposed_in_src=0
  if [[ "${REPO_DIR}" == "${SRC_DIR}/"* ]]; then
    repo_exposed_in_src=1
  else
    local candidate
    while IFS= read -r candidate; do
      if [ "$(readlink -f "$candidate")" = "$(readlink -f "$REPO_DIR")" ]; then
        repo_exposed_in_src=1
        break
      fi
    done < <(find "$SRC_DIR" -mindepth 1 -maxdepth 1 \( -type d -o -type l \) -print 2>/dev/null)
  fi

  if [[ $repo_exposed_in_src -eq 1 ]]; then
    local legacy_link="$SRC_DIR/workcell_builder"
    if [ -L "$legacy_link" ] && [ "$(readlink -f "$legacy_link")" = "$(readlink -f "$REPO_DIR/workcell_builder/workcell_builder")" ]; then
      echo "Removing legacy workcell_builder symlink at ${legacy_link}"
      rm -f "$legacy_link"
    fi
  else
    symlink_repo_package "workcell_builder" "$REPO_DIR/workcell_builder/workcell_builder"
  fi
}

# Ensure external trajopt checkouts fetched via tesseract.repos are ignored
# before invoking colcon. Some upstream snapshots ship a COLCON_IGNORE.repo
# marker instead of the expected COLCON_IGNORE file, which causes colcon to
# discover duplicate packages alongside the patched overlays in
# easy_manipulation_deployment and abort with a circular dependency error.
ensure_colcon_ignore() {
  local dir="$1"
  if [ -d "$dir" ] && [ ! -L "$dir" ] && [ ! -e "$dir/COLCON_IGNORE" ]; then
    echo "Adding COLCON_IGNORE to external checkout at $dir"
    touch "$dir/COLCON_IGNORE"
  fi
}

for duplicate in \
  "$SRC_DIR/trajopt" \
  "$SRC_DIR/trajopt/trajopt" \
  "$SRC_DIR/trajopt/trajopt_common" \
  "$SRC_DIR/trajopt/trajopt_sco"; do
  ensure_colcon_ignore "$duplicate"
done

expose_repo_packages

summarize_exposed_repo_packages() {
  local total_assets=0
  local package_groups=(
    "ur5_moveit_config:UR5 MoveIt config"
    "robotiq_85_moveit_config:Robotiq 85 MoveIt config"
    "ur_description:Robot descriptions"
    "robotiq_85_description:Robot descriptions"
    "workbench_description:Robot descriptions"
  )
  local entry pkg_name label dest target

  while IFS= read -r _; do
    total_assets=$((total_assets + 1))
  done < <(find "$REPO_DIR/assets" -name package.xml -print 2>/dev/null)

  echo
  echo "Workspace layout summary"
  echo "========================"
  echo "Exposed ${#EXPOSED_PACKAGES[@]} package(s) into ${SRC_DIR} during this run; ${total_assets} asset package(s) are available from the repository."

  if [ ${#EXPOSED_PACKAGES[@]} -gt 0 ]; then
    printf '  New links created: %s\n' "$(printf '%s\n' "${EXPOSED_PACKAGES[@]}" | sort | paste -sd ', ' -)"
  else
    echo "  New links created: none (all expected workspace entries were already present)."
  fi

  echo "  Key package status:"
  for entry in "${package_groups[@]}"; do
    pkg_name="${entry%%:*}"
    label="${entry#*:}"
    dest="$SRC_DIR/$pkg_name"
    if path_exists "$dest"; then
      target="$(resolved_path_or_unresolved "$dest")"
      printf '    - %-24s [%s]: %s\n' "$pkg_name" "$label" "$target"
    else
      printf '    - %-24s [%s]: MISSING from src/ (rerun scripts/fix_workspace_layout.sh)\n' "$pkg_name" "$label"
    fi
  done
}

verify_exposed_repo_package() {
  local pkg_name="$1"
  local expected_target="$2"
  local dest="$SRC_DIR/$pkg_name"
  local expected_resolved actual_type actual_resolved

  expected_resolved="$(readlink -f "$expected_target")"
  actual_type="$(path_type "$dest")"
  actual_resolved="$(resolved_path_or_unresolved "$dest")"

  if [ "$actual_type" = "directory" ] || [ "$actual_type" = "symlink" ]; then
    if [ "$actual_resolved" = "$expected_resolved" ]; then
      echo "Verified workspace layout: src/${pkg_name} is present as a ${actual_type} resolving to ${actual_resolved}."
      return
    fi
  fi

  if path_exists "$dest"; then
    echo "Error: expose_repo_packages scanned ${REPO_DIR}/assets but ${dest} is ${actual_type} ($(path_details "$dest")); expected a directory or symlink resolving to ${expected_resolved}. Fix or remove the existing entry before rerunning rosdep install." >&2
  else
    echo "Error: expose_repo_packages scanned ${REPO_DIR}/assets but did not expose the expected workspace package at ${dest}. Existing path type: ${actual_type}; resolved target: ${actual_resolved}; expected target: ${expected_resolved}. rosdep install should not continue until the package is exposed." >&2
  fi
  exit 1
}

while IFS= read -r pkg_xml; do
  pkg_dir="$(dirname "$pkg_xml")"
  pkg_name="$(
    PKG_XML="$pkg_xml" python3 - <<'PY'
import os
import xml.etree.ElementTree as ET

pkg_xml = os.environ["PKG_XML"]
print((ET.parse(pkg_xml).getroot().findtext("name") or "").strip())
PY
  )"
  [ -n "$pkg_name" ] || continue
  verify_exposed_repo_package "$pkg_name" "$pkg_dir"
done < <(find "$REPO_DIR/assets" -name package.xml -print 2>/dev/null | sort)

summarize_exposed_repo_packages

# Install core system dependencies (Boost graph/program_options/serialization and
# TinyXML2) up front so users who only run this script still avoid
# trajopt_common configure errors such as "Could NOT find Boost (missing:
# Boost_INCLUDE_DIR graph)" during subsequent colcon builds.
"${SCRIPT_DIR}/ensure_rosdep_overrides.sh" taskflow
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

  # Skip linking if the repository already provides this package.
  if find "${REPO_DIR}" -path "*/${pkg}/package.xml" -print -quit >/dev/null 2>&1; then
    return
  fi

  for candidate in "${BACKUP_DIR}/${pkg}" "${ORIG_DIR}/${pkg}"; do
    if [ -d "${candidate}" ]; then
      target="${candidate}"
      break
    fi
  done

  if [ -n "${target}" ] && ! path_exists "${SRC_DIR}/${pkg}"; then
    ln -s "${target}" "${SRC_DIR}/${pkg}"
  fi
}

ensure_symlink_target() {
  local dest="$1"
  local target="$2"

  if [ -L "$dest" ] && [ "$(readlink -f "$dest")" = "$(readlink -f "$target")" ]; then
    return
  fi

  if path_exists "$dest"; then
    echo "Updating existing entry at $dest"
    rm -rf "$dest"
  fi

  ln -s "$target" "$dest"
}

# Ensure symlink for trajopt_sco pointing to nested package
ensure_symlink_target \
  "${SRC_DIR}/trajopt_sco" \
  "${SRC_DIR}/easy_manipulation_deployment/trajopt/trajopt_sco"

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

ensure_motion_planners_component_configs "$WORKSPACE_ROOT"
