#!/usr/bin/env bash
set -euo pipefail

WS=${WS:-$HOME/workcell_ws}
SRC="$WS/src"
REPO_DIR="$(cd "$(dirname "$0")" && pwd)"

mkdir -p "$SRC"

# Ensure we are being run from the repository inside the workspace
if [[ "$REPO_DIR" != "$SRC/easy_manipulation_deployment" ]]; then
  echo "Please run this script from ~/workcell_ws/src/easy_manipulation_deployment" >&2
  exit 1
fi

# Select ROS distribution: prefer Humble then Jazzy
for d in humble jazzy; do
  if [[ -f "/opt/ros/$d/setup.bash" ]]; then
    ROS_DISTRO=${ROS_DISTRO:-$d}
    break
  fi
done

if [[ -z ${ROS_DISTRO:-} ]]; then
  echo "No supported ROS distro found (need humble or jazzy)" >&2
  exit 1
fi

# shellcheck source=/dev/null
# Disable nounset while sourcing ROS setup, as it references
# environment variables that may not be defined.
set +u
: "${AMENT_TRACE_SETUP_FILES:=}"
source "/opt/ros/$ROS_DISTRO/setup.bash"
set -u
export LANG=C.UTF-8 LC_ALL=C.UTF-8

# Ensure required tools
APT_GET="sudo apt-get"
command -v sudo >/dev/null 2>&1 || APT_GET="apt-get"

for cmd in git colcon rosdep vcs; do
  if ! command -v "$cmd" >/dev/null 2>&1; then
    echo "Installing missing tool: $cmd"
    $APT_GET update -y
    case "$cmd" in
      git) $APT_GET install -y git;;
      colcon) python3 -m pip install -U colcon-common-extensions;;
      rosdep) $APT_GET install -y python3-rosdep && rosdep init || true;;
      vcs) $APT_GET install -y python3-vcstool;;
    esac
  fi
done

# Ensure TinyXML2 headers/libs are available system-wide.  The upstream
# Tesseract packages rely on CMake's TinyXML2 package during both the
# configure and find_dependency() phases, but rosdep does not always pull in
# libtinyxml2-dev because many ROS distributions satisfy the runtime portion
# via the tinyxml2_vendor package.  Installing the development package up
# front guarantees the FindTinyXML2.cmake lookups succeed even when the
# vendor overlay is absent.
if ! dpkg -s libtinyxml2-dev >/dev/null 2>&1; then
  echo "Installing libtinyxml2-dev to satisfy TinyXML2 find_package() calls"
  $APT_GET update -y
  $APT_GET install -y libtinyxml2-dev
fi

# Import external repositories if tesseract not yet present
if [[ -f "$REPO_DIR/tesseract.repos" ]] && [[ ! -d "$SRC/tesseract" ]]; then
  vcs import --recursive "$SRC" < "$REPO_DIR/tesseract.repos"
fi

# Copy overlays from repo checkout if they exist
# Copy overlays from repo checkout if they exist and immediately reveal packages
HIDDEN_OVERLAY_PREFIXES=("$SRC/tesseract_ros2")
should_skip_marker() {
  local marker=$1
  for prefix in "${HIDDEN_OVERLAY_PREFIXES[@]}"; do
    if [[ $marker == "$prefix"* ]]; then
      return 0
    fi
  done
  return 1
}

for overlay in tesseract trajopt; do
  if [[ -d "$REPO_DIR/$overlay" ]]; then
    mkdir -p "$SRC/$overlay"
    cp -a "$REPO_DIR/$overlay/." "$SRC/$overlay/"
    # Rename ignore markers that may ship with upstream overlays so colcon sees the packages
    while IFS= read -r -d '' marker; do
      if should_skip_marker "$marker"; then
        continue
      fi
      echo "Renaming ignore marker $marker"
      mv -f "$marker" "$marker.repo"
    done < <(find "$SRC/$overlay" -maxdepth 2 \( -name 'COLCON_IGNORE' -o -name 'AMENT_IGNORE' \) -print0)
  fi
done

# Ensure workspace layout is sanitized (handles stray trajopt sources)
WS="$WS" "$REPO_DIR/scripts/fix_workspace_layout.sh"

# Ensure boost_plugin_loader exists
if [[ ! -d "$SRC/boost_plugin_loader" ]]; then
  git clone https://github.com/tesseract-robotics/boost_plugin_loader.git "$SRC/boost_plugin_loader"
fi

# Reveal any hidden Tesseract packages that may remain from upstream checkouts
find "$SRC" -path "$SRC/tesseract*" \( -name COLCON_IGNORE -o -name AMENT_IGNORE \) -print | while read -r f; do
  if should_skip_marker "$f"; then
    continue
  fi
  echo "Renaming ignore marker $f"
  mv -f "$f" "$f.repo"
done

# Remove duplicate package names
declare -A seen
while read -r name path _; do
  if [[ -n ${seen[$name]:-} && ${seen[$name]} != "$path" ]]; then
    echo "Removing duplicate package $name from $path (keeping ${seen[$name]})"
    rm -rf "$path"
  else
    seen[$name]="$path"
  fi
done < <(colcon list --base-paths "$SRC")

# Verify required packages are visible
if ! colcon list --base-paths "$SRC" | grep -E '^(tesseract_common|tesseract_msgs)\b' >/dev/null; then
  echo "tesseract_common or tesseract_msgs not visible to colcon" >&2
  exit 1
fi

# Install dependencies. Pass the explicit package paths reported by colcon to avoid
# rosdep scanning duplicate overlays (e.g., both the repo checkout and the copied
# overlay of trajopt_sco) which would otherwise trigger "Multiple packages found"
# errors.
mapfile -t ROSDEP_PATHS < <(colcon list --base-paths "$SRC" --paths-only)
if [[ ${#ROSDEP_PATHS[@]} -eq 0 ]]; then
  echo "No packages discovered for rosdep installation" >&2
  exit 1
fi

rosdep update
rosdep install --from-paths "${ROSDEP_PATHS[@]}" --ignore-src -yr --rosdistro "$ROS_DISTRO" \
  --skip-keys "tesseract tesseract_process_planners trajopt_ifopt trajopt_sqp trajopt jsoncpp message_generation"

# Enforce C++17
export AMENT_CMAKE_CXX_STANDARD=17
CMAKE_ARGS=( -DCMAKE_CXX_STANDARD=17 -DCMAKE_CXX_STANDARD_REQUIRED=ON -DCMAKE_CXX_EXTENSIONS=OFF )

# Minimal include fix for profile_dictionary
# Use find's -print -quit to grab the first match and avoid non-zero exit when not found
PDH=$(find "$SRC" -path "*/tesseract_command_language/include/tesseract_command_language/profile_dictionary.h" -print -quit 2>/dev/null)
if [[ -f "$PDH" ]] && grep -q '<shared_mutex>' "$PDH" && ! grep -q '<mutex>' "$PDH"; then
  echo "Patching missing <mutex> include in $PDH"
  sed -i '/<shared_mutex>/a #include <mutex>' "$PDH"
fi

cd "$WS"

source_install() {
  if [[ -f install/setup.bash ]]; then
    set +u
    : "${COLCON_TRACE:=}"
    source install/setup.bash
    # Ensure plain CMake packages that ship Colcon environment hooks properly
    # extend CMAKE_PREFIX_PATH even when sourced multiple times.  Some of the
    # vendored Tesseract packages only provide a hook script and do not rely on
    # ament to evaluate it, so manually source any available hooks here.
    if ! declare -F _colcon_prepend_unique_value >/dev/null; then
      _colcon_prepend_unique_value() {
        local listname="$1"
        local value="$2"
        local existing
        existing="$(eval "printf '%s' \"\${$listname:-}\"")"
        if [[ -n "$existing" ]]; then
          case ":$existing:" in
            *":$value:"*) return 0 ;;
          esac
        fi
        if [[ -n "$existing" ]]; then
          eval "export ${listname}=\"$value:$existing\""
        else
          eval "export ${listname}=\"$value\""
        fi
      }
    fi
    if compgen -G "install/*/share/*/hook/cmake_prefix_path.sh" >/dev/null; then
      # shellcheck disable=SC1090
      for hook in install/*/share/*/hook/cmake_prefix_path.sh; do
        source "$hook"
      done
    fi
    set -u
  fi
}

ensure_tesseract_collision_core_config() {
  local package_prefix="$WS/install/tesseract_collision"
  local config_dir="$package_prefix/lib/cmake/tesseract_collision_core"
  local config_file="$config_dir/tesseract_collision_coreConfig.cmake"

  if [[ -f "$config_file" ]] || [[ ! -d "$package_prefix" ]]; then
    return
  fi

  local core_lib=""
  local candidate
  for candidate in \
    "$package_prefix/lib/libtesseract_collision_core.so" \
    "$package_prefix/lib/libtesseract_collision_core.dylib" \
    "$package_prefix/lib/tesseract_collision_core.lib" \
    "$package_prefix/bin/tesseract_collision_core.dll"; do
    if [[ -f "$candidate" ]]; then
      core_lib="$candidate"
      break
    fi
  done

  if [[ -z "$core_lib" ]]; then
    echo "tesseract_collision_core library not found under $package_prefix; cannot synthesize CMake config" >&2
    return
  fi

  local include_dir="$package_prefix/include"
  if [[ ! -d "$include_dir" ]]; then
    include_dir="$package_prefix/include/tesseract_collision"
  fi

  mkdir -p "$config_dir"

  cat >"$config_file" <<EOF
# Generated by fix_and_build.sh to provide a minimal CMake package for
# tesseract_collision_core when the upstream build does not install one.
include(CMakeFindDependencyMacro)
find_dependency(Eigen3)
find_dependency(console_bridge)
find_dependency(tesseract_geometry)
find_dependency(tesseract_common)
find_dependency(yaml-cpp)
find_dependency(octomap)
find_dependency(Boost COMPONENTS filesystem system program_options serialization)
if(NOT TARGET tesseract_collision_core)
  add_library(tesseract_collision_core SHARED IMPORTED)
  set_target_properties(tesseract_collision_core PROPERTIES
    IMPORTED_LOCATION "${core_lib}"
    INTERFACE_INCLUDE_DIRECTORIES "${include_dir}")
endif()
if(NOT TARGET tesseract::tesseract_collision_core)
  add_library(tesseract::tesseract_collision_core ALIAS tesseract_collision_core)
endif()
set(tesseract_collision_core_FOUND TRUE)
EOF

  local package_xml="$package_prefix/share/tesseract_collision/package.xml"
  if [[ ! -f "$package_xml" && -f "$REPO_DIR/tesseract/tesseract_collision/package.xml" ]]; then
    package_xml="$REPO_DIR/tesseract/tesseract_collision/package.xml"
  fi

  local version="0.0.0"
  if [[ -f "$package_xml" ]]; then
    version="$(PACKAGE_XML="$package_xml" python3 - <<'PY'
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

  local version_file="$config_dir/tesseract_collision_coreConfigVersion.cmake"
  cmake -DPACKAGE_VERSION="$version" -DOUTPUT_FILE="$version_file" -P - <<'EOF'
include(CMakePackageConfigHelpers)
write_basic_package_version_file("${OUTPUT_FILE}" VERSION "${PACKAGE_VERSION}" COMPATIBILITY SameMajorVersion)
EOF

  echo "Synthesized missing tesseract_collision_core package configuration at $config_dir"
}

ensure_tesseract_collision_bullet_config() {
  local package_prefix="$WS/install/tesseract_collision"
  local config_dir="$package_prefix/lib/cmake/tesseract_collision_bullet"
  local config_file="$config_dir/tesseract_collision_bulletConfig.cmake"

  if [[ -f "$config_file" ]] || [[ ! -d "$package_prefix" ]]; then
    return
  fi

  local bullet_lib=""
  local candidate
  for candidate in \
    "$package_prefix/lib/libtesseract_collision_bullet.so" \
    "$package_prefix/lib/libtesseract_collision_bullet.dylib" \
    "$package_prefix/lib/tesseract_collision_bullet.lib" \
    "$package_prefix/bin/tesseract_collision_bullet.dll"; do
    if [[ -f "$candidate" ]]; then
      bullet_lib="$candidate"
      break
    fi
  done

  if [[ -z "$bullet_lib" ]]; then
    echo "tesseract_collision_bullet library not found under $package_prefix; cannot synthesize CMake config" >&2
    return
  fi

  local include_dir="$package_prefix/include"
  if [[ ! -d "$include_dir" ]]; then
    include_dir="$package_prefix/include/tesseract_collision"
  fi

  mkdir -p "$config_dir"

  local extras_source="$REPO_DIR/tesseract/tesseract_collision/bullet/cmake/bullet-extras.cmake"
  if [[ -f "$extras_source" ]]; then
    cp -f "$extras_source" "$config_dir/bullet-extras.cmake"
  fi

  cat >"$config_file" <<EOF
# Generated by fix_and_build.sh to provide a minimal CMake package for
# tesseract_collision_bullet when the upstream build does not install one.
include(CMakeFindDependencyMacro)
find_dependency(tesseract_collision COMPONENTS core)
if(EXISTS "\${CMAKE_CURRENT_LIST_DIR}/bullet-extras.cmake")
  include("\${CMAKE_CURRENT_LIST_DIR}/bullet-extras.cmake")
endif()
if(NOT TARGET tesseract_collision_bullet)
  add_library(tesseract_collision_bullet SHARED IMPORTED)
  set_target_properties(tesseract_collision_bullet PROPERTIES
    IMPORTED_LOCATION "${bullet_lib}"
    INTERFACE_INCLUDE_DIRECTORIES "${include_dir}"
    INTERFACE_LINK_LIBRARIES "tesseract::tesseract_collision_core")
endif()
if(NOT TARGET tesseract::tesseract_collision_bullet)
  add_library(tesseract::tesseract_collision_bullet ALIAS tesseract_collision_bullet)
endif()
set(tesseract_collision_bullet_FOUND TRUE)
EOF

  local package_xml="$package_prefix/share/tesseract_collision/package.xml"
  if [[ ! -f "$package_xml" && -f "$REPO_DIR/tesseract/tesseract_collision/package.xml" ]]; then
    package_xml="$REPO_DIR/tesseract/tesseract_collision/package.xml"
  fi

  local version="0.0.0"
  if [[ -f "$package_xml" ]]; then
    version="$(PACKAGE_XML="$package_xml" python3 - <<'PY'
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

  local version_file="$config_dir/tesseract_collision_bulletConfigVersion.cmake"
  cmake -DPACKAGE_VERSION="$version" -DOUTPUT_FILE="$version_file" -P - <<'EOF'
include(CMakePackageConfigHelpers)
write_basic_package_version_file("${OUTPUT_FILE}" VERSION "${PACKAGE_VERSION}" COMPATIBILITY SameMajorVersion)
EOF

  echo "Synthesized missing tesseract_collision_bullet package configuration at $config_dir"
}

# Generate a minimal CMake package configuration for tesseract_state_solver when
# the upstream build fails to install one.  Downstream packages like
# tesseract_kinematics rely on tesseract_state_solverConfig.cmake to discover the
# core, KDL, and OFKT targets.
ensure_tesseract_state_solver_config() {
  local package_prefix="$WS/install/tesseract_state_solver"
  local config_dir="$package_prefix/lib/cmake/tesseract_state_solver"
  local config_file="$config_dir/tesseract_state_solverConfig.cmake"

  if [[ -f "$config_file" ]] || [[ ! -d "$package_prefix" ]]; then
    return
  fi

  local include_dir="$package_prefix/include"
  if [[ ! -d "$include_dir" ]]; then
    include_dir="$package_prefix/include/tesseract_state_solver"
  fi

  local kdl_lib="" ofkt_lib="" core_link
  for candidate in \
    "$package_prefix/lib/libtesseract_state_solver_kdl.so" \
    "$package_prefix/lib/libtesseract_state_solver_kdl.dylib" \
    "$package_prefix/lib/tesseract_state_solver_kdl.lib" \
    "$package_prefix/bin/tesseract_state_solver_kdl.dll"; do
    if [[ -f "$candidate" ]]; then
      kdl_lib="$candidate"
      break
    fi
  done

  for candidate in \
    "$package_prefix/lib/libtesseract_state_solver_ofkt.so" \
    "$package_prefix/lib/libtesseract_state_solver_ofkt.dylib" \
    "$package_prefix/lib/tesseract_state_solver_ofkt.lib" \
    "$package_prefix/bin/tesseract_state_solver_ofkt.dll"; do
    if [[ -f "$candidate" ]]; then
      ofkt_lib="$candidate"
      break
    fi
  done

  core_link="Eigen3::Eigen;tesseract::tesseract_common;tesseract::tesseract_scene_graph;console_bridge::console_bridge"

  mkdir -p "$config_dir"

  cat >"$config_file" <<EOF
# Generated by fix_and_build.sh to provide a minimal CMake package for
# tesseract_state_solver when the upstream build does not install one.
include(CMakeFindDependencyMacro)
find_dependency(Eigen3)
find_dependency(tesseract_scene_graph)
find_dependency(tesseract_common)
find_dependency(console_bridge)
find_dependency(orocos_kdl)

if(NOT TARGET tesseract_state_solver_core)
  add_library(tesseract_state_solver_core INTERFACE IMPORTED)
  set_target_properties(tesseract_state_solver_core PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${include_dir}"
    INTERFACE_LINK_LIBRARIES "${core_link}")
endif()
if(NOT TARGET tesseract::tesseract_state_solver_core)
  add_library(tesseract::tesseract_state_solver_core ALIAS tesseract_state_solver_core)
endif()

if("${kdl_lib}" AND NOT TARGET tesseract_state_solver_kdl)
  add_library(tesseract_state_solver_kdl SHARED IMPORTED)
  set_target_properties(tesseract_state_solver_kdl PROPERTIES
    IMPORTED_LOCATION "${kdl_lib}"
    INTERFACE_INCLUDE_DIRECTORIES "${include_dir}"
    INTERFACE_LINK_LIBRARIES "tesseract::tesseract_state_solver_core;orocos-kdl")
endif()
if(NOT TARGET tesseract::tesseract_state_solver_kdl)
  add_library(tesseract::tesseract_state_solver_kdl ALIAS tesseract_state_solver_kdl)
endif()

if("${ofkt_lib}" AND NOT TARGET tesseract_state_solver_ofkt)
  add_library(tesseract_state_solver_ofkt SHARED IMPORTED)
  set_target_properties(tesseract_state_solver_ofkt PROPERTIES
    IMPORTED_LOCATION "${ofkt_lib}"
    INTERFACE_INCLUDE_DIRECTORIES "${include_dir}"
    INTERFACE_LINK_LIBRARIES "tesseract::tesseract_state_solver_core")
endif()
if(NOT TARGET tesseract::tesseract_state_solver_ofkt)
  add_library(tesseract::tesseract_state_solver_ofkt ALIAS tesseract_state_solver_ofkt)
endif()

set(tesseract_state_solver_FOUND TRUE)
EOF

  local package_xml="$package_prefix/share/tesseract_state_solver/package.xml"
  if [[ ! -f "$package_xml" && -f "$REPO_DIR/tesseract/tesseract_state_solver/package.xml" ]]; then
    package_xml="$REPO_DIR/tesseract/tesseract_state_solver/package.xml"
  fi

  local version="0.0.0"
  if [[ -f "$package_xml" ]]; then
    version="$(PACKAGE_XML="$package_xml" python3 - <<'PY'
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

  local version_file="$config_dir/tesseract_state_solverConfigVersion.cmake"
  cmake -DPACKAGE_VERSION="$version" -DOUTPUT_FILE="$version_file" -P - <<'EOF'
include(CMakePackageConfigHelpers)
write_basic_package_version_file("${OUTPUT_FILE}" VERSION "${PACKAGE_VERSION}" COMPATIBILITY SameMajorVersion)
EOF

  echo "Synthesized missing tesseract_state_solver package configuration at $config_dir"
}

# Build boost_plugin_loader first
colcon build --symlink-install --packages-select boost_plugin_loader --cmake-args "${CMAKE_ARGS[@]}"
source_install
find install -name 'tesseract_commonConfig.cmake' || true

# Build up to tesseract_common and tesseract_msgs
colcon build --symlink-install --packages-up-to tesseract_common tesseract_msgs --cmake-args "${CMAKE_ARGS[@]}"
source_install
find install -name 'tesseract_commonConfig.cmake'

# Build and expose tesseract_state_solver early so downstream packages like
# tesseract_kinematics can reliably locate its exported CMake package
colcon build --symlink-install --packages-select tesseract_state_solver --cmake-args "${CMAKE_ARGS[@]}"
source_install
ensure_tesseract_state_solver_config

# Build up to trajopt_sco
colcon build --symlink-install --packages-up-to trajopt_sco --cmake-args "${CMAKE_ARGS[@]}"
source_install
ensure_tesseract_collision_core_config
ensure_tesseract_collision_bullet_config
ensure_tesseract_state_solver_config
find install -name 'tesseract_commonConfig.cmake'

# Build the whole workspace.  Rebuilding tesseract_collision wipes the
# synthesized tesseract_collision_core/tesseract_collision_bullet config files
# that downstream packages need, so skip reprocessing that package here after it
# has already been built in the trajopt_sco pass above.
COLCON_BUILD_ARGS=(--symlink-install --cmake-args "${CMAKE_ARGS[@]}")
if colcon list --base-paths "$SRC" | grep -q '^tesseract_collision\\b'; then
  echo "Skipping tesseract_collision during final rebuild to preserve synthesized configs"
  COLCON_BUILD_ARGS+=(--packages-skip tesseract_collision)
fi
colcon build "${COLCON_BUILD_ARGS[@]}"
source_install
ensure_tesseract_collision_core_config
ensure_tesseract_collision_bullet_config
ensure_tesseract_state_solver_config
find install -name 'tesseract_commonConfig.cmake'

# Print overlay information
colcon list --paths-only | sort
printf '%s\n' ${CMAKE_PREFIX_PATH//:/\n} | head -n 20

echo "Build completed successfully"
