#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

PROFILE="minimal"
WITH_GUI=0
CLEAN_BUILD=0
WORKSPACE=""
DRY_RUN=0
CHECK_PREREQS=0
INSTALL_PREREQS=0
BUILD_WORKSPACE=0
PARALLEL_WORKERS=""
EPD_UNDERLAY=""
CMAKE_BUILD_TYPE="Release"
SKIP_LOCAL_PROOF_PACKAGES=0
OSQPEIGEN_REPAIRED=0

MUTATING_ACTIONS=()
PACKAGES_INSTALLED=()
PIP_PACKAGES_INSTALLED=()
REPOS_IMPORTED=()
FILES_TOUCHED=()
RODEP_SKIP_KEYS_USED=()
COLCON_SKIP_PACKAGES_USED=()

EPD_UNDERLAY_USED=0
EPD_MSGS_PREFIX=""
EPD_INTEGRATION_ENABLED="unknown"
OSQP_PROVIDER="unknown"
OSQP_VERSION_DETECTED="unknown"
OSQPEIGEN_VERSION_DETECTED="unknown"
TARGET_WS_IN_ENV_BEFORE_BUILD=0
TARGET_WS_ENV_HITS=()

usage() {
  cat <<'USAGE'
Usage: ./fix_and_build_humble.sh [options]

Phase options (choose one or more):
  --check-prereqs            Read-only checks. Verifies tooling and ROS prerequisites.
  --install-prereqs          Mutating phase. Installs missing apt/pip prerequisites.
  --build                    Bootstrap + build phase in the workspace.

General options:
  --workspace <path>         Workspace root (default: current working directory)
  --profile minimal|full     Build profile (default: minimal)
  --with-gui                 Build optional GUI packages in full profile
  --parallel-workers <N>     Optional colcon parallel workers override (default: colcon default)
  --cmake-build-type <type>  CMake build type for colcon/osqp builds (default: Release)
  --epd-underlay <path>      Optional EPD workspace root (sources <path>/install/local_setup.bash)
  --skip-local-proof-packages  Application-only build; excludes local proof packages, NOT Stage-A qualification
  --clean                    Remove build/, install/, and log/ before build
  --dry-run                  Print commands without executing
  -h, --help                 Show this help message

Examples:
  ./fix_and_build_humble.sh --workspace ~/workcell_ws --check-prereqs --install-prereqs --build --profile full --epd-underlay ~/epd_ros2_ws --parallel-workers 4 --skip-local-proof-packages
  ./fix_and_build_humble.sh --workspace ~/workcell_ws --check-prereqs --install-prereqs --build --profile full --clean --epd-underlay ~/epd_ros2_ws
USAGE
}

log_change() { MUTATING_ACTIONS+=("$1"); }

append_unique() {
  local -n arr_ref=$1
  local value="$2"
  local existing
  for existing in "${arr_ref[@]:-}"; do
    [[ "$existing" == "$value" ]] && return 0
  done
  arr_ref+=("$value")
}

join_by() {
  local delim="$1"; shift || true
  local out=""
  local val
  for val in "$@"; do
    [[ -z "$val" ]] && continue
    if [[ -z "$out" ]]; then
      out="$val"
    else
      out+="$delim$val"
    fi
  done
  printf '%s' "$out"
}

run_cmd() {
  local cmd="$*"
  if [[ $DRY_RUN -eq 1 ]]; then
    printf '[dry-run] %s\n' "$cmd"
    return 0
  fi
  eval "$cmd"
}

need_command() { command -v "$1" >/dev/null 2>&1; }

apt_install_package() {
  local pkg="$1"
  local apt_prefix=""
  if command -v sudo >/dev/null 2>&1; then
    apt_prefix="sudo "
  elif [[ ${EUID:-$(id -u)} -ne 0 ]]; then
    echo "Cannot install '$pkg': requires root or sudo." >&2
    exit 1
  fi

  run_cmd "${apt_prefix}apt-get update -y"
  run_cmd "${apt_prefix}apt-get install -y $pkg"
  append_unique PACKAGES_INSTALLED "$pkg"
  log_change "apt install $pkg"
}

check_epd_underlay_layout() {
  [[ -z "$EPD_UNDERLAY" ]] && return 0
  EPD_UNDERLAY="$(cd "$EPD_UNDERLAY" && pwd 2>/dev/null || printf '%s' "$EPD_UNDERLAY")"
  local local_setup="$EPD_UNDERLAY/install/local_setup.bash"
  if [[ ! -f "$local_setup" ]]; then
    echo "--epd-underlay expects an EPD workspace root with install/local_setup.bash. Missing: $local_setup" >&2
    exit 1
  fi
}


# Workcell Builder's embedded Web3D viewer uses Node.js 20 in CI.
# Jammy's default nodejs package is not the pinned/tested runtime.
load_nvm_if_available() {
  if [[ -s "$HOME/.nvm/nvm.sh" ]]; then
    export NVM_DIR="$HOME/.nvm"
    # NVM is sourced under the same nounset guard used for ROS setup.
    # shellcheck disable=SC1091
    set +u
    source "$NVM_DIR/nvm.sh"
    set -u
  fi
}

node20_available() {
  need_command node && need_command npm &&
    [[ "$(node -p 'process.versions.node.split(".")[0]' 2>/dev/null)" == "20" ]]
}

ensure_node20() {
  load_nvm_if_available
  if node20_available; then return 0; fi
  if [[ $INSTALL_PREREQS -ne 1 ]]; then
    echo "Embedded Web3D requires Node.js 20 and npm. Re-run with --install-prereqs, or install Node.js 20 via NVM." >&2
    return 1
  fi
  if [[ ! -s "$HOME/.nvm/nvm.sh" ]]; then
    if [[ -e "$HOME/.nvm" ]]; then
      echo "NVM installation blocked: $HOME/.nvm exists but nvm.sh is absent. Repair it manually; it was not deleted." >&2
      return 1
    fi
    run_cmd "git clone --depth 1 --branch v0.40.3 https://github.com/nvm-sh/nvm.git '$HOME/.nvm'"
    log_change "install per-user nvm v0.40.3"
  fi
  if [[ $DRY_RUN -eq 1 ]]; then
    echo "[dry-run] load NVM and install Node.js 20 + npm"
    return 0
  fi
  load_nvm_if_available
  run_cmd "nvm install 20 && nvm alias default 20"
  node20_available || { echo "Node.js 20 / npm installation failed." >&2; return 1; }
}

apt_package_installed() {
  dpkg -s "$1" 2>/dev/null | grep -qx 'Status: install ok installed'
}

install_prereqs_phase() {
  local missing_tools=()

  need_command git || missing_tools+=(git)
  need_command vcs || missing_tools+=(python3-vcstool)
  need_command rosdep || missing_tools+=(python3-rosdep)

  if ! python3 - <<'PY' >/dev/null 2>&1
import yaml  # noqa: F401
PY
  then
    missing_tools+=(python3-yaml)
  fi

  set +u
  : "${AMENT_TRACE_SETUP_FILES:=}"
  source /opt/ros/humble/setup.bash
  set -u
  if ! ros2 pkg prefix moveit_ros_perception >/dev/null 2>&1; then
    missing_tools+=(ros-humble-moveit-ros-perception)
  fi

  local pkg
  for pkg in "${missing_tools[@]:-}"; do
    apt_install_package "$pkg"
  done

  # Required by Qt Widgets/Svg/OpenGL and the default embedded Web3D Product View.
  for pkg in libqt5svg5-dev libqt5opengl5-dev qtwebengine5-dev; do
    if ! apt_package_installed "$pkg"; then apt_install_package "$pkg"; fi
  done
  ensure_node20

  if ! need_command colcon; then
    need_command pip3 || apt_install_package python3-pip
    run_cmd "python3 -m pip install -U colcon-common-extensions"
    append_unique PIP_PACKAGES_INSTALLED "colcon-common-extensions"
    log_change "pip install colcon-common-extensions"
  fi

  if need_command rosdep; then
    local rosdep_sources="/etc/ros/rosdep/sources.list.d/20-default.list"
    if [[ -f "$rosdep_sources" ]]; then
      echo "rosdep default sources already initialized."
    else
      run_cmd "rosdep init || true"
      log_change "rosdep init"
    fi
  fi
}

check_prereqs_phase() {
  local missing=()
  local ros_setup="/opt/ros/humble/setup.bash"

  [[ -d "$WORKSPACE" ]] || missing+=("workspace directory '$WORKSPACE' does not exist")
  [[ -d "$WORKSPACE/src" ]] || missing+=("workspace is missing src/ directory at '$WORKSPACE/src'")
  [[ -f "$ros_setup" ]] || missing+=("missing ROS setup: $ros_setup")

  need_command git || missing+=("missing command: git")
  need_command vcs || missing+=("missing command: vcs")
  need_command rosdep || missing+=("missing command: rosdep")
  need_command colcon || missing+=("missing command: colcon")
  local qt_pkg
  for qt_pkg in libqt5svg5-dev libqt5opengl5-dev qtwebengine5-dev; do
    apt_package_installed "$qt_pkg" || missing+=("missing Workcell Builder Qt development package: $qt_pkg")
  done
  load_nvm_if_available
  if ! node20_available && ! [[ $DRY_RUN -eq 1 && $INSTALL_PREREQS -eq 1 ]]; then
    missing+=("Node.js 20 and npm required for embedded Web3D (run --install-prereqs)")
  fi

  if ! python3 - <<'PY' >/dev/null 2>&1
import yaml  # noqa: F401
PY
  then
    missing+=("missing Python module: yaml")
  fi

  check_epd_underlay_layout

  set +u
  : "${AMENT_TRACE_SETUP_FILES:=}"
  source /opt/ros/humble/setup.bash
  set -u
  if ! ros2 pkg prefix moveit_ros_perception >/dev/null 2>&1; then
    missing+=("Missing MoveIt perception package required for octomap pointcloud updates: ros-humble-moveit-ros-perception")
  fi

  if [[ ${#missing[@]} -gt 0 ]]; then
    echo "Prerequisite check failed. Missing requirements:" >&2
    printf '  - %s\n' "${missing[@]}" >&2
    exit 1
  fi

  echo "Prerequisite check passed for workspace: $WORKSPACE"
}

osqp_v1_headers_ok() {
  local include_dir="$1"
  local api_types_header="$include_dir/osqp_api_types.h"
  [[ -f "$api_types_header" ]] || return 1
  grep -R -q "OSQPSolver" "$include_dir" || return 1
  grep -R -q "OSQPInt" "$include_dir" || return 1
  grep -R -q "OSQPCscMatrix" "$include_dir" || return 1
}

cleanup_stale_osqp_install() {
  local rm_prefix=""
  if command -v sudo >/dev/null 2>&1; then
    rm_prefix="sudo "
  fi
  run_cmd "${rm_prefix}rm -rf /usr/local/include/osqp /usr/local/lib/libosqp* /usr/local/lib/cmake/osqp /usr/local/lib/pkgconfig/osqp.pc"
}

prepare_osqp_stack() {
  local osqp_include_local="/usr/local/include/osqp"
  local osqp_include_system="/usr/include/osqp"
  local need_local_osqp=1

  if osqp_v1_headers_ok "$osqp_include_local" && [[ -f /usr/local/lib/cmake/osqp/osqp-config.cmake ]]; then
    OSQP_PROVIDER="/usr/local"
    OSQP_VERSION_DETECTED="$(grep -R -E '#define OSQP_VERSION' "$osqp_include_local" | head -n1 | sed -E 's/.*"([^"]+)".*/\1/' || true)"
    need_local_osqp=0
  fi

  if [[ -d "$osqp_include_local" ]] && [[ $need_local_osqp -eq 1 ]]; then
    echo "Existing /usr/local OSQP is incompatible with current TrajOpt sources. Reinstalling pinned OSQP v1 compatibility stack."
    cleanup_stale_osqp_install
  fi

  if [[ $need_local_osqp -eq 1 ]]; then
    echo "Installing pinned OSQP/OsqpEigen compatibility stack for current TrajOpt/Tesseract sources (OSQP v1.x + OsqpEigen v0.11.x)."
    run_cmd "cmake -S src/osqp -B build/_external/osqp -DCMAKE_BUILD_TYPE=$CMAKE_BUILD_TYPE -DCMAKE_INSTALL_PREFIX=/usr/local -DBUILD_SHARED_LIBS=ON"
    run_cmd "cmake --build build/_external/osqp"
    if command -v sudo >/dev/null 2>&1; then
      run_cmd "sudo cmake --install build/_external/osqp"
    else
      run_cmd "cmake --install build/_external/osqp"
    fi
    log_change "install osqp from source (v1.x)"

    OSQP_PROVIDER="/usr/local"
  fi

  # OsqpEigen may have been configured against Humble's old OSQP even when a
  # valid OSQP v1 provider already existed. Its exported v1 definitions must
  # be present before compiling trajopt_sqp.
  local eigen_targets="/usr/local/lib/cmake/OsqpEigen/OsqpEigenTargets.cmake"
  if [[ ! -f "$eigen_targets" ]] ||
    ! grep -Fq "OSQP_EIGEN_OSQP_IS_V1;OSQP_EIGEN_OSQP_IS_V1_FINAL" "$eigen_targets"; then
    echo "Repairing pinned OsqpEigen against OSQP v1 in /usr/local."
    run_cmd "cmake -S src/osqp-eigen -B build/_external/osqp-eigen -UOSQP_IS_V1 -UOSQP_IS_V1_FINAL -DCMAKE_BUILD_TYPE=$CMAKE_BUILD_TYPE -DCMAKE_INSTALL_PREFIX=/usr/local -Dosqp_DIR=/usr/local/lib/cmake/osqp"
    if [[ $DRY_RUN -eq 0 ]]; then
      grep -Eq '^OSQP_IS_V1:INTERNAL=TRUE$' build/_external/osqp-eigen/CMakeCache.txt ||
        { echo "OsqpEigen did not detect OSQP_IS_V1=TRUE; stopping." >&2; return 1; }
      grep -Eq '^OSQP_IS_V1_FINAL:INTERNAL=TRUE$' build/_external/osqp-eigen/CMakeCache.txt ||
        { echo "OsqpEigen did not detect OSQP_IS_V1_FINAL=TRUE; stopping." >&2; return 1; }
    fi
    run_cmd "cmake --build build/_external/osqp-eigen --parallel 4"
    if command -v sudo >/dev/null 2>&1; then
      run_cmd "sudo cmake --install build/_external/osqp-eigen"
      run_cmd "sudo ldconfig"
    else
      run_cmd "cmake --install build/_external/osqp-eigen"
      run_cmd "ldconfig"
    fi
    log_change "repair pinned OsqpEigen OSQP v1 export definitions"
    OSQPEIGEN_REPAIRED=1
  fi
  if [[ $DRY_RUN -eq 0 ]]; then
    grep -Fq "OSQP_EIGEN_OSQP_IS_V1;OSQP_EIGEN_OSQP_IS_V1_FINAL" "$eigen_targets" ||
      { echo "OsqpEigen target is missing required OSQP v1 compile definitions." >&2; return 1; }
  fi

  if [[ -f "/usr/local/include/osqp/osqp.h" ]]; then
    OSQP_VERSION_DETECTED="$(grep -E '#define OSQP_VERSION' /usr/local/include/osqp/osqp.h | head -n1 | sed -E 's/.*"([^"]+)".*/\1/' || true)"
  fi

  if [[ -f "/usr/local/include/OsqpEigen/OsqpEigen.h" ]]; then
    OSQPEIGEN_VERSION_DETECTED="$(grep -E '#define OSQP_EIGEN_VERSION' /usr/local/include/OsqpEigen/OsqpEigen.h | head -n1 | sed -E 's/.*"([^"]+)".*/\1/' || true)"
  elif [[ -f "/usr/include/OsqpEigen/OsqpEigen.h" ]]; then
    OSQPEIGEN_VERSION_DETECTED="$(grep -E '#define OSQP_EIGEN_VERSION' /usr/include/OsqpEigen/OsqpEigen.h | head -n1 | sed -E 's/.*"([^"]+)".*/\1/' || true)"
  fi

  if ! osqp_v1_headers_ok "/usr/local/include/osqp" && ! osqp_v1_headers_ok "/usr/include/osqp"; then
    echo "Incompatible OSQP headers detected: expected OSQP v1 symbols (OSQPSolver, OSQPInt, OSQPCscMatrix)." >&2
    exit 1
  fi

  if [[ ! -f "/usr/local/include/OsqpEigen/OsqpEigen.h" && ! -f "/usr/include/OsqpEigen/OsqpEigen.h" ]]; then
    echo "OsqpEigen compatibility install incomplete: OsqpEigen headers were not found." >&2
    exit 1
  fi

  [[ -d "/usr/local/lib/cmake/osqp" || -d "/usr/lib/cmake/osqp" ]] || {
    echo "OSQP compatibility install incomplete: osqp CMake config directory was not found." >&2
    exit 1
  }
  [[ -d "/usr/local/lib/cmake/OsqpEigen" || -d "/usr/lib/cmake/OsqpEigen" ]] || {
    echo "OsqpEigen compatibility install incomplete: OsqpEigen CMake config directory was not found." >&2
    exit 1
  }
}

hide_source_only_vendor_dependencies() {
  local marker
  local source_only_dirs=("src/osqp" "src/osqp-eigen" "src/cereal")
  for dir in "${source_only_dirs[@]}"; do
    [[ -d "$dir" ]] || continue
    echo "Keeping source-only/vendor dependency hidden from colcon: $dir"
    for marker in COLCON_IGNORE AMENT_IGNORE; do
      if [[ ! -e "$dir/$marker" ]]; then
        run_cmd "touch '$dir/$marker'"
        append_unique FILES_TOUCHED "$WORKSPACE/$dir/$marker"
        log_change "hide source-only dependency $dir via $marker"
      fi
    done
  done
}

validate_cereal_dependency_handling() {
  local rosdep_resolution
  rosdep_resolution="$(rosdep resolve cereal 2>/dev/null || true)"
  if [[ "$rosdep_resolution" != *"libcereal-dev"* ]]; then
    echo "Cereal rosdep validation failed: expected cereal -> libcereal-dev mapping." >&2
    echo "rosdep resolve cereal output:" >&2
    printf '%s\n' "$rosdep_resolution" >&2
    exit 1
  fi
  echo "Validated rosdep override: cereal resolves to libcereal-dev."

  if [[ -d "src/cereal" ]]; then
    local missing_markers=()
    [[ -e "src/cereal/COLCON_IGNORE" ]] || missing_markers+=("src/cereal/COLCON_IGNORE")
    [[ -e "src/cereal/AMENT_IGNORE" ]] || missing_markers+=("src/cereal/AMENT_IGNORE")
    if [[ ${#missing_markers[@]} -gt 0 ]]; then
      echo "Cereal source hiding validation failed: missing ignore markers:" >&2
      printf '  - %s\n' "${missing_markers[@]}" >&2
      exit 1
    fi
    echo "Validated source checkout hiding: src/cereal contains COLCON_IGNORE and AMENT_IGNORE."
  else
    echo "Cereal source checkout not present under src/. Relying on libcereal-dev via rosdep."
  fi

  if colcon list --base-paths src --names-only 2>/dev/null | grep -xq "cereal"; then
    echo "Workspace validation failed: colcon still discovers package named 'cereal'." >&2
    exit 1
  fi
  echo "Validated workspace discovery: no colcon package named cereal is visible."
}

capture_epd_state() {
  if [[ -z "$EPD_UNDERLAY" ]]; then
    EPD_UNDERLAY_USED=0
    EPD_MSGS_PREFIX=""
    return 0
  fi

  EPD_UNDERLAY_USED=1
  if ! EPD_MSGS_PREFIX="$(ros2 pkg prefix epd_msgs 2>/dev/null)"; then
    echo "EPD underlay was requested (--epd-underlay $EPD_UNDERLAY) but 'epd_msgs' was not discoverable." >&2
    echo "Ensure: source /opt/ros/humble/setup.bash && source $EPD_UNDERLAY/install/local_setup.bash exposes epd_msgs." >&2
    exit 1
  fi
}

calculate_colcon_skip_packages() {
  local profile="$1"
  local with_gui="$2"
  local all_packages
  mapfile -t all_packages < <(colcon list --base-paths src --names-only 2>/dev/null || true)

  local requested_skips=()
  if [[ "$profile" == "full" && "$with_gui" -eq 0 ]]; then
    requested_skips+=(tesseract_qt qtadvanceddocking QtADS tesseract_rviz tesseract_ros_examples tesseract_planning_server)
  fi

  if [[ $SKIP_LOCAL_PROOF_PACKAGES -eq 1 ]]; then
    requested_skips+=(workcell_bridge_shutdown_proof workcell_tem_teardown_repro)
    echo "APPLICATION-ONLY: local bridge/TEM proof packages are excluded; Stage-A motion and shutdown qualification remain UNQUALIFIED." >&2
  fi

  # Dependency-safe skip: tesseract_ros_examples must be skipped when tesseract_rviz is skipped.
  if printf '%s\n' "${requested_skips[@]:-}" | grep -Fxq "tesseract_rviz"; then
    requested_skips+=(tesseract_ros_examples)
  fi

  local discovered=()
  local pkg
  for pkg in "${requested_skips[@]}"; do
    if printf '%s\n' "${all_packages[@]}" | grep -Fxq "$pkg"; then
      discovered+=("$pkg")
    fi
  done

  COLCON_SKIP_PACKAGES_USED=("${discovered[@]:-}")
}

capture_target_workspace_env_contamination() {
  local workspace_install="$WORKSPACE/install"
  local var_name var_value split_value
  local env_vars=("AMENT_PREFIX_PATH" "CMAKE_PREFIX_PATH" "COLCON_PREFIX_PATH")

  TARGET_WS_IN_ENV_BEFORE_BUILD=0
  TARGET_WS_ENV_HITS=()
  for var_name in "${env_vars[@]}"; do
    var_value="${!var_name:-}"
    [[ -z "$var_value" ]] && continue
    IFS=':' read -r -a split_value <<< "$var_value"
    local entry
    for entry in "${split_value[@]}"; do
      if [[ "$entry" == "$workspace_install" ]]; then
        TARGET_WS_IN_ENV_BEFORE_BUILD=1
        TARGET_WS_ENV_HITS+=("$var_name=$entry")
      fi
    done
  done

  if [[ $TARGET_WS_IN_ENV_BEFORE_BUILD -eq 1 ]]; then
    echo "Warning: target workspace install path was present in the shell environment before build: $workspace_install"
    printf '  - %s\n' "${TARGET_WS_ENV_HITS[@]}"
    echo "The script will sanitize underlays and continue with only /opt/ros/humble and optional EPD underlay."
  fi
}

capture_emd_epd_integration_flag() {
  local flags_file="$WORKSPACE/build/emd_grasp_planner/CMakeFiles/grasp_planning_interface.dir/flags.make"
  if [[ -f "$flags_file" ]] && grep -q 'EPD_ENABLED=1' "$flags_file"; then
    EPD_INTEGRATION_ENABLED="true"
  elif [[ -f "$flags_file" ]] && grep -q 'EPD_ENABLED=0' "$flags_file"; then
    EPD_INTEGRATION_ENABLED="false"
  else
    EPD_INTEGRATION_ENABLED="unknown"
  fi
}

emit_summary() {
  local summary_file="$WORKSPACE/fix_and_build_summary.json"
  local mut_json pkg_json pip_json repo_json file_json rosdep_skip_json colcon_skip_json

  mut_json=$(printf '%s\n' "${MUTATING_ACTIONS[@]:-}" | python3 -c 'import json,sys; print(json.dumps([l for l in sys.stdin.read().splitlines() if l]))')
  pkg_json=$(printf '%s\n' "${PACKAGES_INSTALLED[@]:-}" | python3 -c 'import json,sys; print(json.dumps([l for l in sys.stdin.read().splitlines() if l]))')
  pip_json=$(printf '%s\n' "${PIP_PACKAGES_INSTALLED[@]:-}" | python3 -c 'import json,sys; print(json.dumps([l for l in sys.stdin.read().splitlines() if l]))')
  repo_json=$(printf '%s\n' "${REPOS_IMPORTED[@]:-}" | python3 -c 'import json,sys; print(json.dumps([l for l in sys.stdin.read().splitlines() if l]))')
  file_json=$(printf '%s\n' "${FILES_TOUCHED[@]:-}" | python3 -c 'import json,sys; print(json.dumps([l for l in sys.stdin.read().splitlines() if l]))')
  rosdep_skip_json=$(printf '%s\n' "${RODEP_SKIP_KEYS_USED[@]:-}" | python3 -c 'import json,sys; print(json.dumps([l for l in sys.stdin.read().splitlines() if l]))')
  colcon_skip_json=$(printf '%s\n' "${COLCON_SKIP_PACKAGES_USED[@]:-}" | python3 -c 'import json,sys; print(json.dumps([l for l in sys.stdin.read().splitlines() if l]))')

  local assets_alias="$WORKSPACE/src/assets"
  local scenes_alias="$WORKSPACE/src/scenes"
  local assets_alias_target scenes_alias_target
  assets_alias_target="$(readlink -f "$assets_alias" 2>/dev/null || true)"
  scenes_alias_target="$(readlink -f "$scenes_alias" 2>/dev/null || true)"
  local json
  json=$(cat <<JSON
{
  "workspace": "${WORKSPACE}",
  "dry_run": ${DRY_RUN},
  "phases": {
    "check_prereqs": ${CHECK_PREREQS},
    "install_prereqs": ${INSTALL_PREREQS},
    "build": ${BUILD_WORKSPACE}
  },
  "profile": "${PROFILE}",
  "with_gui": ${WITH_GUI},
  "clean_build": ${CLEAN_BUILD},
  "skip_local_proof_packages": ${SKIP_LOCAL_PROOF_PACKAGES},
  "parallel_workers": "${PARALLEL_WORKERS}",
  "epd_underlay": {
    "requested": "${EPD_UNDERLAY}",
    "used": ${EPD_UNDERLAY_USED},
    "epd_msgs_prefix": "${EPD_MSGS_PREFIX}",
    "emd_grasp_planner_epd_enabled": "${EPD_INTEGRATION_ENABLED}"
  },
  "target_workspace_in_environment_before_build": {
    "detected": ${TARGET_WS_IN_ENV_BEFORE_BUILD},
    "hits": $(printf '%s\n' "${TARGET_WS_ENV_HITS[@]:-}" | python3 -c 'import json,sys; print(json.dumps([l for l in sys.stdin.read().splitlines() if l]))')
  },
  "osqp_compatibility": {
    "provider": "${OSQP_PROVIDER}",
    "osqp_version": "${OSQP_VERSION_DETECTED}",
    "osqp_eigen_version": "${OSQPEIGEN_VERSION_DETECTED}",
    "expected_matrix": "OSQP=v1.x, OsqpEigen=v0.11.x, TrajOpt/Tesseract=0.33.x"
  },
  "rosdep_skip_keys_used": ${rosdep_skip_json},
  "colcon_skip_packages_used": ${colcon_skip_json},
  "mutating_actions": ${mut_json},
  "packages_installed": ${pkg_json},
  "pip_packages_installed": ${pip_json},
  "repos_imported": ${repo_json},
  "files_touched": ${file_json},
  "workspace_layout": {
    "assets_alias": "${assets_alias_target}",
    "scenes_alias": "${scenes_alias_target}",
    "legacy_asset_symlinks_removed": "reported by scripts/fix_workspace_layout.sh",
    "duplicate_package_check": "enforced by scripts/fix_workspace_layout.sh and scripts/verify_workspace_discovery.sh"
  }
}
JSON
)

  if [[ $DRY_RUN -eq 1 ]]; then
    printf '%s\n' "$json"
  else
    printf '%s\n' "$json" > "$summary_file"
    echo "Wrote summary: $summary_file"
  fi
}

build_phase() {
  cd "$WORKSPACE"

  if [[ $CLEAN_BUILD -eq 1 ]]; then
    run_cmd "rm -rf build install log"
    append_unique FILES_TOUCHED "$WORKSPACE/build"
    append_unique FILES_TOUCHED "$WORKSPACE/install"
    append_unique FILES_TOUCHED "$WORKSPACE/log"
    log_change "removed build/install/log"
  fi
  capture_target_workspace_env_contamination

  local ros_setup="/opt/ros/humble/setup.bash"
  local allow_override="--allow-overriding ruckig tesseract_monitoring tesseract_msgs tesseract_rosutils"

  run_cmd "unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH"
  run_cmd "set +u; : \"\${AMENT_TRACE_SETUP_FILES:=}\"; source '$ros_setup'; set -u"

  if [[ -n "$EPD_UNDERLAY" ]]; then
    run_cmd "set +u; : \"\${AMENT_TRACE_SETUP_FILES:=}\"; source '$EPD_UNDERLAY/install/local_setup.bash'; set -u"
  fi

  run_cmd "vcs import --recursive --skip-existing src < src/easy_manipulation_deployment/dependencies/emd_epd_ws.repos"
  append_unique REPOS_IMPORTED "dependencies/emd_epd_ws.repos"

  run_cmd "./src/easy_manipulation_deployment/scripts/ensure_rosdep_overrides.sh"
  run_cmd "./src/easy_manipulation_deployment/scripts/fix_workspace_layout.sh $([[ $WITH_GUI -eq 1 ]] && echo --with-gui || true)"
  hide_source_only_vendor_dependencies
  validate_cereal_dependency_handling

  local rosdep_skip_keys=(qt_advanced_docking tesseract_visualization taskflow trajopt_ifopt trajopt_sqp osqp osqp_vendor osqp-eigen osqp_eigen qpoases)
  local fallback_keys=""
  if fallback_keys=$(./src/easy_manipulation_deployment/scripts/preflight_tesseract_apt.sh --ros-distro humble --print-rosdep-skip-keys 2>/dev/null); then
    if [[ -n "$fallback_keys" ]]; then
      IFS=',' read -r -a extra_keys <<< "$fallback_keys"
      rosdep_skip_keys+=("${extra_keys[@]}")
    fi
  fi

  # de-duplicate rosdep skip keys
  local dedup_skip=()
  local key
  for key in "${rosdep_skip_keys[@]}"; do
    [[ -z "$key" ]] && continue
    if ! printf '%s\n' "${dedup_skip[@]}" | grep -Fxq "$key"; then
      dedup_skip+=("$key")
    fi
  done
  RODEP_SKIP_KEYS_USED=("${dedup_skip[@]:-}")

  local rosdep_skip_arg
  rosdep_skip_arg="$(join_by ' ' "${dedup_skip[@]:-}")"

  run_cmd "rosdep install --from-paths src --ignore-src -r -y --rosdistro humble --skip-keys '$rosdep_skip_arg'"

  run_cmd "eval \"\$(./src/easy_manipulation_deployment/scripts/ensure_taskflow_cmake_package.sh --export)\""

  prepare_osqp_stack
  run_cmd "./src/easy_manipulation_deployment/scripts/preflight_trajopt_osqp_compatibility.sh \"$WORKSPACE\""

  # Prefer the pinned OSQP v1 provider installed in /usr/local.  ROS Humble
  # also exports an older OSQP package from /opt/ros/humble; without this
  # ordering, trajopt_sco can cache the incompatible Humble provider.
  local trajopt_sco_cache="$WORKSPACE/build/trajopt_sco/CMakeCache.txt"
  local expected_osqp_dir="/usr/local/lib/cmake/osqp"
  if [[ -f "$trajopt_sco_cache" ]]; then
    local cached_osqp_dir
    cached_osqp_dir="$(grep '^osqp_DIR:' "$trajopt_sco_cache" | tail -n1 | cut -d= -f2- || true)"
    if [[ -n "$cached_osqp_dir" && "$cached_osqp_dir" != "$expected_osqp_dir" ]]; then
      echo "Removing stale trajopt_sco CMake cache using OSQP provider: $cached_osqp_dir"
      run_cmd "rm -rf '$WORKSPACE/build/trajopt_sco' '$WORKSPACE/install/trajopt_sco'"
      log_change "removed stale trajopt_sco cache using incompatible OSQP provider"
    fi
  fi

  # Invalidate only SQP's cache when its provider is stale or the target
  # definitions were repaired.
  local trajopt_sqp_cache="$WORKSPACE/build/trajopt_sqp/CMakeCache.txt"
  local cached_eigen_dir=""
  if [[ -f "$trajopt_sqp_cache" ]]; then
    cached_eigen_dir="$(grep '^OsqpEigen_DIR:' "$trajopt_sqp_cache" | tail -n1 | cut -d= -f2- || true)"
  fi
  if [[ $OSQPEIGEN_REPAIRED -eq 1 ]] ||
    { [[ -n "$cached_eigen_dir" ]] && [[ "$cached_eigen_dir" != "/usr/local/lib/cmake/OsqpEigen" ]]; }; then
    echo "Invalidating only trajopt_sqp cache to bind pinned OsqpEigen."
    run_cmd "rm -rf '$WORKSPACE/build/trajopt_sqp' '$WORKSPACE/install/trajopt_sqp'"
    log_change "repair stale trajopt_sqp OsqpEigen cache"
  fi

  export CMAKE_PREFIX_PATH="/usr/local${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"

  # Ensure Node.js 20 is on PATH for the mandatory Web3D freshness target.
  load_nvm_if_available
  ensure_node20
  capture_epd_state

  run_cmd "./src/easy_manipulation_deployment/scripts/verify_workspace_discovery.sh"

  calculate_colcon_skip_packages "$PROFILE" "$WITH_GUI"

  local colcon_cmd="colcon build --symlink-install $allow_override --cmake-args -DCMAKE_BUILD_TYPE=$CMAKE_BUILD_TYPE"
  if [[ -n "$PARALLEL_WORKERS" ]]; then
    colcon_cmd+=" --parallel-workers $PARALLEL_WORKERS"
  fi

  if [[ ${#COLCON_SKIP_PACKAGES_USED[@]} -gt 0 ]]; then
    colcon_cmd+=" --packages-skip $(join_by ' ' "${COLCON_SKIP_PACKAGES_USED[@]}")"
  fi

  run_cmd "$colcon_cmd"
  capture_emd_epd_integration_flag

  append_unique FILES_TOUCHED "$WORKSPACE/build"
  append_unique FILES_TOUCHED "$WORKSPACE/install"
  append_unique FILES_TOUCHED "$WORKSPACE/log"
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --check-prereqs) CHECK_PREREQS=1; shift ;;
    --install-prereqs) INSTALL_PREREQS=1; shift ;;
    --build) BUILD_WORKSPACE=1; shift ;;
    --workspace) WORKSPACE="${2:-}"; shift 2 ;;
    --profile) PROFILE="${2:-}"; shift 2 ;;
    --with-gui|--with-tesseract-qt) WITH_GUI=1; shift ;;
    --parallel-workers) PARALLEL_WORKERS="${2:-}"; shift 2 ;;
    --cmake-build-type) CMAKE_BUILD_TYPE="${2:-}"; shift 2 ;;
    --epd-underlay) EPD_UNDERLAY="${2:-}"; shift 2 ;;
    --skip-local-proof-packages) SKIP_LOCAL_PROOF_PACKAGES=1; shift ;;
    --clean) CLEAN_BUILD=1; shift ;;
    --dry-run) DRY_RUN=1; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown option: $1" >&2; usage; exit 1 ;;
  esac
done

if [[ -z "$WORKSPACE" ]]; then
  WORKSPACE="$PWD"
fi
WORKSPACE="$(cd "$WORKSPACE" && pwd 2>/dev/null || printf '%s' "$WORKSPACE")"

if [[ "$PROFILE" != "minimal" && "$PROFILE" != "full" ]]; then
  echo "Invalid --profile value: '$PROFILE' (expected: minimal or full)" >&2
  exit 1
fi

if [[ -n "$PARALLEL_WORKERS" ]] && { ! [[ "$PARALLEL_WORKERS" =~ ^[0-9]+$ ]] || [[ "$PARALLEL_WORKERS" == "0" ]]; }; then
  echo "--parallel-workers expects a positive integer" >&2
  exit 1
fi

if [[ "$CMAKE_BUILD_TYPE" != "Release" && "$CMAKE_BUILD_TYPE" != "RelWithDebInfo" && "$CMAKE_BUILD_TYPE" != "Debug" ]]; then
  echo "Invalid --cmake-build-type value: '$CMAKE_BUILD_TYPE' (expected: Release, RelWithDebInfo, or Debug)" >&2
  exit 1
fi

if [[ $CHECK_PREREQS -eq 0 && $INSTALL_PREREQS -eq 0 && $BUILD_WORKSPACE -eq 0 ]]; then
  echo "No phase selected. Choose at least one of --check-prereqs, --install-prereqs, --build." >&2
  usage
  exit 1
fi

if [[ $INSTALL_PREREQS -eq 1 ]]; then
  install_prereqs_phase
fi

if [[ $CHECK_PREREQS -eq 1 ]]; then
  check_prereqs_phase
fi

if [[ $BUILD_WORKSPACE -eq 1 ]]; then
  check_prereqs_phase
  build_phase
fi

emit_summary
