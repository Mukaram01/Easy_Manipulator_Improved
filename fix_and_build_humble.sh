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

MUTATING_ACTIONS=()
PACKAGES_INSTALLED=()
PIP_PACKAGES_INSTALLED=()
REPOS_IMPORTED=()
FILES_TOUCHED=()

usage() {
  cat <<'USAGE'
Usage: ./fix_and_build_humble.sh [options]

Phase options (choose one or more):
  --check-prereqs            Read-only checks. Verifies tooling and ROS prerequisites.
  --install-prereqs          Mutating phase. Installs missing apt/pip prerequisites.
  --build                    Build phase only. Runs colcon build in the workspace.

General options:
  --workspace <path>         Workspace root (default: current working directory)
  --profile minimal|full     Build profile (default: minimal)
  --with-gui                 Build optional GUI packages in full profile
  --clean                    Remove build/, install/, and log/ before build
  --dry-run                  Print commands without executing
  -h, --help                 Show this help message

Examples:
  # Local dev: validate, install missing tools, then build
  ./fix_and_build_humble.sh --workspace ~/workcell_ws --check-prereqs --install-prereqs --build --profile full

  # Locked-down enterprise host: read-only preflight only (no apt/pip mutation)
  ./fix_and_build_humble.sh --workspace /opt/workcell_ws --check-prereqs

  # CI runner: deterministic build from pre-provisioned image
  ./fix_and_build_humble.sh --workspace "$GITHUB_WORKSPACE" --check-prereqs --build --profile minimal --clean
USAGE
}

log_change() {
  MUTATING_ACTIONS+=("$1")
}

run_cmd() {
  local cmd="$*"
  if [[ $DRY_RUN -eq 1 ]]; then
    printf '[dry-run] %s\n' "$cmd"
    return 0
  fi
  eval "$cmd"
}

append_unique() {
  local -n arr_ref=$1
  local value="$2"
  local existing
  for existing in "${arr_ref[@]:-}"; do
    [[ "$existing" == "$value" ]] && return 0
  done
  arr_ref+=("$value")
}

emit_summary() {
  local summary_file="$WORKSPACE/fix_and_build_summary.json"
  local mut_json pkg_json pip_json repo_json file_json

  mut_json=$(printf '%s\n' "${MUTATING_ACTIONS[@]:-}" | python3 -c 'import json,sys; print(json.dumps([l for l in sys.stdin.read().splitlines() if l]))')
  pkg_json=$(printf '%s\n' "${PACKAGES_INSTALLED[@]:-}" | python3 -c 'import json,sys; print(json.dumps([l for l in sys.stdin.read().splitlines() if l]))')
  pip_json=$(printf '%s\n' "${PIP_PACKAGES_INSTALLED[@]:-}" | python3 -c 'import json,sys; print(json.dumps([l for l in sys.stdin.read().splitlines() if l]))')
  repo_json=$(printf '%s\n' "${REPOS_IMPORTED[@]:-}" | python3 -c 'import json,sys; print(json.dumps([l for l in sys.stdin.read().splitlines() if l]))')
  file_json=$(printf '%s\n' "${FILES_TOUCHED[@]:-}" | python3 -c 'import json,sys; print(json.dumps([l for l in sys.stdin.read().splitlines() if l]))')

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
  "mutating_actions": ${mut_json},
  "packages_installed": ${pkg_json},
  "pip_packages_installed": ${pip_json},
  "repos_imported": ${repo_json},
  "files_touched": ${file_json}
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

need_command() {
  local cmd="$1"
  if ! command -v "$cmd" >/dev/null 2>&1; then
    return 1
  fi
}

apt_install_package() {
  local pkg="$1"
  local apt_prefix=""
  if command -v sudo >/dev/null 2>&1; then
    apt_prefix="sudo "
  elif [[ ${EUID:-$(id -u)} -ne 0 ]]; then
    echo "Cannot install '$pkg': requires root or sudo. Re-run with sudo access or preinstall package manually." >&2
    exit 1
  fi

  run_cmd "${apt_prefix}apt-get update -y"
  run_cmd "${apt_prefix}apt-get install -y $pkg"
  append_unique PACKAGES_INSTALLED "$pkg"
  log_change "apt install $pkg"
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

  local pkg
  for pkg in "${missing_tools[@]:-}"; do
    apt_install_package "$pkg"
  done

  if ! need_command colcon; then
    if ! need_command pip3; then
      apt_install_package python3-pip
    fi
    run_cmd "python3 -m pip install -U colcon-common-extensions"
    append_unique PIP_PACKAGES_INSTALLED "colcon-common-extensions"
    log_change "pip install colcon-common-extensions"
  fi

  if need_command rosdep; then
    run_cmd "rosdep init || true"
    log_change "rosdep init"
  fi
}

check_prereqs_phase() {
  local missing=()
  local ros_setup="/opt/ros/humble/setup.bash"

  [[ -d "$WORKSPACE" ]] || missing+=("workspace directory '$WORKSPACE' does not exist")
  [[ -d "$WORKSPACE/src" ]] || missing+=("workspace is missing src/ directory at '$WORKSPACE/src'")
  [[ -f "$ros_setup" ]] || missing+=("missing ROS setup: $ros_setup")

  need_command git || missing+=("missing command: git (install package: git)")
  need_command vcs || missing+=("missing command: vcs (install package: python3-vcstool)")
  need_command rosdep || missing+=("missing command: rosdep (install package: python3-rosdep)")
  need_command colcon || missing+=("missing command: colcon (install via pip: colcon-common-extensions)")

  if ! python3 - <<'PY' >/dev/null 2>&1
import yaml  # noqa: F401
PY
  then
    missing+=("missing Python module: yaml (install package: python3-yaml)")
  fi

  if [[ ${#missing[@]} -gt 0 ]]; then
    echo "Prerequisite check failed. Missing requirements:" >&2
    printf '  - %s\n' "${missing[@]}" >&2
    echo "Action: run --install-prereqs to allow this script to install missing tools, or install them manually." >&2
    exit 1
  fi

  echo "Prerequisite check passed for workspace: $WORKSPACE"
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

  local ros_setup="/opt/ros/humble/setup.bash"
  local build_cmd

  if [[ "$PROFILE" == "full" && $WITH_GUI -eq 0 ]]; then
    build_cmd="source '$ros_setup' && colcon build --symlink-install --packages-skip tesseract_qt qtadvanceddocking QtADS tesseract_rviz"
  else
    build_cmd="source '$ros_setup' && colcon build --symlink-install"
  fi

  run_cmd "$build_cmd"
  append_unique FILES_TOUCHED "$WORKSPACE/build"
  append_unique FILES_TOUCHED "$WORKSPACE/install"
  append_unique FILES_TOUCHED "$WORKSPACE/log"
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --check-prereqs)
      CHECK_PREREQS=1
      shift
      ;;
    --install-prereqs)
      INSTALL_PREREQS=1
      shift
      ;;
    --build)
      BUILD_WORKSPACE=1
      shift
      ;;
    --workspace)
      WORKSPACE="${2:-}"
      shift 2
      ;;
    --profile)
      PROFILE="${2:-}"
      shift 2
      ;;
    --with-gui|--with-tesseract-qt)
      WITH_GUI=1
      shift
      ;;
    --clean)
      CLEAN_BUILD=1
      shift
      ;;
    --dry-run)
      DRY_RUN=1
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown option: $1" >&2
      usage
      exit 1
      ;;
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
