#!/usr/bin/env bash
set -euo pipefail

ROS_DISTRO_NAME="${ROS_DISTRO:-humble}"
QUIET=0
PRINT_ROSDEP_SKIP_KEYS=0

usage() {
  cat <<'USAGE'
Usage: preflight_tesseract_apt.sh [options]

Checks whether ROS Tesseract binary packages and QtADS-related dependencies are
available from configured APT repositories before running rosdep.

Options:
  --ros-distro <name>         Override ROS distro (default: $ROS_DISTRO or humble)
  --print-rosdep-skip-keys    Print comma-separated rosdep skip keys for fallback path
  --quiet                     Suppress informational logs (still prints skip keys if requested)
  -h, --help                  Show this help text
USAGE
}

log_info() {
  [[ $QUIET -eq 1 ]] && return
  echo "[INFO] $*"
}

log_warn() {
  [[ $QUIET -eq 1 ]] && return
  echo "[WARN] $*" >&2
}

candidate_available() {
  local pkg="$1"
  local candidate
  candidate="$(apt-cache policy "$pkg" 2>/dev/null | awk '/Candidate:/ {print $2; exit}')"
  [[ -n "$candidate" && "$candidate" != "(none)" ]]
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --ros-distro)
      ROS_DISTRO_NAME="${2:-}"
      shift 2
      ;;
    --print-rosdep-skip-keys)
      PRINT_ROSDEP_SKIP_KEYS=1
      shift
      ;;
    --quiet)
      QUIET=1
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

required_tesseract_packages=(
  "ros-${ROS_DISTRO_NAME}-tesseract"
  "ros-${ROS_DISTRO_NAME}-tesseract-environment"
  "ros-${ROS_DISTRO_NAME}-tesseract-motion-planners"
)

qtads_related_packages=(
  "libqtads-dev"
  "ros-${ROS_DISTRO_NAME}-tesseract-visualization"
)

mapfile -t tesseract_matches < <(apt-cache pkgnames "ros-${ROS_DISTRO_NAME}-tesseract-" | sort -u || true)

missing=()
for pkg in "${required_tesseract_packages[@]}" "${qtads_related_packages[@]}"; do
  if ! candidate_available "$pkg"; then
    missing+=("$pkg")
  fi
done

fallback_skip_keys=(
  qt_advanced_docking
  tesseract_visualization
)

if [[ ${#tesseract_matches[@]} -eq 0 ]]; then
  missing+=("ros-${ROS_DISTRO_NAME}-tesseract-* (no matching package names found)")
fi

if [[ ${#missing[@]} -eq 0 ]]; then
  log_info "APT preflight passed: ROS ${ROS_DISTRO_NAME} Tesseract and QtADS-related packages are discoverable."
  if [[ $PRINT_ROSDEP_SKIP_KEYS -eq 1 ]]; then
    echo ""
  fi
  exit 0
fi

log_warn "APT preflight detected missing package candidates:"
for pkg in "${missing[@]}"; do
  log_warn "  - $pkg"
done
log_warn "Using supported fallback path: build Tesseract/QtADS from source overlay (dependencies/emd_epd_ws.repos)."
log_warn "rosdep should skip keys that assume binary providers for the missing GUI/Qt packages."

if [[ $PRINT_ROSDEP_SKIP_KEYS -eq 1 ]]; then
  (IFS=,; echo "${fallback_skip_keys[*]}")
fi

exit 2
