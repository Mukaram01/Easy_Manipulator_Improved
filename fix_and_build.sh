#!/usr/bin/env bash
set -euo pipefail

# Workspace repair and build pipeline for easy_manipulation_deployment.
# 1. Verifies the repository is checked out at $WS/src/easy_manipulation_deployment and
#    removes stray external Tesseract clones, warning if conflicting packages persist.
# 2. Scrubs every build/install/log directory so diagnostics always reflect the latest run.
# 3. Installs dependencies, performs a single colcon build with override flags, and then
#    inspects exported CMake namespaces so configuration regressions fail fast.

WS=${WS:-"$HOME/workcell_ws"}
SRC="$WS/src"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

mkdir -p "$SRC"

if [[ "$SCRIPT_DIR" != "$SRC/easy_manipulation_deployment" ]]; then
  echo "Expected repository at $SRC/easy_manipulation_deployment" >&2
  exit 1
fi

if [[ -d "$SRC/tesseract" && -d "$SRC/easy_manipulation_deployment/tesseract" ]]; then
  rm -rf "$SRC/tesseract"
fi

mapfile -t COLCON_LINES < <(colcon list --base-paths "$SRC" || true)
declare -A KEPT_PATH
removed_duplicate=false
if [[ ${#COLCON_LINES[@]} -gt 0 ]]; then
  for entry in "${COLCON_LINES[@]}"; do
    name=${entry%% *}
    path=${entry#* }
    path=${path%% *}
    [[ -z "$name" || -z "$path" ]] && continue

    existing=${KEPT_PATH[$name]:-}
    if [[ -z "$existing" ]]; then
      KEPT_PATH[$name]="$path"
      continue
    fi

    if [[ "$existing" == "$path" ]]; then
      continue
    fi

    choose_keep="$existing"
    candidate_remove="$path"
    if [[ "$path" == "$SRC/easy_manipulation_deployment"* ]]; then
      choose_keep="$path"
      candidate_remove="$existing"
    fi

    rel_remove=${candidate_remove#"$SRC/"}
    top_dir=${rel_remove%%/*}
    if [[ -n "$top_dir" && "$top_dir" != "easy_manipulation_deployment" ]]; then
      rm -rf "$SRC/$top_dir"
      removed_duplicate=true
    fi
    KEPT_PATH[$name]="$choose_keep"
  done
fi

if [[ "$removed_duplicate" == true ]]; then
  echo "⚠️ Duplicate detected — removed external clone"
else
  echo "✅ Single Tesseract source detected"
fi

mapfile -t COLCON_LINES < <(colcon list --base-paths "$SRC")
declare -A PACKAGE_COUNTS=( [tesseract_common]=0 [tesseract_geometry]=0 [tesseract_scene_graph]=0 )
for entry in "${COLCON_LINES[@]}"; do
  name=${entry%% *}
  if [[ -n ${PACKAGE_COUNTS[$name]+set} ]]; then
    ((PACKAGE_COUNTS[$name]++))
  fi
done

for pkg in "${!PACKAGE_COUNTS[@]}"; do
  if (( PACKAGE_COUNTS[$pkg] > 1 )); then
    echo "Warning: multiple visible copies of $pkg after cleanup" >&2
    exit 1
  fi
done

rm -rf "$WS/build" "$WS/install" "$WS/log"
find "$SRC" -type d \( -name build -o -name install -o -name log \) -prune -exec rm -rf {} +
echo "🧹 Cleaned build/install/log"

candidate_distros=()
if [[ -n ${ROS_DISTRO:-} ]]; then
  candidate_distros+=("$ROS_DISTRO")
fi
candidate_distros+=(humble jazzy)
for distro in "${candidate_distros[@]}"; do
  if [[ -f "/opt/ros/$distro/setup.bash" ]]; then
    ROS_DISTRO="$distro"
    break
  fi
done

if [[ -z ${ROS_DISTRO:-} ]]; then
  echo "Unable to locate a ROS distribution (expected humble or jazzy)" >&2
  exit 1
fi

set +u
# shellcheck source=/dev/null
source "/opt/ros/$ROS_DISTRO/setup.bash"
set -u

REPO_FILE="$SCRIPT_DIR/tesseract.repos"
if [[ -f "$REPO_FILE" ]]; then
  mapfile -t IMPORT_TARGETS < <(python3 - <<'PY' "$REPO_FILE"
import re
import sys

repo_file = sys.argv[1]
targets = []
with open(repo_file, encoding="utf-8") as fh:
    for line in fh:
        if line.lstrip().startswith('#'):
            continue
        match = re.match(r"\s{2}([\w.-]+):\s*$", line)
        if match:
            targets.append(match.group(1))
print("\n".join(targets))
PY
  )

  missing_targets=()
  for repo_name in "${IMPORT_TARGETS[@]}"; do
    target_dir="$SRC/$repo_name"
    if [[ -d "$target_dir" ]]; then
      if [[ ! -e "$target_dir/.git" ]]; then
        echo "Removing leftover non-repository directory $target_dir"
        rm -rf "$target_dir"
        missing_targets+=("$repo_name")
      fi
    else
      missing_targets+=("$repo_name")
    fi
  done

  if (( ${#missing_targets[@]} )); then
    echo "Fetching missing repositories from $REPO_FILE: ${missing_targets[*]}"
    vcs import --recursive "$SRC" < "$REPO_FILE"
  else
    echo "All repositories from $REPO_FILE already present"
  fi

  for overlay in tesseract trajopt; do
    if [[ -d "$SCRIPT_DIR/$overlay" ]]; then
      mkdir -p "$SRC/$overlay"
      cp -a "$SCRIPT_DIR/$overlay/." "$SRC/$overlay/"
      while IFS= read -r -d '' marker; do
        echo "Renaming ignore marker $marker"
        mv -f "$marker" "$marker.repo"
      done < <(find "$SRC/$overlay" -maxdepth 2 \( -name 'COLCON_IGNORE' -o -name 'AMENT_IGNORE' \) -print0)
    fi
  done
fi

if [[ ! -d "$SRC/boost_plugin_loader" ]]; then
  git -C "$SRC" clone https://github.com/tesseract-robotics/boost_plugin_loader.git
fi

mapfile -t PACKAGE_PATHS < <(colcon list --base-paths "$SRC" --paths-only)
if [[ ${#PACKAGE_PATHS[@]} -eq 0 ]]; then
  echo "No packages discovered for rosdep installation" >&2
  exit 1
fi

rosdep install --from-paths "${PACKAGE_PATHS[@]}" --ignore-src -yr --rosdistro "$ROS_DISTRO" \
  --skip-keys "tesseract_rviz tesseract_ros_examples"

echo "🚀 Building with --allow-overriding eigen tesseract_common boost_plugin_loader"
colcon build --merge-install --event-handlers console_direct+ --allow-overriding eigen tesseract_common boost_plugin_loader \
  --cmake-args -DBUILD_SHARED_LIBS=ON -DTESSERACT_ENABLE_TESTING=OFF

if [[ ! -f "$WS/install/setup.bash" ]]; then
  echo "Expected merged install at $WS/install/setup.bash" >&2
  exit 1
fi

set +u
# shellcheck source=/dev/null
source "$WS/install/setup.bash"
set -u

COMMON_CONFIG="$WS/install/share/tesseract_common/cmake/tesseract_commonConfig.cmake"
GEOMETRY_CONFIG="$WS/install/share/tesseract_geometry/cmake/tesseract_geometryConfig.cmake"

for config in "$COMMON_CONFIG" "$GEOMETRY_CONFIG"; do
  if [[ ! -f "$config" ]]; then
    echo "Missing expected config file: $config" >&2
    exit 1
  fi
done

check_namespace() {
  local file="$1"
  local pkg_label="$2"
  local alt_namespace="$3"
  if grep -q "tesseract::" "$file"; then
    echo "$pkg_label exports tesseract:: namespace"
  elif grep -q "$alt_namespace::" "$file"; then
    echo "$pkg_label exports $alt_namespace:: namespace"
  else
    echo "$pkg_label missing expected namespace" >&2
    return 1
  fi
}

check_namespace "$COMMON_CONFIG" "tesseract_commonConfig.cmake" "tesseract_common"
check_namespace "$GEOMETRY_CONFIG" "tesseract_geometryConfig.cmake" "tesseract_geometry"
