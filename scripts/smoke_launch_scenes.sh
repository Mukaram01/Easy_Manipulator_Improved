#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
LOG_DIR="${SMOKE_LOG_DIR:-${REPO_ROOT}/logs/smoke_launch}"
RESULTS_PATH="${SMOKE_RESULTS_PATH:-${REPO_ROOT}/docs/manuals/latest_smoke_launch_results.tsv}"
SCENE_TIMEOUT_SEC="${SCENE_TIMEOUT_SEC:-30}"

KNOWN_SCENES=(
  ur5_2f_test
  ur5_3f_test
  ur5_airpick4_test
  suction_test
)

DISCOVERED_SCENES=()
if [[ -d "${REPO_ROOT}/scenes" ]]; then
  while IFS= read -r manifest; do
    scene_name="$(basename "$(dirname "${manifest}")")"
    DISCOVERED_SCENES+=("${scene_name}")
  done < <(find "${REPO_ROOT}/scenes" -mindepth 2 -maxdepth 2 -type f \( -name 'scene_manifest.yaml' -o -name 'workcell.yaml' \) | sort)
fi

unique_scenes() {
  awk '!seen[$0]++'
}

if (( $# > 0 )); then
  mapfile -t SCENES_TO_TEST < <(printf '%s\n' "$@" | unique_scenes)
else
  mapfile -t SCENES_TO_TEST < <(
    {
      printf '%s\n' "${DISCOVERED_SCENES[@]:-}"
      printf '%s\n' "${KNOWN_SCENES[@]}"
    } | sed '/^$/d' | unique_scenes
  )
fi

mkdir -p "${LOG_DIR}" "$(dirname "${RESULTS_PATH}")"
run_stamp="$(date -u +"%Y%m%dT%H%M%SZ")"
run_dir="${LOG_DIR}/${run_stamp}"
mkdir -p "${run_dir}"

printf '\n=== run_grasp_execution scene smoke launch ===\n'
printf 'Repository: %s\n' "${REPO_ROOT}"
printf 'Timestamp (UTC): %s\n' "$(date -u +"%Y-%m-%dT%H:%M:%SZ")"
printf 'Timeout per scene: %ss\n' "${SCENE_TIMEOUT_SEC}"
printf 'Log directory: %s\n' "${run_dir}"
printf 'Results file: %s\n\n' "${RESULTS_PATH}"

if ! command -v ros2 >/dev/null 2>&1; then
  echo "ROS check: ros2 command not found. Marking all scenes SKIP."
  : > "${RESULTS_PATH}"
  for scene in "${SCENES_TO_TEST[@]}"; do
    printf '%s\tSKIP\t\tros2 command not found\t\n' "${scene}" >> "${RESULTS_PATH}"
    printf 'SKIP %-24s ros2 command not found\n' "${scene}"
  done
  exit 0
fi

echo "ROS check: ros2 command found."
if [[ -n "${AMENT_PREFIX_PATH:-}" ]]; then
  echo "ROS check: AMENT_PREFIX_PATH is set."
else
  echo "ROS check: AMENT_PREFIX_PATH is not set; workspace may be unsourced (scene discovery/install checks can SKIP)."
fi

declare -a REQUIRED_MARKERS=(
  "Generated workcell context for scene"
  "Trajectory execution is managing controllers"
  "Startup readiness profile selected"
)
declare -a WORLD_MARKERS=(
  "World geometry monitor not started"
  "Starting world geometry update monitor"
)

shutdown_pid() {
  local pid="$1"
  if kill -0 "${pid}" >/dev/null 2>&1; then
    kill -INT "${pid}" >/dev/null 2>&1 || true
    for _ in {1..8}; do
      if ! kill -0 "${pid}" >/dev/null 2>&1; then
        return
      fi
      sleep 0.5
    done
    kill -TERM "${pid}" >/dev/null 2>&1 || true
    for _ in {1..6}; do
      if ! kill -0 "${pid}" >/dev/null 2>&1; then
        return
      fi
      sleep 0.5
    done
    kill -KILL "${pid}" >/dev/null 2>&1 || true
  fi
}

contains_marker() {
  local marker="$1"
  local logfile="$2"
  grep -Fq "${marker}" "${logfile}"
}

result_failures=0
: > "${RESULTS_PATH}"

for scene in "${SCENES_TO_TEST[@]}"; do
  echo
  echo "--- Scene: ${scene} ---"

  scene_log="${run_dir}/${scene}.log"
  if ! ros2 pkg prefix "${scene}" >/dev/null 2>&1; then
    echo "SKIP ${scene}: package not discoverable in current environment."
    printf '%s\tSKIP\t\tScene package not discoverable via ros2 pkg prefix\t%s\n' "${scene}" "${scene_log}" >> "${RESULTS_PATH}"
    continue
  fi

  echo "Launching headless smoke command..."
  cmd=(
    ros2 launch run_grasp_execution grasp_execution.launch.py
    "scene_package:=${scene}"
    "launch_rviz:=false"
  )

  set +e
  "${cmd[@]}" >"${scene_log}" 2>&1 &
  pid=$!
  set -e

  start_epoch="$(date +%s)"
  ready=false

  while true; do
    now="$(date +%s)"
    elapsed=$(( now - start_epoch ))

    if ! kill -0 "${pid}" >/dev/null 2>&1; then
      break
    fi

    all_required=true
    for marker in "${REQUIRED_MARKERS[@]}"; do
      if ! contains_marker "${marker}" "${scene_log}"; then
        all_required=false
        break
      fi
    done

    world_ok=false
    for marker in "${WORLD_MARKERS[@]}"; do
      if contains_marker "${marker}" "${scene_log}"; then
        world_ok=true
        break
      fi
    done

    if [[ "${all_required}" == true && "${world_ok}" == true ]]; then
      ready=true
      break
    fi

    if (( elapsed >= SCENE_TIMEOUT_SEC )); then
      break
    fi

    sleep 1
  done

  marker_hits=()
  for marker in "${REQUIRED_MARKERS[@]}" "${WORLD_MARKERS[@]}"; do
    if contains_marker "${marker}" "${scene_log}"; then
      marker_hits+=("${marker}")
    fi
  done
  marker_text=""
  if (( ${#marker_hits[@]} > 0 )); then
    marker_text="$(printf '%s; ' "${marker_hits[@]}")"
    marker_text="${marker_text%; }"
  fi

  if [[ "${ready}" == true ]]; then
    shutdown_pid "${pid}"
    wait "${pid}" >/dev/null 2>&1 || true
    echo "PASS ${scene}: readiness markers detected and launch shut down cleanly."
    printf '%s\tPASS\t%s\tReadiness markers detected before timeout\t%s\n' "${scene}" "${marker_text}" "${scene_log}" >> "${RESULTS_PATH}"
  else
    shutdown_pid "${pid}"
    wait "${pid}" >/dev/null 2>&1 || true

    if (( elapsed >= SCENE_TIMEOUT_SEC )); then
      note="Timeout waiting for readiness markers (${SCENE_TIMEOUT_SEC}s)"
    else
      note="Launch exited before readiness markers were fully detected"
    fi

    echo "FAIL ${scene}: ${note}."
    printf '%s\tFAIL\t%s\t%s\t%s\n' "${scene}" "${marker_text}" "${note}" "${scene_log}" >> "${RESULTS_PATH}"
    result_failures=$((result_failures + 1))
  fi

done

echo
printf 'Smoke results written to: %s\n' "${RESULTS_PATH}"
if (( result_failures > 0 )); then
  echo "Smoke launch failed for ${result_failures} discoverable scene(s)."
  exit 1
fi

echo "Smoke launch passed for all discoverable scenes."
exit 0
