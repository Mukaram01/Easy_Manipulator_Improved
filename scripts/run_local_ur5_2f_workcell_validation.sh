#!/usr/bin/env bash
# Local Workcell Studio evidence runner for a real ROS 2 Humble workspace.
# Safe-by-default: this script builds Workcell Builder, runs GUI smoke evidence,
# and runs the offline readiness matrix. It never launches robot drivers and it
# only records fake-hardware-first validation evidence.

set -u
set -o pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEFAULT_REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
if [[ -d "${PWD}/scripts" && -f "${PWD}/scripts/run_workcell_builder_scene3d_gui_smoke.py" ]]; then
  DEFAULT_REPO_ROOT="${PWD}"
fi

REPO_ROOT="${DEFAULT_REPO_ROOT}"
WORKSPACE_ROOT="/home/user/workcell_ws"
SCENE="ur5_2f_test"
OUTPUT_DIR="build/workcell_studio/local_validation/ur5_2f_test"
TIMEOUT_SEC="30"

usage() {
  cat <<USAGE
Usage: bash scripts/run_local_ur5_2f_workcell_validation.sh [options]

Collect local ROS Humble Workcell Studio evidence for a supported scene.

Options:
  --repo-root PATH        Repository root (default: current repo/script parent)
  --workspace-root PATH   ROS workspace root (default: /home/user/workcell_ws)
  --scene NAME            Scene name (default: ur5_2f_test)
  --output-dir PATH       Evidence output directory (default: build/workcell_studio/local_validation/ur5_2f_test)
  --timeout-sec SECONDS   Scene3D GUI smoke timeout (default: 30)
  -h, --help              Show this help

Safety:
  - Sources /opt/ros/humble/setup.bash only when present.
  - Builds only the workcell_builder package.
  - Runs Workcell Builder Scene3D smoke and the offline readiness matrix.
  - Does not run ros2 launch, does not launch real robot drivers, and does not
    enable real robot motion.
USAGE
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --repo-root)
      REPO_ROOT="$2"; shift 2 ;;
    --workspace-root)
      WORKSPACE_ROOT="$2"; shift 2 ;;
    --scene)
      SCENE="$2"; shift 2 ;;
    --output-dir)
      OUTPUT_DIR="$2"; shift 2 ;;
    --timeout-sec)
      TIMEOUT_SEC="$2"; shift 2 ;;
    -h|--help)
      usage; exit 0 ;;
    *)
      echo "Unknown argument: $1" >&2
      usage >&2
      exit 64 ;;
  esac
done

REPO_ROOT="$(cd "${REPO_ROOT}" 2>/dev/null && pwd)" || {
  echo "FAIL: repo root does not exist: ${REPO_ROOT}" >&2
  exit 1
}

if [[ "${OUTPUT_DIR}" != /* ]]; then
  OUTPUT_DIR="${REPO_ROOT}/${OUTPUT_DIR}"
fi
mkdir -p "${OUTPUT_DIR}"
OUTPUT_DIR="$(cd "${OUTPUT_DIR}" && pwd)"

BUILD_LOG="${OUTPUT_DIR}/build_workcell_builder.log"
SMOKE_JSON="${OUTPUT_DIR}/scene3d_gui_smoke.json"
SMOKE_PNG="${OUTPUT_DIR}/scene3d_gui_smoke.png"
READINESS_STDOUT="${OUTPUT_DIR}/readiness_matrix_stdout.log"
READINESS_STDERR="${OUTPUT_DIR}/readiness_matrix_stderr.log"
READINESS_DIR="${OUTPUT_DIR}/readiness_matrix"
READINESS_SUMMARY="${OUTPUT_DIR}/readiness_summary.json"
SUMMARY_MD="${OUTPUT_DIR}/validation_summary.md"

STATUS="BLOCKED"
REASON="not_started"
BUILD_RC=""
SMOKE_RC=""
READINESS_RC=""
SMOKE_STATUS="MISSING"
ROS_SETUP="/opt/ros/humble/setup.bash"

quote_cmd() {
  printf '%q ' "$@"
}

source_setup_safely() {
  local setup_path="$1"
  local restore_nounset=0
  local source_rc=0
  case "$-" in
    *u*)
      restore_nounset=1
      set +u
      ;;
  esac
  # ROS/ament setup files may legitimately inspect variables that are unset in
  # the caller. Temporarily disable nounset while sourcing, then restore the
  # runner's strict setting immediately afterwards.
  # shellcheck disable=SC1090
  source "${setup_path}" || source_rc=$?
  if [[ ${restore_nounset} -eq 1 ]]; then
    set -u
  fi
  return "${source_rc}"
}

write_blocked_smoke_json() {
  local blocker="$1"
  local message="$2"
  python3 - "$SMOKE_JSON" "$SCENE" "$REPO_ROOT" "$WORKSPACE_ROOT" "$blocker" "$message" <<'PY'
import json, sys
path, scene, repo, workspace, blocker, message = sys.argv[1:]
payload = {
    "schema": "workcell_studio_scene3d_gui_smoke/v1",
    "status": "BLOCKED",
    "scene": scene,
    "repo_root": repo,
    "workspace_root": workspace,
    "runtime_available": False,
    "screenshot_available": False,
    "blockers": [blocker],
    "blocker_messages": {blocker: message},
    "safety": {
        "fake_hardware_only": True,
        "real_robot_drivers_launched": False,
        "ros2_launch_invoked": False,
    },
}
with open(path, "w", encoding="utf-8") as f:
    json.dump(payload, f, indent=2, sort_keys=True)
    f.write("\n")
PY
}

read_smoke_status() {
  if [[ ! -f "${SMOKE_JSON}" ]]; then
    echo "MISSING"
    return 0
  fi
  python3 - "$SMOKE_JSON" <<'PY'
import json, sys
try:
    with open(sys.argv[1], encoding="utf-8") as f:
        payload = json.load(f)
    print(str(payload.get("status") or payload.get("smoke_status") or "UNKNOWN").upper())
except Exception:
    print("INVALID_JSON")
PY
}

write_summary() {
  local command_line bt fence
  bt="$(printf '\140')"
  fence="${bt}${bt}${bt}"
  command_line="bash scripts/run_local_ur5_2f_workcell_validation.sh --workspace-root ${WORKSPACE_ROOT} --scene ${SCENE} --output-dir ${OUTPUT_DIR} --timeout-sec ${TIMEOUT_SEC}"
  cat > "${SUMMARY_MD}" <<SUMMARY
# Local Workcell Studio validation: ${SCENE}

## Result

- Status: **${STATUS}**
- Reason: ${REASON}
- Scene: ${bt}${SCENE}${bt}
- Repository root: ${bt}${REPO_ROOT}${bt}
- Workspace root: ${bt}${WORKSPACE_ROOT}${bt}
- Output directory: ${bt}${OUTPUT_DIR}${bt}
- Fake hardware only: **yes**
- Real robot drivers launched: **no**
- ros2 launch invoked: **no**

## Evidence files

- ${bt}build_workcell_builder.log${bt}
- ${bt}scene3d_gui_smoke.json${bt} ($([[ -f "${SMOKE_JSON}" ]] && echo present || echo missing))
- ${bt}scene3d_gui_smoke.png${bt} ($([[ -f "${SMOKE_PNG}" ]] && echo present || echo missing))
- ${bt}readiness_matrix_stdout.log${bt}
- ${bt}readiness_matrix_stderr.log${bt}
- ${bt}readiness_summary.json${bt} ($([[ -f "${READINESS_SUMMARY}" ]] && echo present || echo missing))

## Commands run

1. ${bt}source /opt/ros/humble/setup.bash${bt} when available
2. ${bt}colcon build --symlink-install --packages-select workcell_builder${bt}
3. ${bt}source ${WORKSPACE_ROOT}/install/setup.bash${bt}
4. ${bt}python3 scripts/run_workcell_builder_scene3d_gui_smoke.py --repo-root ${REPO_ROOT} --workspace-root ${WORKSPACE_ROOT} --scene ${SCENE} --output ${SMOKE_JSON} --screenshot ${SMOKE_PNG} --timeout-sec ${TIMEOUT_SEC}${bt}
5. ${bt}python3 scripts/run_workcell_studio_scene_readiness_matrix.py --repo-root ${REPO_ROOT} --workspace-root ${WORKSPACE_ROOT} --output-dir ${READINESS_DIR}${bt}

## Next manual GUI check

Run this in the real ROS Humble workspace after collecting automated evidence:

${fence}bash
cd ${WORKSPACE_ROOT}
source /opt/ros/humble/setup.bash
source install/setup.bash
workcell_builder
${fence}

Then manually verify:

1. Open Workcell Builder.
2. Open ${bt}${SCENE}${bt}.
3. Confirm the robot, gripper, and environment are visible.
4. Create an editable layout from the preview.
5. Move an editable item.
6. Save the layout.
7. Close and reopen Workcell Builder.
8. Confirm the moved item persists.

## Re-run command

${fence}bash
cd ${REPO_ROOT}
${command_line}
${fence}
SUMMARY
}

finish() {
  write_summary
  cat <<REPORT

Local Workcell Studio validation result: ${STATUS}
Reason: ${REASON}
Evidence directory: ${OUTPUT_DIR}
Summary: ${SUMMARY_MD}

Next manual GUI check:
  cd ${WORKSPACE_ROOT}
  source /opt/ros/humble/setup.bash
  source install/setup.bash
  workcell_builder
  Then open ${SCENE}; confirm robot/gripper/environment visible; create editable layout from preview; move an editable item; save layout; close/reopen; confirm moved item persists.
REPORT
  case "${STATUS}" in
    PASS) exit 0 ;;
    FAIL) exit 1 ;;
    BLOCKED) exit 2 ;;
    *) exit 1 ;;
  esac
}

if [[ ! -f "${REPO_ROOT}/scripts/run_workcell_builder_scene3d_gui_smoke.py" ]]; then
  STATUS="FAIL"
  REASON="repo_root_missing_scene3d_smoke_runner"
  echo "FAIL: missing ${REPO_ROOT}/scripts/run_workcell_builder_scene3d_gui_smoke.py" | tee "${BUILD_LOG}" >&2
  finish
fi
if [[ ! -f "${REPO_ROOT}/scripts/run_workcell_studio_scene_readiness_matrix.py" ]]; then
  STATUS="FAIL"
  REASON="repo_root_missing_readiness_matrix_runner"
  echo "FAIL: missing ${REPO_ROOT}/scripts/run_workcell_studio_scene_readiness_matrix.py" | tee "${BUILD_LOG}" >&2
  finish
fi

{
  echo "Local Workcell Studio validation for ${SCENE}"
  echo "repo_root=${REPO_ROOT}"
  echo "workspace_root=${WORKSPACE_ROOT}"
  echo "output_dir=${OUTPUT_DIR}"
  echo "safety=fake_hardware_only; no ros2 launch; no real robot drivers"
  echo
} > "${BUILD_LOG}"

if [[ -f "${ROS_SETUP}" ]]; then
  echo "Sourcing ${ROS_SETUP}" | tee -a "${BUILD_LOG}"
  if ! source_setup_safely "${ROS_SETUP}"; then
    STATUS="BLOCKED"
    REASON="ROS Humble setup failed while sourcing ${ROS_SETUP}"
    echo "BLOCKED: ${REASON}" | tee -a "${BUILD_LOG}" >&2
    write_blocked_smoke_json "ros_humble_setup_failed" "${REASON}"
    finish
  fi
elif [[ "${ROS_DISTRO:-}" == "humble" ]]; then
  echo "ROS_DISTRO=humble already present; ${ROS_SETUP} not sourced" | tee -a "${BUILD_LOG}"
else
  STATUS="BLOCKED"
  REASON="ROS Humble is missing: ${ROS_SETUP} was not found and ROS_DISTRO is not humble"
  echo "BLOCKED: ${REASON}" | tee -a "${BUILD_LOG}" >&2
  echo "Readiness matrix skipped because ROS Humble is unavailable." > "${READINESS_STDOUT}"
  : > "${READINESS_STDERR}"
  write_blocked_smoke_json "ros_humble_missing" "${REASON}"
  finish
fi

if ! command -v colcon >/dev/null 2>&1; then
  echo "colcon command is not available in PATH after sourcing ROS Humble." | tee -a "${BUILD_LOG}" >&2
fi

{
  echo
  echo "Running: colcon build --symlink-install --packages-select workcell_builder"
} >> "${BUILD_LOG}"
(
  cd "${WORKSPACE_ROOT}" && colcon build --symlink-install --packages-select workcell_builder
) >> "${BUILD_LOG}" 2>&1
BUILD_RC=$?
if [[ ${BUILD_RC} -ne 0 ]]; then
  STATUS="FAIL"
  REASON="workcell_builder build failed with exit code ${BUILD_RC}"
  echo "FAIL: ${REASON}; see ${BUILD_LOG}" >&2
  finish
fi

INSTALL_SETUP="${WORKSPACE_ROOT}/install/setup.bash"
if [[ -f "${INSTALL_SETUP}" ]]; then
  echo "Sourcing ${INSTALL_SETUP}" >> "${BUILD_LOG}"
  if ! source_setup_safely "${INSTALL_SETUP}"; then
    STATUS="BLOCKED"
    REASON="workspace install setup failed while sourcing ${INSTALL_SETUP}"
    echo "BLOCKED: ${REASON}" | tee -a "${BUILD_LOG}" >&2
    write_blocked_smoke_json "workspace_install_setup_failed" "${REASON}"
    finish
  fi
else
  STATUS="BLOCKED"
  REASON="workspace install setup is missing after build: ${INSTALL_SETUP}"
  echo "BLOCKED: ${REASON}" | tee -a "${BUILD_LOG}" >&2
  write_blocked_smoke_json "workspace_install_setup_missing" "${REASON}"
  finish
fi

SMOKE_CMD=(python3 "${REPO_ROOT}/scripts/run_workcell_builder_scene3d_gui_smoke.py" --repo-root "${REPO_ROOT}" --workspace-root "${WORKSPACE_ROOT}" --scene "${SCENE}" --output "${SMOKE_JSON}" --screenshot "${SMOKE_PNG}" --timeout-sec "${TIMEOUT_SEC}")
echo "Running: $(quote_cmd "${SMOKE_CMD[@]}")" >> "${BUILD_LOG}"
(
  cd "${REPO_ROOT}" && "${SMOKE_CMD[@]}"
) >> "${BUILD_LOG}" 2>&1
SMOKE_RC=$?
SMOKE_STATUS="$(read_smoke_status)"

mkdir -p "${READINESS_DIR}"
READINESS_CMD=(python3 "${REPO_ROOT}/scripts/run_workcell_studio_scene_readiness_matrix.py" --repo-root "${REPO_ROOT}" --workspace-root "${WORKSPACE_ROOT}" --output-dir "${READINESS_DIR}")
echo "Running: $(quote_cmd "${READINESS_CMD[@]}")" >> "${BUILD_LOG}"
(
  cd "${REPO_ROOT}" && "${READINESS_CMD[@]}"
) > "${READINESS_STDOUT}" 2> "${READINESS_STDERR}"
READINESS_RC=$?
if [[ -f "${READINESS_DIR}/scene_readiness_summary.json" ]]; then
  cp "${READINESS_DIR}/scene_readiness_summary.json" "${READINESS_SUMMARY}"
fi
cat "${READINESS_STDOUT}" >> "${BUILD_LOG}"
if [[ -s "${READINESS_STDERR}" ]]; then
  cat "${READINESS_STDERR}" >> "${BUILD_LOG}"
fi

if [[ ${SMOKE_RC} -ne 0 && "${SMOKE_STATUS}" == "BLOCKED" ]]; then
  if python3 - "$SMOKE_JSON" <<'PY'
import json, sys
try:
    payload=json.load(open(sys.argv[1], encoding='utf-8'))
except Exception:
    sys.exit(1)
blockers=set(map(str, payload.get('blockers') or []))
if any('executable' in b or 'MISSING_EXECUTABLE' in b for b in blockers) or str(payload.get('smoke_status','')).upper() == 'MISSING_EXECUTABLE':
    sys.exit(0)
sys.exit(1)
PY
  then
    STATUS="BLOCKED"
    REASON="workcell_builder executable missing after build; see ${SMOKE_JSON}"
  else
    STATUS="FAIL"
    REASON="Scene3D smoke command failed after build/executable resolution; exit code ${SMOKE_RC}; smoke_status=${SMOKE_STATUS}; see ${SMOKE_JSON}"
  fi
  finish
fi

if [[ ${SMOKE_RC} -ne 0 ]]; then
  STATUS="FAIL"
  REASON="Scene3D smoke command crashed or failed after executable resolution; exit code ${SMOKE_RC}; smoke_status=${SMOKE_STATUS}"
  finish
fi

if [[ ! -f "${SMOKE_JSON}" || ! -f "${SMOKE_PNG}" ]]; then
  STATUS="FAIL"
  REASON="Scene3D smoke did not produce both JSON and screenshot evidence"
  finish
fi

if [[ "${SMOKE_STATUS}" == "PASS" || "${SMOKE_STATUS}" == "OK" ]]; then
  STATUS="PASS"
  REASON="Scene3D smoke PASS with JSON and screenshot evidence; readiness matrix collected"
else
  STATUS="FAIL"
  REASON="Scene3D smoke evidence status is ${SMOKE_STATUS}, expected PASS"
fi

if [[ ${READINESS_RC} -ne 0 ]]; then
  STATUS="FAIL"
  REASON="${REASON}; readiness matrix command exited ${READINESS_RC}"
fi

finish
