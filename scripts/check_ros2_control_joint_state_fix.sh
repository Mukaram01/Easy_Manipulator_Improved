#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

log() {
  echo "[check_ros2_control_joint_state_fix] $*"
}

fail() {
  log "FAIL: $*"
  exit 1
}

cleanup() {
  if [[ -n "${LAUNCH_PID:-}" ]] && kill -0 "${LAUNCH_PID}" 2>/dev/null; then
    log "Stopping ros2 launch (pid ${LAUNCH_PID})"
    kill "${LAUNCH_PID}" 2>/dev/null || true
    wait "${LAUNCH_PID}" 2>/dev/null || true
  fi
}

trap cleanup EXIT

cd "${REPO_ROOT}"

pattern="joint_state_controller/JointState"
pattern+="Controller"

if rg -n "${pattern}" -S .; then
  fail "Deprecated JointStateController types found"
fi
log "No deprecated JointStateController types found"

log "Launching ros2_control demo (run_dynamic_safety)"
ros2 launch run_dynamic_safety run_moveit_cpp.launch.py > "${REPO_ROOT}/ros2_control_demo.log" 2>&1 &
LAUNCH_PID=$!

log "Waiting for /controller_manager"
for _ in $(seq 1 30); do
  if ros2 control list_controllers --controller-manager /controller_manager >/dev/null 2>&1; then
    break
  fi
  sleep 1
  if ! kill -0 "${LAUNCH_PID}" 2>/dev/null; then
    fail "ros2 launch exited unexpectedly (see ros2_control_demo.log)"
  fi
done

if ! ros2 control list_controllers --controller-manager /controller_manager >/dev/null 2>&1; then
  fail "controller_manager not available after waiting (see ros2_control_demo.log)"
fi

log "Spawning joint_state_broadcaster"
if ros2 run controller_manager spawner joint_state_broadcaster --controller-manager /controller_manager; then
  log "joint_state_broadcaster spawned"
else
  if ros2 control list_controllers --controller-manager /controller_manager | rg -q "^joint_state_broadcaster"; then
    log "joint_state_broadcaster spawned"
  else
    fail "Failed to spawn joint_state_broadcaster"
  fi
fi

log "Checking /joint_states header stamps"
if timeout 20s ros2 topic echo -n 5 /joint_states | awk '
  /sec:/ { if ($2 + 0 > 0) found=1 }
  /nanosec:/ { if ($2 + 0 > 0) found=1 }
  END { exit found ? 0 : 1 }
'; then
  log "/joint_states publishing with non-zero stamps"
else
  fail "/joint_states did not publish non-zero header stamps"
fi

log "PASS"
