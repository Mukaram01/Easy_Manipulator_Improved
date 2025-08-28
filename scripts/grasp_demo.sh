#!/usr/bin/env bash
set -euo pipefail

cleanup() {
  # Terminate background processes if they were started
  if [ -n "${planner_pid:-}" ]; then
    kill "$planner_pid" 2>/dev/null || true
  fi
  if [ -n "${execution_pid:-}" ]; then
    kill "$execution_pid" 2>/dev/null || true
  fi
}
trap cleanup SIGINT SIGTERM ERR EXIT

# Build the workspace and source it
colcon build --symlink-install
# shellcheck disable=SC1091
source install/setup.bash

# Launch grasp planner and execution demos
ros2 launch run_grasp_planner grasp_planner_3f_launch.py &
planner_pid=$!

ros2 launch run_grasp_execution grasp_execution.launch.py &
execution_pid=$!

# Wait for user to terminate
echo "Demo running. Press Ctrl+C to stop."
wait
