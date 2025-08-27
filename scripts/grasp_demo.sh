#!/usr/bin/env bash
set -euo pipefail

cleanup() {
  kill "${planner_pid:-}" "${execution_pid:-}" 2>/dev/null || true
}
trap cleanup SIGINT SIGTERM ERR EXIT

# Build the workspace and source it
colcon build --symlink-install
source install/setup.bash

# Launch grasp planner and execution demos
ros2 launch run_grasp_planner grasp_planner_3f_launch.py &
planner_pid=$!

ros2 launch run_grasp_execution grasp_execution.launch.py &
execution_pid=$!

# Wait for user to terminate
echo "Demo running. Press Ctrl+C to stop."
wait
