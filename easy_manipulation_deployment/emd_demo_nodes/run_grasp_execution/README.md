# run_grasp_execution

## Launching and launch tests

- Use `ros2 launch run_grasp_execution grasp_execution.launch.py` for normal runtime launch.
- Run launch tests with `colcon test --packages-select run_grasp_execution --ctest-args -R test_grasp_execution_launch`.

The helper test module in `test/test_grasp_execution_launch_helpers.py` is **not** a ROS launch entrypoint.
