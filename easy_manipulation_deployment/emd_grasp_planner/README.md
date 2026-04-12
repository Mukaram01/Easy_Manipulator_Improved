# EMD Grasp Planner

## Performance note

If OpenMP is available, the build enables parallel octomap generation in
`src/common/fcl_functions.cpp` for faster collision setup.

## Camera parameter notes

Under `camera_parameters`, `tf_filter_queue_size` controls the TF message filter
queue depth used by grasp planning subscriptions.

- Default: `20` in the demo `run_grasp_planner` parameter files.
- Increase this value to reduce dropped messages when TF data arrives late.
- Larger values trade memory usage and latency for fewer drops.
