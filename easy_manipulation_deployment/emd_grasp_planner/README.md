# EMD Grasp Planner

## Performance note

If OpenMP is available, the build enables parallel octomap generation in
`src/common/fcl_functions.cpp` for faster collision setup.
