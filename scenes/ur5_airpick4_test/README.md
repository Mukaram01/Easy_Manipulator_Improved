# ur5_airpick4_test dependencies

This package depends on `onrobot_airpick4_description`, which must be available in the
workspace (e.g., vendored under `src/`) or installed as a system package.

If `colcon build` cannot locate the package, point CMake to its install prefix using one of
these options:

```bash
# Example: set the prefix for an installed workspace
export CMAKE_PREFIX_PATH=/path/to/onrobot_airpick4_description/install:$CMAKE_PREFIX_PATH

# Or provide a direct package config path
export onrobot_airpick4_description_DIR=/path/to/onrobot_airpick4_description/install/share/onrobot_airpick4_description/cmake
```

Make sure the package is discoverable before building `ur5_airpick4_test`.
