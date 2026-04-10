# Archived duplicate/reference artifacts

This folder stores legacy duplicate artifacts that were previously kept in
build-relevant source trees under names containing `(copy)` or spaces.

These files are intentionally moved out of active package/config/include paths
so colcon/CMake/package installs do not accidentally pick them up.

If any file here is needed again, restore it under a canonical filename without
spaces in the appropriate package.
