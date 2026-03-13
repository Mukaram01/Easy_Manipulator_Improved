# Patch catalog

- **001-configure-component-override.patch**: Removes the `if(NOT COMMAND configure_component)` guard in `tesseract_common` so the bundled implementation always overrides upstream boilerplate that mishandles `EXPORT_NAME` and `EXPORT_FILE`.
- **002-ricb-cfg-extras-absolute-fix.patch**: Applies to the checked-out `ros_industrial_cmake_boilerplate` dependency and ensures `CFG_EXTRAS` entries are copied from absolute paths into the generated package config directory before installation.
- **003-ricb-octomap-import-location-fix.patch**: Applies to `ros_industrial_cmake_boilerplate` and updates generated `tesseract_geometry` package config files to require `find_dependency(octomap REQUIRED)` before importing targets, validate `octomap`/`octomath` imported locations, set a fallback `IMPORTED_LOCATION` when only config-specific locations exist, and emit a fatal error with recovery guidance when no import location is exported.
