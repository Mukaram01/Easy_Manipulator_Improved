# Patch catalog

- **001-configure-component-override.patch**: Removes the `if(NOT COMMAND configure_component)` guard in `tesseract_common` so the bundled implementation always overrides upstream boilerplate that mishandles `EXPORT_NAME` and `EXPORT_FILE`.
- **002-ricb-cfg-extras-absolute-fix.patch**: Applies to the checked-out `ros_industrial_cmake_boilerplate` dependency and ensures `CFG_EXTRAS` entries are copied from absolute paths into the generated package config directory before installation.
