# Patch catalog

- **001-configure-component-override.patch**: Removes the `if(NOT COMMAND configure_component)` guard in `tesseract_common` so the bundled implementation always overrides upstream boilerplate that mishandles `EXPORT_NAME` and `EXPORT_FILE`.
- **002-motion-planners-export-fix.patch**: Replaces the invalid export/install pattern in `tesseract_motion_planners_core` with standard `install(TARGETS)` and `install(EXPORT ...)` calls so the core targets are installed consistently.
