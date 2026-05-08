# Workcell Studio Asset Onboarding

- `assets/` holds real URDF/Xacro/mesh/MoveIt packages.
- `catalog/` and `workcell_studio_catalog/` hold metadata/index entries.
- `workcell_builder` consumes catalog metadata and should show support/runtime readiness.
- Generated workcell packages reference existing assets and should avoid unnecessary copies.
- Never duplicate the same ROS package in multiple colcon-visible paths.
- Real hardware drivers remain metadata-only and optional; fake hardware remains the default.

## Onboarding flow
1. Add or reuse a robot description package under `assets/` with `package.xml` + `CMakeLists.txt`.
2. Add or reuse a MoveIt config package and mark fake-hardware demo launch readiness.
3. Add grippers/sensors/environment assets under `assets/` and reference them from catalog entries.
4. Update capability YAML with `support_status`, `runtime_status`, asset paths, and readiness flags.
5. Run `python3 scripts/audit_workcell_assets.py --root . --json`.

## Supported vs preview_only
- Use `supported` for assets with existing fake-hardware description + MoveIt paths.
- Use `preview_only` when the package is placeholder/incomplete.

## Driver safety policy
- Record driver metadata (`driver_package`, `driver_repo`, `driver_status`) only.
- Do not add driver packages to required dependencies in this layer.
- Do not generate real-hardware launch commands by default.

## Duplicate package troubleshooting
- Run audit script and resolve duplicate package names by keeping one source package.
- Keep catalog references pointed at the canonical package location.
