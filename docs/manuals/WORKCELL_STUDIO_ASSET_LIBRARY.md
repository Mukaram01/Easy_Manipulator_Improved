# Workcell Studio Asset Library

This catalog powers the Workcell Studio builder selection area for EMD/workcell_builder only.

## Catalog structure
- Root file: `workcell_studio_catalog/catalog.yaml`
- Mesh placeholders: `workcell_studio_catalog/meshes/...`
- Categories include robots, end effectors/tools, sensors, environment, fixtures, conveyors, machines, safety, objects.

## Adding a new robot
1. Add item in `catalog.yaml` with `category: robots`.
2. Set `support_status` and `runtime_status` honestly.
3. Reference real mesh/package only when available.

## Adding a new gripper
1. Add item with `category: end_effectors` or `tools`.
2. Include `tcp_frame`, `flange_frame`, compatibility fields.

## Adding a new STL
- Use `CustomStlMetadata` model in `workcell_asset_catalog.py`.
- Include file path, scale, placement, collision flags, and notes.

## support_status meanings
- `supported`: integrated and validated in EMD flow.
- `preview_only`: shown in builder but not runtime integrated.
- `placeholder`: synthetic primitive mesh.
- `unsupported`: intentionally unavailable.

## runtime_status meanings
- `supported`: runtime launch path exists.
- `fake_hardware_only`: fake-hardware workflows only.
- `preview_only`: selection/preview only.
- `unsupported`: no runtime path.

## Licensing rule
Do not add downloaded third-party meshes without license metadata and attribution.

## Placeholder vs supported
Placeholder assets are clearly tagged to avoid false runtime claims.
