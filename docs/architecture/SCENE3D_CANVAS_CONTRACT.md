# Scene3D Canvas Contract

This document defines the non-optional stability contract for Workcell Builder Scene3D.

## Required Layers

1. `editable_layout`
   - Owns: authoring objects from `environment.yaml`, `layout/workcell_studio_layout.yaml`, and in-memory layout model.
   - Must never own: generated URDF visual source-of-truth.
   - Editability: editable (`editable=true`).

2. `mesh_preview`
   - Owns: mesh-backed visual preview loaded from `generated/scene_visual_mesh_index.json` when safe.
   - Must never own: authoring state unless explicitly mapped by user workflow.
   - Editability: selectable/inspectable, not authoring source.

3. `locked_generated_urdf_visual`
   - Owns: xacro-expanded URDF visual extraction preview for robot/tool/camera/table/fixtures.
   - Must never own: editable layout authoring state.
   - Editability: locked/read-only by default (`editable=false`).

4. `primitive_fallback`
   - Owns: primitive/bounding-box/simple-shape fallback visuals.
   - Must never own: generated mesh authority.
   - Editability: follows originating layout item policy.

5. `overlay`
   - Owns: visualization overlays only (FOV/zones/safety guidance).
   - Must never own: scene generation data.
   - Editability: non-authoring guidance visuals.

## Source-of-Truth Rules

- Authoring source-of-truth is `editable_layout` only.
- `mesh_preview` and `locked_generated_urdf_visual` are preview layers and must not mutate layout YAML/environment YAML implicitly.
- `overlay` is visual-only and cannot become data ownership.

## Fallback Rules

- Primitive fallback must remain available whenever mesh preview is missing, unsafe, or disabled.
- If xacro expansion fails, attempt safe mesh preview and retain primitive fallback.
- If mesh index is unsafe, logs must report unsafe mesh preview and confirm fallback retained.

## Refresh Rules

- Existing open/refresh/generate scene flows must preserve layer counts and not silently drop items.
- No-silent-disappearance: items cannot disappear across refresh without diagnostics indicating reason and replacement path.

## Stable Item Schema (contract fields)

Scene3D items expose/track:

- `id`, `label`, `source_layer`, `active_visual_source`, `editable`, `locked_reason`
- `transform_source`, `source_scene`, `source_file`, `parent_link`
- `mesh_uri`, `resolved_mesh_path`, `primitive_geometry`
- `pose_xyz`, `pose_rpy`, `scale`, `diagnostics`

Allowed `source_layer` values:

- `editable_layout`
- `mesh_preview`
- `locked_generated_urdf_visual`
- `primitive_fallback`
- `overlay`

Allowed `active_visual_source` values:

- `mesh`
- `expanded_urdf_mesh`
- `primitive`
- `placeholder`
- `overlay`

Additional invariants:

- IDs must be stable across repeated loads.
- IDs must not contain unresolved `${...}` placeholders when safe xacro-expanded extraction is available.
- Locked generated URDF items default `editable=false`.
- Editable layout items remain `editable=true`.
- Primitive fallback cannot be deleted solely because mesh index exists.
- If a higher-fidelity visual supersedes fallback rendering, diagnostics must retain fallback relationship metadata.

## Additive Change Rule

**Future 3D canvas PRs must add capabilities as layers or extend existing layers. They must not remove primitive fallback, mesh preview, xacro-expanded preview, or refresh behavior unless the Scene3D contract is intentionally updated and all regression tests are updated.**

