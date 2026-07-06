# Scene3D Canvas Contract

This document defines the non-optional stability contract for Workcell Builder Scene3D.

## Required Layers

1. `editable_layout`
   - Owns: authoring objects from `environment.yaml`, `layout/workcell_studio_layout.yaml`, and in-memory layout model.
   - Must never own: generated URDF visual source-of-truth.
   - Editability: editable (`editable=true`).

2. `mesh_preview`
   - Owns: mesh-backed visual preview loaded from `generated/scene_visual_mesh_index.json` when safe. This JSON file is generated cache/build output, not tracked source-of-truth, and must not be committed under `scenes/*/generated/`.
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

## Generated cache and web export placement

- `generated/scene_visual_mesh_index.json` is generated cache/build output used by Scene3D/Web preview refresh, not canonical scene state.
- Do not commit `generated/scene_visual_mesh_index.json` under `scenes/*/generated/`; the GUI/Web viewer refresh path regenerates it automatically when the file is missing, stale, or produced by an older extractor.
- Source-of-truth scene state remains in tracked YAML, xacro, URDF, layout, task/config, and manifest inputs.
- Web scene JSON exports should be written under `build/workcell_studio_web_scene/` or another ignored output location, never as committed scene artifacts under `scenes/*/generated/`.

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

## Camera, FOV, and perception overlay contract

- Camera markers may appear in Scene3D from either editable layout metadata or generated preview sources.
- When camera marker source is `editable_layout`, pose/metadata edits are allowed through existing authoring controls.
- When camera marker source is generated preview (`locked_generated_urdf_visual` or `mesh_preview` without explicit mapping), the marker is inspectable only and remains locked/read-only.
- Camera FOV frustum must render on the `overlay` layer as read-only guidance visuals and must not become an authoring transform owner.
- Detection snapshot overlays (for example projected detections, historical perception frames, and similar snapshot annotations) must render on the `overlay` layer as read-only visuals.
- Overlay visuals are preview-only and non-authoritative for scene generation: they must not mutate `environment.yaml`, `layout/workcell_studio_layout.yaml`, task intent files, or generated URDF/mesh artifacts.
- This feature scope excludes live perception-device launches. No live EPD/RealSense startup, streaming session creation, or hardware bring-up is triggered from Scene3D camera/FOV/detection overlays.

## Additive Change Rule

**Future 3D canvas PRs must add capabilities as layers or extend existing layers. They must not remove primitive fallback, mesh preview, xacro-expanded preview, or refresh behavior unless the Scene3D contract is intentionally updated and all regression tests are updated.**

## Selection and transform editing rules

- Selection tracks visual identity only and does not change layer ownership.
- `editable_layout` is the only layer that owns authoring transforms.
- `locked_generated_urdf_visual` is inspectable but remains locked/read-only.
- `mesh_preview` is a visual aid and does not become authoring state by selection.
- `primitive_fallback` must remain available even when mesh/URDF preview is present.
- Inspector transform writes must target authoring layout sources only and must not mutate generated artifacts (`scene.urdf.xacro`, expanded preview data, mesh index JSON, or generated preview files).

## Translate gizmo and drag editing rules

- Drag translation is available only for editable authoring-backed items.
- Generated visuals remain inspectable/selectable but locked (no drag gizmo).
- `mesh_preview` remains a visual aid by default and only participates in editing when explicitly mapped to editable layout state.
- `primitive_fallback` can be editable only when linked to editable layout state.
- Dragging provides preview movement only; authoring YAML is not written on every mouse move.
- Drag commit writes only authoring layout XYZ transforms and preserves other transform fields.
- Drag editing must not mutate generated artifacts (`generated/scene.urdf.xacro`, expanded URDF preview data, mesh index files, generated preview artifacts).

## Mesh ingestion and preview expectations

- Scene3D first attempts generated URDF visual mesh rendering from the authoritative visual mesh index, such as `generated/scene_visual_mesh_index.json`, before using lower-fidelity visuals.
- Assimp is the preferred mesh ingestion path when it is available, and should be used for STL, DAE, and OBJ visual assets.
- Lightweight fallback parsers are only a backup path for environments where Assimp is unavailable or cannot load an otherwise previewable asset.
- Heavy but valid workcell assets may use simplified preview meshes or semantic visual surrogates when that keeps the canvas responsive without changing the authoritative scene model.
- Truly unsafe assets should be rejected only after scale-, origin-, and transform-aware bounds checks show that the effective preview bounds are unsafe.
- Scene3D is a visual preview renderer. It is not a collision model, safety certificate, validation report, or proof of real-hardware readiness.

### Expected `ur5_2f_test` behavior

For `ur5_2f_test`, Scene3D should render the UR5 and Robotiq visual assets as real meshes when the generated visual mesh index resolves their mesh files. The table should render as either a real or simplified workbench mesh when available, or as a semantic table surrogate when a safe mesh is not available. The D435 camera should render as a real mesh when loadable, or as a labelled RealSense visual surrogate when the mesh cannot be safely loaded.
