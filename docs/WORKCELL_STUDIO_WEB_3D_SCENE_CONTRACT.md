# Workcell Studio Web 3D Scene Contract

## Purpose

Workcell Studio is moving toward a browser-based Web Studio experience for 3D scene review and editing. That browser UI needs a stable, dependency-light JSON scene contract so it can exchange scene state with the Workcell Studio backend without depending on ROS, Qt Scene3D internals, generated package implementation details, or a specific frontend framework.

This contract exists to define the boundary between:

- source-of-truth Workcell Studio scene files and generated backend metadata;
- browser-friendly scene JSON consumed by a future Web Studio client; and
- backend ROS package generation, validation, and simulation workflows.

This PR defines the contract only. It does **not** build the web editor, introduce a browser application, or change the existing Workcell Builder runtime flow.

## Contract goals

The web 3D scene contract should be:

- **Stable:** clients can rely on predictable top-level fields, units, coordinate conventions, item identity, provenance, editability, and warnings.
- **Dependency-light:** the exported scene JSON should be readable with standard JSON tooling and should not require ROS, Qt, npm, Vite, React, Three.js, Babylon.js, or other frontend build systems.
- **Traceable:** each exported item should carry enough provenance to identify whether it came from authored scene data, layout editor state, generated preview metadata, or backend-generated package metadata.
- **Safe by default:** the contract is a scene data exchange format, not a robot execution interface.
- **Backend-owned for generation:** the browser may request or submit scene edits through this contract, but package generation remains a backend responsibility.

## Source-of-truth ownership

Workcell Studio source-of-truth scene data remains in the existing authored and generated scene files, including:

- `environment.yaml` for authored environment and layout-adjacent scene data;
- `layout/workcell_studio_layout.yaml` for Workcell Builder editor state where applicable;
- `cell_definition.yaml` for the generated exchange model and canonical generated cell definition;
- `scene_manifest.yaml` for scene package indexing and contract summaries; and
- generated backend metadata such as visual mesh indexes, validation outputs, readiness metadata, and generated package metadata.

The browser UI reads and writes scene state through the web scene contract. The contract is the browser-facing interchange layer, not the canonical owner of the full workcell model.

Backend responsibilities remain unchanged:

- Workcell Studio backend code remains responsible for validating source files and generating ROS 2 scene packages.
- Generated ROS packages remain the backend output for launch, validation, planning, and simulation workflows.
- RViz/MoveIt remains the planning and simulation truth for robot structure, generated visuals, planning groups, and fake-hardware simulation behavior.
- EPD/RealSense remains the perception truth for RealSense input, detection, localization, tracking, classification, and perception-specific runtime state.
- Qt Scene3D remains legacy/debug preview only. It is not the product direction for the primary web 3D editor and is not the authoritative source for scene data.

## Browser-facing contract boundary

A web scene payload should describe browser-relevant scene state, including:

- schema version and scene identity;
- units and coordinate-system conventions;
- source file presence and provenance;
- robot and tool summary metadata;
- authored physical assets such as tables, bins, conveyors, cameras, and objects;
- generated visual preview items when available;
- editability and lock state so the browser can distinguish editable layout items from locked generated preview items;
- warnings for missing optional inputs, missing meshes, unsupported fields, or degraded preview behavior; and
- backend actions that the UI may expose as requests to the backend rather than local frontend side effects.

The contract should preserve the near-term canvas model:

```text
Editable layout items = user can select, move, rotate, edit, save.
Generated URDF preview items = locked visual preview, not directly edited.
Inspector = authoritative manual XYZ/RPY editing surface.
Layout YAML = editor state.
Generated scene files = regenerated from source-of-truth data.
```

## Exporter behavior

The exporter should produce deterministic, browser-readable JSON from existing Workcell Studio scene inputs. Exported web scene output belongs under `build/` by default or under an explicit user-specified output path.

Exporter behavior must remain side-effect-light:

- no frontend dependencies;
- no React, Three.js, Babylon.js, Vite, or npm dependency installation;
- no Qt Scene3D visual fixes;
- no ROS launch or build side effects;
- no scene package regeneration unless a separate backend action explicitly performs generation; and
- no writes into canonical scene source files unless an explicit import/apply-edit workflow is added later.

Missing optional inputs should be reported in the exported JSON as warnings rather than causing the exporter to crash. Missing required inputs for future stricter workflows should produce clear, actionable errors that name the responsible file and field.

## Non-goals for this contract PR

This contract PR does not:

- implement the Web Studio editor;
- add frontend scaffolding;
- add React, Three.js, Babylon.js, Vite, npm, or equivalent browser build tooling;
- replace Workcell Builder;
- fix Qt Scene3D/Product View visual quality issues;
- change RViz, MoveIt, ROS launch files, controller configs, runtime services, or hardware parameters;
- build or launch ROS packages;
- validate real robot readiness; or
- change EPD/RealSense ownership boundaries.

## Safety

Fake-hardware-first behavior is unchanged. This contract is a data interchange format for scene review/editing and backend requests; it does not enable real robot motion, automatic runtime send, uncontrolled topic/service publishing, or real-hardware execution.

Real-hardware operation remains explicitly guarded by existing backend safety gates and future guarded workflows. RViz/MoveIt fake-hardware validation remains the foundation for simulation and planning truth.
