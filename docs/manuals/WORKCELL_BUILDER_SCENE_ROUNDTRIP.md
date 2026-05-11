# Workcell Builder Scene Round-trip (Load → Edit → Regenerate)

This workflow keeps the existing Qt `workcell_builder` and adds support for opening existing `workcell_scene/v1` scenes, editing them, and regenerating safely.

## Open Existing Scene

Use **Open Existing Scene** in the builder scene flow and pick an existing generated scene folder containing `environment.yaml`.

Use **Reload Scene From YAML** to refresh UI/model state from the same file.

## What Gets Restored Into UI/Models

The round-trip loader restores available metadata from `workcell_scene/v1`:
- scene/package identity
- robot and tool selections
- compatibility metadata (`COMPATIBLE`, `UNKNOWN_COMPATIBILITY`, `INCOMPATIBLE`)
- placed objects and mesh references (`generated_objects`, imported/custom meshes)
- camera/perception metadata
- task/grasp metadata
- workspace bounds/zones
- safety flags and schema/readiness notes

Missing or legacy fields produce **Legacy/partial scene warning** and keep manual override/editing enabled.

## Regenerate Existing Scene (Safe Defaults)

Use **Regenerate Existing Scene** after edits. The process preserves safe defaults:
- `fake_hardware_first: true`
- `real_hardware_enabled: false`
- `runtime_execution_enabled: false`

It also preserves existing generated/imported mesh references and scene metadata when present.

## Scene Round-trip Status

**Scene Round-trip Status** indicates whether the loaded scene was recognized as:
- **Loaded from workcell_scene/v1**
- **Legacy/partial scene warning**

## Validation Dashboard Integration

After load/edit/regenerate, use **Run Offline Validation** to review:
- schema status
- catalog and compatibility warnings
- readiness blockers/warnings

This is offline-only safety validation; no ROS launch, no MoveIt planning/execution, and no real hardware enablement.
