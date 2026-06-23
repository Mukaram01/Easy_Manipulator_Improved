# Workcell Studio large-file wrapper plan

This is the wrapper-extraction baseline for the oversized Workcell Studio files.
It deliberately avoids a broad rewrite. Each follow-up PR should move one cohesive
area out of a huge file, wire it into production, and keep the existing product
flow working.

## Rules

- Do not split files just to split files.
- Extract one cohesive responsibility per PR.
- Keep `workcell_builder` as the primary product UI.
- Keep EPD separate; do not merge the EPD GUI into Workcell Builder.
- Preserve fake-hardware-first behaviour.
- Do not add another validator loop unless it catches a real escaped bug.

## Current first-class wrap targets

Run the audit with:

```bash
python3 scripts/audit_workcell_large_files.py --json
python3 scripts/audit_workcell_large_files.py --output build/workcell_large_file_audit.md
```

The initial known targets are:

| File | First wrapper seams |
| --- | --- |
| `workcell_builder/workcell_builder/gui/mainwindow.cpp` | scene loading and manifest parsing; Scene3D visual-index ingestion; save/generate action orchestration; task-intent and zone editor wiring |
| `workcell_builder/workcell_builder/gui/scene3d_viewport_widget.cpp` | mesh loading/cache policy; render diagnostics/counter emission; camera fitting and bounds calculation; selection/picking helpers |
| `scripts/run_workcell_builder_scene3d_gui_smoke.py` | payload normalization; UR5 final viewport contract checks; wrapper process execution/log capture |
| `scripts/generate_workcell_from_cell_definition.py` | thin CLI wrapper; package contract renderer; asset/environment renderer; validation/report writer |

## Safe next extraction sequence

1. Move `scripts/run_workcell_builder_scene3d_gui_smoke.py` payload-only helpers into a module such as `scripts/scene3d_smoke_payload.py`. This is the lowest-risk first split because it is Python and already has unit tests.
2. Move generated-workcell rendering helpers out of `scripts/generate_workcell_from_cell_definition.py`, leaving a thin CLI and `generate_package()` orchestration entrypoint.
3. Extract `mainwindow.cpp` Scene3D visual-index ingestion into a C++ helper with no Qt widget ownership.
4. Extract `mainwindow.cpp` save/generate command orchestration into a helper that returns explicit command/result payloads.
5. Extract `scene3d_viewport_widget.cpp` render diagnostics into a small data collector object so smoke output does not depend on scattered counter writes.

## Acceptance for each wrapper PR

- The old product behaviour remains available through the existing entrypoints.
- The moved logic has direct unit coverage.
- The old huge file gets smaller or stops receiving new unrelated logic.
- No real robot motion path is added.
- The PR validates with a focused pytest/C++ test plus `colcon build --symlink-install --packages-select workcell_builder` when the change touches C++.
