# Workcell Studio UI Layout

Workcell Studio now uses a **progressive disclosure** layout:

- Each page keeps one primary action and only a few secondary actions visible.
- Advanced/secondary actions are grouped under a **More Actions** dropdown.
- The Scene Builder is **canvas-first** with resizable/collapsible side panels.
- Mode indicators are shown in compact chips:
  - Design
  - Preview
  - Plan
  - Simulate
  - Hardware Guarded
- Safety gates remain enforced with compact messaging:
  - Runtime disabled by default
  - Fake hardware by default
  - Guarded execution
  - No uncontrolled robot motion

This keeps demo flows cleaner without removing any existing safety constraints or action coverage.


## Premium Digital Twin Canvas

The Scene Builder canvas is now the primary visual workspace with premium dark industrial styling, compact controls, and progressive disclosure.

- **Canvas modes**: Select, Place, Move, Inspect.
- **Snap grid**: Snap Grid toggle with visible step label (default 0.05 m) for predictable placement/move edits.
- **Drag/move/save**: Layout items are moved in-canvas and persisted through **Save Layout** into `environment_layout.yaml`.
- **Ghost placement**: Place mode uses a ghost preview flow before committing item placement.
- **Overlays**: Reach, camera FOV, pick/place, trajectory placeholder, warnings, labels.
- **Minimap**: Compact minimap card offers a scene overview of placed assets.
- **Plan & Simulate relation**: Canvas remains preview-safe and integrates with Plan & Simulate (MoveIt + RViz fake-hardware workflow) without enabling real hardware runtime.

This keeps `environment_layout.yaml` as the source of truth for scene layout persistence while preserving ROS 2 Humble-compatible preview-only safety defaults.
