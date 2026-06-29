# Workcell Studio Web 3D Pivot

## Decision

Qt Scene3D/Product View is now treated as a Debug 3D Preview and legacy experimental preview. It may remain useful for diagnostics, local inspection, and regression triage, but it is no longer the product direction for the main 3D editing experience.

## What remains supported

Workcell Studio continues to support the core Workcell Builder workflow:

- scene selection
- opening and saving layout state
- generating scene packages
- validation and readiness scripts
- ROS 2 Humble package generation

This pivot does not remove the existing scene authoring, generation, validation, or ROS package outputs that downstream RViz/MoveIt workflows depend on.

## Source of truth

The authoritative scene and package state remains in the repository data and generated artifacts, including:

- layout YAML, including editor/layout state such as `layout/workcell_studio_layout.yaml` where applicable
- generated or canonical cell definitions, including `cell_definition.yaml`
- scene manifests, including `scene_manifest.yaml`
- generated URDF/Xacro and package outputs, including files such as `urdf/scene.urdf.xacro` and launch/package files

Qt Scene3D is not authoritative. It must not become the canonical source for robot, tool, environment, task, manifest, package, or generated URDF/Xacro state.

## Planning/visual truth

RViz/MoveIt fake-hardware simulation remains the validation foundation for planning and visual truth. Product-facing claims about planning readiness, generated robot structure, MoveIt visibility, and fake-hardware simulation should continue to be validated through the ROS 2 Humble package path rather than through Qt Scene3D appearance alone.

## Future direction

The main 3D editor should be browser-based. The intended long-term direction is a web-native 3D editor that can better support modern scene interaction, inspection, layout editing, and product-quality visualization.

Do not start React, Three.js, Babylon.js, Streamlit, or any other new web/dashboard implementation in this PR. This document records the product direction only; implementation should be planned in later scoped PRs.

## Perception boundary

EPD/RealSense remains separate from Workcell Studio. EPD owns RealSense input, detection, localization, tracking, classification, and perception-specific UI/configuration. Workcell Studio consumes perception input only, such as normalized object detections, poses, classes, confidence values, timestamps, and task bindings.

Perception output may inform Workcell Studio validation, replay, task binding, or runtime readiness checks, but it must not make EPD own scene authoring, scene generation, task planning, Workcell Builder UI state, or generated package state.

## Safety

Safety defaults are unchanged:

- no real robot motion by default
- fake hardware remains the default validation and simulation path
- real hardware requires explicit guarded flags and setup
- generated validation/readiness reports are not safety certificates
- no automatic runtime send or uncontrolled robot execution path is introduced by this pivot

Any future implementation work must preserve fake-hardware-first behavior and keep real-hardware mode explicitly guarded.

## Out of scope

This pivot document does not include:

- Qt visual-quality fixes
- new visual topology gates
- new smoke screenshots or screenshot fixtures
- new browser-based 3D implementation work
- React, Three.js, Babylon.js, Streamlit, or dashboard scaffolding
- changes to launch files, controllers, runtime services, or hardware parameters
