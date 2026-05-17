# Workcell Builder Object Placement Manager

The Qt Workcell Builder includes an **Object Placement Manager** workflow for managing imported STL assets and generated primitive meshes in a scene.

## What this feature does

- Browse/select STL assets from repository/workspace asset trees.
- Import an external STL into a managed library path:
  - `easy_manipulation_deployment/assets/environment/custom_meshes/`
- Add assets as placed objects.
- Edit object pose (`x y z roll pitch yaw`) and metadata.
- Duplicate/remove placed objects.
- Persist placed objects into generated scene summary/preview/readiness artifacts.

## Where meshes are stored

- Imported external STL meshes are copied to:
  - `easy_manipulation_deployment/assets/environment/custom_meshes/`
- Generated primitive STL meshes remain in:
  - `meshes/generated_objects/`

## Generated artifacts

Placed objects are represented in generated output such as `environment.yaml`, `workcell_studio_summary.json`, `workcell_studio_summary.md`, and preview files. Example section:

```yaml
placed_objects:
  - name: table_01
    source: asset_stl
    mesh: package://easy_manipulation_deployment/assets/environment/custom_meshes/table_01.stl
    pose: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
```

## Operator flow

1. Open workcell_builder.
2. Select/create scene.
3. Select robot.
4. Select end effector.
5. Open Object Placement Manager.
6. Import external STL or select existing STL asset.
7. Set object pose.
8. Duplicate/remove/edit as needed.
9. Generate Files.
10. Build generated scene.
11. Launch with `use_fake_hardware:=true`.

## Safety note

This is generation-time/offline-only workflow. No MoveIt planning call, no trajectory execution, and no real-hardware enablement is introduced by object placement management.


## Preview real STL assets in RViz

- Object Placement Manager can import/select STL assets and place them with XYZ/RPY pose metadata.
- Use **Open RViz STL Preview** to generate offline preview artifacts in `/tmp/workcell_builder_preview/<scene_name>/`.
- Generated files include `placed_objects_preview.yaml`, `placed_objects_preview.urdf.xacro`, `preview_scene.launch.py`, and `README_PREVIEW.md`.
- This preview is visual-only/offline-only and does not use MoveIt, controllers, trajectories, or robot motion.
- Full generated scene flow still requires **Generate YAML** and **Generate Files** in Workcell Studio.


## Apply RViz pose feedback

- Interactive RViz edits produce `placed_objects_feedback.yaml` in `/tmp/workcell_builder_preview/<scene_name>/`.
- The Object Placement Manager can import this feedback for operator review before any pose update is applied.
- Only feedback entries that match existing placed object names are pose-updated.
- Unknown object names, malformed pose rows, and other invalid entries are warned and skipped.
- There is no automatic overwrite of `environment.yaml`, `scene_manifest.yaml`, `cell_definition.yaml`, or generated package files.
- **Save/Generate remains an explicit user action** after review and optional apply.
- No MoveIt planning, no controller execution, and no real hardware execution are performed by this workflow.


## Persisting placed objects to real generated scenes
1. Add/import STL object.
2. Open STL preview.
3. Open interactive RViz preview.
4. Drag/rotate object.
5. Import RViz Pose Feedback.
6. Review/apply valid updates.
7. **Save Placed Objects to Scene YAML**.
8. Generate YAML / Generate Files.
9. Build scene package.
10. Launch generated scene in RViz/MoveIt fake hardware mode.

Preview files are temporary. `environment.yaml` is the scene persistence point. Real scene files are updated only by explicit save/generate actions. This workflow introduces no controller execution and no real hardware execution.
