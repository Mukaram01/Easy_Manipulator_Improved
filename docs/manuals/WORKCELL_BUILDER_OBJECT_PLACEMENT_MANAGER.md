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

## Placed object end-to-end generation

1. Import an STL file in Object Placement Manager (or select an existing managed STL asset).
2. Confirm the STL is stored in the managed asset path and place it into the scene with initial XYZ/RPY pose values.
3. Open RViz STL Preview to generate temporary preview artifacts under `/tmp/workcell_builder_preview/<scene_name>/...`.
4. Optionally refine object pose in interactive RViz preview and export feedback to `placed_objects_feedback.yaml` in the same temporary preview location.
5. Import RViz pose feedback into Object Placement Manager and review/apply valid pose updates by object name.
6. Save placed objects to scene YAML so `environment.yaml` is updated as the persistence source of record.
7. Run Generate YAML / Generate Files to regenerate scene artifacts from persisted data.
8. Confirm generated placed-object URDF/Xacro outputs are produced; these generated URDF/Xacro files are the runtime source consumed by RViz/MoveIt visualization flows.
9. Build the generated scene package/workspace so runtime launch files resolve the updated generated artifacts.
10. Launch the demo/preview runtime (typically fake hardware mode) and verify the placed objects appear correctly in RViz/MoveIt.

**Important runtime boundary**

- Preview files are temporary and live under `/tmp/workcell_builder_preview/...`; they are not persistent scene truth.
- `environment.yaml` is the persistence source for placed object data.
- Generated URDF/Xacro is the runtime source used by RViz/MoveIt after generation/build.
- This feature does not enable robot motion, controller execution, or real hardware execution.



## Camera placement and frustum preview
1. Add camera.
2. Set XYZ/RPY.
3. Save Cameras to Scene YAML.
4. Open Camera Frustum Preview.
5. Generate YAML.
6. Confirm cell_definition.yaml camera block.
7. Later use metadata for EPD/RealSense integration.

This is visual/configuration only: it does not start RealSense hardware, does not start EPD, and does not enable robot motion.
