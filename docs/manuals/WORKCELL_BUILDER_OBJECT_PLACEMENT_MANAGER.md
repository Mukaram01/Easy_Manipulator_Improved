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
