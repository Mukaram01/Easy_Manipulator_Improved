# Workcell Studio Web 3D transform convention

Generated mesh renderables exported by `scripts/export_workcell_studio_web_scene.py` are rendered in ROS world, Z-up coordinates. For generated preview meshes, the exporter publishes one canonical final pose under `final_transform` (also mirrored as `world_from_visual` for explicit naming). That transform is the baked world-from-visual pose: the link world pose multiplied by the URDF visual origin before the payload reaches browser code.

The web viewer must treat `final_transform` / `world_from_visual` as the complete render transform for a generated mesh. Browser code must not re-apply joint origins, link transforms, or `visual_origin` to generated mesh renderables, because those components are already baked into the canonical transform. Diagnostic fields such as `transform_source`, `link_world_pose`, `visual_origin`, `mesh_scale`, `baked_world_visual_matrix`, and `baked_world_visual_quaternion` are included only for inspection and troubleshooting.

Fallback order for generated preview mesh transforms is:

1. `baked_world_visual_pose` exported as `final_transform` / `world_from_visual`.
2. `pose` when no baked world visual pose is available.
3. `world_pose` when neither baked nor pose fields are available.

Authored layout/environment items may continue to use their authored `pose`, `pose_xyz`, and `pose_rpy` fields because they are not generated URDF mesh renderables.
