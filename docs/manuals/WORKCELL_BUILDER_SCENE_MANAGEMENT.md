# Workcell Builder Scene Management

- Default scenes folder: `<repo_root>/scenes`.
- Default assets folder: `<repo_root>/assets`.
- Existing scenes are discovered by scanning subfolders under `scenes/` and accepting any folder containing at least one marker: `package.xml`, `scene_manifest.yaml`, `environment.yaml`, `urdf/scene.urdf.xacro`, or `launch/demo.launch.py`.
- Use **Browse Scenes Folder** to switch scene roots for this session.
- Use **Refresh Scenes** to rescan without restarting.
- New scene output defaults to `scenes/<safe_scene_name>`.
- `/tmp` remains for temporary export/demo usage, not default package generation.
- Existing scene names are checked; collisions are blocked and user must rename or cancel.
- Safe package names are lowercased, spaces become `_`, only `[a-z0-9_]` kept, and leading digits are prefixed with `scene_`.
