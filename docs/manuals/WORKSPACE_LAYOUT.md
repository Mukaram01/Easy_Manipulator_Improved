# Humble Workspace Layout

Intended `~/workcell_ws/src` layout:

- `easy_manipulation_deployment`
- `assets -> easy_manipulation_deployment/assets`
- `scenes -> easy_manipulation_deployment/scenes`
- other dependency repositories

## What changed

- `scripts/fix_workspace_layout.sh` now creates only top-level `src/assets` and `src/scenes` aliases.
- Individual asset package symlinks in `src/` are no longer created.
- Legacy generated asset symlinks are safely pruned only when they are symlinks that resolve inside `easy_manipulation_deployment/assets`.
- Real directories, user folders, dependency repositories, and symlinks outside repo assets are never deleted.

## Why

This keeps workspace layout clean and avoids duplicate package discovery while preserving workcell_builder scene/asset discovery and generated-scene workflows.

## Manual validation

```bash
cd ~/workcell_ws/src/easy_manipulation_deployment

PYTHONPATH=$PWD:$PYTHONPATH PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q \
  tests/test_workspace_layout_assets_scenes_aliases.py \
  tests/test_humble_build_regressions.py \
  tests/test_workcell_builder_asset_picker.py \
  tests/test_workcell_builder_scene_manager_ux.py \
  tests/test_workcell_builder_healthcheck.py

cd ~/workcell_ws

./src/easy_manipulation_deployment/fix_and_build_humble.sh \
  --workspace ~/workcell_ws \
  --check-prereqs \
  --build \
  --profile full

ls -la ~/workcell_ws/src | grep -E "assets|scenes"
find ~/workcell_ws/src -maxdepth 1 -type l -printf "%f -> %l\n" | sort
colcon list --base-paths ~/workcell_ws/src --names-only | sort | uniq -d

source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select workcell_builder --event-handlers console_direct+

cd ~/workcell_ws/src/easy_manipulation_deployment
python3 scripts/validate_workcell_builder_healthcheck.py \
  --repo-root . \
  --workspace ~/workcell_ws \
  --skip-colcon \
  --skip-launch
```
