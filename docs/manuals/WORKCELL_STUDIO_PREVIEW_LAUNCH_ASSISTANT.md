# Workcell Studio Preview Launch Assistant

Preview Launch Assistant now provides an operational, QProcess-backed, fake-hardware-only preview console.

## Safety gates
- Fake hardware only (`use_fake_hardware:=true`).
- Real hardware and runtime execution flags are blocked.
- PREVIEW_ONLY placeholder scenes are not launch-enabled.
- BLOCKED acceptance scenes are not launch-enabled.
- Preview launch requires explicit user click and confirmation dialog.

## Run Build
Run Build executes (only when explicitly clicked):
- `cd <workspace_root> && source /opt/ros/humble/setup.bash && colcon build --symlink-install --packages-select <scene_name>`

Build logs stream live into the Preview Launch console, and results are recorded in transcript artifacts.

## Run Fake-Hardware Preview
Run Fake-Hardware Preview executes:
- `cd <workspace_root> && source install/setup.bash && ros2 launch <scene_name> demo.launch.py use_fake_hardware:=true`

Before launch:
- scene validity and preview eligibility are checked,
- command safety denylist is enforced,
- confirmation dialog shows exact command and safety text.

## Stop Preview
Stop Preview only controls the local preview process started by the assistant:
1. `terminate()` request,
2. short wait,
3. `kill()` fallback if still running.

No controllers, hardware drivers, or robot motion commands are issued.

## Transcripts and logs
Written under `<scene_dir>/preview_launch/`:
- `build_session.json`
- `build_summary.txt`
- `preview_launch_session.json`
- `preview_launch_summary.txt`
- `latest_console.log`

## Copy and manual fallback
Copy buttons are available for build/source/launch/all commands.
If workspace root detection fails, Run Build/Run Preview are disabled and copy/manual mode remains available.

## Troubleshooting build failure
- Re-run acceptance and demo readiness first.
- Verify workspace root contains `src/` and selected scene package exists.
- Re-source ROS and workspace setup scripts.
- Inspect `preview_launch/build_summary.txt` and `preview_launch/latest_console.log`.

## Not supported
- Real robot hardware launch.
- Runtime execution enablement.
- Motion command dispatch.
