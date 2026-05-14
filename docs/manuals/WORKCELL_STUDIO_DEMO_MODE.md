# Workcell Studio Demo Mode

Demo Mode provides a guided **offline** workflow inside Workcell Studio for investor/operator demonstrations.

## What Demo Mode does
- Validates the selected generated scene (acceptance validator).
- Reads offline smoke report status (report-only).
- Generates consolidated demo artifacts under `<scene>/demo/`.
- Provides copyable build and fake-hardware launch commands.
- Opens a clean offline dashboard HTML.

## What Demo Mode does not do
- Does **not** launch ROS nodes automatically.
- Does **not** call MoveIt services.
- Does **not** command robot motion.
- Does **not** enable runtime execution.

## Generated artifacts
- `demo/workcell_studio_demo_summary.txt`
- `demo/workcell_studio_demo_report.json`
- `demo/workcell_studio_demo_dashboard.html`

## Commands surfaced in Demo Mode
- Build command:
  - `colcon build --symlink-install --packages-select <scene_name>`
- Fake-hardware launch command:
  - `ros2 launch <scene_name> demo.launch.py use_fake_hardware:=true`

## Investor demo usage
1. Create/select a generated scene from Workcell Studio.
2. Open **Demo Mode**.
3. Click **Run Demo Readiness**.
4. Open dashboard and share summary/commands.

## Safety notes
- No robot motion commanded.
- Offline/fake-hardware preview only.
- Runtime execution remains disabled unless enabled elsewhere.
- EPD workflows remain separate from Demo Mode.
