# Workcell Studio Quick Start

## Assumptions
- Workspace: `~/workcell_ws`
- Repo path: `~/workcell_ws/src/easy_manipulation_deployment`
- Temp acceptance output: `/tmp/workcell_studio_acceptance`

## Launch Workcell Studio
```bash
cd ~/workcell_ws
source install/setup.bash
ros2 run workcell_builder workcell_builder
```

## Basic flow
1. Create or select a scene.
2. Configure layout in canvas.
3. Generate/Update Task Intent.
4. Generate Scene Package.
5. Run offline validation.

## Plan & Simulate command
```bash
ros2 launch <scene_name> demo.launch.py use_fake_hardware:=true launch_rviz:=true
```

## Acceptance gate quick command
```bash
python3 scripts/run_workcell_studio_acceptance_gate.py --mode scratch --workspace ~/workcell_ws --output /tmp/workcell_studio_acceptance
```
