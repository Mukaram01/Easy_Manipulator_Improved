# Workcell Studio Plan & Simulate

Plan & Simulate mode replaces legacy Preview wording.

## Standard launch
```bash
ros2 launch <scene_name> demo.launch.py use_fake_hardware:=true launch_rviz:=true
```

## Controls
- Open RViz2 / MoveIt
- Run Fake-Hardware Simulation
- Stop Simulation

## Notes
- `launch_rviz:=true` is recommended for visibility.
- `use_fake_hardware:=true` is the default simulation path.
- Real hardware execution requires explicit guarded setup and is not launched from this mode.
