# Workcell Studio Plan & Simulate Mode

Workcell Studio now exposes motion-readiness as explicit operating modes:
- **Design**
- **Plan**
- **Simulate**
- **Hardware Guarded**

## Mode intent

- **Design**: edit scene/canvas/task metadata.
- **Plan**: open RViz2/MoveIt launch for planning workflows.
- **Simulate**: run fake-hardware launch and observe simulation motion.
- **Hardware Guarded**: real hardware remains locked by default and is not launched from this page.

## Default launch behavior

Plan & Simulate uses fake-hardware-first launch commands from the selected scene package:

```bash
source install/setup.bash
ros2 launch <scene_name> demo.launch.py use_fake_hardware:=true launch_rviz:=true
```

If a scene `launch/demo.launch.py` declares `launch_task_preview`, Studio appends `launch_task_preview:=true` for compatibility with preview-capable scenes.

## Allowed vs locked behavior

- **Allowed in this mode:** simulated motion through fake hardware, RViz2/MoveIt planning UI, launch command copy.
- **Locked in this mode:** real hardware execution (`use_fake_hardware:=false` is not wired to a launch button).

## Safety rationale

`use_fake_hardware:=true` remains default to preserve safe-first operation, reproducible planning checks, and compatibility with generated Workcell Studio scenes without requiring Gazebo/Isaac as mandatory runtime dependencies.
