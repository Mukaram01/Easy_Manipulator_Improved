# Workcell Studio Scene Readiness Gate

The canonical gate runner orchestrates fake-hardware-only RViz/MoveIt launch validation and the all-scenes Workcell Studio audit, then emits a unified readiness report.

> This is fake-hardware simulation readiness only, not real-hardware validation.

Safety posture:
- Launch validation is driven through fake hardware (`use_fake_hardware:=true`).
- The runner is not a real-hardware validator and must not be used to start real drivers.
- Missing ROS runtime or missing launch evidence cannot be treated as PASS readiness.

## Commands

Dry-run:

```bash
python3 scripts/run_workcell_studio_scene_readiness_gate.py --dry-run-launches
```

Fake-hardware simulation run:

```bash
python3 scripts/run_workcell_studio_scene_readiness_gate.py --timeout-sec 45
```

Manual GUI run:

```bash
python3 scripts/run_workcell_studio_scene_readiness_gate.py --launch-rviz --timeout-sec 60
```

Strict CI run:

```bash
python3 scripts/run_workcell_studio_scene_readiness_gate.py --dry-run-launches --strict
```
