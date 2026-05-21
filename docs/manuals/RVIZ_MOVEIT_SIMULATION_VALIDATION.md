# RViz/MoveIt Simulation Validation

## Purpose and Scope

This manual defines validation steps for **RViz/MoveIt simulation launch only**. It verifies that the simulation stack starts correctly, key nodes come up, and expected simulation-only behavior is observable.

This document is **not** a procedure for validating real robot hardware, field safety, or production readiness.

## Validation Modes

### 1) Headless (CI-friendly) Validation

Use headless mode when validating launch behavior without a GUI.

Example commands:

```bash
ros2 launch <your_package> <your_launch_file>.launch.py launch_rviz:=false use_fake_hardware:=true
```

```bash
ros2 launch <your_package> <your_launch_file>.launch.py launch_rviz:=false use_fake_hardware:=true --show-args
```

Recommended for automation:
- CI pipelines
- Remote test environments
- Fast launch-regression checks

### 2) Manual GUI Validation (RViz)

Use GUI mode when a human operator needs to visually verify planning scene and interaction behavior.

Example commands:

```bash
ros2 launch <your_package> <your_launch_file>.launch.py launch_rviz:=true use_fake_hardware:=true
```

```bash
ros2 launch <your_package> <your_launch_file>.launch.py launch_rviz:=true use_fake_hardware:=true rviz_config:=<path_to_config.rviz>
```

Recommended for:
- Visual sanity checks
- Interactive planning checks in simulation
- Operator sign-off for simulation workflows

## Result Definitions

Use the following standardized outcomes:

- **PASS**: Launch completed and expected simulation components behaved correctly.
- **WARN**: Launch succeeded with non-blocking issues (for example: recoverable warnings, delayed non-critical topics).
- **FAIL**: Validation objective not met (launch failure, critical node missing, fatal runtime issue).
- **SKIP**: Validation intentionally not executed (for example: dependency unavailable, test explicitly deferred).

## JSON Report Requirements

Store machine-readable results at:

- `artifacts/reports/rviz_moveit_simulation_validation.json`

Minimum required key fields:

- `validation_name` (string)
- `timestamp_utc` (string, ISO-8601)
- `mode` (string: `headless` or `gui`)
- `launch_rviz` (boolean)
- `use_fake_hardware` (boolean)
- `status` (string: `PASS` | `WARN` | `FAIL` | `SKIP`)
- `summary` (string)
- `checks` (array of per-check objects)
- `errors` (array)
- `warnings` (array)

## Real Hardware Exclusion (Explicit)

This validation is **simulation-only** and is **not real-hardware validation**.

No result from this document alone may be used as evidence that physical robot hardware is safe, calibrated, or ready for operation.

## Hardware Gate (Explicit)

Set `use_fake_hardware:=false` **only after**:

1. Manual readiness gates are completed and approved, **and**
2. A separate, documented hardware validation process is executed.

Any attempt to run with `use_fake_hardware:=false` before these conditions is out of scope for this manual.
