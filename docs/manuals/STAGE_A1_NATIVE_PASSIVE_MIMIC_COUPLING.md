# Stage-A1 — Native passive mimic coupling for the simulated Robotiq 2F-85

This supersedes the mimic-follower effort-cap hypothesis in Draft PR #3191, which remains unmerged.

## Problem

The URDF tree breaks the 2F-85 four-bar. Both pads hang off passive mimic
followers (`gripper_finger*_inner_knuckle_joint`, `gripper_finger*_finger_tip_joint`).
Stock `ign_ros2_control` enforces each follower separately, with a velocity
servo capped by that follower's `<limit effort="0.1">`. So in the stock
model the 0.1 N·m caps set grip force, not the actuated leader. That is
the mechanism behind the 2025-09-25 `runtime::part_07` / grasp 7 loss of
`gripper_finger1_finger_tip_link` contact at `EXECUTE_LIFT`
(`evidence/stage_a1/retirement_20260925/release_failure/report.json`).

## Correction

`scripts/simulator_backend.py` (opt-in via `WORKCELL_SIMULATOR_PHYSICS_ROOT`)
writes each authored flat ±1 URDF mimic into the bound SDF. A pinned, patched
DART physics plugin (`scripts/simulator_physics_repro/passive-joint-coupling.patch`)
enforces it as a bilateral constraint row with unbounded impulse. That row
transmits the equal and opposite reaction to the driven joint. Followers have no
independent actuator, and their limits are unchanged. The only actuator bound is
the leader, `gripper_finger1_joint`, at 10.114 N·m: half the rated 235 N grasp
force times the minimum 0.08608 m/rad pad-gap Jacobian, about 117 N per pad.
This is a simulation bound, not a measured motor torque.

## Why the follower effort-cap change (#3191) is not adopted

| Configuration | Coupling mechanism | Grip authority | Note |
|---|---|---|---|
| Stock, source URDF (historical) | 4 independent follower servos | ≤ ~1.75 N per pad (0.1 N·m caps) | reproduces LIFT contact loss |
| #3191 on stock | same servos, caps raised to τ_l/\|m\| = 1000 N·m | ≤ ~17.5 kN per pad | ~75× above the 2F-85 maximum; not realistic |
| #3191 applied on this branch | servos not used for passive mimics | caps would be 10.114 N·m, but have no effect | redundant |
| **This branch (native)** | DART bilateral constraint, reaction to leader | leader 10.114 N·m ≈ 117 N per pad | within the 20–235 N spec |

No 1000 N·m follower authority is introduced.

## Evidence (workstation, 2026-10-04, stock-versus-native build `simulator_physics_stage_a1_20261004_passive_union`)

- Measured coupling over 15,524 physics frames: maximum |q_f − m·q_l| is 6.26e-8 rad,
  against the unchanged 0.02 rad guard.
- Stationary retention PASS, `runtime::part_06` grasp 1: 5.0 s simulated time,
  5,003 samples, 10.1 µm / 1.5e-4 rad tool-object motion, clean shutdown.
- That run is stationary only. Lift, transfer, release and home are not yet
  qualified on this coupling.

No contact, slip, retention, freshness, coupling, ACM or pile threshold is changed.
