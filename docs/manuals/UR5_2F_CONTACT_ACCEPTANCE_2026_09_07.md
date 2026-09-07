# UR5 / 2F contact continuation — 2026-09-07

`INDUSTRIAL_PICK_PLACE_V1 = PARTIAL`

The operator confirmed that the connected camera is loose and calibration is
unavailable. Its live observations cannot currently establish valid world-frame
pick coordinates. No trajectory was sent during this continuation. Physical
move-and-replan and the complete fake-hardware sequence remain unverified.

## Camera evidence

The saved authored pose is unchanged: XYZ `[0.4, -0.25, 0.85]` m, RPY
`[0, 1.570000000000127, 0]` rad. The RealSense macro applies this pose to
`camera_bottom_screw_frame`, not directly to the body or optical frame.
The runtime TF tree and MoveIt FK agree:

| Frame | World XYZ (m) |
| --- | --- |
| camera_bottom_screw_frame | 0.400000, -0.250000, 0.850000 |
| camera_link | 0.412508, -0.232500, 0.839410 |
| camera_color_optical_frame | 0.412508, -0.217500, 0.839410 |

The body collision box remains `0.02505 × 0.09 × 0.025` m, at the URDF's
specified local offset. Earlier `wrist_2_link ↔ camera_link` contacts concern
that modeled installation. There is no evidence here of a runtime FK/TF
disagreement causing those contacts, and no calibrated evidence that this
model matches the current physical installation. The earlier experimental
camera relocation was discarded. A collision-free real mounting pose has not
been established. RViz visual acceptance was not repeated in this continuation.

## Grasp and lifecycle fixes

- Resolve either a source directory or an installed ROS package before loading
  `cell_definition.yaml`. Missing/invalid TCP metadata now fails explicitly;
  an unresolved package name no longer silently supplies a zero TCP offset.
- Generate approach positions in the grasp frame, then convert using
  `T_world_tool = T_world_grasp × inverse(T_tool_grasp)`. The full authored
  translation and rotation are used. The URDF's fixed downstream joint does
  not reinterpret an IK goal addressed to `tool0`. The experimental 4 cm cap
  was discarded; authored approach and retreat distances are preserved.
- A narrow 229 mm long bottle is not rejected by the former unrelated 200 mm
  planar-length cap. Aperture, destination dimensions, and MoveIt collisions
  continue to gate the actual candidate. Size rejection is reported directly.
- APPROACH uses the normal world collision object. GRASP CONTACT requires a
  complete collision-checked Cartesian path and temporarily permits only the
  selected ID against the authored `allowed_touch_links`. Existing matrix
  entries/defaults are preserved. The original matrix is restored on success
  and on planning, execution, or attachment failure. With the arm stopped,
  normal collision checking measures actual contact; missing contact or any
  non-designated pair rejects attachment.
- ATTACH uses only those designated links, replacing the previous broad list
  containing tool, palm/base, and finger-body links. Environment collisions
  remain enabled during lift, transfer, and place.
- Detachment propagates the achieved FK transform, including rotation and local
  geometry offsets, instead of writing the ideal destination directly into the
  object's pose. A straight departure permits only the initial fingertip/target
  contact, then restores the matrix and validates separation before HOME.
- Initial-state checks and initial/final HOME handling were added. A failed
  transfer preserves its attachment for recovery instead of deleting held
  geometry. Fake placement retains bridge ownership until explicit scene
  recovery/reset because fake motion does not move the physical bottle; live
  observations must not teleport the simulated object back to the pick site.

These execution stages are implemented but have not passed the full runtime
sequence. Contact-force physics and a successful gripper grasp are not proven
by these code changes or unit tests.

## Execution controls

The launch default remains `allow_trajectory_execution:=false`. Explicit opt-in
is accepted only with `use_fake_hardware:=true`; the executor separately requires
every reported hardware component to be `mock_components/GenericSystem`.
Mixed mock/unknown hardware is rejected.

After physical calibration and planning validation, the fake execution launch
supports:

```bash
ros2 launch ur5_2f_test demo.launch.py use_fake_hardware:=true \
  allow_trajectory_execution:=true launch_rviz:=true
```

This option alone is not acceptance of an uncalibrated live target. The current
session did not execute that target or complete the pick/place cycle.

## Validation and limitations

Evidence directory on the workstation:
`/home/ubuntu/workcell_ws/industrial_v1_evidence/continuation/`.

- Focused existing suite plus regression cases: **91 passed** (`pytest.log`).
- Canonical workspace build of `workcell_builder` and `ur5_2f_test`:
  **2 packages finished** (`build.log`).
- Fake launch and initial whole-robot state validity: **PASS**, no contacts
  (`frame_audit.json`). Three authored world objects remained present.
- MoveIt ACM service round trip: **PASS**, original matrix restored and initial
  state still valid (`acm_roundtrip.json`). This used an absent probe ID and
  added no object geometry; it is a service test, not grasp-contact acceptance.
- Default executor gate: **rejected as intended**, no trajectory attempted
  (`default_execution_guard.json`).
- Real-hardware execution launch: **rejected before nodes started**
  (`real_execution_rejected.log`).
- Explicit fake opt-in: **PASS**, two GenericSystem components and valid initial
  state, no trajectory sent (`fake_opt_in_audit.json`).
- RealSense reopened color/depth at **640×480, 15 FPS** with aligned depth;
  point cloud and IMU streams disabled (`realsense.log`).
- Read-only IK diagnostic using the previously recorded bottle pose: corrected
  grasp Z `0.665235` m corresponds to tool Z `0.805235` m; all eight top
  candidates returned **NO_IK_SOLUTION (-31)**
  (`historical_pose_ik_diagnostic.json`). No historical object was injected into
  the PlanningScene. This does not establish current live reachability.
- Current live A→B same-ID update/new plan: **NOT RUN**, camera is loose and its
  world transform is unknown.
- Full contact/attach/transfer/place/detach/retreat/HOME sequence: **NOT RUN**
  against a calibrated live target. The historic camera/path contacts are not
  declared resolved by changing the software's installation coordinates.

The remaining physical prerequisite is a fixed, measured camera mounting
transform relative to the authored cell. Then the actual perceived target must
pass approach/contact planning before fake trajectory execution acceptance.
