# Workcell Studio: RGB-D camera source selection (Feature 3, first slice)

## Product behaviour

Use the existing Workcell Builder **Task → Object input** controls. The
**Perception mode** remains Off (commissioning objects), Replayed observations,
or Live (normalized EPD adapter snapshot). When Live is selected, **Live camera
source** offers:

- **Simulated RGB-D (Gazebo + EPD)**: `simulated_rgbd`
- **Intel RealSense D435i (EPD)**: `realsense_d435i`

Only live mode enables the camera selector. A non-live selection clears the
camera source. The selected source is stored in the existing authoritative
`config/workcell_builder_task_intent.yaml` under
`perception.camera_source`, alongside `perception.mode` and the explicit
`perception.replay_path` input binding. It round-trips through the existing
Save Layout/Save Task and normal task validation/generation. It is **not** a
second task model or a physical camera pose authority: physical camera
mounting, pose and frames remain in the existing scene/environment contract.

## Fail-closed live input binding

`scripts/task_intent_authoring.py::planning_input` consumes only the saved
observation path. For `live_epd`, it requires a selected camera source and
an actual `detected_objects/v1` snapshot with a source declaration agreeing
with the selection, for example:

```yaml
schema_version: detected_objects/v1
source:
  mode: live_epd
  camera_source: simulated_rgbd  # or realsense_d435i
objects:
  # Current, measured, world-transformed, collision-usable observations
```

An existing source with a missing or mismatched `source.camera_source` is
blocked as `CAMERA_SOURCE_MISMATCH`, not inferred from a filename, camera
topic or replay mode. Live observations declaring `replayed_snapshot` are
rejected before camera checking. Missing input is blocked. The existing
object freshness, class, zone, geometry, TF and full-cycle MoveIt constraints
remain authoritative. Selecting a source does not start a camera, create
depth/collision geometry, apply TF automatically, or enable robot motion.

The existing wizard's `live_epd_realsense` source now seeds
`camera_source: realsense_d435i`; existing Off and Replay paths continue
without a live camera source. Previously-authored `live_epd` tasks must
explicitly select their source before consuming live observations.

## Qualification and remaining work

This is **configuration and pre-plan input validation only**. It does not
claim a live D435i session, Gazebo bridge commissioning, segmentation mask
accuracy, 3D collision-geometry acceptance, real-time camera preview,
physical grasp retention or real robot actuation. EPD remains a separate
repository and owns camera inference. Simulator and D435i adapters must
publish the correct, measured source provenance; they must not be modified
to forge identity or accept stale/frozen simulated frames as live. Their
existing qualification blockers remain active.

The subsequent workstation acceptance is:

1. Build `workcell_builder` on Ubuntu 22.04 / ROS 2 Humble and run the Qt
   TaskIntentEditor test suite, including the new save/reopen/source test.
2. Run `python3 -m pytest -q tests/test_task_intent_authoring.py`.
3. Create a fresh UR5+2F cell in the GUI, set both sources in turn, Save,
   close and reopen, and check exact selection and source bindings.
4. Verify missing, mismatched, replay-stamped and stale live observations
   block before planning, with no replay substitution.
5. Only after genuine adapter qualification, demonstrate separately each
   camera's colour/depth topics, calibrated TF, measured collision geometry,
   live preview, and plan-only runtime; record evidence before claiming the
   full Feature 3 acceptance. Fake hardware remains default.

No camera or controller is automatically launched and no execution action
goals are sent by this feature.
