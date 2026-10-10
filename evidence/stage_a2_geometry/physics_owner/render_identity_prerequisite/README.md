# Renderer identity prerequisite — BLOCKED

Baseline: `917f39dced28d8c40a60aa29d22d6e476a7f32aa`, isolated PR #3176.
**Stop gate reached before any genuine capture.** The existing separated-cube
RGB-D world explicitly selects `<render_engine>ogre</render_engine>` (Ogre 1).
Its installed rendering plugin cannot create a segmentation camera through the
supported public API. No physical-object-to-pixel witness is available on this
path, so EPD identity, capture-state penetration and contact evidence remain
BLOCKED. Historical six-state numerical qualification is not applied to a new
camera frame.

## Working prerequisite probe

`stage_a_render_identity_probe` is an opt-in target in the existing capture
CMake project, selectable without linking EPD or constructing a physics world.
It uses `ignition::rendering::engine`, `RenderEngine::CreateScene` and
`Scene::CreateSegmentationCamera` against the actual installed rendering plugin.
It creates an empty scene, attempts camera construction, records the result and
actual loaded rendering library paths, then destroys the scene. It never renders
an image, steps physics, publishes robot commands or grants contact authority.

`ogre.json` is the actual native result:

```json
{"decision":"BLOCKED","engine_name":"ogre",
 "segmentation_camera_created":false,
 "reason":"SEGMENTATION_CAMERA_UNSUPPORTED",
 "contact_authority":false,"execution_goals":0,"moveit_started":false}
```

The loaded plugin is
`/usr/lib/x86_64-linux-gnu/ign-rendering-6/engine-plugins/libignition-rendering6-ogre.so.6.6.4`.
`provenance.json` records its SHA-256, supporting Ogre/core libraries, probe
binary hash and the original world definition/hash. This is a standalone
capability observation, not a loaded-world capture. `unknown.json` records the
independent unsupported-engine rejection. Every outcome, including successful
camera construction on another backend, remains BLOCKED: API availability alone
cannot qualify frame alignment, identity association or physical penetration.

Installed `BaseScene::CreateSegmentationCameraImpl` reports unsupported and
returns a null camera; `OgreScene` has no override. Installed `Ogre2Scene` does
have an override. Thus the generic SegmentationCamera header and installed
segmentation sensor package do not imply support in the selected renderer.
Changing to Ogre 2 would change the camera rendering path and require fresh
RGB-D, identity and timing qualification. That alternative was not silently
substituted for the existing genuine pipeline.

## Lifecycle inspection: correspondence remains unqualified

Matching Fortress 6.18.0 upstream `Sensors.cc` and `RenderUtil.cc` were inspected;
URLs/hashes are recorded. Their correspondence to all operations of the
installed binary has not been established and no timing claim is based solely
on matching version labels.

The inspected source sequence is:

1. DART contact evaluation uses the pre-integration state; DART then integrates
   positions. The existing owner separately records pre/post transforms.
2. `Sensors::PostUpdate` copies ECM data using `RenderUtil::UpdateFromECM` and
   records simulation time. `RenderUtil::Update` later applies queued entity
   poses on the render thread.
3. `SensorsPrivate::RunOnce` emits `PreRender`, sets the applied scene time,
   calls `scene->PreRender`, runs active sensors at `updateTimeApplied`, then
   calls `scene->PostRender` and emits `PostRender`.
4. Existing capture freezes synchronized RGB/depth/calibration messages and
   subsequently invokes genuine external EPD inference. Its snapshot timestamp
   denotes the image acquisition stamp, not the inference completion time.

Source-level event ordering alone does not witness the actual image's scene
poses or identify a DART step. Equal message stamps are insufficient. In
particular, an event subscription alone must not assume an old scene-time value
is the timestamp about to be assigned. No new render callback, motion enclosure,
frame association or synchronization authority was implemented after the
unsupported-renderer gate was demonstrated.

## Focused tests and build

Two native regression tests pass after a red/green cycle:

- Unknown engine → BLOCKED, no authority or execution goals.
- Actual existing Ogre backend → null segmentation camera, BLOCKED, loaded
  Ogre plugin witnessed, no authority or MoveIt.

The opt-in CMake target compiled successfully. The original capture build path
is unchanged when the option is OFF (default). Tests skip explicitly when the
rendering development packages or graphics context are unavailable; skipped CI
checks must not be presented as native renderer evidence.

Reproduce on this workstation with its existing disposable graphics display:

```sh
cmake -S scripts/stage_a_rgbd -B /tmp/stage_a_render_probe \
  -DSTAGE_A_RENDER_PROBE_ONLY=ON
cmake --build /tmp/stage_a_render_probe --target stage_a_render_identity_probe -j1
/tmp/stage_a_render_probe/stage_a_render_identity_probe ogre /tmp/new_render_probe.json
# Expected exit 2 and reason SEGMENTATION_CAMERA_UNSUPPORTED.
python3 -m pytest -q tests/test_stage_a_render_identity_probe.py
```

No Gazebo RGB-D/EPD capture was performed. The requested mask-association safety
suite, current-capture numeric evaluation and successful-association positive
case were not reached. A fabricated mask witness or synthetic PASS would not
resolve this prerequisite. All original EPD poses, dimensions, conservative
collision envelopes, 100 µm limit and downstream gates remain unchanged.
No installed-library edits, Stage-A1 edits, bridge bypass, MoveIt or execution.

**Exact next action:** qualify a disposable Ogre 2 RGB-D plus segmentation
camera path sharing one witnessed render-scene update, before attempting the
physical-object-to-genuine-EPD-mask association capture.
