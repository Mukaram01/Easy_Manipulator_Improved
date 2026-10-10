# Live Gazebo MRT integration — BLOCKED before draw

Baseline: `1cb3f6f1`. Exactly **one** disposable native experiment was executed.
No retry, external EPD, MoveIt, controller, execution goal or contact permission.

## Implemented path (build PASS, draw NOT REACHED)

`gazebo_fragment_capture.cpp` installs an opt-in `ISystemPostUpdate` capture owner
through `Server::AddSystem`. That owner holds Gazebo's actual `RenderUtil`, calls
`UpdateFromECM` / `Update` on the render thread and accesses its public
`SceneManager()`. It does not borrow another System's private memory or create
a second physics engine.

The compiled draw integration looks up each visual with `VisualById(Entity)`,
checks the complete model/link/visual/collision inventory against the existing
owner record, joins the exact collision/ShapeNode binding, and obtains the
existing geometry's public `OgreObject()`. It requires a uniquely attached Ogre
`Item`; it creates neither replacement geometry nor an ID overlay. The actual
native `Visual::Id()` is bound as a uint uniform, with a separate full-width
Gazebo entity map. Unexpected Items, legacy Entities or ambiguous ownership fail
closed. Native material bindings are checked after acquisition as well.

The proven compositor, reservation, scene-graph preparation, typed attachment
clear and lossless readback are shared in `fragment_mrt.hh`; the original fixture
retains its geometry and acceptance checks. Both executables compile. The
previously executed successful fixture ELF remains untouched at SHA256
`3e998fd6cd255f8330a83f918c7a1235159bd76dfa35f53b55cb99084530fd9c`.
No new standalone fixture run occurred.

The disposable live RGB profile is **opaque unlit diffuse RGB**, using the actual
Gazebo material colour in the same fragment shader that emits R32_UINT identity.
It is genuine rendered RGB, not an ID colourisation or simulator mask. It is not
claimed byte-equivalent to stock PBR appearance or qualified for EPD inference.
The exact captured RGB would be retained without image alteration for eventual
EPD input. This shader/draw path has not yet executed successfully.

## Actual experiment

Executed ELF SHA256:
`df95b1e81e5660732b99f30f787540eb9995bc53144a17218aa88eab4f2d4d6c`.

```sh
python3 scripts/stage_a_gazebo_fragment.py \
  --world /tmp/workcell_stage_a2_separated/physical_world.sdf \
  --binary /tmp/stage_a2_live_fragment_build/stage_a_gazebo_fragment_capture \
  --owner /tmp/stage_a2_owner_build/libworkcell_owner_physics.so \
  --output /tmp/stage_a2_live_identity_20261010_1cb3f6f1
python3 scripts/stage_a_gazebo_fragment.py \
  --output /tmp/stage_a2_live_identity_20261010_1cb3f6f1 --run-prepared
```

The runner exclusively created its output, verified the ELF/source/world hashes,
and claimed one execution before invoking `LIBGL_ALWAYS_SOFTWARE=1 timeout 60s`
with a unique transport partition. It never automatically retries. The retained
input world and derived world prove that the two original separated cube models
were copied unchanged; support/bin were omitted from this identity-only world.
No production world or planning envelope changed. This is not a support-contact
experiment and no historical penetration result applies.

Actual stderr:

```text
Failed to load system plugin [/tmp/stage_a2_owner_build/libworkcell_owner_physics.so] : couldn't load library on path [...].
Tried to convert SDF [world] into [plugin]
```

The native failure report says `missing/stale owner record`. No `owner.jsonl`
was produced. The process terminated with **SIGILL**, subprocess return code
**-4**; `timeout` also reported a core dump. The precise SIGILL cause is unknown.
`ldd -r` on the hash-matched owner ELF reports no missing dependencies or
unresolved symbols; it does not prove Gazebo plugin registration/loading.

Actual `RenderUtil` updates were recorded at iterations **1 / 2**, simulation
times **1,000,000 / 2,000,000 ns**, with corresponding `RenderUtil::SimTime`.
These observations alone do **not** establish a live DART step, physical/render
alignment or synchronization authority. The owner was absent. The mapping,
MRT draw, attachment readback and pixel acceptance were never reached.

- New RGB/ID buffers: **NOT PRODUCED**. New pixel counts: **unavailable**.
- Visual → collision → ShapeNode → draw result: **BLOCKED**.
- Renderer/physics timing, EPD association and contact authority: **BLOCKED**.
- Loaded paths were witnessed before Server teardown and retained in the native
  report; `result.json.gz` contains post-process file hashes of those paths.
  The owner ELF hash is preflight provenance only, **not loaded-library proof**.
- Driver string: **not recorded before failure**. Software rendering was
  requested; no new llvmpipe image-production PASS is claimed.

## Focused checks

Both the live target and refactored original fixture build PASS. **72 focused
Python tests PASS**, including retained original GPU bytes, >32-bit entity IDs,
ownership ambiguity, stale render updates, unexpected/replaced geometry,
incorrect RGB/ID correspondence, disposable-world preflight and refusal of a
second execution claim. These tests are CPU checks, not live draw qualification.

```sh
cmake -S scripts/stage_a_rgbd -B /tmp/stage_a2_live_fragment_build \
  -DSTAGE_A_RENDER_PROBE_ONLY=ON -DSTAGE_A_GAZEBO_FRAGMENT_CAPTURE=ON
cmake --build /tmp/stage_a2_live_fragment_build \
  --target stage_a_gazebo_fragment_capture stage_a_fragment_mrt -j1
python3 -m pytest -q tests/test_stage_a_gazebo_fragment_runner.py \
  tests/test_stage_a_fragment_identity.py \
  tests/test_stage_a_visual_collision_identity.py \
  tests/test_stage_a_live_fragment_identity.py
```

**Single next action:** repair and validate the existing owner ELF's loading
through Gazebo `SystemLoader` in a graphics-free preflight. Surface the actual
loader error before any further rendering experiment. No installed-library edit
or alternative physics backend is warranted by this failure.
