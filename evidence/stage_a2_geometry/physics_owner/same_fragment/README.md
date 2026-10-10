# Same-fragment RGB / integer-ID fixture — production PASS; physics mapping BLOCKED

## Preserved manual success (baseline 2753f7ab)

The user's existing one-line `sm->updateSceneGraph()` change was preserved without reset/stash/clean or overwrite. The existing capture `/tmp/stage_a2_fixed_20261010_200705/` was independently read and validated; **no new GPU execution** was needed. ELF SHA256 matches the user's verified executed identity: **`3e998fd6cd255f8330a83f918c7a1235159bd76dfa35f53b55cb99084530fd9c`**. Current source SHA256 is `aa3d89c5c8fba4f2f03a6aa77cb78116e8ec4de64a297f675b7d1047d7213aed`. The manual exit 0 is user-reported; no independent process exit log is retained. Native `fixture_result=PASS` and image contents are independently verified. Hashes were verified at preservation, not falsely described as a new pre-run witness.

- Actual nonblank RGB: 256×256×3 uint8, 196608 bytes.
- Actual IDs: 256×256 uint32, 262144 bytes, exactly **0: 40612 pixels; 16777217: 21756 pixels; 4000000022: 3168 pixels**. No unexpected/invalid IDs or floating-point round trip.
- Centre [128,128]: **16777217**, the intended nearer cube in the overlapping fixture. Both objects remain visible. This qualifies tested fixture occlusion, not arbitrary scene visibility or numeric depth accuracy.
- Actual colour attachment formats: GL_RGBA8 (32856) and GL_R32UI (33334), each 256×256. One shared scene-pass callback; post-pass GL depth-test and write states both true; materials explicitly configure depth check/write and disable blending/MSAA.
- Driver: **llvmpipe (LLVM 15.0.7, 256 bits)**; GL4.5 Core Mesa23.2.1. Exact original capture and buffers are retained losslessly compressed under [accepted_manual](accepted_manual/acceptance.json). Actual loaded paths are preserved; their current file hashes are separately labelled as preservation-time hashes.
- Existing strict CPU diagnostic executed against the **actual** RGB/ID buffers: PASS. Scene-graph ordering regression added, checking omission/misordering in memory without altering the manual source. **25 focused CPU tests PASS**. Fixture-derived masks used for diagnostic checks are test inputs, not EPD inference.

Only same-fragment production and the tested fixture's occlusion/uint32 readback are PASS. Gazebo visual/collision mapping, genuine EPD association, DART/render alignment, physical penetration and extraction remain BLOCKED/NOT RUN. No new motion goals, permissions, planning geometry or safety changes. Older failure attempts below remain historical.

## Reservation fix and sole run (baseline 214a30e2)

Inserted **`nd->setNumTargetPass(1)` immediately before `nd->addTargetPass("fragment_mrt")`**, matching the installed Ogre2.2 public API's obligatory reservation. Added the explicitly requested static ordering regression: observed RED before the fix, then **24 focused CPU tests PASS** after it. Opt-in MRT target build PASS. The only additional fixture changes record existing callback count and post-pass GL depth-test/write state for acceptance; no rendering refactor or shader/material change. Camera parent fix remains unchanged.

Pre-run source SHA256: `b01a706970a34df62e9ca90e02d33d0722823f595b61f25cb7a1be581a13f476`.
Executed ELF SHA256: **`6aca19a505cce4a0e10a2a9382c446969fd9968167f13f2fb960887f0d4c397c`**.
Both were saved before execution; output/report/RGB/ID paths were checked absent.

```bash
LIBGL_ALWAYS_SOFTWARE=1 timeout 60s /tmp/stage_a2_fragment_build/stage_a_fragment_mrt /tmp/stage_a2_fragment_reserved.json
```

**BLOCKED:** this task's sole execution terminated SIGABRT (Python return **-6**) after **0.875 s**. The target-pass assertion is cleared; execution reached scene culling, then asserted:

```text
stage_a_fragment_mrt: /build/ogre-next-UFfg83/ogre-next-2.2.5+dfsg3/OgreMain/src/OgreSceneManager.cpp:1291: virtual void Ogre::SceneManager::_cullPhase01(Ogre::Camera*, Ogre::Camera*, const Ogre::Camera*, Ogre::uint8, Ogre::uint8, bool): Assertion `!mEntitiesMemoryManagerCulledList.empty()' failed.
```

[Exact retained evidence](reservation_attempt/result.json) contains preflight/ELF/source hashes, build log, full stdout/stderr, elapsed time/exit status and artifact hashes. Stdout is empty; stderr also contains the EGL software-rendering warning and timeout core-dump diagnostic. No native report or image buffers exist. Actual RGB, IDs/counts, attachment formats, pass count, depth-state readback, driver/loaded-library witness, uint32 preservation and occlusion are **NOT REACHED / unavailable**. No second GPU execution. Strict captured-buffer diagnostics/mutation tests were NOT RUN because no captured buffers exist. CPU fixtures are not substituted for native evidence.

**One next fix:** prepare the native SceneManager with **`sm->updateSceneGraph()` before the manual workspace update**. Narrow source inspection shows `updateSceneGraph()` calls `highLevelCull()`, which populates the asserted culled-manager list, then updates transforms/bounds; the public installed header describes it as scene preparation. Merely reserving compositor targets cannot establish this state. That next fix was **not implemented or tested** here. No broader investigation, renderer replacement or GPU draw inspection.

The acceptance milestone remains BLOCKED. Gazebo visual/collision mapping, genuine EPD association, DART/render synchronisation, physical penetration and extraction are unchanged and unqualified. No Gazebo/EPD, MoveIt, controller or execution goals; no collision permissions, installed-library or protected Stage-A1 modifications. Original EPD poses/envelopes and 0.1 mm threshold preserved. PR #3176 stays draft. Older attempts below are historical and do not replace this run's failure.

## Corrected-binary verification attempt (baseline 36c2637f)

**BLOCKED before rendering.** Exactly one newly authorised bounded execution used the unchanged corrected ELF SHA256 `091aa50cb926b4a97e13df01ab5c5f27ef215afd69e08c49a33b2633f3e5b5c6` and source SHA256 `c2cceb47fbb5d57186f3b591ec564cbe083e677a5386b4c61d8247f8bc1e3b93`. HEAD/clean checkout, committed source equality, camera attachment fix, unchanged installed swrast hash and all-new output paths were verified before execution. No rebuild or source modification preceded this run.

```bash
LIBGL_ALWAYS_SOFTWARE=1 timeout 60s /tmp/stage_a2_fragment_build/stage_a_fragment_mrt /tmp/stage_a2_fragment_verified.json
```

The supervised command terminated with **SIGABRT** (Python subprocess return code **-6**) after approximately **0.795 s**. Complete stderr:

```text
libEGL warning: Not allowed to force software rendering when API explicitly selects a hardware device.
stage_a_fragment_mrt: /build/ogre-next-UFfg83/ogre-next-2.2.5+dfsg3/OgreMain/src/Compositor/OgreCompositorNodeDef.cpp:46: Ogre::CompositorTargetDef* Ogre::CompositorNodeDef::addTargetPass(const String&, Ogre::uint32): Assertion `mTargetPasses.size() < mTargetPasses.capacity() && "setNumTargetPass called improperly!"' failed.
timeout: the monitored command dumped core
```

Stdout is empty. No native JSON report, RGB bytes or ID bytes were produced. Actual driver identity, loaded-library inventory, attachment formats, render-pass count, depth-test/write state, visible IDs/counts, exact uint32 preservation and occlusion are **unavailable / NOT REACHED**, not PASS. The pre-run installed swrast hash is not a substitute for a run-time loaded-library or llvmpipe identity witness. Zero pixels qualify. Actual-buffer diagnostics and retained-capture mutation tests were **NOT RUN because no buffers exist**; previous synthetic CPU tests are not native evidence.

The installed public `OgreCompositorNodeDef.h` documents `setNumTargetPass` as obligatory. The minimal next fix is **`nd->setNumTargetPass(1)` before `nd->addTargetPass("fragment_mrt")`**. It was not implemented or rerun in this execution-only task. This assertion is not evidence that MRT is unsupported. Stop here; no further investigation or graphics execution was attempted.

[Retained attempt](verified_attempt/result.json) includes preflight hashes, exact invocation/termination, full stdout/stderr and artifact hashes. Historical evidence below remains unchanged. Baseline 36c2637f had Humble, Jazzy and both security checks PASS at inspection; the evidence follow-up commit has separate CI. Gazebo visual/collision mapping, genuine EPD association, DART/render synchronisation, current-frame physical penetration and extraction remain BLOCKED/NOT RUN. No installed libraries, protected Stage-A1, planning poses/envelopes, 0.1 mm threshold or collision permissions were changed. No Gazebo/EPD, MoveIt, controller or execution goals.

Baseline `ab4be990c674b2259d82aa9896e9236a13970373`. **Supported API components compile; same-fragment output, occlusion and lossless runtime readback remain UNVERIFIED.** One bounded native fixture attempt failed before rendering. No Gazebo/EPD capture, MoveIt, controller or execution goals; no physics-owner, installed-library or protected Stage-A1 changes. No contact authority, envelope change or penetration-limit change.

## Implementation

Opt-in target `stage_a_fragment_mrt`, built only under existing `STAGE_A_RENDER_PROBE_ONLY`. Uses public Ogre2 SceneManager, v1 prefab cube/material, high-level GLSL program, TextureGpu and compositor RTV/workspace APIs. A single scene pass targets RGBA8_UNORM RGB and R32_UINT IDs with one D32_FLOAT depth attachment. One fragment shader writes both outputs; no second camera, segmentation pass, nominal projection or majority vote. Opaque depth-tested/depth-writing materials, no blending, no MSAA. The intended RGB is simple face-shaded red/green geometry rendering, not an ID-coloured segmentation replacement. This fixture does not claim equivalence to stock Gazebo PBS/RGB imagery or inference quality.

Two renderer-local integer IDs (16777217 and 4000000022) deliberately exceed exact binary32 integer range. They travel through uint uniforms and an integer output/readback; background 0, invalid 4294967295. They are **not Gazebo entities or collision IDs**. Native Entity inventory is enumerated and checked against the two allocated entities. An attachment callback verifies actual texture formats/dimensions, not per-draw GPU bindings. This is limited MRT attachment validation; the stopped draw-inspection route is not resumed. Source and loaded-library provenance are saved.

Ogre2.2 GL3Plus colour clear uses float clear operations. The implementation avoids them for the integer output: initialise the owned attachments with public typed `glClearTexImage`, then load both in the scene pass and clear only depth. Readback uses native R32_UINT and RGBA8, packs RGB without image transformation, and saves sibling buffers. The intended center pixel is covered by the near cube and also lies in the farther cube projection; both IDs must remain visible. These checks have **not reached execution**.

`stage_a_fragment_identity.py` supplies CPU consistency and strict mask diagnostics: reject unsupported formats/material contract, duplicate IDs/native entities, stale session/frame, missing/unknown visible IDs and changed sibling byte hashes. A nonempty boolean mask must contain exactly one non-background/non-invalid ID at unchanged RGB coordinates. It returns only a renderer-local integer, no planning pose or collision permission. Capture flags/hashes are input consistency checks, **not independent evidence that a GPU contract or EPD preprocessing is qualified**. The routine is not wired to EPD/native contact admission. It generates no masks.

## Sole run and corrected build

The executable SHA256 was saved **before** the one run: `37bc1a9cde0682a23816b1d8058aa5601bca200cde76889eb559ebcdeb8ac931`.

Command actually attempted:

```bash
LIBGL_ALWAYS_SOFTWARE=1 timeout 60s /tmp/stage_a2_fragment_build/stage_a_fragment_mrt /tmp/stage_a2_fragment_fixture.json
```

The run threw `Object already attached to a SceneNode or a Bone` before shader/material construction and before any RGB/ID image. Ogre2's `SceneManager::createCamera` already attaches the camera to the dynamic root, as documented in installed OgreCamera.h and [pinned SceneManager source](https://github.com/OGRECave/ogre-next/blob/0e0c47ed70091e7bdead5fb1ca01e1cae5857ef4/OgreMain/src/OgreSceneManager.cpp#L293). The probe incorrectly attached it again. The final source preserves and checks the existing root attachment. That correction **builds**, but was **not run** to obey the one-fixture maximum. The original failed-run ELF was copied before rebuilding; its pre-execution hash and the corrected ELF hash are separate. Failed-run source bytes are retained compressed. No corrected-build hash is substituted for the failed capture.

`failed_capture.json.gz`, `driver.log`, build logs and `provenance.json` preserve this failure and loaded libraries. Loaded swrast proves the software driver library was present; no successful image or llvmpipe rendering result is claimed from this attempt. **Zero qualified pixels; no RGB/ID images produced.** Previous camera/projection bounds do not qualify this new custom path.

## Validation and downstream gates

- Initial and corrected opt-in target builds: PASS (existing Ogre debug-configuration pragma note).
- `python3 -m pytest -q tests/test_stage_a_fragment_identity.py`: **23 PASS**. These are CPU record/array tests, not graphics/inference qualification. They cover two large integer IDs, mixed/empty/background masks, duplicate/missing IDs, duplicate native entity, incomplete inventory, stale session/frame, unsupported material/MSAA/depth/format, changed RGB/ID bytes and wrong dtype/shape.
- Native occlusion/shared-visibility/attachment/lossless-readback checks: **NOT REACHED**. No invented PASS or second fixture.
- Authoritative Gazebo visual → parent link/model → collision → owner ShapeNode mapping: **BLOCKED**, not implemented. Existing owner has collision-to-ShapeNode evidence, but this standalone fixture has no Gazebo visual inventory or current render/physics-step correspondence. No fixture label/name/order is substituted for that missing chain. Invalid mapping rejection cannot be presented as tested runtime behaviour of an absent mapping implementation.
- Genuine EPD association: **NOT RUN**, because the fixture and mapping prerequisites did not pass. Exact EPD preprocessing/image provenance is still required before use of the CPU mask diagnostic.
- Current-frame DART penetration, renderer/physics timing and continuous extraction: remain separate BLOCKED gates. Historical bounds are not imported. Original EPD poses/envelopes and the 0.1 mm threshold remain unchanged.

**One next action:** run the corrected, pre-hashed native MRT fixture once in the next explicitly bounded validation task; require actual RGB/uint-ID/occlusion evidence before implementing the Gazebo mapping or invoking EPD. No further GPU draw instrumentation is proposed. PR #3176 stays draft. Rollback removes this opt-in target, diagnostics and evidence only.
