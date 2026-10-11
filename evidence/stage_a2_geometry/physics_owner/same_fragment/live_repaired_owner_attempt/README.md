# Repaired-owner live Gazebo attempt — BLOCKED at inventory join

Baseline `eba8be03`. **Exactly one** fresh disposable live Gazebo experiment,
under `LIBGL_ALWAYS_SOFTWARE=1`, a unique transport partition, batch GDB and
`timeout 60s`. No retries. Two iterations; child exit **2**, debugger/supervisor
exit **2**, no SIGILL/SIGSEGV. No MRT draw, RGB/ID buffers or pixel counts.

## Implementation and preflight

The opt-in runner now consumes the prior graphics-free `owner_loader` acceptance:
its exact registered WorkcellOwnerPhysics class, clean exit, supported System
interfaces and the actual mapped owner ELF hash must agree with the current ELF.
It verifies retained qualification source hashes and prior inventory/readback
source hashes, and pins all consumed provenance files in the fresh preflight.
Immediately before the run it rechecks provenance, live source hashes, executable,
owner, debugger and world hashes; the single Physics System declaration,
session/trace configuration, two original cube inventories and absence of
sensors/joints/controller plugins; and fresh artifacts/execution claim.
Historical qualification and failed-run evidence are unchanged.

The **owner ELF is unchanged**, including physics stepping and mappings:
`0f4a2dda8d7bfd67745b03493254a9d91ea66bb36dcb2d78440fbf5809ff072a`.
Executed live ELF:
`86168c05f49b4070d98601bd39c33b1da5c042210ad030b72bfb10157c77065d`.
GDB and exact source/world hashes are in `preflight.json`.

Flushed stderr markers cover Server construction, the physics/PostUpdate boundary,
owner trace observation, RenderUtil initialization/update, inventory join, MRT
preparation/draw/readback and teardown. Physics Configure/Update observation is
explicitly **inferred from a valid owner step record**, not a new callback inside
the unchanged owner ELF. Native and GDB-side `/proc` maps survive absent final
JSON. GDB records fatal signals, registers, all-thread backtraces and shared
libraries if a signal occurs, then quits without continuing the inferior. It
starts the inferior once; no in-process signal handler or suppression is added.
No crash stack was produced this time because no fatal signal occurred.
The prior native failure JSON existed, and the previous source wrote it after
the Server scope; its remaining uninstrumented boundary was final capture-owner
destruction/process exit. This narrows the old failure phase, not its cause.

An oblique authored camera and actual native BOX-ray/pixel occlusion diagnostic
are implemented for the existing cubes. They were **not reached or qualified**.
The optional MRT lifecycle callback leaves the original fixture path unchanged.
No new standalone GPU fixture ran; the historical successful binary is preserved.

Preflight/build: live target and unchanged owner PASS; **91 focused Python tests**
and **2 native owner tests PASS**. New CPU tests cover stale/substituted owner
qualification, wrong class/loaded hash, duplicate plugins, session/trace mismatch,
unsupported controller/sensor structure, one execution claim and fatal child
reporting without a final capture JSON. A source-order regression caught marker
placement during preparation; its ordering was restored before the green tests,
final build and sole live execution. No failed prerequisite was carried into the
live run.

## Reproduction of the one actual attempt

```sh
python3 scripts/stage_a_gazebo_fragment.py \
  --world /tmp/workcell_stage_a2_separated/physical_world.sdf \
  --binary /tmp/stage_a2_live_fragment_build/stage_a_gazebo_fragment_capture \
  --owner /tmp/stage_a2_owner_build/libworkcell_owner_physics.so \
  --output /tmp/stage_a2_live_identity_repaired_20261010_eba8be03
python3 scripts/stage_a_gazebo_fragment.py \
  --output /tmp/stage_a2_live_identity_repaired_20261010_eba8be03 --run-prepared
```

The consumed original cube XML was independently compared byte for byte after
serialization with the disposable world. Physical geometry, materials and poses
were unchanged; support/bin remain omitted as in the prior identity-only task.
No historical support/penetration result applies to this frame.

## Actual evidence

- Session `606b9ed756004d28ab411647dd0c82f7`, Gazebo world entity **1**.
- Owner complete at physics step **2**, timestamp **2,000,000 ns**; DART frames 2,
  DART time 0.002 s. Actual owner and DART/ODE libraries were mapped and hashed.
  One owner plugin; no stock Physics System DSO in the retained mapped paths.
- Owner runtime backend: `ignition::physics::dartsim::Plugin`, path
  `/usr/lib/x86_64-linux-gnu/ign-physics-5/engine-plugins/libignition-physics-dartsim-plugin.so`.
- Owner collision **7** → physics shape **6** → ShapeNode **0x56a8856062d0**;
  collision **11** → physics shape **7** → ShapeNode **0x56a88560f810**.
  These pointers are valid only as identities in this recorded session.
- Owner topology includes cube visuals **6 / 10**, plus visual **13**, parent **12**,
  with `UNSUPPORTED_OR_MISSING` geometry and `opaque=false`. Its role is unproven.
- RenderUtil initialization and updates at iterations **1 / 2** completed.
  The observer read and validated the complete owner trace, then rejected
  `inventory == owner["identity_inventory"]` with **owner/render ECM inventory changed**.
- The render-side inventory value was not retained by this existing failure path;
  the exact entity/field difference is therefore **UNKNOWN**. The extra owner
  visual alone is not proof of the equality failure's cause. JsonCpp in-memory
  unsigned IDs versus parsed numeric types is also a possible representation
  issue, not a demonstrated runtime root cause. Neither is silently ignored.
- No SceneManager mapping, Item binding, compositor, attachment readback or
  occlusion check was reached. Renderer-local IDs and live pixel counts are
  **unavailable**. No live draw-to-collision PASS is claimed.
- Server scope exited and final native JSON was flushed. GDB independently
  observed child exit 2; the old SIGILL did not recur. This finite supervised run
  does not establish the historical SIGILL cause or universal crash freedom.
- Mesa software rendering was requested; loaded paths include swrast. EGL
  warnings are retained verbatim. GL_RENDERER was not acquired before this stop;
  no new image-production/driver qualification is claimed.

Matching observer/owner timestamps do **not** establish synchronization authority.
EPD association, render/physics timing, penetration and contact/extraction remain
BLOCKED. No EPD, ROS/MoveIt/controller/trajectory execution, ACM changes or robot
motion. Original envelopes and the 0.1 mm threshold remain unchanged.

## Artifacts and one next action

Raw stdout/stderr, native/runner results, owner trace, process maps, execution
claim, world, debugger script and independent exit reports are retained. Larger
or whitespace-bearing files use deterministic gzip; `summary.json` records their
**uncompressed SHA256**. Use `gzip -dc` to inspect original bytes.

**One next action:** implement a lossless, schema-aware inventory comparison that
retains both snapshots and reports exact entity/field differences at this join,
with CPU regressions for JSON round trips and the observed third visual. Preserve
complete coverage and reject every unexplained/nonphysical ownership ambiguity;
do not bypass the mismatch or assume visual 13 is harmless. No second live run
was conducted in this task.
