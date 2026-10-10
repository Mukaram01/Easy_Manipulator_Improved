# Graphics-free owner SystemLoader repair — CONFIRMED

Baseline `b1b8ea6d629c1f58b4a7ba3bd772192e2451d916`. One post-repair
acceptance, exit **0**, no renderer execution. This closes plugin loading only.
The preceding live experiment's **SIGILL (-4) remains unresolved**.

## Root cause and minimal repair

The unchanged owner ELF (`feeb888374b0b6cf8a51eff6854e4ea31abe9a7547146b4860ac935cf9b377e0`)
loads through Fortress SystemLoader in isolation (`diagnostic.json.gz`). Its class,
registration macros, aliases and System interfaces were present; `ldd -r` alone
could not expose the actual failure.

The live executable has a DT_NEEDED dependency on the Ogre2 engine plugin
(`live_executable_needed.txt`). Both DSOs export `IgnitionPluginHook` with
DEFAULT visibility. Loading that engine DSO with RTLD_NOW | RTLD_GLOBAL, without
constructing an engine, reproduces the failure: the global hook resolves to
Ogre2, the owner's registration table is empty and SystemLoader returns no
instance (`global_diagnostic.*`). That diagnostic process subsequently received
SIGSEGV (shell reported Segmentation fault; timeout reported a core dump).
This is **not proof of the earlier SIGILL's cause**.

`prepare.py` now emits ELF `.protected IgnitionPluginHook` immediately before
owner registration. This binds the owner's own registration calls locally while
keeping its hook exported for the loader. The exact generated-source diff is
`registration.diff.gz`; no physics construction, stepping, solver, DART readback or
collision-to-ShapeNode logic changes. Reference generation is unchanged. No
installed library was modified; no library-wide symbolic binding was added.

## Actual acceptance

The new graphics-free `owner_loader_preflight` reads the **same retained world
plugin declaration**, absolute ELF path and requested alias. It calls actual
Gazebo `SystemLoader::LoadPlugin` before separately inspecting registration.
It instantiates a System but never calls Configure, constructs a Server, steps
physics, initializes a render engine or creates a graphics context.

The sole post-repair execution uses the global Ogre2 DSO stress condition:

```sh
cmake -S scripts/stage_a_rgbd/physics_owner -B /tmp/stage_a2_owner_build
cmake --build /tmp/stage_a2_owner_build \
  --target workcell_owner_physics owner_loader_preflight -j1
# Use a new, absent JSON path for any separately authorised future acceptance.
timeout 20s /tmp/stage_a2_owner_build/owner_loader_preflight \
  evidence/stage_a2_geometry/physics_owner/same_fragment/live_gazebo_attempt/world.sdf \
  /tmp/stage_a2_owner_loader_acceptance_b1b8ea6d/result.json \
  /usr/lib/x86_64-linux-gnu/ign-rendering-6/engine-plugins/libignition-rendering6-ogre2.so.6.6.4
```

`pre_run.json` records the actual absolute command, timestamp and hashes before
execution; the wrapper exclusively created its output directory and checked that
the result did not exist. Full stdout/stderr, exit code and mapped-library hashes
are retained. Result:

- Exact class `ignition::gazebo::v6::systems::WorkcellOwnerPhysics` loaded.
- Exactly one owner registration, with exactly the two expected aliases.
- System / ISystemConfigure / ISystemUpdate present; PreUpdate / PostUpdate absent.
- Global hook still resolves to Ogre2; its separate registration table contains
  only `ignition::rendering::v6::Ogre2RenderEnginePlugin`, with no owner pollution.
- Owner ELF mapped while the instance is alive; no stock Physics System DSO
  mapped. Exactly one world plugin declaration; no duplicate Physics System.
- No DRI driver mapped, no GPU device file descriptors, zero physics steps and
  zero execution goals. Graphics libraries are mapped **for symbol-scope
  reproduction only**, not initialized. Clean destruction and process exit 0.

Owner ELF SHA256:
`0f4a2dda8d7bfd67745b03493254a9d91ea66bb36dcb2d78440fbf5809ff072a`.
Preflight executable SHA256:
`bc7022b65c789a24db5cfe99dfc51070d5466a8e089cfdddc806a3e01854e5e0`.
`hook_symbols.txt` proves GLOBAL PROTECTED export. Loaded DART paths are
provenance only: no DART world/backend was constructed by this acceptance.

## Focused validation

- Registration regression: three tests failed before implementation (`red.log`)
  and pass after repair, including missing/duplicate-anchor rejection.
- `python3 -m pytest -q tests/test_stage_a_owner_registration.py
  tests/test_stage_a_owner_geometry.py tests/test_stage_a_visual_collision_identity.py`:
  **45 passed**.
- Native build: owner, loader preflight and existing owner test targets PASS.
- `ctest --test-dir /tmp/stage_a2_owner_build -R '^owner_(bindings|identity_inventory)$'
  --output-on-failure`: **2 passed**, graphics-free.
- No new GPU fixture, Gazebo Server, EPD or MoveIt run.

## Scope and next action

Live draw-to-entity identity, renderer/physics synchronization, EPD association,
physical penetration and extraction authority remain **BLOCKED**. Original
planning poses/envelopes, 0.1 mm threshold and contact rules remain unchanged.
Historical owner hashes in `identity_metadata/result.json` are intentionally
preserved; the live runner still rejects this new ELF against that older record.

**One next action:** explicitly consume this newly qualified owner provenance
in the opt-in runner, then perform one separately authorised bounded disposable
Gazebo identity experiment, retaining crash diagnostics if the live SIGILL
recurs. No claim that the live crash is repaired follows from this loader PASS.

Raw files with trailing whitespace are stored as deterministic gzip to preserve their exact bytes; use `gzip -dc` to inspect them.
