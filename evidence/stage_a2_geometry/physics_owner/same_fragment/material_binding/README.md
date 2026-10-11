# Live Gazebo material binding and RGB/ID capture — PASS in tested scope

Baseline `41ab8565bdea3b4fcc48191fb64cb4af6f871f97` plus manual GLX initialization,
explicit EGL query resolution and exact numeric world/collision-ID comparisons.
All manual modifications were preserved and included; the initial diff is retained
as `manual_before.patch.gz`. The previous manual failure is retained independently
in `manual_before_attempt/`. No reset, clean, stash or historical overwrite.

## Confirmed root cause and minimal fix

Fortress SceneManager assigns materials with `geom->SetMaterial(material)`.
Installed ignition-rendering BaseVisual::Material returns a separate visual-level
field populated by Visual::SetMaterial. Thus a null Visual::Material is not proof
of absent geometry material. The previous run rejected the disjunction without
retaining which condition held. This run confirms BOTH cube Visual::Material
pointers are null, while BOTH Geometry::Material pointers exist, have transparency
zero and exactly match the live ECM diffuse colour.

Source: https://raw.githubusercontent.com/gazebosim/gz-sim/ignition-gazebo6_6.18.0/src/rendering/SceneManager.cc
(CreateVisual). The installed BaseVisual.hh was inspected directly.

Before any replacement, the capture records both exact Gazebo visual IDs, native
renderer IDs, original Ogre Item IDs, live ECM Material/Transparency components,
visual/geometry material availability and values, and original SubItem datablock
identity/type/blending/alpha-test state. Both are PBS HLMS-backed, have no legacy
Ogre material, opaque ONE/ZERO blending and disabled alpha testing.

The unlit MRT shader now receives the actual live ECM diffuse colour, keyed by
exact full-width visual identity. This requires present ECM components, zero
transparency, finite in-range RGBA, opaque ambient/diffuse alpha, no script/PBR
profile, matching native geometry colour/transparency, and opaque PBS native state.
An absent visual-level material is permitted; an absent geometry material or any
observed disagreement is rejected. No colours are invented. Existing Item,
geometry, scene, collision and ShapeNode gates remain intact. Original live Items
are reused; only their opt-in disposable unlit materials change. This does not
claim equivalence with stock lit RGB, shader precision or full image registration.

## One bounded runtime attempt

Fresh output `/tmp/stage_a2_material_41ab8565_20261010`; software GLX/llvmpipe,
`timeout 60s`, single execution claim, supervisor and child exit 0. No retry.
Native result `PASS_TESTED_VISUAL_DRAW_PRODUCTION`; strict CPU result
`PASS_TESTED_LIVE_GAZEBO_IDENTITY_ONLY`.

Session `b973ddbbca2c41639d5350db47bdac74`. Exact owner step 2; matching owner and
render ECM inventories and complete SceneManager map. This records step metadata,
not independent render-to-physics timing authority.

| Gazebo visual | Link | Model | Collision | Physics shape | Original Item | Native uint32 ID | Visible pixels |
| --- | --- | --- | --- | --- | --- | --- | --- |
| 6 | 5 | 4 | 7 | 6 | 0 | 65523 | 1736 |
| 10 | 9 | 8 | 11 | 7 | 1 | 65517 | 211 |

Exact live ShapeNode pointer identities are retained in capture, owner trace and
strict mappings. They are scoped to this session/process, not persistent IDs.
Background ID 0 has 63,589 pixels; total 65,536. No unexpected IDs.

Actual output: 256x256 RGB8 (196,608 bytes) and lossless little-endian uint32 IDs
(262,144 bytes). RGB has values [0,0,0] and [184,112,41], so is nonblank. The two
cubes share their authored colour; their distinct IDs are not inferred from colour.
The new capture exercises the actual IDs above, not arbitrary large sentinel IDs;
prior native fixture evidence separately tested large uint32 labels.

One shared scene pass; actual attachment formats GL_RGBA8 (32856) and GL_R32UI
(33334), each 256x256. Native post-pass depth-test/write states are true. The
retained overlapping-interior pixel (132,127) contains near ID 65523 rather than
far ID 65517. This is a measured native BOX-ray diagnostic, not a numerical
registration/coverage bound. Same-pass RGB and IDs use the original Items.

GL context: GLX current, EGL not current; GL 4.5 Core Mesa 23.2.1,
llvmpipe LLVM 15.0.7, prior/query glGetError zero. Detailed thread/context identity
and loaded-library hashes are retained.

## Provenance

Executed ELF SHA256:
`4fb15a27dcfc277da55fe4985e98f8d4f72663b8dd5d623a9c601fb56651945e`.

Unchanged owner ELF SHA256:
`0f4a2dda8d7bfd67745b03493254a9d91ea66bb36dcb2d78440fbf5809ff072a`.

RGB SHA256: `920560881e06ae7285a33fd79d93e906a4001aac33f49583cf4af07016d1f526`.

ID SHA256: `cba6227c654aa690e9064057b51dd932162d604b5c5e0b112aa81b8c2ce60020`.

`attempt/preflight.json` pins both new material headers and all capture sources,
world, owner and executable. Actual source/ELF hashes were rechecked after capture.
`attempt/result.json.gz` retains loaded-library hashes and strict validation.
Raw buffers, process maps, complete stdout/stderr, owner trace, execution claim
and checked capture are retained, using deterministic gzip where appropriate.
Each directory's raw_sha256.json hashes the original uncompressed artifacts.

## Focused validation

Build succeeded; 99 focused Python tests passed across runner, live fragment,
fragment identity, visual/collision identity and owner registration suites.
Six native tests passed: material contract, GL context classification, renderer
identity, inventory comparison, owner bindings and owner identity inventory.
New CPU tests accept null visual material with qualifying geometry/ECM state and
reject missing material/transparency, nonopaque alpha/transparency, invalid colour,
native colour disagreement, scripts/PBR and unsupported HLMS/blend/alpha-test state.
Initial regression failed against an unimplemented checker. Two initial compile
errors (IdString API and const Colour indexing) were corrected before preflight;
all build logs are retained. Neither was a graphics execution.

The strict CPU diagnostic was also independently run against the actual retained
buffers. Six CPU-only mutations were rejected: changed RGB, altered IDs, duplicate
Item binding, missing Item binding, changed session and missing ShapeNode. No GPU
reruns. Manual GLX/EGL/numeric-ID files remain byte-identical; light/grid settings
and original cube geometry remain preserved.

## Scope and exact next action

PASS is limited to this disposable two-cube visual/collision/ShapeNode draw identity
and RGB/uint32-ID production. EPD association NOT RUN; render/physics timing,
physical penetration and extraction authority remain BLOCKED. No contact/timing
permission follows from RGB. No EPD, MoveIt, controllers, execution goals, ACM
changes, installed-library edits or protected Stage-A1 edits. Original EPD poses,
conservative envelopes and 0.1 mm threshold unchanged; PR remains draft.

Next action: run genuine external EPD inference on this exact retained RGB and
bind its masks to the sibling integer-ID buffer using verified preprocessing
coordinates and session/image hashes; reject mixed or unresolved IDs. Keep timing
and contact authority blocked independently.
