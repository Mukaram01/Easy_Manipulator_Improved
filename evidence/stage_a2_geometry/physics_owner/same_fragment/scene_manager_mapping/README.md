# Live SceneManager mapping diagnostics — BLOCKED

Baseline: `3ee7952284697a4a3e33171da7d2f92da521a214`. One new disposable,
lightless two-cube experiment was performed, under software rendering and
`timeout 60s`. Child and supervisor exited normally with code 2; no crash.
No retries, RGB/ID buffers, EPD inference or motion goals occurred.

## Implementation

`RendererIdentity` now records every expected full-width Gazebo visual/link/model
ID, actual SceneManager lookups, native renderer IDs and parent relationships,
scene/root/world identity, duplicate observations and the complete native visual
inventory. A pure CPU checker returns all failures and the first condition/path.
The capture saves this incomplete evidence in `capture.json` before throwing.
No unexpected visual is filtered or exempted. No hierarchy correction was made:
the recorded relationships support the original hierarchy for both cubes.

The initial manual runner light-removal change and its tests are preserved.
`manual_before.patch.gz` records the initial patch; the manual tests file remained
byte-identical during this task. Both identity adapter headers are now pinned in
preflight source provenance. Physics-owner code and ELF were unchanged.

## Actual runtime observations

Session: `f5e043de0b7c45dc9a513a889967d9ad`; owner step 2, stamp 2,000,000 ns.
SceneManager world entity 1; renderer scene 0; root renderer ID 65535.
These observations do not qualify render/physics synchronisation.

| Gazebo visual | Native visual | Gazebo/native link | Gazebo/native model |
| --- | --- | --- | --- |
| 6 | 65519 | 5 / 65521 | 4 / 65523 |
| 10 | 65513 | 9 / 65520 | 8 / 65522 |

All expected lookups exist, parents match the exact looked-up objects, model
parents match the scene root, and owners are unique. Owner/render ECM inventories
are equivalent. The qualified owner ELF is present in the process mapping.

The sole failed condition is **`unexpected_geometry_visual`**, at
`/scene_visuals/0`: native visual **65526**, parent **65535**, geometry count **1**,
not mapped to either expected Gazebo visual. Its runtime geometry type is UNKNOWN.
Complete SceneManager mapping, actual draw association and buffer validation
remain BLOCKED. Existing metadata/ShapeNode joins do not establish draw authority.

Source review of matching Fortress `RenderUtil.cc` shows `scene.Grid()` with
sensors disabled invokes `ShowGrid()`, which attaches a geometry-bearing visual
to the scene root. This is consistent with the unexpected visual, but its grid
classification is INFERRED, not directly observed. The prepared SDF has no
explicit grid setting. No safety gate was weakened to proceed.

## Provenance and validation

Executed capture ELF SHA256:
`9f61a8513655ddeb46b76ed5d12f7ee61a408d34bd2e14d7491586abc20e4049`.

Owner ELF SHA256:
`0f4a2dda8d7bfd67745b03493254a9d91ea66bb36dcb2d78440fbf5809ff072a`.

`preflight.json` pins source, executable, owner and world hashes.
`result.json.gz` contains the command, process result and loaded-library hashes;
`capture.json.maps.gz` retains actual process maps. Raw stdout/stderr, owner trace,
debugger script, world and execution claim are retained. `raw_sha256.json` hashes
the original uncompressed artifacts. Gzip files use deterministic compression.

Focused builds succeeded. 93 Python tests passed across the runner, live identity,
fragment identity, visual/collision identity and owner registration suites.
Four native CTest targets passed: owner bindings, owner identity inventory,
lossless inventory comparison and renderer identity diagnostics. The latter
covers missing/duplicate/ambiguous mappings, parent/world mismatches, unexpected
geometry and full-width IDs. Validation logs include the initial failing native
regression and successful build/test output. The first Python run exposed an
outdated static source assertion after extracting the native ID helper; it was
corrected before the successful preflight.

## Single next action

Explicitly disable the optional grid using `<scene><grid>false</grid></scene>`
in the disposable capture-world generator, with a focused regression proving
cube geometry is unchanged and the complete-inventory gate remains strict.
Then request a separately bounded experiment. Do not exempt native visual 65526
or infer identity from names, proximity or renderer numbering.

Original EPD poses/envelopes, the 0.1 mm limit and all contact permissions remain
unchanged. Physical contact, EPD association, timing and extraction authority
remain BLOCKED. No installed libraries or protected Stage-A1 files were edited.
