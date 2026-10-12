# Asset relationships and mounting acceptance

Fixed asset mounting extends the existing Workcell Builder and physical
`environment.yaml` contract. `parent_object`, `parent_link`, fixed `joint_type`
and relative `origin` own each mounting relationship. Saved layout and generated
world poses are checked projections. Robot/tool joints are unchanged.

The Inspector's **Mount To / Relative Pose…** dialog offers world/asset parents,
named anchors, metres/degrees, and Detach. Reparent/detach preserve world poses
unless relative pose is explicitly applied. `origin` is implicit; reviewed
support surfaces also provide `surface`. Additional named anchors use the
existing asset record's `mounting_anchors` mapping (`pose_xyz`/`pose_rpy`).
Cycles, duplicate IDs, missing parents/anchors, unsupported joints, nonfinite
poses and stale projections fail closed. Task zones retain destination ownership;
mount their physical target instead. Parent deletion requires detaching children.

## Reproduce

On Ubuntu 22.04 / ROS 2 Humble, source ROS and the workspace dependency underlay,
build `workcell_builder`, then run its installed executable against an isolated
workspace containing this checkout:

```bash
workcell_builder --asset-mounting-acceptance --workspace "$WS" \
  --scene mounting_acceptance --smoke-output "$EVIDENCE"
workcell_builder --asset-mounting-acceptance --workspace "$WS" \
  --scene mounting_acceptance --smoke-output "$EVIDENCE" --reopen
```

The existing Qt acceptance harness drives the real New Cell Wizard, library
placement, mounting dialog, relative controls, Inspector, history, Save Layout,
close/reopen and GUI Generate. It refuses to overwrite an existing scene.
It writes JSON and screenshots; failures exit nonzero. No motion goals are sent.

## Workstation evidence — 2026-10-13 (London)

**PASS for fixed mounting.** Final cell: `mounting_acceptance_v12`, created with
UR5 + Robotiq 2F through the real Qt New Cell Wizard. The embedded Web3D renderer
was live. Actual dialog, Inspector, history and Save Layout controls were used.
A separate Builder process reopened the saved cell and invoked GUI Generate.

- Bin → table `surface`; camera → library camera support `origin`.
- Relative Z/pitch edited through mounting controls; both parents translated and
  rotated 20°. Children followed; full graph undo/redo, reparent and detach were
  checked. Reopen preserved all physical IDs, parent/anchor IDs and XYZ/RPY to 1e-8.
- Bin world XYZ: `[0.371404848486, 0.105517596109, 0.100899999499]`, yaw 20°.
- Camera mount XYZ: `[0.410186787085, -0.314165049413, 1.2]`, pitch 90°, yaw 20°.
- ROS Humble Builder and generated scene package builds passed. Cell validation
  passed. All required generated files exist, with no readiness blockers.
- Real fake-hardware RViz/MoveIt launch: three active controllers, 188 joint-state
  messages, three collision objects with matching world positions/rotations,
  three mesh markers, camera TF matching expanded generated xacro within 1e-7.
  Sensor macro offsets are included; camera-link XYZ is
  `[0.415947592336, -0.293445176758, 1.1894]`.
- No runtime errors occurred before intentional shutdown. The raw log's final
  controller/MoveIt exit `-2` lines follow the probe's SIGINT shutdown.
- Canonical `ur5_2f_test` built/launched with all three controllers, joint states,
  collision objects and RViz. Its evidence notes the correction for the existing
  camera macro offset when evaluating the captured TF.
- 58 focused tests passed in 23.44s. Independent review found no remaining
  concrete defect. `git diff --check` passed.

Evidence: [authoring](asset-mounting-v1/authoring.json),
[reopen](asset-mounting-v1/reopen.json),
[screenshot](asset-mounting-v1/mounted_scene.png),
[cell validation](asset-mounting-v1/cell-validation.json),
[mounted runtime](asset-mounting-v1/mounted-runtime.json),
[runtime log (trailing whitespace trimmed)](asset-mounting-v1/mounted-runtime.log),
[canonical runtime](asset-mounting-v1/canonical-runtime.json).

Focused tests (use a current Node runtime; this workstation used Node 24):

```bash
python3 -m pytest -q tests/test_asset_mounting.py \
  tests/test_workcell_studio_layout_merge.py tests/test_physical_destination.py \
  tests/test_new_cell_physical_profile.py
```

After building and sourcing the generated package, from the repository root:

```bash
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_DOMAIN_ID=87 python3 \
  workcell_builder/workcell_builder/test/asset_mounting_runtime_acceptance.py \
  mounting_acceptance scenes/mounting_acceptance /tmp/mounting-runtime
```

The probe reads live services, joint states, markers and TF; it sends no execution
or motion goals. Catalog STL copies use the existing `assets/imported` resource
contract and binary STL for RViz. Conversion preserves geometry and source files.

## Limits and existing issues

FastDDS service calls intermittently timed out on this workstation; final runtime
acceptance used installed CycloneDDS. Six broader generation-contract fixture
failures also reproduce on clean main `7a5cd224` (invalid destination references
and zone bounds); validation was not relaxed to hide them. Baseline comparison:
`/tmp/workcell-relationships-main-tests.log`.

Generated readiness is WARN for the new cell's missing task fallback/recipe;
this feature verifies mounting and fake-hardware launch, not pick/place execution.
The existing camera-support catalog entry is a simple placeholder with camera
classification, and the Builder breadcrumb can retain a previous scene name.
Acceptance checks the canonical selected scene path explicitly. No manual GUI
step remains for the supported mounting flow; additional hardware/task workflows
are outside this acceptance. Test scenes and full intermediate logs remain in
`/tmp/workcell-relationships-evidence` and `/tmp/workcell-relationships-*`.
