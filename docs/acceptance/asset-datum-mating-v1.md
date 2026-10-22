# Datum mating on fixed asset mounting (v1)

Datum mating extends [fixed asset mounting](asset-mounting-v1.md); it is not a
second relationship system. The relation stays on the child record in
`environment.yaml`: `parent_object`, `parent_link` (the parent **datum**), fixed
`joint_type`, `origin` (the offset) and, new, an optional `mate` mapping. World
poses, layout poses, generated objects, collision objects and TF remain checked
projections of that one relation.

## Contract

- **Datums** are named, explicit local frames in an asset's `mounting_anchors`,
  now with `datum_type`: `point`, `axis` or `plane` (`origin` is implicit;
  untyped v1 anchors are points; the reviewed `surface` from `surface_z_m` is a
  plane). A plane lies in its local XY with +Z as the outward normal; an axis
  runs along local +Z. Datums are never inferred from mesh triangles.
- **Reviewed catalog datums** (`catalog/workcell_studio_asset_datums.yaml`) are
  offered for `bin_standard` (bottom, rim, centre axis) and `camera_stand`
  (base, top, mast axis) only while the asset's saved mesh calibration matches
  the reviewed one. When a mate uses one, its frame is copied into that asset's
  `mounting_anchors`, so a saved cell never reads the catalog again.
- **Mate**: `mate: {child_datum, alignment}` with `coincident` (datum frames
  line up) or `flush` (plane to plane, outward normals opposed). The offset
  `origin` is expressed in the parent datum frame (Z along its normal, yaw about
  it): `child = parent · parent_datum · origin · alignment · child_datum⁻¹`.
  Without `mate` the v1 formula is unchanged, and an origin/coincident choice is
  stored in the v1 format.
- **Rejected, atomically, with an actionable message**: flush without two
  planes, axis↔plane coincidence, missing child/parent datums or parents,
  cycles, a second parent or `mates` list (over-constraint), a mate on a world
  asset, unknown alignment/datum type, nonfinite values, offsets beyond 1000 m
  and stale projections. Detach and reparent keep the world pose unless the
  user chooses to move the asset. Destination zones keep their R1.9
  target-local placement and follow their physical target; TaskIntent EXACT is
  untouched.

## GUI

The existing Inspector **Mount To / Relative Pose…** dialog now has Parent
datum, Child datum (with type and label), Alignment and Offset fields, and a
live preview computed by the same resolver: the resulting pose and movement,
the offset that will be stored, the assets and destination zones that follow,
or the rejection reason with Apply disabled. Return applies (it never
detaches). The hierarchy nests mated children with their relation (for
example "bottom flush on surface") and the Inspector and selection card state
the full relation and offset. Undo/redo, Save, reopen and Generate use the
existing paths.

Two related fixes found during acceptance: Inspector Apply keeps untouched
fields at their exact value (a Y edit used to round Z to the 3-decimal display,
sinking a flush bin by 0.1 mm), and a Product View pose echo on selection no
longer records a no-op mounting edit or marks the layout dirty.

## Workstation evidence — 2026-10-22 (London), PR branch on top of #3201

Fresh UR5 + Robotiq 2F cell created through the real New Cell Wizard
(`datum_mating_acceptance`), driven through the real dialog, Inspector, history
and Save Layout controls (`--datum-mating-acceptance`), then reopened in a
separate Builder process. Embedded Web3D and native Scene3D both **PASS**.

- Destination bin `bottom` (plane) flush on table `surface`; the dialog moved it
  0.9 mm onto the top (Z 0.0991). Camera `origin` on library camera-stand `top`
  (plane), offset 0.15/0.09/0.05 m, pitch 90° — an independent second relation.
- Table moved +5 cm and rotated 20°, stand moved and rotated 15°: children
  followed, table-local bin offset unchanged to 1e-15 m, camera unaffected by
  the table. Two undos/redos restored both graphs exactly; detach and reparent
  kept world poses and were undone.
- Rejected in the dialog with Apply disabled and the scene unchanged: flush with
  an origin datum, axis-to-plane coincidence, a table→bin cycle.
- Four Saves: the mated graph, then bin Y −0.02, −0.07, −0.01 (the #3201
  regression). Each save: disk equals the editor, mate unchanged, destination
  zones `placement_local` stay (0, 0, 0.01), bin bottom stays on the table top.
- Reopen: every ID, parent, datum, mate, offset and world pose equal the
  authoring session (1e-8) and `environment.yaml`; selecting the bin leaves the
  layout clean; GUI Generate and Validate ran.
- [Projection check](asset-datum-mating-v1/projection-check.json): datum frames
  hold in world (≤1e-16), zones are current, generated environment objects and
  the MoveIt manifest carry the same relations/poses, camera pose mirrors its
  mounted asset.
- [Fake-hardware runtime](asset-datum-mating-v1/runtime.json) (ROS 2 Humble,
  CycloneDDS): package built; three controllers active; collision objects at
  the mated world poses; camera TF equals the generated chain (1e-7); RViz and
  mesh markers up; no motion or execution goals sent.
- Click-driven at 1366×768 (Home → Open → hierarchy → Mount To, Return,
  Undo): [reopened selection](asset-datum-mating-v1/click-reopened-bin-selected.png),
  [Coincident preview](asset-datum-mating-v1/click-dialog-coincident-preview.png);
  closed without saving, disk unchanged.

Evidence: [authoring](asset-datum-mating-v1/authoring.json),
[reopen](asset-datum-mating-v1/reopen.json),
[native authoring](asset-datum-mating-v1/native-authoring.json),
[native reopen](asset-datum-mating-v1/native-reopen.json),
[mate dialog](asset-datum-mating-v1/dialog-bin-on-table.png),
[rejected relation](asset-datum-mating-v1/dialog-invalid-flush-origin.png).

## Reproduce

In an isolated workspace whose `src/easy_manipulation_deployment` is this
checkout, with ROS and the workspace underlay sourced, from the repository root:

```bash
workcell_builder --datum-mating-acceptance --workspace "$WS" --scene datum_mating_acceptance --smoke-output "$EVIDENCE"
workcell_builder --datum-mating-acceptance --workspace "$WS" --scene datum_mating_acceptance --smoke-output "$EVIDENCE" --reopen
python3 workcell_builder/workcell_builder/test/datum_mating_projection_check.py scenes/datum_mating_acceptance
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_DOMAIN_ID=87 python3 \
  workcell_builder/workcell_builder/test/asset_mounting_runtime_acceptance.py \
  datum_mating_acceptance scenes/datum_mating_acceptance /tmp/datum-runtime
python3 -m pytest -q tests/test_asset_datum_mating.py tests/test_asset_mounting.py \
  tests/test_workcell_studio_layout_merge.py tests/test_physical_destination.py tests/test_new_cell_physical_profile.py
```

## Limits

- Fixed relations only; one parent per asset. Not a CAD solver: no face picking,
  STEP, constraint loops, revolute/prismatic joints, flexible mechanisms or
  physics. Datum frames come from asset records or the reviewed catalog.
- The preview is numeric (pose, offset, followers); the 3D view updates on
  Apply and Undo restores it.
- Reviewed catalog datums exist for the standard bin and the placeholder camera
  stand only; other assets offer `origin`, `surface` and their own anchors.
- Pre-existing, not changed here: the embedded Web3D Builder can crash with heap
  corruption while shutting down (first Web3D reopen: PASS written, then exit
  139; backtrace in `~OwnedProductViewServer` during `ScenePreviewWidget`
  teardown; second reopen exited 0); the harness-load breadcrumb can show the
  previously selected cell; the new cell's validation reports
  `TASK_INPUT_UNAVAILABLE` until a task input is chosen; the camera stand is not
  drawn in Web3D until the cell is saved.
