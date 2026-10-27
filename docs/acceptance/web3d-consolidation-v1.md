# Web3D consolidation and Asset Library clean-up v1

**Scope:**
- Web3D becomes the only engineering 3D viewport.
- The native Scene3D renderer is retired.
- The Asset Library is inventoried and cleaned.

**Evidence level:**
- Static analysis, headless builds, headless unit and contract tests.
- **No GUI was opened.** The user was working on LENLEG.
- Everything visual is **PENDING USER-AUTHORIZED GUI ACCEPTANCE** (see the end of this document).

## 1. Native Scene3D retirement

### What was removed

| Area | Removed |
|---|---|
| Renderer | `gui/scene3d_viewport_widget.{h,cpp}` (QOpenGLWidget, about 5,300 lines), `gui/scene3d_diagnostics.*`, the empty `gui/scene3d_mesh_cache.*` and `scene3d_transform_math.cpp` stubs, and the unused `src_scene3d_visual_backend.cpp` / `include/scene3d_visual_backend.hpp` |
| Backend selection | Environment variable `WORKCELL_BUILDER_PRODUCT_VIEW_BACKEND=native_scene3d`, the non-WebEngine construction branch, `is_native_product_view_backend()`, `ProductViewBackend::NativeScene3D` |
| Silent fallback | `activate_native_compatibility_preview` (a second renderer created on a Web3D failure), the `CompatibilityReady` state, and "Preview available in compatibility mode" |
| Hidden native-only controls | Mesh Preview, Labels, Mode, View (Top/Front/Side/Iso) and the 20-entry Overlays combo in the Product View toolbar; Visual Modes → *Label mode* and *Mesh preview mode*. These were reachable through **View** under 1280 px. |
| Duplicate edit paths | Native gizmo `transform_changed_cb` and `asset_drop_cb`. They wrote the canvas directly with no undo entry. The Web3D `preview_item_transform_changed` and asset-drop paths are the only ones now. |
| Native-only audits | `audit_ur5_2f_test_committed_viewport_items`, the UR5 final-append check, native payload and draw log lines, and the `preview_chip_status` computation (it was `Q_UNUSED`) |
| Smoke runner | The roughly 950-line `Scene3DSmokeRunner`. `--web3d-scene-sequence` has its own small runner. Plain `--scene3d-smoke` now writes `status: RETIRED`, blocker `native_scene3d_renderer_retired`, and exits 3. |
| Dependencies | Qt5 `OpenGL`, `OpenGL::GL`, `find_package(OpenGL)`, and `libqt5opengl5-dev` in `package.xml` |
| Tests | The native-only gtests `scene3d_stl_parser`, `scene3d_render_role_classifier`, `scene3d_ur5_baked_transform` and `scene3d_mesh_preview_regression` (they tested the deleted widget's internals) |

### What was rewired

- **Fit Selection** (formerly "Fit Robot") and the Scene Hierarchy **Focus** called the native widget, so under Web3D they did nothing. They now call the viewer's `fitSelection()` through `ScenePreviewWidget::frame_selection()`.
- **Fit Scene** calls `fitScene()`.
- **Reload Meshes** refreshes the Product View.
- A **read-only diagnostic preview** now disables the Gizmo mode selector. It previously disabled a hidden native combo.
- **A Web3D failure stays a failure.** It shows the explicit preparation-error page, or "Web3D Product View — failed, Retry", with the Retry button. No other renderer is substituted.
- **CMake:** WebEngine is REQUIRED for the GUI. `-DWORKCELL_BUILDER_ALLOW_NATIVE_3D_FALLBACK=ON` survives only so that `scripts/build_commissioning_capability.sh` (unchanged, qualified) can configure the build directory for its non-GUI targets. It no longer provides any 3D fallback.

### What was kept

| Kept | Why |
|---|---|
| `scene3d_candidate_assembly.*`, `scene3d_visual_classification.*`, `preview_item_suppression.*` | Data and classification code that drives the Web3D layer filter, the payload, health counts and the hierarchy |
| `scene3d_transform_math.h` | Documents the URDF visual-origin convention shared by Web3D and the MoveIt manifest (tested) |
| Overlay models (`TaskOverlayModel`, camera, reachability, collision, EPD detections) | Still read by the 2D Layout and status reporting; the menu is now labelled **2D Layout Overlays** |
| `load_scene_for_scene3d_smoke` | The deterministic scene loader used by the Web3D acceptance runners |
| 2D Layout | A separate view, not a fallback |

ROS, RViz, MoveIt and URDF code never referenced the widget and is untouched.

### Dependencies that block complete retirement (documented, not removed)

1. **3D overlays.**
   - The reach, collision, work-envelope, task-route, camera-FOV, pick-coverage and EPD-detection overlay *models* are not drawn in Web3D.
   - The native drawer was their only 3D renderer, and it was reachable only by forcing the native backend.
   - Web3D shows only the helper items exported in the scene JSON, behind its debug-overlay toggle.
   - Next step: a `setOverlayModels` bridge to draw them in Web3D. That needs GUI acceptance.
2. **Native-smoke script family.** These measured the retired renderer and now receive `RETIRED`:
   - `scripts/run_workcell_builder_scene3d_gui_smoke*.py`, `scene3d_smoke_payload.py`;
   - `validate_scene3d_rviz_parity.py`, `run_scene3d_generated_canvas_acceptance.py`, `run_scene3d_visual_quality_screenshots.py`;
   - the native parts of `run_workcell_studio_scene_readiness_gate.py` and `run_local_ur5_2f_workcell_validation.sh`;
   - their source-text tests.

   The CI release-gate wrapper tests still pass. Removing the family is a scripted follow-up.
3. **GUI acceptance harnesses.** `studio_full_cycle_acceptance.hpp` and `multi_select_acceptance.inc` set `native = false`. Their native-only branches are now dead code and will be removed when the harnesses are next run with a GUI.
4. **Static Python tests.** Many tests assert source text of the removed native paths. Updated tests are listed in Tests; the rest are listed by the regression comparison in the PR.

## 2. Asset Library

The Builder's Asset Library is a recursive crawl of the asset roots, one entry per ROS package (`gui/asset_catalog_model.cpp`). `scripts/audit_asset_library.py` reproduces that crawl headlessly and inspects every mesh:
- triangles and degenerate faces;
- extent and units;
- open and non-manifold edges;
- signed volume (inside-out detection);
- stored-normal agreement;
- COLLADA unit, materials and textures;
- references across scenes, catalogs, descriptions, scripts, tests and evidence;
- package licence and README.

Full per-file results are in `asset-library-inventory-v1/inventory.json` (`class` = KEEP / FIX / REPLACE / REMOVE, with the reason).

| | Before (main 3679a4f5) | After |
|---|---|---|
| Asset Library entries | **103** (the GUI showed 105 with per-scene imports) | **29** |
| Mesh files inspected | 290 | 241 |
| Files with geometry problems | 65 | 11 (see REPLACE and KEEP below) |

### Root causes of the "distorted triangles"

- **48 single-triangle STL stubs.** Each is a 1 m × 1 m × 0 flat triangle. They stood in for bins, conveyors, fixtures, safety equipment, machines, cameras and primitive objects (`assets/environment/{bins,conveyors,fixtures,safety,machines,cameras}/meshes`, `assets/objects/primitives/meshes`). Each was listed as a separate product.
- **Six camera-mount packages, one geometry.** The flat-plate bracket, overhead gantry, pipe stand, table clamp, T-slot frame and V-slot frame were all the same generic 0.8 × 0.6 × 1.6 m box.
- **Duplicate entries.** Each builder-internal primitive was listed twice (mesh and xacro), and a stale 70-file copy of the gripper and robot packages under `workcell_builder/workcell_builder/assets/{end_effectors,robots}` appeared as extra entries.

### Classification (mesh files)

| Class | Count | What |
|---|---|---|
| **REMOVE** | 49 | The 48 single-triangle stubs and the empty `assets/imported/imported_demo_asset.stl`, with its importer-profile entry. No scene, URDF/Xacro, generated configuration or qualification evidence referenced them. Their catalog generator entries and tests were updated. |
| **FIX** | 5 + package | Robotiq 3F `GRIPPER_CLOSED` and `GRIPPER_OPEN_FINGER_2` (visual and collision) were closed but inside-out: rewound, normals negated. The curated pallet `pallet.obj` was inside-out: rewound. Robotiq 3F descriptions referenced the non-existent package `robotiq_3f_gripper_visualization`; the 18 resolvable URIs now use `robotiq_3f_gripper_description`, and 5 visual `.STL` references point at the existing `.dae`. |
| **REPLACE** | 21 | The six camera-mount packages and `simple_conveyor_description` (a 19-byte non-mesh) are marked in their own `package.xml` (`<workcell_asset_library status="replace" reason="..."/>`) and withheld from the library. They stay on disk: the conveyor package name is used by the conveyor-sorting wizard, and the mount frames are reusable. Also the legacy `workcell_studio_catalog/catalog.yaml`: 155 items (UR3–UR30, Fanuc, grippers…) all point at one 2-triangle placeholder square. The Builder library does not use it; it feeds older Python catalog paths and release-gate validators. |
| **KEEP** | 215 | Real vendor and CAD models (UR, Robotiq 2F-85 and 3F, AirPick4, single suction, Panda, Fanuc, D435/D435i, sorting bin, table/workbench) and the 17 curated generic primitives (honest boxes with real dimensions, used by Add Object). The UR3 collision `shoulder.stl` / `wrist1.stl` have 5–9 % zero-area triangles, which is upstream and harmless for collision. |

### Asset Library after the change (29 entries)

- **11 real models:** RealSense D435, Robotiq 3F, Robotiq 85, Panda, UR, table, workbench, Fanuc, AirPick4, single suction, sorting bin.
- **18 curated generic primitives:** pallet, bins, camera stand, conveyor placeholder, pedestal, fence panel, tables, tote, tray, workbench, calibration cube, cylinder, fixture plate, pick box, robot base plate, table.

### Library rules changed

- A package without `package.xml` is grouped by its `<name>_description` directory, so its mesh and xacro are one entry. Its stable ID (for example `tray`) is unchanged.
- A package whose `package.xml` declares `status="replace"` or `"remove"` is withheld. The reason is kept in `AssetCatalogModel::withheld_packages`.
- The builder-internal root is now `assets/environment` only, not its stale `robots` and `end_effectors` copies.

### Also found, not changed here

| Finding | Status |
|---|---|
| `catalog/capabilities/environment_assets/*.yaml` names four `mesh_path`s that do not exist (`table_standard.stl`, `conveyor_2m.stl`, `camera_mount_overhead_a.stl`, `cnc_tending_fixture_a.stl`) | REPLACE |
| The stale builder-internal package copies declare **Apache-2.0** where the canonical packages declare **BSD**; `ur_description` declares no licence | Provenance review needed |
| The template instantiator resolves robot and tool xacros from the stale copies first | Deleting them needs a headless instantiation test |
| Robotiq 3F ROS 1 entry files (`$(find …)/cfg/…`, `robotiq_gripper_upload.launch.py`) are broken | Not used by `ur5_3f_test`, which loads `robotiq-3f-gripper_articulated.urdf.xacro` |
| UR is represented in the library by the UR10 `base.dae` (crawl order) | Thumbnail choice only |

## 3. Duplicate UI pathways removed

| Removed | Survivor |
|---|---|
| Native gizmo transform callback | Web3D `preview_item_transform_changed` (undo, mounting, hierarchy) |
| Native asset-drop callback | Web3D `AssetDropWebEngineView` / `embedded_asset_placement_requested` |
| Native selection callbacks (two copies) | Web3D editor events |
| Product View Snap/Gizmo native branches | Web3D `setSnap` / `setMode` |
| Six hidden native toolbar combos, Visual Modes label/mesh menus | none (no Web3D meaning) |
| "Approach/Retreat" overlay toggle (native only) | none |
| Native "Fit Robot" / hierarchy "Focus" | Web3D `fitSelection` |

## 4. Web3D viewer tests (CI check "Browser Product View supported-scenes matrix")

The check failed on `main` and on every PR. The main cause:
- `viewer.js` called `window.__WORKCELL_VIEWER_LIFECYCLE__.registerCleanup` unconditionally;
- the test harness loads `viewer.js` without the bridge, so most tests died at load;
- the call is now guarded and a missing bridge is recorded;
- `viewer.bundle.js` was rebuilt.

`tests/test_workcell_studio_web_viewer_static.py`, run with Node 20 (the system Node 12 cannot parse the viewer):

| | Failed tests |
|---|---|
| main | 57 |
| this branch | 3 |

Three source-text tests were updated to the current viewer (bundle import chain, the scene_ready fit block, the select/clear command path).

All three remaining failures are now resolved (CI static step: 199 passed, 0 failed).

**Target-local destination ownership (defect fixed).**

| | |
|---|---|
| Exported fields | `ur5_2f_test`'s place zones are `editable: true`, `source_layer: editable_authored_physical`, with `target_ref` and `placement_local` |
| Defect | `canEditItem(zone)` was true, and `markDirtyTransform(zone, …)` moved the zone 0.5 m on its own while the bin stayed put. Only the incidental overlay classification kept the UI paths away from it. |
| Fix | A target-local destination now has no edit authority of its own (`editAuthorityForItem` → `follows_physical_target`). It moves only as a linked member of its target. Undo and redo replay it with the target. The selection reports the target as `editOwnerItemId`. |
| Not restored | The old zone→bin edit *routing* stays removed (`797508d8`): routing an absolute pose (Qt bridge `setItemPose`) through the zone would move the bin to the zone's pose. |
| Proof | New headless test `test_target_local_destination_moves_only_with_its_physical_target`: independent move refused, bridge refused, bin move keeps the local offset (0.01, 0, 0.03) through undo and redo, only the bin is persisted. The stale routing assertion was replaced. |

**Rotation (`canonicalRotatePreviewTransform` is required).**

- **History:** `0cc458f4` removed it and added the "absent" assertion. `390473da` deliberately reintroduced it for pivot (physical-centre) rotation, with its own positive contract test (one ROS RPY component per ring).
- **Proof:** new test `test_pivot_rotation_preview_apply_undo_and_persisted_rpy_agree`. Preview, displayed pose, committed edit and saved patch are one orientation, the position is kept and undo is exact, including yaw past π and pitch past π/2.
- **Remaining interaction gap:** the direct-attach path (parts without a physical centre) applies a true rotation about the ring axis, while the pivot path adds to one RPY component. These differ only for the X/Y rings of parts that already have other non-zero angles in world space. Choosing one is a visible interaction decision and is listed below as pending GUI acceptance; behaviour was not changed blind.

## 5. Tests run (headless only)

| Check | Result |
|---|---|
| Build `workcell_builder` (2 jobs) | PASS. No warnings in changed files. |
| gtests (53 binaries, `QT_QPA_PLATFORM=offscreen`, run from the package directory) | 42 pass outright. The new contract and library-rule tests pass: `ToolbarOffersOnlyWeb3DControls`, `PreparationFailureStaysAFailureWithRetry`, `ProductViewTransformsEditOnlyEditableLayoutItems`, `PackageMarkedReplaceIsWithheldWithItsReason`, `UnpackagedDescriptionMeshAndXacroAreOneEntry`. The other failures are pre-existing (identical on the #3221 base, or in targets that compile no changed source and use temp fixtures): `scene_preview_widget_ui` (8, previously 9), `preview_item_suppression`, `directory_inspection`, `scene_identity`, `scene_select_paths`, `scene_xacro_tool_attachment`, `canvas_model_mesh`, `visual_mesh_source_resolver`; `generated_environment_asset_writer`, `support_contact` and `task_intent_editor` need ctest's environment. |
| Python, 493 test files compared with main (per-file 60 s cap, Node 20) | Failing test IDs: main 664 → branch 611. The two "new" IDs depend on generated scene artifacts that another test creates during the run; both pass with those artifacts present. |
| `tests/test_workcell_studio_web_viewer_static.py` (Node 20) | main 57 failed → branch 0 (138 passed, including 2 new) |
| Web3D bundle | Rebuilt; `check_stale_bundle.mjs` current |
| Inventory tool | `tests/test_audit_asset_library.py` 3/3 |

Test-suite changes:
- About 30 static tests that only inspected the deleted renderer were removed.
- Tests that pinned the old fallback were inverted to guard the retirement.
- Tests that sliced source by removed `#ifdef` markers were re-pointed.
- Several test modules read the deleted widget at module level, which made the whole module fail to collect and hid its valid tests. They were pruned so the valid tests run again.

### Asset integrity checks (headless)

- **Removed files:** none of the 49 is referenced by any scene, cell definition, scenario pack, catalog, description, script, test, or qualified evidence directory (`r4_1_evidence`, `r4_1b_evidence`, `r4_2_evidence`, `stage_b_evidence`, `integration_evidence`, `r3_integration_evidence`). Two basename matches point at different, surviving paths.
- **Rewound meshes:** identical vertices per triangle, identical triangle counts and bounding boxes, negated normals, positive volume. No origin changed.
- **Robotiq 3F:** `ur5_3f_test` and the 3F MoveIt config load the unchanged `robotiq-3f-gripper_articulated.urdf.xacro`, whose 10 mesh URIs resolve. The edited legacy variants now resolve to the same meshes.
- **Marked packages:** the 7 marked `package.xml` files parse with `catkin_pkg`.
- **Withheld assets:** no authored scene uses a withheld or removed asset, and scene loading does not consult the library list.
- **Curated profiles:** all 17 curated Add Object profiles resolve.
- **Not changed:** the legacy `catalog.yaml` and the stale builder-internal package copies stay documented for later cleanup.

## 6. PENDING USER-AUTHORIZED GUI ACCEPTANCE

Not visually verified:
1. **Product View with no native code:** loads, edits, Retry on a forced failure, and no other renderer appears.
2. **Fit Selection, hierarchy Focus and Fit Scene** frame the Web3D view.
3. **Product View toolbar:** Gizmo, Snap, Undo, Redo, Fit and ⓘ only; the Gizmo selector is disabled in the read-only AirPick preview.
4. **View menu under 1280 px:** no Label mode / Mesh preview mode; "2D Layout Overlays" affects the 2D Layout only.
5. **Asset Library:** 29 entries, none drawn as a triangle; Robotiq 3F gripper meshes and the pallet are no longer inside-out.
6. **`--web3d-scene-sequence`** still reaches scene_ready per scene.
7. **Destination zones:** selecting a place zone shows it and its target; no gizmo or keyboard move applies to the zone. Moving the bin carries the zone.
8. **Rotation rings:** decide one interpretation for X/Y rings on parts with non-zero other angles in world space (pivot path: RPY component; direct path: true axis rotation).
