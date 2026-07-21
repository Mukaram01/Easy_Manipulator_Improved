from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
VIEWER = ROOT / "workcell_studio_web/viewer"
INDEX = VIEWER / "index.html"
COLLISION = VIEWER / "collision_placement_validation.js"
BUNDLE = VIEWER / "dist/viewer.bundle.js"


def test_collision_validation_loads_after_surface_placement_and_task_gizmo():
    html = INDEX.read_text(encoding="utf-8")
    ordered = [
        "./dist/viewer.bundle.js",
        "./rviz_light_baseline.js",
        "./support_surface_placement.js",
        "./workcell_task_gizmo.js",
        "./collision_placement_validation.js",
        "./simple_product_ui.js",
    ]
    positions = [html.index(token) for token in ordered]
    assert positions == sorted(positions)
    assert html.count("./collision_placement_validation.js") == 1


def test_collision_validation_uses_visible_rendered_mesh_bounds_and_clearance():
    source = COLLISION.read_text(encoding="utf-8")
    for token in [
        "CLEARANCE_M = 0.01",
        "OVERLAP_EPSILON_M = 1e-5",
        "node.geometry.computeBoundingBox",
        "local.clone().applyMatrix4(node.matrixWorld)",
        "objectBox.clone().expandByScalar(CLEARANCE_M)",
        "penetrationDepth(a, b)",
        "depth.x > OVERLAP_EPSILON_M",
        "depth.y > OVERLAP_EPSILON_M",
        "depth.z > OVERLAP_EPSILON_M",
        "clearanceBox.intersectsBox(candidate.box)",
    ]:
        assert token in source


def test_helpers_support_surfaces_and_debug_geometry_are_not_collision_objects():
    source = COLLISION.read_text(encoding="utf-8")
    for token in [
        "new Set(['workbench_body', 'table_surface', 'tabletop'])",
        "SUPPORT_KINDS.has(supportKind(item))",
        "current instanceof TransformControls",
        "current.isGridHelper",
        "current.isAxesHelper",
        "data.exclude_from_physical_bounds === true",
        "data.helper_overlay === true",
        "item?.debug_overlay === true",
        "isDescendantOf(node, selectedRoot)",
    ]:
        assert token in source


def test_feedback_distinguishes_valid_near_and_overlapping_placement():
    source = COLLISION.read_text(encoding="utf-8")
    for token in [
        "severity = collisions.length ? 'invalid' : (nearby.length ? 'warning' : 'valid')",
        "Valid placement · 10 mm clearance",
        "less than 10 mm clearance",
        "Invalid: overlaps",
        "VALID_COLOR = 0x2f8f5b",
        "NEAR_COLOR = 0xb26b00",
        "INVALID_COLOR = 0xc43434",
        "new THREE.Box3Helper",
        "collision-validation-status",
        "exclude_from_fit_bounds = true",
        "exclude_from_physical_bounds = true",
    ]:
        assert token in source


def test_invalid_drag_numeric_edit_and_export_cannot_be_committed():
    source = COLLISION.read_text(encoding="utf-8")
    for token in [
        "canvas.addEventListener('pointerup', onCanvasPointerFinishCapture, true)",
        "api?.setMode?.('select')",
        "restore(active?.root, active?.start)",
        "previous pose restored",
        "runtime.gizmo.dispatchEvent({ type: 'objectChange' })",
        "editorApi()?.undo?.()",
        "numeric edit rejected",
        "getEditPatch?.()?.edits",
        "event.stopImmediatePropagation()",
        "export blocked",
        "event.key !== 'Escape'",
        "Placement cancelled · previous pose restored",
    ]:
        assert token in source


def test_validation_includes_fixed_meshes_without_item_metadata():
    source = COLLISION.read_text(encoding="utf-8")
    for token in [
        "if (ancestorItemNode(node)) return",
        "node.name || node.parent?.name || 'fixed geometry'",
        "runtime.scene.traverse(node =>",
        "if (!node?.isMesh",
    ]:
        assert token in source


def test_collision_module_is_small_preview_only_and_does_not_rebuild_bundle():
    source = COLLISION.read_text(encoding="utf-8").lower()
    for forbidden in [
        "fetch(",
        "xmlhttprequest",
        "websocket",
        "execute_trajectory",
        "getmotionplan",
        "/plan_kinematic_path",
        "environment.yaml",
        "object_placement.yaml",
        "writefile",
    ]:
        assert forbidden not in source

    assert len(COLLISION.read_text(encoding="utf-8").splitlines()) < 560
    assert BUNDLE.exists()
    assert "__WORKCELL_COLLISION_PLACEMENT_V1__" in COLLISION.read_text(encoding="utf-8")
    assert "workcell:collision-placement" in COLLISION.read_text(encoding="utf-8")
