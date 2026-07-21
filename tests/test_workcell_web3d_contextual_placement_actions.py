from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
VIEWER = ROOT / "workcell_studio_web/viewer"
INDEX = VIEWER / "index.html"
ACTIONS = VIEWER / "contextual_placement_actions.js"
CSS = VIEWER / "contextual_placement_actions.css"
BUNDLE = VIEWER / "dist/viewer.bundle.js"


def test_contextual_actions_load_after_collision_validation_and_simple_ui():
    html = INDEX.read_text(encoding="utf-8")
    ordered = [
        "./dist/viewer.bundle.js",
        "./support_surface_placement.js",
        "./workcell_task_gizmo.js",
        "./collision_placement_validation.js",
        "./simple_product_ui.js",
        "./contextual_placement_actions.js",
    ]
    positions = [html.index(token) for token in ordered]
    assert positions == sorted(positions)
    assert 'href="contextual_placement_actions.css"' in html
    assert html.count("./contextual_placement_actions.js") == 1


def test_actions_are_contextual_and_only_available_for_editable_items():
    source = ACTIONS.read_text(encoding="utf-8")
    for token in [
        "state.selectedEditable === true",
        "state.selectedItemId",
        "Quick placement",
        "Place on nearest surface",
        "Centre on surface",
        "Align left",
        "Align right",
        "Align front",
        "Align back",
        "Move to pick area",
        "Move to place area",
        "button.disabled = !availability[name]",
    ]:
        assert token in source


def test_surface_actions_use_rendered_bounds_top_height_and_edge_margin():
    source = ACTIONS.read_text(encoding="utf-8")
    for token in [
        "new Set(['workbench_body', 'table_surface', 'tabletop'])",
        "new THREE.Box3().setFromObject(root)",
        "top_surface_z_m",
        "EDGE_MARGIN_M = 0.01",
        "support.box.min.x + EDGE_MARGIN_M",
        "support.box.max.x - EDGE_MARGIN_M",
        "support.box.min.y + EDGE_MARGIN_M",
        "support.box.max.y - EDGE_MARGIN_M",
        "support.top - objectBox.min.z",
        "clampCenterToSupport",
        "inner.minX + inner.size.x / 2",
        "inner.maxX - inner.size.x / 2",
        "inner.minY + inner.size.y / 2",
        "inner.maxY - inner.size.y / 2",
    ]:
        assert token in source


def test_pick_and_place_actions_use_existing_task_zone_metadata():
    source = ACTIONS.read_text(encoding="utf-8")
    for token in [
        "item?.zone_kind",
        "item?.zone_type",
        "item?.task_zone",
        "item?.task_zone_name",
        "zoneEntries('pick')",
        "zoneEntries('place')",
        "zone = zoneEntries(kind)[0]",
        "containingOrNearestSupport(boxCenter(zone.box))",
        "target.copy(clampCenterToSupport(boxCenter(zone.box), inner))",
    ]:
        assert token in source


def test_actions_commit_once_through_existing_transform_editor_and_reject_collisions():
    source = ACTIONS.read_text(encoding="utf-8")
    for token in [
        "fields.x.value = Number(position.x).toFixed(6)",
        "fields.y.value = Number(position.y).toFixed(6)",
        "fields.z.value = Number(position.z).toFixed(6)",
        "fields.x.dispatchEvent(new Event('input', { bubbles: true }))",
        "__WORKCELL_COLLISION_PLACEMENT_V1__?.validateItem",
        "editorApi()?.undo?.()",
        "action reverted",
        "workcell:contextual-placement-action",
    ]:
        assert token in source


def test_action_refresh_has_a_stable_signature_instead_of_a_dom_loop():
    source = ACTIONS.read_text(encoding="utf-8")
    for token in [
        "function sectionSignature",
        "section.dataset.signature = sectionSignature",
        "if (existing?.dataset.signature === signature) return",
        "new MutationObserver(() => queueMicrotask(refreshActions))",
    ]:
        assert token in source


def test_structural_actions_are_explicitly_deferred_until_patch_persistence_exists():
    source = ACTIONS.read_text(encoding="utf-8")
    assert "structural_actions: 'deferred_until_persistent_add_remove_patch_support'" in source
    assert "Duplicate" not in source
    assert "Delete item" not in source


def test_contextual_actions_are_preview_only_and_do_not_rebuild_the_bundle():
    source = ACTIONS.read_text(encoding="utf-8").lower()
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
        "unlink(",
    ]:
        assert forbidden not in source

    css = CSS.read_text(encoding="utf-8")
    assert ".contextual-placement-grid" in css
    assert "grid-template-columns: repeat(2, minmax(0, 1fr))" in css
    assert "body.embedded-mode .contextual-placement-actions" in css
    assert len(ACTIONS.read_text(encoding="utf-8").splitlines()) < 500
    assert len(CSS.read_text(encoding="utf-8").splitlines()) < 140
    assert BUNDLE.exists()
    assert "__WORKCELL_CONTEXTUAL_PLACEMENT_ACTIONS_V1__" in ACTIONS.read_text(encoding="utf-8")
