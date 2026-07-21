from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
VIEWER = ROOT / "workcell_studio_web/viewer"
INDEX = VIEWER / "index.html"
CSS = VIEWER / "simple_product_ui.css"
UI = VIEWER / "simple_product_ui.js"
BUNDLE = VIEWER / "dist/viewer.bundle.js"


def test_simple_ui_loads_after_viewer_placement_and_task_gizmo():
    html = INDEX.read_text(encoding="utf-8")
    ordered = [
        "./dist/viewer.bundle.js",
        "./rviz_light_baseline.js",
        "./support_surface_placement.js",
        "./workcell_task_gizmo.js",
        "./simple_product_ui.js",
    ]
    positions = [html.index(token) for token in ordered]
    assert positions == sorted(positions)
    assert 'href="simple_product_ui.css"' in html
    assert html.count("./simple_product_ui.js") == 1


def test_common_actions_stay_visible_and_advanced_controls_move_under_more():
    html = INDEX.read_text(encoding="utf-8")
    assert "Workcell Studio Product View" in html
    assert "Arrange, review and validate the workcell scene." in html
    assert ">Open scene" in html
    assert ">Fit</button>" in html
    assert ">Undo</button>" in html
    assert ">Redo</button>" in html
    assert ">Export edits</button>" in html
    assert 'id="task-gizmo-slot"' in html
    assert 'id="more-tools"' in html

    more = html.split('id="more-tools"', 1)[1].split("</details>", 1)[0]
    for control in [
        'id="clear-edits"',
        'id="snap-toggle"',
        'id="translation-snap"',
        'id="rotation-snap"',
        'id="labels-toggle"',
        'id="debug-overlays-toggle"',
        'id="show-initial-pose"',
    ]:
        assert control in more


def test_side_panels_use_progressive_disclosure_instead_of_permanent_clutter():
    html = INDEX.read_text(encoding="utf-8")
    for token in [
        "Scene items",
        'id="scene-details"',
        ">Scene details</summary>",
        "Selected item",
        'id="warnings-details"',
        'id="warning-count"',
    ]:
        assert token in html

    source = UI.read_text(encoding="utf-8")
    for token in [
        "PRIMARY_INSPECTOR_ROWS",
        "Technical details (",
        "details.hidden = count === 0",
        "if (count > 0) details.open = true",
        "moveTaskGizmoControl",
        "more.open = false",
    ]:
        assert token in source


def test_inspector_exposes_only_normal_workcell_pose_fields_by_default():
    source = UI.read_text(encoding="utf-8")
    for token in [
        "Place item",
        "Left / right (X)",
        "Forward / back (Y)",
        "Height (Z)",
        "Turn (Yaw)",
        "hold Shift for fine adjustment",
        "setTransformFieldVisible('z', freeHeight)",
        "setTransformFieldVisible('roll', false)",
        "setTransformFieldVisible('pitch', false)",
        "setTransformFieldVisible('scale_x', false)",
        "setTransformFieldVisible('scale_y', false)",
        "setTransformFieldVisible('scale_z', false)",
    ]:
        assert token in source


def test_more_menu_and_side_disclosures_remain_bounded_without_page_scrolling():
    css = CSS.read_text(encoding="utf-8")
    for token in [
        ".toolbar-more-panel",
        "max-height: min(70vh, 30rem)",
        "overflow-x: hidden",
        "overflow-y: auto",
        "overscroll-behavior: contain",
        ".panel-disclosure",
        ".technical-details",
        ".simple-ui-hidden-field",
        "body.embedded-mode .toolbar-more",
    ]:
        assert token in css


def test_simple_ui_is_small_preview_only_and_does_not_rebuild_the_bundle():
    source = UI.read_text(encoding="utf-8").lower()
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

    assert len(UI.read_text(encoding="utf-8").splitlines()) < 260
    assert len(CSS.read_text(encoding="utf-8").splitlines()) < 320
    assert BUNDLE.exists()
    assert "__WORKCELL_SIMPLE_PRODUCT_UI_V1__" in UI.read_text(encoding="utf-8")
