from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
VIEWER = ROOT / "workcell_studio_web/viewer"
INDEX = VIEWER / "index.html"
CONSOLIDATOR = VIEWER / "post_bundle_viewer_modules.js"

MODULES = [
    ("rviz_light_baseline.js", "__WORKCELL_RVIZ_LIGHT_BASELINE__"),
    ("support_surface_placement.js", "__WORKCELL_SUPPORT_PLACEMENT_V1__"),
    ("workcell_task_gizmo.js", "__WORKCELL_TASK_GIZMO_V1__"),
    ("collision_placement_validation.js", "__WORKCELL_COLLISION_PLACEMENT_V1__"),
    ("simple_product_ui.js", "__WORKCELL_SIMPLE_PRODUCT_UI_V1__"),
    ("contextual_placement_actions.js", "__WORKCELL_CONTEXTUAL_PLACEMENT_ACTIONS_V1__"),
]


def test_post_bundle_consolidator_loads_once_after_every_existing_extension():
    html = INDEX.read_text(encoding="utf-8")
    ordered = ["./dist/viewer.bundle.js", *[f"./{name}" for name, _ in MODULES], "./post_bundle_viewer_modules.js"]
    positions = [html.index(token) for token in ordered]
    assert positions == sorted(positions)
    assert html.count("./post_bundle_viewer_modules.js") == 1
    for token in ordered:
        assert html.count(token) == 1


def test_consolidator_has_one_explicit_inventory_for_all_post_bundle_modules():
    source = CONSOLIDATOR.read_text(encoding="utf-8")
    for filename, global_name in MODULES:
        assert f"file: '{filename}'" in source
        assert f"global: '{global_name}'" in source
        assert (VIEWER / filename).exists()
    assert "const MODULES = Object.freeze([" in source
    assert "modules: MODULES" in source
    assert "missing.length === 0" in source


def test_consolidated_api_preserves_existing_module_contracts():
    source = CONSOLIDATOR.read_text(encoding="utf-8")
    for token in [
        "window.__WORKCELL_POST_BUNDLE_VIEWER_V1__ = api",
        "getStatus",
        "getEditorState: editorState",
        "clearTransientFeedback",
        "refreshUi",
        "validateSelected",
        "setFreeHeight",
        "runPlacementAction",
        "syncSceneIdentity",
        "__WORKCELL_SUPPORT_PLACEMENT_V1__?.clear?.()",
        "__WORKCELL_COLLISION_PLACEMENT_V1__?.clear?.()",
        "__WORKCELL_SIMPLE_PRODUCT_UI_V1__?.refresh?.()",
        "__WORKCELL_CONTEXTUAL_PLACEMENT_ACTIONS_V1__?.refresh?.()",
        "__WORKCELL_COLLISION_PLACEMENT_V1__?.validateItem?.(id)",
        "__WORKCELL_TASK_GIZMO_V1__?.setFreeHeight?.(Boolean(enabled))",
        "__WORKCELL_CONTEXTUAL_PLACEMENT_ACTIONS_V1__?.run?.(String(action || ''))",
    ]:
        assert token in source


def test_scene_switch_cleanup_and_status_publication_are_bounded_and_event_driven():
    source = CONSOLIDATOR.read_text(encoding="utf-8")
    for token in [
        "sceneId !== lastSceneId",
        "clearTransientFeedback()",
        "workcell:post-bundle-viewer-status",
        "workcell_post_bundle_viewer_status",
        "signature === lastAnnouncement",
        "beforeunload",
        "visibilitychange",
        "queueMicrotask(() =>",
    ]:
        assert token in source
    assert "setInterval(" not in source
    assert "requestAnimationFrame(" not in source
    assert "new MutationObserver" not in source


def test_consolidation_does_not_add_an_eighth_renderer_patch_or_side_effectful_io():
    source = CONSOLIDATOR.read_text(encoding="utf-8").lower()
    for forbidden in [
        "webglrenderer",
        "prototype.render",
        "fetch(",
        "xmlhttprequest",
        "websocket",
        "environment.yaml",
        "workcell_studio_layout.yaml",
        "writefile",
        "unlink(",
        "execute_trajectory",
        "getmotionplan",
        "/plan_kinematic_path",
        "ros2 launch",
    ]:
        assert forbidden not in source
    assert len(CONSOLIDATOR.read_text(encoding="utf-8").splitlines()) < 220
