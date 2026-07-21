from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
INDEX = ROOT / "workcell_studio_web/viewer/index.html"
PLACEMENT = ROOT / "workcell_studio_web/viewer/support_surface_placement.js"
BUNDLE = ROOT / "workcell_studio_web/viewer/dist/viewer.bundle.js"


def test_support_placement_loads_after_the_existing_viewer_and_light_baseline():
    html = INDEX.read_text(encoding="utf-8")
    bundle = './dist/viewer.bundle.js'
    baseline = './rviz_light_baseline.js'
    placement = './support_surface_placement.js'

    assert bundle in html
    assert baseline in html
    assert placement in html
    assert html.index(bundle) < html.index(baseline) < html.index(placement)


def test_support_placement_uses_real_bounds_top_height_and_xy_boundaries():
    source = PLACEMENT.read_text(encoding="utf-8")

    for token in [
        "new Set(['workbench_body', 'table_surface', 'tabletop'])",
        "support_surface_kind",
        "top_surface_z_m",
        "new THREE.Box3().setFromObject(object)",
        "support.box.min.x + EDGE_MARGIN_M",
        "support.box.max.x - EDGE_MARGIN_M",
        "support.box.min.y + EDGE_MARGIN_M",
        "support.box.max.y - EDGE_MARGIN_M",
        "const correctionZ = support.top - objectBox.min.z",
        "moveObjectInWorld(object, new THREE.Vector3(correctionX, correctionY, 0))",
        "moveObjectInWorld(object, new THREE.Vector3(0, 0, correctionZ))",
        "objectSize.x > availableX + EPSILON",
        "objectSize.y > availableY + EPSILON",
    ]:
        assert token in source


def test_support_placement_is_move_only_and_keeps_existing_edit_commit_path():
    source = PLACEMENT.read_text(encoding="utf-8")

    for token in [
        "state.mode !== 'move'",
        "state.selectedEditable !== true",
        "positionChanged(active.object, active.lastPosition)",
        "canvas.addEventListener('pointermove', onPointerMove)",
        "canvas.addEventListener('pointerup', finishPlacement)",
        "Preview-only placement helper; final poses remain committed by the existing edit-patch workflow.",
    ]:
        assert token in source

    forbidden = [
        "environment.yaml",
        "workcell_studio_layout.yaml",
        "buildEditPatch",
        "exportEditPatch",
        "fetch(",
        "scale.set(",
        "execute_trajectory",
        "real_hardware_enabled: true",
    ]
    assert all(token not in source for token in forbidden)


def test_support_placement_feedback_is_temporary_and_excluded_from_scene_fit():
    source = PLACEMENT.read_text(encoding="utf-8")

    for token in [
        "new THREE.Box3Helper",
        "exclude_from_fit_bounds = true",
        "exclude_from_physical_bounds = true",
        "helper_overlay = true",
        "Valid placement on",
        "No table or workbench under the object",
        "clearPlacementFeedback(900)",
        "workcell:support-placement",
        "window.__WORKCELL_SUPPORT_PLACEMENT_V1__",
    ]:
        assert token in source


def test_support_placement_stays_a_small_non_generated_change():
    source_lines = PLACEMENT.read_text(encoding="utf-8").splitlines()
    assert len(source_lines) < 420
    assert BUNDLE.exists()
