from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
INDEX = ROOT / "workcell_studio_web/viewer/index.html"
GIZMO = ROOT / "workcell_studio_web/viewer/workcell_task_gizmo.js"


def test_task_gizmo_loads_after_viewer_visuals_and_surface_placement():
    html = INDEX.read_text(encoding="utf-8")
    ordered = [
        "./dist/viewer.bundle.js",
        "./rviz_light_baseline.js",
        "./support_surface_placement.js",
        "./workcell_task_gizmo.js",
    ]
    positions = [html.index(token) for token in ordered]
    assert positions == sorted(positions)
    assert html.count("./workcell_task_gizmo.js") == 1


def test_task_gizmo_defaults_to_xy_move_and_yaw_only_rotation():
    source = GIZMO.read_text(encoding="utf-8")

    for token in [
        "state.mode === 'move'",
        "state.selectedEditable === true",
        "configureAxes(gizmo, 'translate', true, true, runtime.freeHeight)",
        "configureAxes(gizmo, 'rotate', false, false, true)",
        "if (gizmo.mode !== mode) gizmo.setMode(mode)",
        "if (gizmo.space !== 'world') gizmo.setSpace('world')",
        "if (gizmo.showX !== showX) gizmo.showX = showX",
        "if (gizmo.showY !== showY) gizmo.showY = showY",
        "if (gizmo.showZ !== showZ) gizmo.showZ = showZ",
        "rotation_axes: ['z']",
    ]:
        assert token in source

    assert "setMode('scale')" not in source


def test_free_height_is_explicit_and_pauses_surface_snapping():
    source = GIZMO.read_text(encoding="utf-8")

    for token in [
        "free-height-toggle",
        "Surface move (XY)",
        "Free height (XYZ)",
        "support-surface placement controls Z",
        "window.__WORKCELL_SUPPORT_PLACEMENT_V1__?.clear?.()",
        "if (runtime.freeHeight) queueMicrotask(clearSupportPlacement)",
        "runtime.toggle.disabled = !editableMove",
        "state.mode !== 'move' && runtime.freeHeight",
        "runtime.selectedItemId !== state.selectedItemId",
    ]:
        assert token in source


def test_shift_temporarily_enables_fine_snap_and_restores_user_values():
    source = GIZMO.read_text(encoding="utf-8")

    for token in [
        "FINE_TRANSLATION_M = 0.001",
        "FINE_ROTATION_DEG = 1",
        "event.key === 'Shift'",
        "setFineMode(true)",
        "setFineMode(false)",
        "window.addEventListener('blur'",
        "runtime.savedTranslationValue = translation.value",
        "runtime.savedRotationValue = rotation.value",
        "translation.value = String(FINE_TRANSLATION_M)",
        "rotation.value = String(FINE_ROTATION_DEG)",
        "translation.value = runtime.savedTranslationValue",
        "rotation.value = runtime.savedRotationValue",
        "THREE.MathUtils.degToRad",
        "snapDiffers(gizmo.translationSnap, desiredTranslation)",
        "snapDiffers(gizmo.rotationSnap, rotation)",
    ]:
        assert token in source


def test_task_gizmo_is_preview_only_and_does_not_add_motion_or_source_writes():
    source = GIZMO.read_text(encoding="utf-8").lower()

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
        "setmode('scale')",
    ]:
        assert forbidden not in source

    assert "__WORKCELL_TASK_GIZMO_V1__" in GIZMO.read_text(encoding="utf-8")
    assert "workcell:gizmo-task-mode" in GIZMO.read_text(encoding="utf-8")
