from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
ASSEMBLY_CPP = ROOT / "workcell_builder/workcell_builder/gui/scene3d_candidate_assembly.cpp"
CLASSIFIER_CPP = ROOT / "workcell_builder/workcell_builder/gui/scene3d_visual_classification.cpp"
TOKENS_YAML = ROOT / "workcell_builder/workcell_builder/config/scene3d_helper_tokens.yaml"


def test_product_view_treats_helper_rows_as_overlay_only() -> None:
    source = CLASSIFIER_CPP.read_text(encoding="utf-8")
    assembly_source = ASSEMBLY_CPP.read_text(encoding="utf-8")
    tokens_yaml = TOKENS_YAML.read_text(encoding="utf-8")

    assert "bool identity_contains_helper_overlay_token" in source
    assert "bool is_helper_overlay_identity" in source
    assert "const bool is_overlay_or_helper = workcell_builder::scene3d_visual_classification::is_helper_overlay_identity(item);" in assembly_source
    assert "helper_tokens:" in tokens_yaml

    for token in (
        'QStringLiteral("safety_zone")',
        'QStringLiteral("pick_zone")',
        'QStringLiteral("place_zone")',
        'QStringLiteral("robot_reach")',
        'QStringLiteral("warning_anchor")',
        'QStringLiteral("warning_badge")',
        'QStringLiteral("camera_fov")',
        'QStringLiteral("pick_coverage")',
        'QStringLiteral("reachability")',
        'QStringLiteral("collision")',
        'QStringLiteral("work_envelope")',
        'QStringLiteral("task_route")',
        'QStringLiteral("approach_retreat")',
        'QStringLiteral("epd_detection")',
        'QStringLiteral("detection_label")',
        'QStringLiteral("bounds_box")',
        'QStringLiteral("bounding_box")',
    ):
        assert token in source


def test_product_view_keeps_helper_overlays_behind_overlay_layer() -> None:
    source = ASSEMBLY_CPP.read_text(encoding="utf-8")
    body = source.split("bool include_preview_item_for_scene3d", 1)[1]
    body = body.split("Scene3DLayerVisibilityDefaults compute_scene3d_default_layer_visibility", 1)[0]

    assert 'if (is_overlay_or_helper) return enabled_layers.contains("overlay");' in body
    assert "out.overlay = false;" in source


def test_include_preview_item_combined_includes_source_identity() -> None:
    source = ASSEMBLY_CPP.read_text(encoding="utf-8")
    body = source.split("bool include_preview_item_for_scene3d", 1)[1]
    body = body.split("Scene3DLayerVisibilityDefaults compute_scene3d_default_layer_visibility", 1)[0]

    assert "source_layer +" in body
    assert "visual_source +" in body
    assert 'const QString source_layer = token(item.source_layer);' in body
    assert 'const QString visual_source = token(item.active_visual_source);' in body
    assert 'const QString role = token(item.role);' in body
    assert 'const QString category = token(item.category);' in body
    assert 'token(item.status)' in body
    assert 'item.warnings.join("|").toLower()' in body
