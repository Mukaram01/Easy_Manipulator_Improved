from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
ASSEMBLY_CPP = ROOT / "workcell_builder/workcell_builder/gui/scene3d_candidate_assembly.cpp"


def test_product_view_treats_zone_rows_as_helper_overlays() -> None:
    source = ASSEMBLY_CPP.read_text(encoding="utf-8")

    assert "bool is_product_helper_overlay_role" in source
    assert "const bool is_overlay_or_helper = is_product_helper_overlay_role(role, category, combined);" in source

    for token in (
        'role == QStringLiteral("pick_zone")',
        'role == QStringLiteral("place_zone")',
        'role == QStringLiteral("robot_reach")',
        'role == QStringLiteral("warning_anchor")',
        'role == QStringLiteral("warning_badge")',
        'category == QStringLiteral("pick_zone")',
        'category == QStringLiteral("place_zone")',
    ):
        assert token in source


def test_product_view_keeps_helper_overlays_behind_overlay_layer() -> None:
    source = ASSEMBLY_CPP.read_text(encoding="utf-8")
    body = source.split("bool include_preview_item_for_scene3d", 1)[1]
    body = body.split("Scene3DLayerVisibilityDefaults compute_scene3d_default_layer_visibility", 1)[0]

    assert 'if (is_overlay_or_helper) return enabled_layers.contains("overlay");' in body
    assert "out.overlay = false;" in source
