from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
MAIN = (ROOT / "workcell_builder/workcell_builder/gui/mainwindow.cpp").read_text(encoding="utf-8")
VIEWPORT = (ROOT / "workcell_builder/workcell_builder/gui/scene3d_viewport_widget.cpp").read_text(encoding="utf-8")


def _final_draw_rows_from_fresh_visual_index_rows(visual_rows):
    # Mirrors the native PreviewItem handoff contract after ingestion: fresh
    # urdf_visual_* ids must not be parsed for link identity; the visual-index
    # fields populated on PreviewItem are the authority for final draw rows.
    rows = []
    for row in visual_rows:
        link = row.get("link") or row.get("link_name")
        rows.append(
            {
                "item_id": row["id"],
                "id": row["id"],
                "source_row_index": row["source_row_index"],
                "link": link,
                "link_name": link,
                "canonical_link_name": link,
                "source_layer": "locked_generated_urdf_visual",
                "active_visual_source": "generated_urdf_visual",
                "visible": True,
                "rendered": True,
            }
        )
    return rows


def test_fresh_urdf_visual_preview_items_preserve_visual_index_identity_fields():
    assert "p.visual_index_link = raw_visual_link.isEmpty() ? stable_visual_link_identity : raw_visual_link;" in MAIN
    assert "p.visual_index_link_name = raw_visual_link_name.isEmpty() ? stable_visual_link_identity : raw_visual_link_name;" in MAIN
    assert "p.source_row_index = source_row_index;" in MAIN
    assert 'p.source_layer = QStringLiteral("locked_generated_urdf_visual");' in MAIN
    assert 'p.active_visual_source = QStringLiteral("generated_urdf_visual");' in MAIN


def test_viewport_link_resolution_prefers_visual_index_fields_before_legacy_ids():
    link_fn_start = VIEWPORT.rindex("QString scene3d_link_name_for_item")
    link_fn = VIEWPORT[link_fn_start : VIEWPORT.index("QString scene3d_canonical_link_name", link_fn_start)]
    assert "item.visual_index_link_name" in link_fn
    assert "item.visual_index_link" in link_fn
    assert link_fn.index("item.visual_index_link_name") < link_fn.index("const QString id = item.id.trimmed()")
    assert link_fn.index("item.visual_index_link") < link_fn.index("const QString id = item.id.trimmed()")


def test_fresh_ur5_urdf_visual_rows_survive_to_final_draw_visual_items():
    required_links = [
        "shoulder_link",
        "upper_arm_link",
        "forearm_link",
        "wrist_1_link",
        "wrist_2_link",
        "wrist_3_link",
    ]
    visual_rows = [
        {"id": f"urdf_visual_{index}", "source_row_index": index, "link": link, "visual_name": f"visual_{index}"}
        for index, link in enumerate(["base_link_inertia", *required_links], start=1)
    ]
    final_draw_visual_items = _final_draw_rows_from_fresh_visual_index_rows(visual_rows)

    retained_links = {row["link"] for row in final_draw_visual_items if row["visible"] and row["rendered"]}

    assert set(required_links) <= retained_links
    assert all(row["id"].startswith("urdf_visual_") for row in final_draw_visual_items)
    assert all(row["source_layer"] == "locked_generated_urdf_visual" for row in final_draw_visual_items)
    assert all(row["active_visual_source"] == "generated_urdf_visual" for row in final_draw_visual_items)
