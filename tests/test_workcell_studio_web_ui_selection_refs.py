import pytest

from scripts import export_workcell_studio_web_scene as exporter


def empty_generated():
    return {
        "robots": [],
        "tools": [],
        "assets": [],
        "sensors": [],
        "zones": [],
        "frames": [],
    }


def test_selection_owner_registry_rejects_ambiguous_configured_robot():
    with pytest.raises(exporter.BlockingExportError, match="unambiguous robot owner"):
        exporter._selection_owner_registry(
            [{"id": "robot_a"}, {"id": "robot_b"}],
            [{"id": "tool"}],
            [],
            [],
        )


def test_generated_camera_and_table_receive_authored_ui_identity():
    generated = empty_generated()
    generated["sensors"].append(
        {
            "id": "generated_camera_visual",
            "link": "camera_link",
            "source_kind": "generated_preview",
        }
    )
    generated["assets"].append(
        {
            "id": "generated_table_visual",
            "link": "workbench_table",
            "source_kind": "generated_preview",
        }
    )

    exporter._annotate_generated_ui_selection_refs(
        generated,
        [
            {
                "id": "realsense_overhead",
                "role": "sensor",
                "display_name": "RealSense overhead camera",
                "source_kind": "user_authored",
            }
        ],
        [
            {
                "id": "support_surface_table",
                "role": "support_surface",
                "display_name": "Workbench table",
                "source_kind": "user_authored",
            }
        ],
    )

    camera = generated["sensors"][0]
    table = generated["assets"][0]

    assert camera["camera_id"] == "realsense_overhead"
    assert camera["canonical_scene_item_id"] == "realsense_overhead"
    assert table["support_surface_ref"] == "support_surface_table"
    assert table["canonical_scene_item_id"] == "support_surface_table"


def test_ambiguous_authored_identity_is_not_guessed():
    generated = empty_generated()
    generated["sensors"].append(
        {
            "id": "generated_camera_visual",
            "link": "camera_link",
            "source_kind": "generated_preview",
        }
    )

    exporter._annotate_generated_ui_selection_refs(
        generated,
        [
            {"id": "camera_a", "role": "camera", "source_kind": "user_authored"},
            {"id": "camera_b", "role": "camera", "source_kind": "user_authored"},
        ],
        [],
    )

    assert "camera_id" not in generated["sensors"][0]
    assert "canonical_scene_item_id" not in generated["sensors"][0]


def test_layout_identity_wins_over_generic_top_level_metadata():
    generated = empty_generated()
    generated["sensors"].append(
        {
            "id": "generated_camera_visual",
            "link": "camera_link",
            "source_kind": "generated_preview",
        }
    )
    generated["assets"].append(
        {
            "id": "generated_table_visual",
            "link": "workbench_table",
            "source_kind": "generated_preview",
        }
    )

    exporter._annotate_generated_ui_selection_refs(
        generated,
        [
            # Generic capability metadata: not a hierarchy row.
            {
                "id": "camera",
                "camera_id": "realsense_overhead",
            },
            {
                "id": "realsense_overhead",
                "role": "camera",
                "source_kind": "user_authored",
                "source_layer": "editable_layout",
            },
            {
                "id": "legacy_camera_metadata",
                "role": "camera",
                "source_kind": "user_authored",
                "source_layer": "editable_authored_physical",
            },
        ],
        [
            {
                "id": "support_surface_table",
                "role": "support_surface",
                "source_kind": "user_authored",
                "source_layer": "editable_layout",
            },
            {
                "id": "legacy_workbench_metadata",
                "role": "support_surface",
                "source_kind": "user_authored",
                "source_layer": "editable_authored_physical",
            },
        ],
    )

    assert generated["sensors"][0]["camera_id"] == "realsense_overhead"
    assert (
        generated["sensors"][0]["canonical_scene_item_id"]
        == "realsense_overhead"
    )
    assert (
        generated["assets"][0]["support_surface_ref"]
        == "support_surface_table"
    )
    assert (
        generated["assets"][0]["canonical_scene_item_id"]
        == "support_surface_table"
    )


def test_non_support_table_text_does_not_block_support_surface_mapping():
    generated = empty_generated()
    generated["assets"].append(
        {
            "id": "generated_table_visual",
            "link": "table",
            "source_kind": "generated_preview",
        }
    )

    exporter._annotate_generated_ui_selection_refs(
        generated,
        [],
        [
            {
                "id": "support_surface_table",
                "type": "support_surface",
                "role": "support_surface",
                "category": "work_surface",
                "source_kind": "user_authored",
                "source_layer": "editable_layout",
            },
            {
                "id": "table_keepout_note",
                "type": "safety_zone",
                "role": "keepout",
                "category": "safety_zone",
                "display_name": "Robot/Table Keepout Boundary",
                "source_kind": "user_authored",
                "source_layer": "editable_layout",
            },
        ],
    )

    table = generated["assets"][0]
    assert table["canonical_scene_item_id"] == "support_surface_table"
    assert table["support_surface_ref"] == "support_surface_table"
