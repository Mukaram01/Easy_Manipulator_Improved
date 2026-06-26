import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
MAINWINDOW = ROOT / "workcell_builder/workcell_builder/gui/mainwindow.cpp"
UR5_INDEX = ROOT / "scenes/ur5_2f_test/generated/scene_visual_mesh_index.json"


def _token(value: str) -> str:
    return "".join(ch.lower() if ch.isalnum() else "_" for ch in (value or "")).strip("_")


def _final_identity(item: dict, source_row_index: int) -> str:
    visual = item.get("visual") or item.get("visual_name") or item.get("id") or "visual_missing"
    row_index = item.get("source_row_index")
    if row_index in (None, ""):
        row_index = source_row_index
    return "generated_urdf::{}::{}::{}".format(
        _token(item.get("link") or item.get("link_name") or item.get("object_id") or item.get("object") or "link_missing"),
        _token(visual),
        _token(str(row_index)),
    )


def _is_lower_fidelity_fallback(item: dict) -> bool:
    source = _token(item.get("source", ""))
    geometry = _token(item.get("geometry_type", ""))
    category = _token(item.get("category", ""))
    transform_status = _token(item.get("transform_status", ""))
    return source != "urdf_flattened" and (
        geometry != "mesh"
        or "fallback" in category
        or "fallback" in source
        or "static" in source
        or bool(item.get("primitive_fallback"))
        or transform_status in {"static_fallback", "static_fallback_parent"}
    )


def _retained_rows(items: list[dict]) -> list[dict]:
    flattened_links = {
        _token(item.get("link") or item.get("object_id") or item.get("object") or item.get("id"))
        for item in items
        if _token(item.get("source", "")) == "urdf_flattened" and _token(item.get("geometry_type", "")) == "mesh"
    }
    retained_ids = set()
    retained_rows = []
    for source_row_index, item in enumerate(sorted(items, key=lambda i: _token(i.get("source", "")) != "urdf_flattened")):
        identity = _final_identity(item, source_row_index)
        link_key = _token(item.get("link") or item.get("object_id") or item.get("object") or item.get("id"))
        if identity in retained_ids:
            continue
        if link_key in flattened_links and _is_lower_fidelity_fallback(item):
            continue
        retained_ids.add(identity)
        retained_rows.append({**item, "_final_identity": identity})
    return retained_rows


def _filtered_links(items: list[dict]) -> set[str]:
    return {_token(item.get("link", "")) for item in _retained_rows(items)}


def test_mainwindow_visual_loader_distinguishes_identity_fallback_and_shared_mesh_uri() -> None:
    src = MAINWINDOW.read_text(encoding="utf-8")
    assert "generated_urdf::%1::%2::%3" in src
    assert "source_row_index" in src
    assert "visual_item_is_lower_fidelity_fallback" in src
    assert "suppressed_by_urdf_flattened_visual_mesh" in src
    assert "retention check passed for ur5_2f_test" in src


def test_ur5_2f_visual_loader_filter_retains_robot_and_robotiq_rows() -> None:
    items = json.loads(UR5_INDEX.read_text(encoding="utf-8"))["visual_items"]
    retained_rows = _retained_rows(items)
    retained = {_token(item.get("link", "")) for item in retained_rows}
    identities = [item["_final_identity"] for item in retained_rows]
    valid_mesh_rows = [
        item
        for item in items
        if _token(item.get("source", "")) == "urdf_flattened" and _token(item.get("geometry_type", "")) == "mesh"
    ]

    assert len(items) >= 18
    assert len(retained_rows) >= 18
    assert len(retained_rows) <= len(items)
    assert len(identities) == len(set(identities))
    assert {"base_link", "base_link_inertia"} & retained
    assert {
        "shoulder_link",
        "upper_arm_link",
        "forearm_link",
        "wrist_1_link",
        "wrist_2_link",
        "wrist_3_link",
    } <= retained
    assert "gripper_base_link" in retained
    for token in ("finger_link", "knuckle_link", "finger_tip_link"):
        assert any(token in link for link in retained)
    assert "table" in retained
    assert "camera_link" in retained


def test_fresh_real_xacro_style_urdf_visual_ids_retain_ur5_and_robotiq_rows() -> None:
    links = [
        "base_link_inertia",
        "shoulder_link",
        "upper_arm_link",
        "forearm_link",
        "wrist_1_link",
        "wrist_2_link",
        "wrist_3_link",
        "gripper_base_link",
        "left_inner_finger",
        "right_inner_finger",
    ]
    items = [
        {
            "id": f"urdf_visual_{index}",
            "source_row_index": index,
            "link": link,
            "visual_name": f"visual_{index}",
            "source": "urdf_flattened",
            "category": "locked_generated_urdf_visual",
            "geometry_type": "mesh",
        }
        for index, link in enumerate(links)
    ]

    retained = _filtered_links(items)

    assert {
        "shoulder_link",
        "upper_arm_link",
        "forearm_link",
        "wrist_1_link",
        "wrist_2_link",
        "wrist_3_link",
    } <= retained
    assert "gripper_base_link" in retained
    assert any("finger" in link for link in retained)


def test_old_stale_generated_urdf_style_rows_still_retain_ur5_rows() -> None:
    links = ["shoulder_link", "upper_arm_link", "forearm_link", "wrist_1_link", "wrist_2_link", "wrist_3_link"]
    items = [
        {
            "id": f"generated_urdf::{link}::visual_{index}::{index}",
            "source_row_index": index,
            "link": link,
            "visual_name": f"visual_{index}",
            "source": "urdf_flattened",
            "category": "locked_generated_urdf_visual",
            "geometry_type": "mesh",
        }
        for index, link in enumerate(links, start=1)
    ]

    assert set(links) <= _filtered_links(items)


def test_editable_layout_semantic_ids_do_not_suppress_generated_urdf_rows() -> None:
    items = [
        {
            "id": "robot_base",
            "link": "robot_base",
            "visual": "editable_robot_base",
            "source": "layout",
            "category": "editable_layout",
            "geometry_type": "box",
        },
        {
            "link": "base_link",
            "visual": "base_link_visual",
            "visual_index": 0,
            "source": "urdf_flattened",
            "category": "locked_generated_urdf_visual",
            "geometry_type": "mesh",
        },
    ]

    retained = _filtered_links(items)

    assert "robot_base" in retained
    assert "base_link" in retained


def test_final_payload_preserves_visual_index_row_identity_metadata() -> None:
    src = MAINWINDOW.read_text(encoding="utf-8")
    viewport = (ROOT / "workcell_builder/workcell_builder/gui/scene3d_viewport_widget.cpp").read_text(encoding="utf-8")
    assert "p.visual_index_link" in src
    assert "p.source_row_index" in src
    assert "p.visual_index_link_chain" in src
    assert 'row["link"] = !item.visual_index_link' in viewport
    assert 'row["source_row_index"] = item.source_row_index' in viewport
    assert 'row["source"] = item.visual_index_source' in viewport

    items = json.loads(UR5_INDEX.read_text(encoding="utf-8"))["visual_items"]
    retained = _retained_rows(items)
    first_by_row = {row.get("source_row_index", idx): row for idx, row in enumerate(retained)}

    assert first_by_row[0]["link"] == "base_link_inertia"
    assert first_by_row[1]["link"] == "shoulder_link"
    assert first_by_row[2]["link"] == "upper_arm_link"
    assert [row["link"] for row in retained[:7]] == [
        "base_link_inertia",
        "shoulder_link",
        "upper_arm_link",
        "forearm_link",
        "wrist_1_link",
        "wrist_2_link",
        "wrist_3_link",
    ]
    assert not any(row["link"] in {"table", "camera_link", "gripper_base_link"} for row in retained[:7])
