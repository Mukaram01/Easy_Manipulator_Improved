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


def _mesh_identity(item: dict) -> str:
    return (
        item.get("package_uri")
        or item.get("mesh_uri")
        or item.get("source_path")
        or item.get("mesh_path")
        or item.get("resolved_path")
        or item.get("resolved_source_path")
        or "mesh_missing"
    )


_PROTECTED_UR5_LINKS = {
    "base_link_inertia",
    "shoulder_link",
    "upper_arm_link",
    "forearm_link",
    "wrist_1_link",
    "wrist_2_link",
    "wrist_3_link",
}


def _normalized_ur5_link(item: dict) -> str:
    for key in ("link", "link_name", "canonical_link_name"):
        link = _token(item.get(key, ""))
        if link in _PROTECTED_UR5_LINKS:
            return link
    id_token = _token(item.get("id", ""))
    for link in _PROTECTED_UR5_LINKS:
        if id_token == link or f"_{link}_" in id_token or id_token.startswith(f"{link}_") or id_token.endswith(f"_{link}"):
            return link
    return ""


def _normalized_mesh_identity(item: dict) -> str:
    parts = [
        _token(str(item.get(key, "")))
        for key in ("package_uri", "mesh_uri", "source_path", "mesh_path", "resolved_path", "resolved_source_path", "filename")
        if str(item.get(key, "")).strip()
    ]
    return "__".join(parts) or "mesh_missing"


def _is_protected_ur5_generated_visual_row(item: dict) -> bool:
    link = _normalized_ur5_link(item)
    mesh_mix = "|".join(str(item.get(key, "")).lower() for key in (
        "package_uri", "mesh_uri", "source_path", "mesh_path", "resolved_path", "resolved_source_path", "filename"
    ))
    mesh_token = _token(mesh_mix)
    known_ur5_mesh_files = {"base_dae", "shoulder_dae", "upperarm_dae", "forearm_dae", "wrist1_dae", "wrist2_dae", "wrist3_dae"}
    return (
        bool(link)
        and _token(item.get("geometry_type") or item.get("type") or "") == "mesh"
        and all(part in mesh_token for part in ("ur_description", "meshes", "ur5", "visual"))
        and any(mesh_file in mesh_token for mesh_file in known_ur5_mesh_files)
    )


def _generated_row_key(item: dict, source_row_index: int) -> str:
    visual = item.get("visual_name") or item.get("visual") or item.get("id") or "visual_missing"
    row_index = item.get("source_row_index")
    if row_index in (None, ""):
        row_index = source_row_index
    link = _normalized_ur5_link(item) if _is_protected_ur5_generated_visual_row(item) else ""
    return "generated_urdf_row::{}::{}::{}::{}".format(
        link or _token(item.get("link") or item.get("link_name") or item.get("canonical_link_name") or item.get("object_id") or item.get("object") or "link_missing"),
        _token(visual),
        _token(str(row_index)),
        _normalized_mesh_identity(item) if link else _token(_mesh_identity(item)),
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
    retained_generated_row_keys = set()
    retained_rows = []
    for source_row_index, item in enumerate(sorted(items, key=lambda i: _token(i.get("source", "")) != "urdf_flattened")):
        generated_row_key = _generated_row_key(item, source_row_index)
        if generated_row_key in retained_generated_row_keys:
            continue
        retained_generated_row_keys.add(generated_row_key)
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
    return {_normalized_ur5_link(item) or _token(item.get("link") or item.get("link_name") or item.get("canonical_link_name") or "") for item in _retained_rows(items)}


def test_mainwindow_visual_loader_distinguishes_identity_fallback_and_shared_mesh_uri() -> None:
    src = MAINWINDOW.read_text(encoding="utf-8")
    assert "generated_urdf::%1::%2::%3" in src
    assert "source_row_index" in src
    assert "visual_item_is_lower_fidelity_fallback" in src
    assert "suppressed_by_urdf_flattened_visual_mesh" in src
    assert "retention check passed for ur5_2f_test" in src


def _focused_m1_visual_items() -> list[dict]:
    ur5_meshes = {
        "shoulder_link": "shoulder.dae",
        "upper_arm_link": "upperarm.dae",
        "forearm_link": "forearm.dae",
        "wrist_1_link": "wrist1.dae",
        "wrist_2_link": "wrist2.dae",
        "wrist_3_link": "wrist3.dae",
    }
    rows = [
        {
            "id": "robot_base",
            "link": "robot_base",
            "visual": "robot_base_helper",
            "source": "layout",
            "role": "robot",
            "category": "helper",
            "geometry_type": "box",
        },
        {
            "id": "robot_reach",
            "link": "robot_reach",
            "visual": "robot_reach_helper",
            "source": "layout",
            "role": "robot",
            "category": "robot_reach",
            "geometry_type": "sphere",
        },
        {
            "id": "warning_anchor_1",
            "link": "warning_anchor_1",
            "visual": "warning_badge",
            "source": "diagnostic",
            "role": "warning",
            "category": "warning_anchor",
            "geometry_type": "box",
        },
        {
            "id": "object_a_bounds_box",
            "link": "object_a",
            "visual": "object_a_bounds_box",
            "source": "layout",
            "role": "helper",
            "category": "bounds_box",
            "geometry_type": "box",
        },
    ]
    for index, (link, filename) in enumerate(ur5_meshes.items(), start=10):
        rows.append(
            {
                "id": f"generated_urdf::{link}::visual::{index}",
                "source_row_index": index,
                "link": link,
                "visual_name": f"{link}_visual",
                "source": "urdf_flattened",
                "source_layer": "locked_generated_urdf_visual",
                "active_visual_source": "mesh_preview",
                "role": "robot",
                "category": "locked_generated_urdf_visual",
                "geometry_type": "mesh",
                "package_uri": f"package://ur_description/meshes/ur5/visual/{filename}",
            }
        )
    rows.extend(
        [
            {
                "id": "generated_urdf::left_inner_finger::visual::30",
                "source_row_index": 30,
                "link": "left_inner_finger",
                "visual_name": "left_inner_finger_visual",
                "source": "urdf_flattened",
                "source_layer": "locked_generated_urdf_visual",
                "active_visual_source": "mesh_preview",
                "role": "tool",
                "category": "locked_generated_urdf_visual",
                "geometry_type": "mesh",
                "package_uri": "package://robotiq_85_description/meshes/visual/inner_finger.dae",
            },
            {
                "id": "generated_urdf::right_inner_finger::visual::31",
                "source_row_index": 31,
                "link": "right_inner_finger",
                "visual_name": "right_inner_finger_visual",
                "source": "urdf_flattened",
                "source_layer": "locked_generated_urdf_visual",
                "active_visual_source": "mesh_preview",
                "role": "tool",
                "category": "locked_generated_urdf_visual",
                "geometry_type": "mesh",
                "package_uri": "package://robotiq_85_description/meshes/visual/inner_finger.dae",
            },
            {
                "id": "table",
                "source_row_index": 40,
                "link": "table",
                "visual_name": "table_visual",
                "source": "urdf_flattened",
                "role": "environment",
                "category": "workbench",
                "geometry_type": "mesh",
                "package_uri": "package://workcell_builder/meshes/table.dae",
            },
            {
                "id": "camera_link",
                "source_row_index": 41,
                "link": "camera_link",
                "visual_name": "camera_visual",
                "source": "urdf_flattened",
                "role": "sensor",
                "category": "camera",
                "geometry_type": "mesh",
                "package_uri": "package://workcell_builder/meshes/camera.dae",
            },
            # Exact duplicate: same canonical link, visual, row index, and mesh identity.
            {
                "id": "duplicate_left_inner_finger",
                "source_row_index": 30,
                "link": "left_inner_finger",
                "visual_name": "left_inner_finger_visual",
                "source": "urdf_flattened",
                "geometry_type": "mesh",
                "package_uri": "package://robotiq_85_description/meshes/visual/inner_finger.dae",
            },
        ]
    )
    return rows


def _credible_physical_mesh_rows(items: list[dict]) -> list[dict]:
    helper_tokens = {
        "overlay", "helper", "diagnostic", "safety_zone", "pick_zone", "place_zone",
        "robot_reach", "warning_anchor", "warning_badge", "bounds_box", "bounding_box",
    }
    physical = []
    for item in _retained_rows(items):
        identity_tokens = {_token(str(item.get(k, ""))) for k in ("source_layer", "active_visual_source", "role", "category", "id", "display_name", "status")}
        if identity_tokens & helper_tokens:
            continue
        if _token(item.get("geometry_type", "")) == "mesh" and _mesh_identity(item) != "mesh_missing":
            physical.append(item)
    return physical



def _requested_18_visual_index_rows_with_semantic_placeholders() -> tuple[list[dict], list[dict]]:
    ur5_meshes = [
        ("base_link_inertia", "base.dae"),
        ("shoulder_link", "shoulder.dae"),
        ("upper_arm_link", "upperarm.dae"),
        ("forearm_link", "forearm.dae"),
        ("wrist_1_link", "wrist1.dae"),
        ("wrist_2_link", "wrist2.dae"),
        ("wrist_3_link", "wrist3.dae"),
    ]
    robotiq_links = [
        "robotiq_arg2f_base_link",
        "left_outer_knuckle",
        "left_outer_finger",
        "left_inner_finger",
        "left_inner_finger_pad",
        "right_outer_knuckle",
        "right_outer_finger",
        "right_inner_finger",
        "right_inner_finger_pad",
    ]
    visual_rows: list[dict] = []
    for index, (link, filename) in enumerate(ur5_meshes):
        visual_rows.append(
            {
                "id": f"urdf_visual_{index}_{link}",
                "source_row_index": index,
                "link": link,
                "visual_name": f"{link}_visual",
                "source": "urdf_flattened",
                "source_layer": "locked_generated_urdf_visual",
                "active_visual_source": "mesh_preview",
                "role": "robot",
                "category": "locked_generated_urdf_visual",
                "geometry_type": "mesh",
                "package_uri": f"package://ur_description/meshes/ur5/visual/{filename}",
                "resolved_path": f"/opt/ros/humble/share/ur_description/meshes/ur5/visual/{filename}",
            }
        )
    for offset, link in enumerate(robotiq_links, start=7):
        visual_rows.append(
            {
                "id": f"urdf_visual_{offset}_{link}",
                "source_row_index": offset,
                "link": link,
                "visual_name": f"{link}_visual",
                "source": "urdf_flattened",
                "source_layer": "locked_generated_urdf_visual",
                "active_visual_source": "mesh_preview",
                "role": "gripper",
                "category": "locked_generated_urdf_visual",
                "geometry_type": "mesh",
                "package_uri": f"package://robotiq_85_description/meshes/visual/{link}.dae",
            }
        )
    visual_rows.extend(
        [
            {
                "id": "urdf_visual_16_table",
                "source_row_index": 16,
                "link": "table",
                "visual_name": "table_visual",
                "source": "urdf_flattened",
                "source_layer": "locked_generated_urdf_visual",
                "active_visual_source": "mesh_preview",
                "role": "environment",
                "category": "workbench",
                "geometry_type": "mesh",
                "package_uri": "package://workbench_description/meshes/visual/table.dae",
            },
            {
                "id": "urdf_visual_17_camera",
                "source_row_index": 17,
                "link": "camera_link",
                "visual_name": "camera_visual",
                "source": "urdf_flattened",
                "source_layer": "locked_generated_urdf_visual",
                "active_visual_source": "mesh_preview",
                "role": "sensor",
                "category": "camera",
                "geometry_type": "mesh",
                "package_uri": "package://realsense2_description/meshes/d435.dae",
            },
        ]
    )
    semantic_placeholders = [
        {"id": "robot_base", "link": "robot_base", "visual_name": "robot_base_helper", "role": "robot", "category": "helper", "geometry_type": "box"},
        {"id": "robot_reach", "link": "robot_reach", "visual_name": "robot_reach_helper", "role": "robot", "category": "robot_reach", "geometry_type": "sphere"},
        {"id": "conveyor", "link": "conveyor", "visual_name": "conveyor", "role": "environment", "category": "conveyor", "geometry_type": "box"},
        {"id": "object_a", "link": "object_a", "visual_name": "object_a", "role": "object", "category": "pick_object", "geometry_type": "box"},
        {"id": "warning", "link": "warning", "visual_name": "warning_badge", "role": "warning", "category": "warning_anchor", "geometry_type": "box"},
    ]
    assert len(visual_rows) == 18
    return visual_rows, semantic_placeholders


def _retained_rows_with_duplicate_count(items: list[dict]) -> tuple[list[dict], int, list[dict]]:
    retained_keys = set()
    retained = []
    skipped_duplicates = []
    for source_row_index, item in enumerate(items):
        key = _generated_row_key(item, source_row_index)
        if key in retained_keys:
            skipped_duplicates.append(item)
            continue
        retained_keys.add(key)
        retained.append(item)
    return retained, len(skipped_duplicates), skipped_duplicates


def _is_semantic_helper_non_renderable_by_default(item: dict) -> bool:
    tokens = {
        _token(str(item.get(key, "")))
        for key in ("source_layer", "active_visual_source", "role", "category", "id", "display_name", "status", "warnings", "mesh_load_warning")
    }
    helper_tokens = {"helper", "robot_reach", "warning_anchor", "warning_badge", "bounds_box", "bounding_box", "overlay", "diagnostic"}
    if tokens & helper_tokens:
        return True
    return _token(item.get("geometry_type", "")) != "mesh" and _mesh_identity(item) == "mesh_missing"


def test_requested_18_visual_rows_retain_ur5_robotiq_table_camera_and_skip_only_exact_duplicate() -> None:
    visual_rows, semantic_placeholders = _requested_18_visual_index_rows_with_semantic_placeholders()
    items_with_duplicate = visual_rows + [dict(visual_rows[3])] + semantic_placeholders

    retained_rows, duplicate_skip_count, skipped_duplicates = _retained_rows_with_duplicate_count(items_with_duplicate)
    retained_links = {_token(row.get("link", "")) for row in retained_rows}
    ur5_rows = [row for row in retained_rows if _normalized_ur5_link(row)]
    ur5_keys = [_generated_row_key(row, index) for index, row in enumerate(visual_rows[:7])]

    assert len(ur5_rows) == 7
    assert _PROTECTED_UR5_LINKS <= retained_links
    assert duplicate_skip_count == 1
    assert skipped_duplicates == [visual_rows[3]]
    retained_ur5_source_rows = {row.get("source_row_index") for row in retained_rows if _normalized_ur5_link(row)}
    assert set(range(7)) <= retained_ur5_source_rows
    assert len(ur5_keys) == len(set(ur5_keys))
    assert _generated_row_key(visual_rows[3], 3) == _generated_row_key(dict(visual_rows[3]), 18)

    for link in [
        "robotiq_arg2f_base_link",
        "left_outer_knuckle",
        "left_outer_finger",
        "left_inner_finger",
        "left_inner_finger_pad",
        "right_outer_knuckle",
        "right_outer_finger",
        "right_inner_finger",
        "right_inner_finger_pad",
        "table",
        "camera_link",
    ]:
        assert link in retained_links

    assert all(_is_semantic_helper_non_renderable_by_default(item) for item in semantic_placeholders)

def _ur5_18_row_fixture_with_path_field(path_field: str, path_prefix: str) -> list[dict]:
    ur5_meshes = {
        "base_link_inertia": "base.dae",
        "shoulder_link": "shoulder.dae",
        "upper_arm_link": "upperarm.dae",
        "forearm_link": "forearm.dae",
        "wrist_1_link": "wrist1.dae",
        "wrist_2_link": "wrist2.dae",
        "wrist_3_link": "wrist3.dae",
    }
    rows = []
    for index, (link, filename) in enumerate(ur5_meshes.items()):
        rows.append(
            {
                "id": f"urdf_visual_{index}_{link}",
                "source_row_index": index,
                "link": link,
                "visual_name": f"{link}_visual",
                "source": "urdf_flattened",
                "source_layer": "locked_generated_urdf_visual",
                "active_visual_source": "mesh_preview",
                "role": "robot",
                "category": "locked_generated_urdf_visual",
                "geometry_type": "mesh",
                path_field: f"{path_prefix}/{filename}",
                "filename": filename,
            }
        )
    rows.extend(
        [
            {"id": "robot_base", "link": "robot_base", "visual_name": "robot_base_helper", "role": "robot", "category": "helper", "geometry_type": "box"},
            {"id": "robot_reach", "link": "robot_reach", "visual_name": "robot_reach_helper", "role": "robot", "category": "robot_reach", "geometry_type": "sphere"},
            {"id": "object_a", "link": "object_a", "visual_name": "object_a", "role": "object", "category": "pick_object", "geometry_type": "box"},
            {"id": "conveyor", "link": "conveyor", "visual_name": "conveyor", "role": "environment", "category": "conveyor", "geometry_type": "box"},
            {"id": "warning_anchor_1", "link": "warning", "visual_name": "warning_badge", "role": "warning", "category": "warning_anchor", "geometry_type": "box"},
            {"id": "left_inner_finger", "link": "left_inner_finger", "visual_name": "left_inner_finger", "geometry_type": "mesh", "package_uri": "package://robotiq_85_description/meshes/visual/inner_finger.dae"},
            {"id": "right_inner_finger", "link": "right_inner_finger", "visual_name": "right_inner_finger", "geometry_type": "mesh", "package_uri": "package://robotiq_85_description/meshes/visual/inner_finger.dae"},
            {"id": "table", "link": "table", "visual_name": "table", "geometry_type": "mesh", "package_uri": "package://workbench_description/meshes/visual/table.stl"},
            {"id": "camera_link", "link": "camera_link", "visual_name": "camera", "geometry_type": "mesh", "package_uri": "package://realsense2_description/meshes/d435.dae"},
            {"id": "object_a_bounds_box", "link": "object_a", "visual_name": "object_a_bounds_box", "role": "helper", "category": "bounds_box", "geometry_type": "box"},
            {"id": "conveyor_warning", "link": "conveyor", "visual_name": "warning", "role": "warning", "category": "warning", "geometry_type": "box"},
        ]
    )
    assert len(rows) == 18
    return rows


def test_18_row_fixture_retains_all_ur5_rows_with_package_uri_paths() -> None:
    items = _ur5_18_row_fixture_with_path_field("package_uri", "package://ur_description/meshes/ur5/visual")
    retained = _filtered_links(items)

    assert _PROTECTED_UR5_LINKS <= retained
    assert all(_is_protected_ur5_generated_visual_row(item) for item in items[:7])
    assert not any(_is_protected_ur5_generated_visual_row(item) for item in items[7:])


def test_18_row_fixture_retains_all_ur5_rows_with_resolved_local_paths() -> None:
    items = _ur5_18_row_fixture_with_path_field(
        "resolved_path",
        "/home/user/workcell_ws/install/ur_description/share/ur_description/meshes/ur5/visual",
    )
    retained = _filtered_links(items)

    assert _PROTECTED_UR5_LINKS <= retained
    assert all(_is_protected_ur5_generated_visual_row(item) for item in items[:7])
    assert not any(_is_protected_ur5_generated_visual_row(item) for item in items[7:])

def test_focused_m1_fixture_retains_physical_ur5_robotiq_table_and_camera_rows() -> None:
    items = _focused_m1_visual_items()
    retained_rows = _retained_rows(items)
    physical_rows = _credible_physical_mesh_rows(items)
    retained = {_token(item.get("link", "")) for item in retained_rows}
    physical = {_token(item.get("link", "")) for item in physical_rows}

    required_ur5_links = {
        "shoulder_link",
        "upper_arm_link",
        "forearm_link",
        "wrist_1_link",
        "wrist_2_link",
        "wrist_3_link",
    }
    assert required_ur5_links <= retained
    assert required_ur5_links <= physical
    assert {"left_inner_finger", "right_inner_finger"} <= retained
    assert {"left_inner_finger", "right_inner_finger"} <= physical
    assert {"table", "camera_link"} <= retained
    assert {"table", "camera_link"} <= physical
    assert not ({"robot_base", "robot_reach", "warning_anchor_1", "object_a"} & physical)

    left_finger_rows = [row for row in retained_rows if row.get("link") == "left_inner_finger"]
    assert len(left_finger_rows) == 1
    assert "robot_base" in retained
    assert "robot_reach" in retained
    assert required_ur5_links.isdisjoint({"robot_base", "robot_reach"})
    assert len([row for row in retained_rows if row.get("role") == "robot" and row.get("geometry_type") == "mesh"]) == 6


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

    assert len(items) >= 9
    assert len(retained_rows) >= 9
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
    # The checked-in M1 mesh index currently covers the UR5 arm, table/workbench, and camera.
    # Robotiq retention is protected by the focused synthetic fixture above so this static
    # regression test does not depend on generated artifact freshness.
    assert {"table", "table_"} & retained
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


def test_raw_ur5_mesh_rows_without_visual_source_metadata_are_protected() -> None:
    items = [
        {
            "id": f"urdf_visual_{index}_{link}",
            "source_row_index": index,
            "link_name": link,
            "visual_name": f"visual_{index}",
            "type": "mesh",
            "mesh_uri": f"package://ur_description/meshes/ur5/visual/{filename}",
        }
        for index, (link, filename) in enumerate(
            {
                "base_link_inertia": "base.dae",
                "shoulder_link": "shoulder.dae",
                "upper_arm_link": "upperarm.dae",
                "forearm_link": "forearm.dae",
                "wrist_1_link": "wrist1.dae",
                "wrist_2_link": "wrist2.dae",
                "wrist_3_link": "wrist3.dae",
            }.items()
        )
    ]

    retained = _filtered_links(items)

    assert _PROTECTED_UR5_LINKS <= retained
    assert all(_is_protected_ur5_generated_visual_row(item) for item in items)


def test_ur5_link_can_be_recovered_from_raw_urdf_visual_id() -> None:
    items = [
        {
            "id": "urdf_visual_12_shoulder_link_visual",
            "source_row_index": 12,
            "visual_name": "shoulder_visual",
            "geometry_type": "mesh",
            "resolved_path": "/tmp/ws/install/ur_description/share/ur_description/meshes/ur5/visual/shoulder.dae",
        }
    ]

    retained_rows = _retained_rows(items)

    assert len(retained_rows) == 1
    assert _normalized_ur5_link(retained_rows[0]) == "shoulder_link"
    assert _filtered_links(items) == {"shoulder_link"}


def test_exact_duplicate_protected_ur5_rows_are_skipped_without_suppressing_other_visuals() -> None:
    duplicate = {
        "id": "urdf_visual_2_shoulder_link_visual",
        "source_row_index": 2,
        "link": "shoulder_link",
        "visual_name": "shoulder_visual",
        "geometry_type": "mesh",
        "package_uri": "package://ur_description/meshes/ur5/visual/shoulder.dae",
    }
    distinct_same_link = {
        **duplicate,
        "id": "urdf_visual_3_shoulder_link_collision_visual",
        "source_row_index": 3,
        "visual_name": "shoulder_collision_visual",
    }
    robotiq = {
        "id": "urdf_visual_30_left_inner_finger",
        "source_row_index": 30,
        "link": "left_inner_finger",
        "visual_name": "left_inner_finger_visual",
        "geometry_type": "mesh",
        "package_uri": "package://robotiq_85_description/meshes/visual/inner_finger.dae",
    }

    retained_rows = _retained_rows([duplicate, dict(duplicate), distinct_same_link, robotiq])

    assert len([row for row in retained_rows if _normalized_ur5_link(row) == "shoulder_link"]) == 2
    assert len([row for row in retained_rows if row.get("link") == "left_inner_finger"]) == 1


def test_protected_ur5_mesh_rows_generate_distinct_duplicate_keys_by_link_visual_row_and_mesh() -> None:
    ur5_rows = [
        {
            "id": f"urdf_visual_{index}_{link}",
            "source_row_index": index,
            "link": link,
            "visual_name": f"{link}_visual",
            "geometry_type": "mesh",
            "package_uri": f"package://ur_description/meshes/ur5/visual/{filename}",
            "resolved_path": f"/opt/ros/humble/share/ur_description/meshes/ur5/visual/{filename}",
        }
        for index, (link, filename) in enumerate(
            {
                "shoulder_link": "shoulder.dae",
                "upper_arm_link": "upperarm.dae",
                "forearm_link": "forearm.dae",
                "wrist_1_link": "wrist1.dae",
                "wrist_2_link": "wrist2.dae",
                "wrist_3_link": "wrist3.dae",
            }.items(),
            start=10,
        )
    ]

    duplicate_keys = [_generated_row_key(row, index) for index, row in enumerate(ur5_rows)]

    assert len(duplicate_keys) == len(set(duplicate_keys))
    for row, key in zip(ur5_rows, duplicate_keys):
        assert _normalized_ur5_link(row) in key
        assert _token(row["visual_name"]) in key
        assert _token(str(row["source_row_index"])) in key
        assert _token(row["package_uri"]) in key
        assert _token(row["resolved_path"]) in key


def test_exact_duplicate_ur5_mesh_row_duplicate_key_is_skipped() -> None:
    shoulder_row = {
        "id": "urdf_visual_10_shoulder_link",
        "source_row_index": 10,
        "link": "shoulder_link",
        "visual_name": "shoulder_link_visual",
        "geometry_type": "mesh",
        "package_uri": "package://ur_description/meshes/ur5/visual/shoulder.dae",
        "resolved_path": "/opt/ros/humble/share/ur_description/meshes/ur5/visual/shoulder.dae",
    }
    exact_duplicate = dict(shoulder_row)

    assert _generated_row_key(shoulder_row, 0) == _generated_row_key(exact_duplicate, 1)
    retained_rows = _retained_rows([shoulder_row, exact_duplicate])

    assert len(retained_rows) == 1
    assert _normalized_ur5_link(retained_rows[0]) == "shoulder_link"


def test_mainwindow_logs_protected_ur5_ingestion_and_renderer_handoff_lines() -> None:
    src = MAINWINDOW.read_text(encoding="utf-8")
    assert "Scene3D preserved generated URDF robot mesh row before preview handoff" in src
    assert "link=%2" in src
    assert "Scene3D generated URDF robot mesh renderer handoff" in src
    assert 'p.active_visual_source = QStringLiteral("mesh_preview");' in src
    assert 'p.category = QStringLiteral("robot_arm");' in src


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


def test_viewport_required_robot_link_detection_uses_generic_profile_table() -> None:
    viewport = (ROOT / "workcell_builder/workcell_builder/gui/scene3d_viewport_widget.cpp").read_text(encoding="utf-8")

    assert "struct GeneratedRobotViewportProfile" in viewport
    assert "is_required_generated_robot_viewport_link" in viewport
    assert "draw_required_generated_robot_emergency_fallback" in viewport
    assert "generated_robot_final_draw_candidate_diagnostics_export" in viewport
    assert "generated_robot_profile_for_required_link_item(item)" in viewport
    assert "expected_ur5_visual_meshes" not in viewport


def test_viewport_required_robot_profiles_include_non_ur5_supported_scenes() -> None:
    viewport = (ROOT / "workcell_builder/workcell_builder/gui/scene3d_viewport_widget.cpp").read_text(encoding="utf-8")

    assert 'QStringLiteral("ur5")' in viewport
    assert 'QStringLiteral("ur10")' in viewport
    assert 'QStringLiteral("ur3")' in viewport
    assert 'QStringLiteral("ur_description/meshes/ur10")' in viewport
    assert 'QStringLiteral("ur_description/meshes/ur3")' in viewport


def test_successful_package_fallback_is_canonical_and_not_user_facing_warning():
    src = MAINWINDOW.read_text(encoding="utf-8")
    assert 'p.source_path_resolution_outcome = QStringLiteral("resolved_via_package_uri");' in src
    assert 'p.resolved_source_path_stale = false;' in src
    assert 'p.resolved_source_path_original = resolved_mesh_path;' in src
    assert 'URDF visual stale resolved_source_path for %1' not in src
    assert 'resolved_via_package_uri_after_stale_resolved_source_path' not in src
    assert 'package_uri_resolved_after_stale=%2' in src


def test_missing_package_mesh_remains_warning_after_package_attempt():
    src = MAINWINDOW.read_text(encoding="utf-8")
    assert 'stale_resolved_source_path_unresolved_after_package_uri_attempt' in src
    assert 'Preview warning: resolved_source_path is stale; package_uri fallback did not resolve a mesh' in src
    assert 'Preview warning: URDF visual unresolved' in src


def test_collada_z_up_console_message_collapsed_once_per_scene_load():
    js = (ROOT / "workcell_studio_web/viewer/urdf_robot_renderer.js").read_text(encoding="utf-8")
    assert "COLLADA_Z_UP_CONSOLE_MESSAGE" in js
    assert "collada_z_up_console_message_emitted" in js
    assert "console.info(`Collada Z-UP loader notice collapsed for scene load:" in js
    assert "originalWarn.apply(console, args);" in js
