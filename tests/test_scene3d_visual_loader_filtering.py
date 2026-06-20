import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
MAINWINDOW = ROOT / "workcell_builder/workcell_builder/gui/mainwindow.cpp"
UR5_INDEX = ROOT / "scenes/ur5_2f_test/generated/scene_visual_mesh_index.json"


def _token(value: str) -> str:
    return "".join(ch.lower() if ch.isalnum() else "_" for ch in (value or "")).strip("_")


def _final_identity(item: dict) -> str:
    visual = item.get("visual") or item.get("visual_name") or "visual_missing"
    parts = [
        "urdf_visual_final",
        _token(item.get("link") or "link_missing"),
        _token(visual),
    ]
    if item.get("visual_index") not in (None, ""):
        parts.append("visual_index_" + _token(str(item["visual_index"])))
    if item.get("source"):
        parts.append("source_" + _token(item["source"]))
    if item.get("category"):
        parts.append("category_" + _token(item["category"]))
    return "__".join(parts)


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


def _filtered_links(items: list[dict]) -> set[str]:
    flattened_links = {
        _token(item.get("link") or item.get("object_id") or item.get("object") or item.get("id"))
        for item in items
        if _token(item.get("source", "")) == "urdf_flattened" and _token(item.get("geometry_type", "")) == "mesh"
    }
    retained_ids = set()
    retained_links = set()
    for item in sorted(items, key=lambda i: _token(i.get("source", "")) != "urdf_flattened"):
        identity = _final_identity(item)
        link_key = _token(item.get("link") or item.get("object_id") or item.get("object") or item.get("id"))
        if identity in retained_ids:
            continue
        if link_key in flattened_links and _is_lower_fidelity_fallback(item):
            continue
        retained_ids.add(identity)
        retained_links.add(_token(item.get("link", "")))
    return retained_links


def test_mainwindow_visual_loader_distinguishes_identity_fallback_and_shared_mesh_uri() -> None:
    src = MAINWINDOW.read_text(encoding="utf-8")
    assert "source_%1" in src and "category_%1" in src
    assert "uri_%1" not in src and "row_%1" not in src
    assert "visual_item_is_lower_fidelity_fallback" in src
    assert "suppressed_by_urdf_flattened_visual_mesh" in src
    assert "retention check passed for ur5_2f_test" in src


def test_ur5_2f_visual_loader_filter_retains_robot_and_robotiq_rows() -> None:
    items = json.loads(UR5_INDEX.read_text(encoding="utf-8"))["visual_items"]
    retained = _filtered_links(items)
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
