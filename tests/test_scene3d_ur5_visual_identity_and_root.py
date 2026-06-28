from __future__ import annotations

import json
import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
MESH_INDEX_PATH = ROOT / "scenes" / "ur5_2f_test" / "generated" / "scene_visual_mesh_index.json"
GUI_LOADER_PATH = ROOT / "workcell_builder" / "workcell_builder" / "gui" / "mainwindow.cpp"


def _canonical_scene3d_token(value: object) -> str:
    """Python mirror of mainwindow.cpp canonical_scene3d_token for loader-equivalent checks."""
    normalized = str(value or "").strip().lower().replace("-", "_").replace(" ", "_")
    if normalized in {
        "generated_preview",
        "generated_urdf_visual",
        "locked_generated_urdf",
        "locked_generated_urdf_visual",
    }:
        return "locked_generated_urdf_visual"
    if normalized == "legacy_static_fallback":
        return "primitive_fallback"
    if normalized in {"overlays", "helper_overlay"}:
        return "overlay"
    return normalized


def _visual_item_final_identity_key(node: dict, source_row_index: int) -> str:
    """Mirror the GUI loader's visual_item_final_identity_key lambda."""

    def value(key: str) -> str:
        raw = node.get(key, "")
        return str(raw).strip() if raw is not None else ""

    link = value("link")
    visual = value("visual") or value("visual_name")
    visual_index = value("visual_index")
    uri = value("package_uri") or value("mesh_uri") or value("source_path")
    row_index = value("source_row_index") or str(source_row_index)

    return "generated_urdf::{}::{}::{}".format(
        _canonical_scene3d_token(link) if link else "link_missing",
        _canonical_scene3d_token(visual) if visual else "visual_missing",
        _canonical_scene3d_token(row_index),
    )


def _load_visual_items() -> list[dict]:
    payload = json.loads(MESH_INDEX_PATH.read_text(encoding="utf-8"))
    items = payload["visual_items"]
    assert payload["scene_name"] == "ur5_2f_test"
    assert payload["extraction_mode"] == "urdf_flattened"
    assert len(items) >= 9
    return items


def _ordered_visual_items(items: list[dict]) -> list[dict]:
    # std::stable_sort comparator places source=urdf_flattened rows before lower-fidelity rows.
    return sorted(items, key=lambda item: _canonical_scene3d_token(item.get("source")) != "urdf_flattened")


def _retained_rows(items: list[dict]) -> list[dict]:
    """Loader-equivalent static subset for the generated urdf_flattened mesh index rows."""
    retained: list[dict] = []
    seen: set[str] = set()
    for source_row_index, item in enumerate(_ordered_visual_items(items)):
        identity = _visual_item_final_identity_key(item, source_row_index)
        if identity in seen:
            continue
        seen.add(identity)
        retained.append({**item, "_final_scene3d_identity": identity})
    return retained


def _robot_base_frame_score(frame: str) -> int:
    token = _canonical_scene3d_token(frame)
    if not token:
        return -100000
    if token == "world":
        return 10000
    if token == "base_link":
        return 9000
    if token == "base_link_inertia":
        return 8000
    non_robot_terms = (
        "gripper_base_link",
        "robotiq",
        "finger",
        "knuckle",
        "tool0",
        "tool_",
        "_tool",
        "tool_link",
        "end_effector",
        "endeffector",
        "ee_link",
        "eef",
        "camera",
        "realsense",
        "d435",
        "d455",
        "table",
        "workbench",
    )
    if any(term in token for term in non_robot_terms):
        return -9000
    if token == "root_link":
        return 7000
    if token.endswith("_root_link") or "root_link" in token:
        return 6500
    if token.endswith("_base_link") or "base_link" in token:
        return 6000
    if token == "base" or token.endswith("_base"):
        return 5500
    if (token.startswith("ur") or "_ur" in token) and ("root" in token or "base" in token):
        return 5200
    if "shoulder_link" in token:
        return 1000
    return 100


def _selected_robot_base_frame(items: list[dict]) -> str:
    best_frame = ""
    best_score = -100000

    def consider(frame: object, bonus: int = 0) -> None:
        nonlocal best_frame, best_score
        candidate = str(frame or "").strip()
        if not candidate:
            return
        score = _robot_base_frame_score(candidate) + bonus
        if score > best_score:
            best_frame = candidate
            best_score = score

    for item in _ordered_visual_items(items):
        consider(item.get("root_link"), 300)
        consider(item.get("parent_link") or item.get("base_frame"), 150)
        consider(item.get("link"), 0)
        for chain_index, entry in enumerate(item.get("link_chain") or []):
            consider(entry, 200 - chain_index)
    return best_frame


def test_final_scene3d_identities_are_unique_with_gui_loader_helper() -> None:
    source = GUI_LOADER_PATH.read_text(encoding="utf-8")
    assert "visual_item_final_identity_key" in source
    assert "generated_urdf::%1::%2::%3" in source
    assert "source_row_index" in source

    retained = _retained_rows(_load_visual_items())
    identities = [row["_final_scene3d_identity"] for row in retained]

    assert len(retained) == len(_load_visual_items())
    assert len(identities) == len(set(identities))
    assert all(identity.startswith("generated_urdf::") for identity in identities)


def test_selected_robot_base_frame_is_not_tool_camera_or_table() -> None:
    frame = _selected_robot_base_frame(_load_visual_items())
    token = _canonical_scene3d_token(frame)

    assert frame in {"base_link", "base_link_inertia", "world"}
    assert token != "gripper_base_link"
    assert not re.search(r"gripper|finger|knuckle|tool|camera|realsense|d435|d455|table|workbench", token)


def test_retained_rows_include_ur5_arm_visuals() -> None:
    links = {row["link"] for row in _retained_rows(_load_visual_items())}

    assert "base_link_inertia" in links or "base_link" in links
    assert {
        "shoulder_link",
        "upper_arm_link",
        "forearm_link",
        "wrist_1_link",
        "wrist_2_link",
        "wrist_3_link",
    }.issubset(links)


def test_retained_rows_include_current_physical_table_and_camera_rows() -> None:
    retained = _retained_rows(_load_visual_items())
    links = {row.get("link", "") for row in retained}

    assert "table_" in links
    assert "camera_link" in links


def test_regression_guard_rejects_dropping_required_physical_visual_rows() -> None:
    retained = _retained_rows(_load_visual_items())
    simulated_regression = [row for row in retained if row.get("link") not in {"table_", "camera_link"}]

    assert len(retained) >= 9
    assert len(simulated_regression) < len(retained)
    assert len(simulated_regression) != len(retained), "A loader that drops physical table/camera rows must fail this guard."
