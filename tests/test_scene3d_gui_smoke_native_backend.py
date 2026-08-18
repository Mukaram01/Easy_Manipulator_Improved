from __future__ import annotations

import json
import os

from scripts import run_workcell_builder_scene3d_gui_smoke as smoke


def test_scene3d_gui_smoke_forces_native_product_view_backend(monkeypatch):
    """The native Scene3D smoke must not inherit the normal Web3D default/override."""
    monkeypatch.setenv("WORKCELL_BUILDER_PRODUCT_VIEW_BACKEND", "embedded_web3d")

    smoke._configure_scene3d_smoke_environment()

    assert os.environ["WORKCELL_BUILDER_PRODUCT_VIEW_BACKEND"] == "native_scene3d"


def _healthy_failed_payload(blockers: list[str]) -> dict[str, object]:
    """Match the real wrapper schema emitted on the user's ROS Humble workstation."""
    return {
        "status": "FAIL",
        "wrapper_status": "FAIL",
        "runtime_available": True,
        "child_returncode": 0,
        "timed_out": False,
        "screenshot_available": True,
        "scene": "ur5_2f_test",
        "blockers": blockers,
        "warnings": ["no_item_selected_by_default"],
        "counters": {
            "scene3d_viewport_widget_found": True,
            "viewport_received_count": 30,
            "visible_count": 30,
            "rendered_count": 29,
            "render_cache_count": 14,
            "hierarchy_rows_count": 30,
            "last_paint_completed": True,
        },
    }


def test_legacy_topology_only_failure_is_downgraded_when_runtime_is_healthy(tmp_path):
    output = tmp_path / "smoke.json"
    output.write_text(
        json.dumps(
            _healthy_failed_payload(
                [
                    "scene3d_rendered_mesh_adjacency_failed",
                    "ur5_final_draw_bbox_regression_failed",
                ]
            )
        ),
        encoding="utf-8",
    )

    assert smoke._downgrade_legacy_topology_only_failure(output) is True

    payload = json.loads(output.read_text(encoding="utf-8"))
    assert payload["status"] == "PASS"
    assert payload["wrapper_status"] == "PASS"
    assert payload["blockers"] == []
    assert payload["legacy_topology_diagnostics_downgraded"] is True
    assert set(payload["legacy_topology_diagnostic_blockers"]) == {
        "scene3d_rendered_mesh_adjacency_failed",
        "ur5_final_draw_bbox_regression_failed",
    }
    assert "scene3d_rendered_mesh_adjacency_failed" in payload["warnings"]
    assert "ur5_final_draw_bbox_regression_failed" in payload["warnings"]


def test_legacy_topology_failure_stays_hard_when_an_actual_runtime_blocker_exists(tmp_path):
    output = tmp_path / "smoke.json"
    output.write_text(
        json.dumps(
            _healthy_failed_payload(
                [
                    "scene3d_rendered_mesh_adjacency_failed",
                    "scene_rendered_no_physical_items",
                ]
            )
        ),
        encoding="utf-8",
    )

    assert smoke._downgrade_legacy_topology_only_failure(output) is False

    payload = json.loads(output.read_text(encoding="utf-8"))
    assert payload["status"] == "FAIL"
    assert "scene_rendered_no_physical_items" in payload["blockers"]


def test_legacy_topology_failure_stays_hard_without_real_screenshot_evidence(tmp_path):
    output = tmp_path / "smoke.json"
    payload = _healthy_failed_payload(["scene3d_rendered_mesh_adjacency_failed"])
    payload["screenshot_available"] = False
    output.write_text(json.dumps(payload), encoding="utf-8")

    assert smoke._downgrade_legacy_topology_only_failure(output) is False
