from scripts.run_workcell_builder_scene3d_gui_smoke import _apply_ur5_final_draw_bbox_regression


def _row(item_id, center):
    x, y, z = center
    half = 0.01
    return {
        "item_id": item_id,
        "final_draw_bbox_min": [x - half, y - half, z - half],
        "final_draw_bbox_max": [x + half, y + half, z + half],
    }


def test_ur5_final_draw_bbox_regression_fails_exploded_render():
    payload = {
        "scene": "ur5_2f_test",
        "status": "PASS",
        "blockers": [],
        "final_draw_visual_items": [
            _row("urdf_visual_0_base_link", [0.0, 0.0, 0.0]),
            _row("urdf_visual_1_shoulder_link", [0.0, 0.0, 0.1]),
            _row("urdf_visual_2_upper_arm_link", [3.0, 0.0, 0.1]),
        ],
    }

    _apply_ur5_final_draw_bbox_regression(payload)

    assert payload["status"] == "FAIL"
    assert payload["ur5_final_draw_bbox_status"] == "FAIL"
    assert "ur5_final_draw_bbox_regression_failed" in payload["blockers"]
    assert payload["ur5_final_draw_bbox_errors"]


def test_ur5_final_draw_bbox_regression_passes_compact_render():
    payload = {
        "scene": "ur5_2f_test",
        "status": "PASS",
        "blockers": [],
        "final_draw_visual_items": [
            _row("urdf_visual_0_base_link", [0.0, 0.0, 0.0]),
            _row("urdf_visual_1_shoulder_link", [0.0, 0.0, 0.1]),
            _row("urdf_visual_2_upper_arm_link", [0.2, 0.0, 0.2]),
            _row("urdf_visual_3_forearm_link", [0.6, 0.0, 0.2]),
            _row("urdf_visual_4_wrist_1_link", [0.9, 0.0, 0.2]),
            _row("urdf_visual_5_wrist_2_link", [0.95, 0.0, 0.2]),
            _row("urdf_visual_6_wrist_3_link", [1.0, 0.0, 0.2]),
        ],
    }

    _apply_ur5_final_draw_bbox_regression(payload)

    assert payload["status"] == "PASS"
    assert payload["ur5_final_draw_bbox_status"] == "PASS"
    assert payload["ur5_final_draw_bbox_errors"] == []
