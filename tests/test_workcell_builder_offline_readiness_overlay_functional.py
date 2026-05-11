from pathlib import Path


def test_overlay_functional_helpers_exist_and_cover_all_required_checks():
    hdr = Path('workcell_builder/workcell_builder/include/offline_readiness_overlay.hpp').read_text(encoding='utf-8')
    src = Path('workcell_builder/workcell_builder/src_offline_readiness_overlay.cpp').read_text(encoding='utf-8')
    for marker in [
        'evaluate_offline_readiness_overlay',
        'evaluate_reach_warnings',
        'evaluate_workspace_bounds',
        'evaluate_simple_overlap_warnings',
        'evaluate_camera_placement_warnings',
        'evaluate_task_pick_place_reach',
        'SAFETY_ZONE_WARNING',
        'object overlaps robot base exclusion zone',
        'missing camera topic warnings',
        'unknown TCP/mount link warnings',
        'incompatible robot/tool blocker',
    ]:
        assert marker in hdr or marker in src


def test_visual_layout_editor_refresh_overlay_markers_present():
    txt = Path('workcell_builder/workcell_builder/gui/environment_layout_editor.cpp').read_text(encoding='utf-8')
    for marker in [
        'Readiness Overlay',
        'Refresh Readiness Overlay',
        'Show Reach Envelope',
        'Show Workspace Bounds',
        'Show Safety Zones',
        'Reach Warning',
        'Workspace Warning',
        'Overlap Warning',
        'Camera Warning',
        'Task Target Warning',
        'Safety Zone Warning',
        'update_model_from_item_move',
    ]:
        assert marker in txt


def test_summary_readiness_preview_include_overlay_metadata_and_offline_safety_notes():
    txt = Path('workcell_builder/workcell_builder/gui/scene_select.cpp').read_text(encoding='utf-8')
    for marker in [
        'readiness_overlay_status',
        'readiness_overlay_warning_count',
        'readiness_overlay_blocker_count',
        'reach_warnings',
        'workspace_warnings',
        'overlap_warnings',
        'camera_warnings',
        'task_target_warnings',
        'safety_zone_warnings',
        'Offline approximate readiness only',
        'no MoveIt planning',
        'no robot motion commanded',
        'No real hardware enabled',
    ]:
        assert marker in txt


def test_no_forbidden_runtime_and_no_epd_runtime_coupling_or_pyyaml_added():
    targets = [
        'workcell_builder/workcell_builder/src_offline_readiness_overlay.cpp',
        'workcell_builder/workcell_builder/gui/environment_layout_editor.cpp',
        'workcell_builder/workcell_builder/gui/scene_select.cpp',
    ]
    merged = '\n'.join(Path(t).read_text(encoding='utf-8').lower() for t in targets)
    for forbidden in ['getmotionplan', 'execute_trajectory', 'followjointtrajectory', '/plan_kinematic_path', 'import yaml', 'pyyaml']:
        assert forbidden not in merged
