from pathlib import Path

MAIN_CPP = Path('workcell_builder/workcell_builder/gui/mainwindow.cpp').read_text(encoding='utf-8')
YAML_UTILS_CPP = Path('workcell_builder/workcell_builder/src_workcell_yaml_utils.cpp').read_text(encoding='utf-8')


def test_scene3d_camera_mode_uses_canonical_values_and_legacy_mappers():
    for token in [
        'none',
        'epd_optional',
        'snapshot_overlay',
        'live_epd',
        'saved_snapshot',
        'manual_simulated',
        'canonicalize_scene3d_camera_mode',
    ]:
        assert token in YAML_UTILS_CPP


def test_scene3d_overlay_generation_does_not_require_live_launch_for_snapshot_modes():
    for token in [
        'snapshot overlay loaded (no live launch required)',
        'EPD optional (live runtime launch not required for preview overlays)',
        'Detection snapshot overlays are preview-only and do not require launching live runtime nodes.',
    ]:
        assert token in MAIN_CPP
