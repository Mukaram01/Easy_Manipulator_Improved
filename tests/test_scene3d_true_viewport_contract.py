from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
VIEWPORT_H = (ROOT / 'workcell_builder/workcell_builder/gui/scene3d_viewport_widget.h').read_text(encoding='utf-8')
VIEWPORT_CPP = (ROOT / 'workcell_builder/workcell_builder/gui/scene3d_viewport_widget.cpp').read_text(encoding='utf-8')
PREVIEW_CPP = (ROOT / 'workcell_builder/workcell_builder/gui/scene_preview_widget.cpp').read_text(encoding='utf-8')
MAIN_CPP = (ROOT / 'workcell_builder/workcell_builder/gui/mainwindow.cpp').read_text(encoding='utf-8')
CANVAS_MODEL_CPP = (ROOT / 'workcell_builder/workcell_builder/src_workcell_studio_canvas_model.cpp').read_text(encoding='utf-8')


def test_true_3d_viewport_class_contract_present():
    assert 'class Scene3DViewportWidget : public QOpenGLWidget' in VIEWPORT_H
    assert 'void initializeGL() override;' in VIEWPORT_H
    assert 'void paintGL() override;' in VIEWPORT_H


def test_perspective_depth_and_camera_controls_present():
    for token in [
        'out_proj.perspective(',
        'glEnable(GL_DEPTH_TEST)',
        'mouseMoveEvent(QMouseEvent * e)',
        'wheelEvent(QWheelEvent * e)',
        'set_isometric_view()',
        'set_top_view()',
        'set_front_view()',
        'set_side_view()',
        'reset_view()',
        'fit_scene()',
    ]:
        assert token in VIEWPORT_CPP or token in VIEWPORT_H


def test_grid_axes_selection_and_highlight_path_present():
    for token in [
        'draw_ground_grid_pass()',
        'draw_world_axes_pass()',
        'pick_item_at_screen(',
        'selected_id',
        'emit preview_item_selected',
    ]:
        assert token in VIEWPORT_CPP or token in VIEWPORT_H or token in PREVIEW_CPP


def test_layout_roles_mesh_fallback_and_layer_separation_contract_present():
    for token in [
        'pick_zone',
        'place_zone',
        'robot_base',
        'camera',
        'conveyor',
        'layout/workcell_studio_layout.yaml',
        'scene_visual_mesh_index.json',
        'locked',
        'editable',
        'draw_mesh_preview_if_available',
        'warn_mesh_fallback_once',
    ]:
        haystacks = (VIEWPORT_CPP, MAIN_CPP, CANVAS_MODEL_CPP)
        assert any(token in text for text in haystacks)


def test_malformed_metadata_warns_not_crash_contract_present():
    for token in [
        'Malformed layout/workcell_studio_layout.yaml; falling back safely',
        'failed to parse generated/scene_visual_mesh_index.json',
        'YAML parse exception in preview loader',
        'std exception in preview loader',
    ]:
        assert token in CANVAS_MODEL_CPP or token in MAIN_CPP


def test_no_real_hardware_tokens_in_canvas_and_generated_preview_path():
    banned = [
        'use_fake_hardware:=false',
        'fake_hardware:=false',
        'ur_robot_driver',
        'ethercat',
        'canopen',
    ]
    # Scope this check to 3D preview/canvas rendering implementation files.
    # mainwindow.cpp contains safety validators that intentionally reference forbidden launch tokens.
    scan = '\n'.join([VIEWPORT_CPP, VIEWPORT_H, PREVIEW_CPP, CANVAS_MODEL_CPP]).lower()
    for token in banned:
        assert token not in scan
