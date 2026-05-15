from pathlib import Path


def test_task_binding_buttons_connected_and_handlers_exist():
    cpp = Path('workcell_builder/workcell_builder/gui/mainwindow.cpp').read_text()
    header = Path('workcell_builder/workcell_builder/gui/mainwindow.h').read_text()

    for label in [
        'Use Selected as Pick Zone',
        'Use Selected as Place Zone',
        'Use Selected as Camera',
    ]:
        assert label in cpp

    assert '&MainWindow::bind_selected_item_as_pick_zone' in cpp
    assert '&MainWindow::bind_selected_item_as_place_zone' in cpp
    assert '&MainWindow::bind_selected_item_as_camera' in cpp

    assert 'void bind_selected_item_as_pick_zone();' in header
    assert 'void bind_selected_item_as_place_zone();' in header
    assert 'void bind_selected_item_as_camera();' in header


def test_task_binding_writer_targets_yaml_and_keys_and_safety_text():
    cpp = Path('workcell_builder/workcell_builder/gui/mainwindow.cpp').read_text()

    assert 'config" / "workcell_builder_task_intent.yaml' in cpp or 'config"/"workcell_builder_task_intent.yaml' in cpp
    assert '{"pick", "source", "id"}' in cpp
    assert '{"place", "target", "id"}' in cpp
    assert '{"perception", "camera", "id"}' in cpp

    assert 'preview_only' in cpp
    assert 'use_fake_hardware' in cpp
    assert 'no_robot_motion' in cpp
    assert 'Fake Hardware | No Robot Motion | Preview Only' in cpp
    assert 'malformed YAML' in cpp
    assert '.malformed.' in cpp
