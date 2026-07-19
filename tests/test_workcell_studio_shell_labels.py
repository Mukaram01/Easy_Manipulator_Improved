from pathlib import Path


def test_workcell_studio_qss_exists():
    qss = Path('workcell_builder/workcell_builder/gui/resources/workcell_studio_dark.qss')
    assert qss.exists(), 'Expected Workcell Studio dark theme stylesheet to exist'


def test_workcell_studio_labels_present_in_source():
    source = Path('workcell_builder/workcell_builder/gui/mainwindow.cpp').read_text(encoding='utf-8')
    for token in [
        'Workcell Studio',
        'Scene Builder',
        'Existing Scenes',
        'Full Screen',
        'Validate',
        'Generate Scene',
        'Asset Catalog',
        'Readiness',
    ]:
        assert token in source, f'Missing UI token: {token}'



def test_scene_builder_header_controls_are_layout_owned_and_short():
    source = Path('workcell_builder/workcell_builder/gui/mainwindow.cpp').read_text(encoding='utf-8')
    header = source[
        source.index('void MainWindow::build_studio_header_actions()'):
        source.index('void MainWindow::refresh_scene_bundle_export_panel()')
    ]

    assert 'scenes_open_button->setText("Files");' in header
    assert 'top_bar->addWidget(scenes_open_button);' in header
    assert 'scenes_open_button->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Preferred);' in header
    assert 'scenes_open_button->move(' not in header
    assert 'scenes_open_button->setGeometry(' not in header

    assert 'product_view_help_label->setText("Product View");' in header or '"Product View",\n    this' in header
    assert 'rviz_truth_preview_help_label = new QLabel("RViz / MoveIt Preview", this);' in header
    assert 'product_view_help_label->setWordWrap(false);' in header
    assert 'rviz_truth_preview_help_label->setWordWrap(false);' in header
    assert 'Native Scene3D compatibility preview: lightweight editable layout preview; not guaranteed RViz-equivalent.' in header
    assert 'new QLabel(\n    "Native Scene3D compatibility preview: lightweight editable layout preview; not guaranteed RViz-equivalent.",' not in header
    assert 'new QLabel(\n    "RViz Truth Preview: authoritative generated scene preview using ROS/MoveIt/RViz stack.",' not in header

    for connection in [
        'connect(action_workspace_open_scene_builder_, &QAction::triggered',
        'connect(action_generate_yaml_, &QAction::triggered',
        'connect(action_generate_task_intent_, &QAction::triggered',
        'connect(action_generate_package_, &QAction::triggered',
        'connect(action_simulate_plan_preview_, &QAction::triggered',
    ]:
        assert connection in header
