from pathlib import Path


def test_asset_discovery_helper_paths_and_statuses_present():
    text = Path('workcell_builder/workcell_builder/src_asset_discovery_helper.cpp').read_text(encoding='utf-8')
    assert 'easy_manipulation_deployment/assets/robots' in text
    assert 'easy_manipulation_deployment/assets/end_effectors' in text
    assert 'assets/environment_objects' in text
    assert 'READY' in text
    assert 'MISSING_MOVEIT_CONFIG' in text
    assert 'MISSING_DESCRIPTION' in text
    assert 'easy_manipulation_deployment/assets/environment' in text
    assert 'easy_manipulation_deployment/assets/environment_objects' in text
    assert '/src/assets/environment' in text
    assert '/src/assets/environment_objects' in text


def test_empty_asset_catalog_diagnostic_includes_searched_paths():
    main = Path('workcell_builder/workcell_builder/gui/mainwindow.cpp').read_text(encoding='utf-8')
    assert 'No catalog assets found' in main
    assert 'discover_asset_catalog returned no placeable or previewable assets.' in main


def test_add_to_canvas_button_is_disabled_then_enabled_via_selection_signal():
    main = Path('workcell_builder/workcell_builder/gui/mainwindow.cpp').read_text(encoding='utf-8')
    assert 'add_to_canvas_button_->setEnabled(false);' in main
    assert 'connect(asset_catalog_tree_, &QTreeWidget::currentItemChanged, this, [this](QTreeWidgetItem *, QTreeWidgetItem *){ validate_asset_catalog_selection(); update_asset_library_preview(); });' in main
    assert 'validate_asset_catalog_selection();' in main
    assert 'add_to_canvas_button_->setEnabled(can_add);' in main


def test_end_effector_type_inference_keywords():
    text = Path('workcell_builder/workcell_builder/src_asset_discovery_helper.cpp').read_text(encoding='utf-8')
    assert 'robotiq' in text and 'finger' in text
    assert 'airpick' in text and 'suction' in text


def test_environment_stl_discovery_and_skip_folders():
    text = Path('workcell_builder/workcell_builder/src_asset_discovery_helper.cpp').read_text(encoding='utf-8')
    assert 'build' in text and 'install' in text and 'log' in text
    assert '.stl' in text


def test_ui_labels_and_guidance_and_no_unknown_spam():
    addrobot = Path('workcell_builder/workcell_builder/gui/addrobot.ui').read_text(encoding='utf-8')
    addee = Path('workcell_builder/workcell_builder/gui/addendeffector.ui').read_text(encoding='utf-8')
    addobj = Path('workcell_builder/workcell_builder/gui/addobject.ui').read_text(encoding='utf-8')
    scene_cpp = Path('workcell_builder/workcell_builder/gui/scene_select.cpp').read_text(encoding='utf-8')
    assert 'Select Robot Asset' in addrobot
    assert 'Select End Effector Asset' in addee
    assert 'Select Existing STL' in addobj
    assert 'Asset discovery paths' in scene_cpp
    assert 'use_fake_hardware:=true' in scene_cpp
    assert 'unknown_description' not in scene_cpp
    assert 'unknown_moveit_config' not in scene_cpp
