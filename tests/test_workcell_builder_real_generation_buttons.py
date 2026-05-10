from pathlib import Path


def test_validate_generate_buttons_are_clear_and_wired():
    ui = Path('workcell_builder/workcell_builder/gui/scene_select.ui').read_text(encoding='utf-8')
    assert 'Save / Generate environment.yaml' in ui
    assert 'Generate Full Scene Package' in ui
    assert 'Save / Generate environment.yaml' in ui

    cpp = Path('workcell_builder/workcell_builder/gui/scene_select.cpp').read_text(encoding='utf-8')
    assert 'ui->generate_files->hide();' in cpp
    assert 'on_generate_canonical_files_clicked()' in cpp and 'on_generate_yaml_clicked();' in cpp
    assert 'on_generate_workcell_package_clicked()' in cpp and 'on_generate_files_clicked();' in cpp


def test_generate_workcell_package_guidance_mentions_build_source_launch():
    cpp = Path('workcell_builder/workcell_builder/gui/scene_select.cpp').read_text(encoding='utf-8')
    assert 'Scene package generated/updated successfully.' in cpp
    assert 'colcon build --symlink-install --packages-select ' in cpp
    assert 'source install/setup.bash' in cpp
    assert 'ros2 launch ' in cpp and 'use_fake_hardware:=true' in cpp
