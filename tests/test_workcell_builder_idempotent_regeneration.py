from pathlib import Path


def test_copydir_merges_existing_directories_and_preserves_files():
    header = Path('workcell_builder/workcell_builder/include/file_functions.h').read_text(encoding='utf-8')
    assert 'Merging into existing directory %s' in header
    assert 'exists and is not a directory' in header
    assert 'fs::copy_file(current, dst, fs::copy_option::overwrite_if_exists);' in header


def test_generation_success_message_and_build_steps_are_present():
    cpp = Path('workcell_builder/workcell_builder/gui/scene_select.cpp').read_text(encoding='utf-8')
    assert 'Scene package generated/updated successfully.' in cpp
    assert 'Build before launching so ROS 2 can discover updated package files.' in cpp
    assert 'colcon build --symlink-install --packages-select ' in cpp
    assert 'source install/setup.bash' in cpp
    assert 'ros2 launch ' in cpp and 'demo.launch.py use_fake_hardware:=true' in cpp


def test_generate_launch_error_message_is_real_failure_not_existing_directory_false_alarm():
    cpp = Path('workcell_builder/workcell_builder/gui/scene_select.cpp').read_text(encoding='utf-8')
    assert 'Failed to generate/merge launch files. Check filesystem permissions and destination path: ' in cpp
    assert 'destination already exists or could not be created' not in cpp
