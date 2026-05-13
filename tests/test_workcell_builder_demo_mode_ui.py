from pathlib import Path
UI = Path('workcell_builder/workcell_builder/gui/scene_select.ui').read_text()
CPP = Path('workcell_builder/workcell_builder/gui/scene_select.cpp').read_text()

def test_demo_mode_tab_and_buttons_exist():
    for token in [
        '<string>Demo Mode</string>',
        'demo_mode_table',
        'demo_create_scene_button',
        'demo_validate_button',
        'demo_generate_button',
        'demo_export_preview_button',
        'demo_copy_build_command_button',
        'demo_copy_launch_command_button',
        'demo_open_scene_folder_button',
        'demo_open_readiness_report_button',
        'demo_one_click_button',
    ]:
        assert token in UI

def test_demo_names_and_safety_text_exist():
    for token in [
        'UR5 + Robotiq 2F Pick & Place Demo',
        'UR5 + Suction Pick Demo',
        'Conveyor Sorting + EPD Metadata Preview',
        'Camera Inspection Preview',
        'fake_hardware_first=true | runtime_execution_enabled=false',
        'PREVIEW_ONLY',
    ]:
        assert token in CPP
