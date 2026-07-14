import xml.etree.ElementTree as ET
from pathlib import Path

UI = Path('workcell_builder/workcell_builder/gui/scene_select.ui')
CPP = Path('workcell_builder/workcell_builder/gui/scene_select.cpp')

def root(): return ET.parse(UI).getroot()
def text(w):
    n=w.find("property[@name='text']/string")
    if n is None:
        n=w.find("attribute[@name='title']/string")
    return n.text if n is not None else ''
def widgets(cls=None):
    q = './/widget' if cls is None else f".//widget[@class='{cls}']"
    return root().findall(q)

def by_name(name):
    return root().find(f".//widget[@name='{name}']")

def test_home_is_first_visible_workflow_tab():
    tabs = root().find(".//widget[@class='QTabWidget'][@name='workflow_tabs']")
    first = tabs.find('widget')
    assert first.get('name') == 'start_tab'
    assert first.find("attribute[@name='title']/string").text == 'Home'

def test_primary_action_strip_contract():
    names = ['primary_edit_layout_button','primary_save_button','primary_validate_button','primary_generate_package_button','primary_refresh_preview_button']
    labels = [text(by_name(n)) for n in names]
    assert labels == ['Edit Layout','Save','Validate','Generate Package','Refresh Preview']
    assert len(labels) <= 5
    assert text(by_name('more_actions_button')) == 'More'

def test_home_keeps_create_open_and_summary_visible_contract():
    for name in ['cell_name','output_folder','browse_output_folder','add_scene','existing_scenes_group','scene_list','refresh_scenes_button','browse_scenes_folder','edit_scene','selected_cell_summary_group']:
        assert by_name(name) is not None
    assert text(by_name('add_scene')) in {'New Cell','Create Cell'}
    assert text(by_name('edit_scene')) == 'Open Cell'

def test_advanced_operations_are_configured_under_more_not_home_runtime():
    cpp = CPP.read_text()
    for label in ['Export & Open Web 3D Viewer','Validate Web Edit Patch','Run All-Scenes Readiness','Live EPD preview controls','Conveyor Sorting Run Console']:
        assert f'add("{label}"' in cpp or f'addAction("{label}"' in cpp
    for widget in ['run_all_scenes_readiness','enable_live_epd_button','disable_live_epd_button','open_conveyor_sorting_run_console_button','generate_full_scene_package_start']:
        assert f'ui->{widget},' in cpp or f'ui->{widget}->hide()' in cpp

def test_primary_state_centralized_and_disabled_without_cell():
    cpp = CPP.read_text()
    assert 'void SceneSelect::refresh_primary_workflow_state' in cpp
    assert 'Open or create a cell before using this primary action.' in cpp
    assert 'button->setEnabled(false)' in cpp
    assert 'ui->primary_save_button->setEnabled(unsaved)' in cpp
    assert 'ui->primary_validate_button->setEnabled(!unsaved)' in cpp

def test_unsaved_and_blocked_feedback_are_textual():
    cpp = CPP.read_text()
    assert 'Unsaved edits: %1' in cpp and 'YES — save before validate/generate' in cpp
    assert 'Status: BLOCKED' in cpp
    assert 'Outcome:' in UI.read_text()
    assert 'Fake hardware by default' in UI.read_text()
