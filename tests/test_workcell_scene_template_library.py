from pathlib import Path
import json, subprocess

def test_template_library_end_to_end(tmp_path: Path):
    repo=Path('.')
    catalog=repo/'workcell_builder/workcell_builder/config/scene_templates/scene_templates.json'
    assert catalog.exists()
    data=json.loads(catalog.read_text())
    ids={t['template_id'] for t in data['templates']}
    for i in ['ur5_pick_place_cell','ur5_sorting_cell','camera_inspection_cell','conveyor_pick_placeholder_cell','palletizing_placeholder_cell']:
        assert i in ids
    assert (repo/'workcell_builder/workcell_builder/include/scene_template_library.hpp').exists()
    assert (repo/'workcell_builder/workcell_builder/src_scene_template_library.cpp').exists()
    cmake=(repo/'workcell_builder/workcell_builder/CMakeLists.txt').read_text()
    assert 'src_scene_template_library.cpp' in cmake
    assert (repo/'scripts/generate_workcell_scene_from_template.py').exists()
    ui=(repo/'workcell_builder/workcell_builder/gui/scene_select.cpp').read_text()
    for s in ['Create Scene From Template','Scene Template Library','Template Validation Status']:
        assert s in ui
    out=tmp_path/'out'
    subprocess.run(['python3','scripts/generate_workcell_scene_from_template.py','--template','ur5_pick_place_cell','--scene-name','template_pick_place_test','--output-dir',str(out),'--validate','--print-summary'],check=True)
    env=(out/'template_pick_place_test'/'environment.yaml').read_text()
    for s in ['schema_version: workcell_scene/v1','robot:\n  name: ur5','tool:\n  name: robotiq_2f85','camera:','task:','workspace:','safety:','fake_hardware_first: true','real_hardware_enabled: false']:
        assert s in env
    assert 'pick_box' in env and 'bin_small' in env
    subprocess.run(['python3','scripts/validate_workcell_scene.py','--scene-dir',str(out/'template_pick_place_test')],check=False)


def test_no_bom_or_forbidden_additions():
    text='\n'.join(Path(p).read_text(encoding='utf-8',errors='ignore').lower() for p in ['scripts/generate_workcell_scene_from_template.py','workcell_builder/workcell_builder/src_scene_template_library.cpp'])
    for forbidden in ['bill of materials','bom','streamlit','import yaml','pyyaml','getmotionplan','execute_trajectory','real_hardware_enabled: true']:
        assert forbidden not in text
