from pathlib import Path
import json, subprocess

TEMPLATES=[
    "ur5_pick_place_cell",
    "ur5_sorting_cell",
    "camera_inspection_cell",
    "conveyor_pick_placeholder_cell",
    "palletizing_placeholder_cell",
]

def test_template_generation_acceptance(tmp_path: Path):
    out=tmp_path/'generated'
    for template in TEMPLATES:
        scene_name=f"{template}_test"
        subprocess.run([
            'python3','scripts/generate_workcell_scene_from_template.py',
            '--template',template,'--scene-name',scene_name,'--output-dir',str(out),'--validate','--print-summary'
        ],check=True)
        scene=out/scene_name
        env=scene/'environment.yaml'
        assert env.exists()
        txt=env.read_text(encoding='utf-8')
        assert 'schema_version: workcell_scene/v1' in txt
        for key in ['scene:','robot:','tool:','compatibility:','placed_objects:','camera:','task:','workspace:','safety:','metadata:']:
            assert key in txt
        for marker in ['fake_hardware_first: true','real_hardware_enabled: false','runtime_execution_enabled: false','motion_command_sent: false','moveit_plan_service_called: false']:
            assert marker in txt
        assert (scene/'config/task_recipe.yaml').exists()
        assert (scene/'workcell_template_summary.json').exists() and (scene/'workcell_template_summary.md').exists()
        assert (scene/'workcell_studio_summary.json').exists() and (scene/'workcell_studio_summary.md').exists()
        assert (scene/'preview/workcell_preview.svg').exists() and (scene/'preview/workcell_preview.html').exists()

        summary=json.loads((scene/'workcell_studio_summary.json').read_text(encoding='utf-8'))
        assert 'validation_command' in summary
        assert 'fake_hardware_smoke' in summary

        subprocess.run(['python3','scripts/validate_workcell_scene.py','--scene-dir',str(scene)],check=True)
        subprocess.run(['python3','scripts/run_workcell_fake_hardware_smoke.py','--scene-dir',str(scene),'--skip-launch','--print-summary'],check=True)


def test_template_scope_forbidden_additions():
    files=[
        'scripts/generate_workcell_scene_from_template.py',
        'workcell_builder/workcell_builder/config/scene_templates/scene_templates.json',
        'docs/manuals/WORKCELL_SCENE_TEMPLATE_LIBRARY.md',
    ]
    text='\n'.join(Path(f).read_text(encoding='utf-8',errors='ignore').lower() for f in files)
    for forbidden in ['bill of materials','streamlit','import yaml','pyyaml','getmotionplan','execute_trajectory','real_hardware_enabled: true']:
        assert forbidden not in text
