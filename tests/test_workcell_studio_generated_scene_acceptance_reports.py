import json, subprocess, sys
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]
SCRIPT=ROOT/'scripts/validate_workcell_studio_generated_scene.py'

def test_report_content(tmp_path: Path):
    scene=tmp_path/'ur5_robotiq_demo'
    (scene/'config').mkdir(parents=True)
    (scene/'launch').mkdir(); (scene/'preview').mkdir(); (scene/'smoke').mkdir(); (scene/'urdf').mkdir()
    (scene/'package.xml').write_text('x'); (scene/'CMakeLists.txt').write_text('x')
    (scene/'environment.yaml').write_text('robot: {name: ur5}\nend_effector: {name: robotiq}\nmount_rpy: "-1.5708 -1.5708 0"\n')
    (scene/'scene_manifest.yaml').write_text('x')
    (scene/'config/task_recipe.yaml').write_text('safety:\n  fake_hardware_first: true\n  runtime_execution_enabled: false\n  motion_command_sent: false\n')
    (scene/'config/workcell_builder_task_intent.yaml').write_text('safety:\n  fake_hardware_first: true\n  runtime_execution_enabled: false\n  motion_command_sent: false\n')
    (scene/'launch/demo.launch.py').write_text('use_fake_hardware')
    (scene/'preview/static_preview.html').write_text('<html/>')
    (scene/'smoke/offline_smoke_summary.txt').write_text('ok')
    (scene/'workcell_studio_summary.json').write_text('{}')
    p=subprocess.run([sys.executable,str(SCRIPT),str(scene),'--json'],capture_output=True,text=True,check=False)
    out=json.loads(p.stdout)
    assert 'use_fake_hardware:=true' in '\n'.join(out['next_commands'])
    assert out['safety_flags']['fake_hardware_first'] is True
    assert (scene/'acceptance/generated_scene_acceptance.html').read_text().lower().find('no robot motion commanded')!=-1
