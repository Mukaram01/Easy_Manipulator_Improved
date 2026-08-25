from pathlib import Path
from scripts.workcell_builder_acceptance_check import build_golden_scene


def test_golden_headless_workflow_outputs_expected_files(tmp_path):
    scene = build_golden_scene('golden_ur5_2f_cell', tmp_path / 'scenes', tmp_path / 'assets')
    expected = [
        'environment.yaml','scene_manifest.yaml','layout/workcell_studio_layout.yaml','generated/cell_definition.yaml','generated/environment_layout.yaml',
        'generated/task_recipe.yaml','generated/grasp_strategy.yaml','package.xml','CMakeLists.txt',
        'launch/demo.launch.py','urdf/scene.urdf.xacro','urdf/arm_hand.srdf.xacro','generated/preview.txt','generated/readiness_summary.md'
    ]
    for rel in expected:
        assert (scene / rel).exists(), rel
