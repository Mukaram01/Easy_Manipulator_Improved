from scripts.workcell_builder_acceptance_check import build_golden_scene, launch_smoke


def test_generated_scene_launch_smoke_acceptance(tmp_path):
    scene = build_golden_scene('golden_ur5_2f_cell', tmp_path / 'scenes', tmp_path / 'assets')
    ok, errs = launch_smoke(scene)
    assert ok, errs
