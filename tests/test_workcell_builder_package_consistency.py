from scripts.workcell_builder_acceptance_check import build_golden_scene, package_consistency


def test_generated_package_consistency(tmp_path):
    scene = build_golden_scene('golden_ur5_2f_cell', tmp_path / 'scenes', tmp_path / 'assets')
    ok, errs = package_consistency(scene)
    assert ok, errs
