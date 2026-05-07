from pathlib import Path
import importlib.util
import sys
import tempfile

REPO_ROOT = Path(__file__).resolve().parents[1]

def _load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[name] = mod
    assert spec and spec.loader
    spec.loader.exec_module(mod)
    return mod

renderer = _load('render_workcell_builder_metadata', REPO_ROOT / 'scripts' / 'render_workcell_builder_metadata.py')
validator = _load('validate_builder_generated_scene', REPO_ROOT / 'scripts' / 'validate_builder_generated_scene.py')


def test_metadata_template_has_expected_fields():
    text = (REPO_ROOT / 'workcell_builder/workcell_builder/templates/workcell_builder_metadata_template.yaml').read_text()
    assert 'schema_version: workcell_builder_metadata/v1' in text
    assert 'runtime_io_applied: false' in text
    assert 'metadata_only: true' in text


def test_renderer_resolves_ur5_robotiq_and_suction_and_preview_variants():
    m = renderer.render_metadata('UR5', 'Robotiq 2F', 'RealSense D435i', 'finger_pinch_basic')
    assert m['robot']['capability_id'] == 'ur5'
    assert m['end_effector']['capability_id'] == 'robotiq_2f_85'

    s = renderer.render_metadata('ur5', 'OnRobot Airpick', 'realsense', 'suction_top_basic')
    assert s['end_effector']['capability_id'] == 'onrobot_airpick_style'
    assert s['end_effector']['runtime_io_applied'] is False
    assert s['grasp_strategy']['metadata_only'] is True

    p = renderer.render_metadata('generic delta', 'suction', 'realsense', 'suction_top_basic')
    assert p['robot']['preview_only'] is True

    g = renderer.render_metadata('generic gantry/cartesian', 'suction', 'realsense', 'suction_top_basic')
    assert g['robot']['capability_id'] == 'generic_gantry_xyz'


def test_validator_accepts_scene_with_metadata_and_warns_without_metadata():
    with tempfile.TemporaryDirectory() as d:
      root = Path(d)
      (root / 'package.xml').write_text('<package/>')
      (root / 'CMakeLists.txt').write_text('cmake_minimum_required(VERSION 3.5)')
      (root / 'environment.yaml').write_text('robot: {name: ur5}\nend_effector: {name: robotiq}\nobjects: {}\n')
      (root / 'workcell_builder_metadata.yaml').write_text('{}')
      ok = validator.validate_scene(root)
      assert ok['ok'] is True
      (root / 'workcell_builder_metadata.yaml').unlink()
      warn = validator.validate_scene(root)
      assert warn['ok'] is True
      assert any(c['check'] == 'workcell_builder_metadata.yaml present' and c.get('optional') is True for c in warn['checks'])
      assert any('workcell_builder_metadata.yaml missing (optional)' in w for w in warn['warnings'])


def test_scene_builder_readme_and_export_helper_are_documented():
    text = (REPO_ROOT / "workcell_builder/workcell_builder/gui/scene_select.cpp").read_text(encoding="utf-8")
    assert "export_workcell_studio_sources.sh" in text
    assert "export_builder_scene_to_cell_definition.py" in text
    assert "Export Workcell Studio source files" in text
