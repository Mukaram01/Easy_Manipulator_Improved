from pathlib import Path

CPP = Path('workcell_builder/workcell_builder/gui/mainwindow.cpp').read_text()


def test_save_layout_readiness_files_are_explicit():
    for token in [
        'environment_layout.yaml',
        'layout/workcell_studio_layout.yaml',
        'environment.yaml',
        '"Save Layout"',
    ]:
        assert token in CPP


def test_rviz_readiness_blockers_are_distinct_and_placeholder_is_honest():
    for token in [
        'package.xml',
        'CMakeLists.txt',
        'launch/demo.launch.py',
        'urdf/scene.urdf.xacro',
        'Placeholder launch only — not RViz truth preview ready.',
        'Generate real scene URDF/RViz launch is not implemented for this scene yet',
    ]:
        assert token in CPP


def test_open_rviz_preview_keeps_package_xml_check_distinct_from_urdf_check():
    assert 'const bool package_xml_ready = has("package.xml")' in CPP
    assert 'const bool scene_xacro_ready = has("urdf/scene.urdf.xacro") || s.has_scene_urdf_xacro;' in CPP


def test_primary_actions_and_advanced_labels_present():
    for token in [
        '"Save Layout"',
        '"Generate YAML"',
        '"Generate Scene Package"',
        '"Validate"',
        '"Open RViz Truth Preview"',
        '"Copy Launch Command"',
    ]:
        assert token in CPP


def test_native_scene3d_and_2d_labels_demoted():
    assert 'Native Scene3D compatibility preview: lightweight editable layout preview; not guaranteed RViz-equivalent.' in CPP
    assert '2D Layout Draft' in CPP


def test_scene_package_generation_protects_scene_source_files_contract():
    for token in [
        'ensured environment.yaml, cell_definition.yaml, scene_manifest.yaml, environment_layout.yaml',
        'Generate ROS Scene Package: Generate YAML first.',
    ]:
        assert token in CPP
