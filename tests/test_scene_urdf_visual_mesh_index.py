from pathlib import Path
import json
import os
import subprocess

import yaml

ROOT = Path(__file__).resolve().parents[1]


def test_xacro_env_preserves_and_extends_package_lookup(monkeypatch, tmp_path):
    import scripts.extract_scene_urdf_visual_mesh_index as mesh_index

    workspace_root = tmp_path / 'workcell_ws'
    workspace_install = workspace_root / 'install'
    workspace_install.mkdir(parents=True)
    inherited_prefix = os.pathsep.join(['/existing/overlay', str(workspace_install), '/existing/overlay'])
    inherited_ros_package_path = os.pathsep.join([str(ROOT / 'assets'), '/extra/packages'])
    monkeypatch.setenv('AMENT_PREFIX_PATH', inherited_prefix)
    monkeypatch.setenv('ROS_PACKAGE_PATH', inherited_ros_package_path)

    original_exists = mesh_index.Path.exists

    def fake_exists(path):
        if str(path) == '/opt/ros/humble':
            return True
        return original_exists(path)

    monkeypatch.setattr(mesh_index.Path, 'exists', fake_exists)

    env = mesh_index.xacro_env(ROOT / 'scenes' / 'ur5_2f_test', workspace_root=workspace_root)

    ament_entries = env['AMENT_PREFIX_PATH'].split(os.pathsep)
    assert ament_entries == ['/existing/overlay', str(workspace_install), '/opt/ros/humble']

    ros_package_entries = env['ROS_PACKAGE_PATH'].split(os.pathsep)
    assert str(ROOT / 'assets') in ros_package_entries
    assert str(ROOT / 'workcell_builder' / 'workcell_builder' / 'assets') in ros_package_entries
    assert '/extra/packages' in ros_package_entries
    assert ros_package_entries.count(str(ROOT / 'assets')) == 1


def _write_package_xml(pkg_dir: Path, name: str):
    pkg_dir.mkdir(parents=True, exist_ok=True)
    (pkg_dir / 'package.xml').write_text(
        f'<package format="3"><name>{name}</name><version>0.0.0</version><description>test</description><maintainer email="test@example.com">Test</maintainer><license>Apache-2.0</license></package>',
        encoding='utf-8',
    )


def test_discover_package_map_prefers_ros_resolution_before_scan_fallback(monkeypatch, tmp_path):
    import scripts.extract_scene_urdf_visual_mesh_index as mesh_index

    scene_dir = tmp_path / 'scene'
    fallback_pkg = scene_dir / 'generated' / 'custom_description'
    resolved_pkg = tmp_path / 'install' / 'share' / 'custom_description'
    _write_package_xml(fallback_pkg, 'custom_description')
    _write_package_xml(resolved_pkg, 'custom_description')

    def fake_resolve(package_name, workspace_root=None):
        if package_name == 'custom_description':
            return resolved_pkg, 'ament_index', {'package_name': package_name, 'attempts': [{'source_tier': 'ament_index', 'status': 'resolved'}]}
        return None, '', {'package_name': package_name, 'attempts': []}

    monkeypatch.setattr(mesh_index, 'resolve_ros_package_share', fake_resolve)

    package_map, diagnostics = mesh_index.discover_package_map(scene_dir)

    assert package_map['custom_description'] == resolved_pkg
    resolved = [p for p in diagnostics['resolved_packages'] if p['package_name'] == 'custom_description']
    assert resolved and resolved[0]['source_tier'] == 'ament_index'
    shadowed = [p for p in diagnostics['shadowed_packages'] if p['package_name'] == 'custom_description']
    assert shadowed and shadowed[0]['source_tier'] == 'scene_generated_package'


def test_discover_package_map_preserves_repo_local_asset_precedence(monkeypatch, tmp_path):
    import scripts.extract_scene_urdf_visual_mesh_index as mesh_index

    installed_pkg = tmp_path / 'install' / 'share' / 'robotiq_85_description'
    _write_package_xml(installed_pkg, 'robotiq_85_description')

    def fake_resolve(package_name, workspace_root=None):
        if package_name == 'robotiq_85_description':
            return installed_pkg, 'ament_index', {'package_name': package_name, 'attempts': [{'source_tier': 'ament_index', 'status': 'resolved'}]}
        return None, '', {'package_name': package_name, 'attempts': []}

    monkeypatch.setattr(mesh_index, 'resolve_ros_package_share', fake_resolve)

    package_map, diagnostics = mesh_index.discover_package_map(ROOT / 'scenes' / 'ur5_2f_test')

    assert package_map['robotiq_85_description'] == ROOT / 'assets' / 'end_effectors' / 'robotiq_85_gripper' / 'robotiq_85_description'
    resolved = [p for p in diagnostics['resolved_packages'] if p['package_name'] == 'robotiq_85_description']
    assert resolved and resolved[0]['source_tier'] == 'repo_assets'


def test_discover_package_map_resolves_referenced_packages_without_scanned_package_xml(monkeypatch, tmp_path):
    import scripts.extract_scene_urdf_visual_mesh_index as mesh_index

    scene_dir = tmp_path / 'scene'
    scene_dir.mkdir()
    resolved_pkg = tmp_path / 'install' / 'share' / 'uri_only_description'
    _write_package_xml(resolved_pkg, 'uri_only_description')

    def fake_resolve(package_name, workspace_root=None):
        if package_name == 'uri_only_description':
            return resolved_pkg, 'ros2_pkg_prefix', {'package_name': package_name, 'attempts': [{'source_tier': 'ros2_pkg_prefix', 'status': 'resolved'}]}
        return None, '', {'package_name': package_name, 'attempts': []}

    monkeypatch.setattr(mesh_index, 'resolve_ros_package_share', fake_resolve)

    package_map, diagnostics = mesh_index.discover_package_map(
        scene_dir,
        package_names=mesh_index.extract_referenced_package_names('<mesh filename="package://uri_only_description/meshes/tool.stl"/>'),
    )

    assert package_map['uri_only_description'] == resolved_pkg
    resolved = [p for p in diagnostics['resolved_packages'] if p['package_name'] == 'uri_only_description']
    assert resolved and resolved[0]['source_tier'] == 'ros2_pkg_prefix'


def test_resolve_ros_package_share_uses_ros2_pkg_prefix_when_ament_unavailable(monkeypatch, tmp_path):
    import scripts.extract_scene_urdf_visual_mesh_index as mesh_index

    prefix = tmp_path / 'ros_overlay'
    share = prefix / 'share' / 'prefix_only_description'
    share.mkdir(parents=True)

    monkeypatch.setattr(mesh_index.importlib.util, 'find_spec', lambda name: None)
    monkeypatch.setattr(mesh_index.shutil, 'which', lambda name: '/usr/bin/ros2' if name == 'ros2' else None)

    class Result:
        returncode = 0
        stdout = str(prefix) + '\n'
        stderr = ''

    def fake_run(cmd, **kwargs):
        assert cmd == ['/usr/bin/ros2', 'pkg', 'prefix', 'prefix_only_description']
        return Result()

    monkeypatch.setattr(mesh_index.subprocess, 'run', fake_run)

    resolved, source_tier, diagnostics = mesh_index.resolve_ros_package_share('prefix_only_description')

    assert resolved == share
    assert source_tier == 'ros2_pkg_prefix'
    assert diagnostics['attempts'][-1]['source_tier'] == 'ros2_pkg_prefix'


def _xyz_from_item(item: dict):
    pose = item.get('world_pose') or item.get('computed_world_pose') or item.get('pose') or {}
    xyz = pose.get('xyz')
    if isinstance(xyz, list) and len(xyz) == 3:
        return xyz
    return [0, 0, 0]


def test_scene_urdf_visual_mesh_index_generation():
    script = ROOT / 'scripts' / 'extract_scene_urdf_visual_mesh_index.py'
    proc = subprocess.run(['python3', str(script), '--all', '--prefer-xacro'], capture_output=True, text=True)
    assert proc.returncode == 0, proc.stderr

    report = ROOT / 'build' / 'workcell_studio_urdf_visual_mesh_index_report.json'
    assert report.exists()
    data = json.loads(report.read_text())

    # 1) Report exists, has resolved meshes, and expected baseline totals.
    scene_dirs = [p for p in (ROOT / 'scenes').iterdir() if p.is_dir()]
    assert data['scene_count'] == len(scene_dirs)
    assert data['scene_count'] >= 8
    assert data['visual_count'] >= data['scene_count']
    assert data['resolved'] > 0
    assert 'mesh_format_counts' in data
    assert data.get('renderable_item_count', data.get('emitted_visual_count', 0)) > 0

    assert data.get('candidate_mesh_count', 0) >= data.get('emitted_visual_count', 0)
    assert 'unresolved_placeholder_count' in data

    catalog = yaml.safe_load((ROOT / 'scenes' / 'supported_scenes.yaml').read_text(encoding='utf-8'))
    blocked_scenes = {
        entry.get('scene_name')
        for entry in catalog.get('scenes', [])
        if entry.get('status') == 'blocked'
    }

    # Gather per-scene index data for data-driven validation.
    scene_payloads = []
    all_items = []
    unresolved_warnings = 0

    for scene in scene_dirs:
        idx = scene / 'generated' / 'scene_visual_mesh_index.json'
        assert idx.exists()
        payload = json.loads(idx.read_text())
        for key in ['generated_at','extractor_version','extraction_mode','xacro_available','source_urdf_xacro_path','source_mtime','unresolved_placeholder_count','has_transform_collapse_warning','candidate_mesh_count','emitted_visual_count','transform_status_counts','safe_for_preview']:
            assert key in payload
        items = payload.get('visual_items', [])
        if not items:
            assert scene.name in blocked_scenes
            continue
        all_items.extend(items)
        scene_payloads.append((scene.name, items))
        unresolved_warnings += sum(
            1 for i in items if (not i.get('resolved', False)) and i.get('warning')
        )

    # 2) At least one scene has transform_status == resolved.
    # Backward-compatible fallback: if transform status is not emitted yet, ensure resolved meshes exist.
    transform_status_values = [
        i.get('transform_status')
        for _, items in scene_payloads
        for i in items
        if 'transform_status' in i
    ]
    if transform_status_values:
        assert ('resolved' in transform_status_values) or ('partial' in transform_status_values)
    else:
        assert any(i.get('resolved') for i in all_items)

    # 3) At least one scene has non-zero and non-identical computed world pose.xyz.
    has_world_pose_fields = any(
        ('world_pose' in i) or ('computed_world_pose' in i)
        for _, items in scene_payloads
        for i in items
    )
    if has_world_pose_fields:
        has_nonzero_nonidentical_world_xyz = any(
            any(any(abs(v) > 1e-9 for v in xyz) for xyz in (_xyz_from_item(i) for i in items))
            and len({tuple(_xyz_from_item(i)) for i in items}) > 1
            for _, items in scene_payloads
        )
        assert has_nonzero_nonidentical_world_xyz

    # 4) No scene with many visuals should have all world xyz zero or all identical.
    many_visual_scene_threshold = 4
    if has_world_pose_fields:
        for scene_name, items in scene_payloads:
            if len(items) < many_visual_scene_threshold:
                continue
            xyzs = [_xyz_from_item(i) for i in items]
            all_zero = all(not any(abs(v) > 1e-9 for v in xyz) for xyz in xyzs)
            all_identical = len({tuple(xyz) for xyz in xyzs}) == 1
            assert not all_zero, f'{scene_name} has all world pose.xyz == [0,0,0]'
            assert not all_identical, f'{scene_name} has all identical world pose.xyz'

    # 5) Unresolved mesh paths remain warnings and do not crash generation.
    # Non-fatal unresolved meshes should be represented as unresolved+warning entries.
    assert unresolved_warnings >= data.get('unresolved', 0)
    assert data.get('unresolved', 0) >= 0
    mesh_counts = data.get('mesh_format_counts', {})
    dae_present = any((i.get('mesh_extension') or '').lower() == '.dae' for i in all_items)
    if dae_present:
        assert mesh_counts.get('.dae', 0) > 0

    bad_resolved = [i for i in all_items if i.get('transform_status') == 'resolved' and any(tok in str(i.get(k,'')) for k in ['id','link','parent_link'] for tok in ['${','$(arg '])]
    assert not bad_resolved
    mesh_resolved = [i for i in all_items if i.get('geometry_type') == 'mesh' and i.get('resolved')]
    if data.get('renderable_mesh_count', 0) > 0:
        assert mesh_resolved
    assert all((i.get('source_path') or i.get('resolved_source_path') or '').strip() for i in mesh_resolved)
    assert all(Path(i.get('resolved_source_path')).exists() for i in mesh_resolved if (i.get('resolved_source_path') or '').strip())
    primitive_items = [i for i in all_items if i.get('geometry_type') in ('box', 'cylinder', 'sphere')]
    for i in primitive_items:
        if i.get('geometry_type') == 'box':
            assert isinstance(i.get('size'), list) and len(i.get('size')) == 3
        if i.get('geometry_type') == 'cylinder':
            assert 'radius' in i and 'length' in i
        if i.get('geometry_type') == 'sphere':
            assert 'radius' in i
    skipped = [i for i in all_items if i.get('render_expected') and i.get('geometry_type') == 'mesh' and not (i.get('source_path') or '').strip()]
    assert len(skipped) <= max(2, len(all_items) // 4)
    for i in all_items:
        if not i.get('render_expected') or i.get('geometry_type') == 'unknown' or (i.get('geometry_type') == 'mesh' and not i.get('resolved')):
            assert (i.get('render_skip_reason') or i.get('warning') or '').strip()


def test_scene3d_visual_diagnostics_report_generation():
    proc = subprocess.run(['python3', str(ROOT / 'scripts' / 'validate_scene3d_visual_diagnostics.py')], capture_output=True, text=True)
    assert proc.returncode == 0, proc.stderr
    assert 'Traceback' not in (proc.stdout + proc.stderr)

    report = ROOT / 'build' / 'workcell_studio_scene3d_visual_diagnostics.json'
    assert report.exists()
    data = json.loads(report.read_text())
    assert 'scenes' in data and data['scenes']
    first = data['scenes'][0]
    for key in ['item_count','mesh_item_count','loaded_mesh_count','failed_mesh_count','world_bounds','largest_mesh','smallest_mesh','warnings']:
        assert key in first


def test_scene_option_and_local_smoke_missing_xacro_graceful():
    script = ROOT / 'scripts' / 'extract_scene_urdf_visual_mesh_index.py'
    proc = subprocess.run(['python3', str(script), '--scene', 'ur5_2f_test', '--prefer-xacro'], capture_output=True, text=True)
    assert proc.returncode == 0, proc.stderr
    assert 'Traceback' not in (proc.stdout + proc.stderr)

    smoke = ROOT / 'scripts' / 'run_scene3d_local_xacro_smoke.py'
    assert smoke.exists()
    proc2 = subprocess.run(['python3', str(smoke)], capture_output=True, text=True)
    assert proc2.returncode in (0, 2)
    assert 'Traceback' not in (proc2.stdout + proc2.stderr)

def test_ur5_2f_index_requires_non_empty_mesh_visuals():
    script = ROOT / 'scripts' / 'extract_scene_urdf_visual_mesh_index.py'
    proc = subprocess.run(['python3', str(script), '--scene', 'ur5_2f_test', '--prefer-xacro', '--require-xacro'], capture_output=True, text=True)
    assert proc.returncode in (0, 2), proc.stderr
    if proc.returncode == 2:
        return
    idx = ROOT / 'scenes' / 'ur5_2f_test' / 'generated' / 'scene_visual_mesh_index.json'
    assert idx.exists()
    data = json.loads(idx.read_text())
    assert data.get('safe_for_preview') is True
    assert not data.get('fallback_reason')
    items = data.get('visual_items', [])
    assert len(items) > 0
    mesh_items = [i for i in items if i.get('geometry_type') == 'mesh']
    assert mesh_items
    assert any((i.get('package_uri') or i.get('source_path')) for i in mesh_items)
    assert any(Path(i.get('resolved_source_path')).exists() for i in mesh_items if i.get('resolved_source_path'))


def test_require_xacro_strict_rejects_best_effort_modes():
    import sys
    import scripts.extract_scene_urdf_visual_mesh_index as mesh_index

    original_argv = sys.argv
    try:
        sys.argv = ['extract_scene_urdf_visual_mesh_index.py', '--all', '--prefer-xacro', '--require-xacro', '--no-write']
        rc = mesh_index.main()
    finally:
        sys.argv = original_argv

    if rc == 2:
        # Environment has no xacro binary; strict non-zero behavior is covered by the failure-path test.
        return

    assert rc == 0
    report = ROOT / 'build' / 'workcell_studio_urdf_visual_mesh_index_report.json'
    data = json.loads(report.read_text())
    assert data.get('best_effort_count', 0) == 0
    assert data.get('xacro_expanded_count', 0) == data.get('scene_count', 0)
    assert all(scene.get('extraction_mode') == 'xacro_expanded' for scene in data.get('scenes', []))


def test_require_xacro_strict_nonzero_on_simulated_xacro_failure(monkeypatch):
    import sys
    import scripts.extract_scene_urdf_visual_mesh_index as mesh_index

    monkeypatch.setattr(mesh_index.shutil, 'which', lambda _: '/usr/bin/xacro')

    def _fake_expand(_path):
        return None, 'best_effort', ['xacro expansion failed: simulated failure']

    monkeypatch.setattr(mesh_index, 'expand_xacro', _fake_expand)
    monkeypatch.setattr(
        mesh_index,
        'SCENES_ROOT',
        ROOT / 'tests' / 'fixtures' / 'scene_readiness' / 'duplicate_mesh_workcell' / 'scenes',
    )

    original_argv = sys.argv
    try:
        sys.argv = ['extract_scene_urdf_visual_mesh_index.py', '--all', '--prefer-xacro', '--require-xacro', '--no-write']
        rc = mesh_index.main()
    finally:
        sys.argv = original_argv
    assert rc != 0


def test_static_ur_robot_fallback_is_gated_by_expansion_mode_and_mesh_presence():
    import scripts.extract_scene_urdf_visual_mesh_index as mesh_index

    source_xacro = '<robot xmlns:xacro="http://ros.org/wiki/xacro"><xacro:ur_robot name="ur5"/></robot>'
    items = []
    assert mesh_index.append_static_robot_primitive_fallbacks(
        items,
        '<robot/>',
        'skipped unresolved macros: ur_robot',
        'xacro_expanded',
        source_xacro,
    ) == 0
    assert items == []

    items = [
        {
            'geometry_type': 'mesh',
            'package_uri': 'package://ur_description/meshes/ur5/visual/base.dae',
            'source_path': 'package://ur_description/meshes/ur5/visual/base.dae',
        }
    ]
    assert mesh_index.append_static_robot_primitive_fallbacks(
        items,
        '<robot/>',
        'skipped unresolved macros: ur_robot',
        'xacro_lite_expanded',
        source_xacro,
    ) == 0
    assert len(items) == 1

    items = []
    added = mesh_index.append_static_robot_primitive_fallbacks(
        items,
        '<robot/>',
        'skipped unresolved macros: ur_robot',
        'xacro_lite_expanded',
        source_xacro,
    )
    assert added > 0
    assert all(item.get('category') == 'robot_static_primitive_fallback' for item in items)


def test_main_marks_xacro_lite_static_ur5_fallback_preview_unsafe(monkeypatch, tmp_path):
    import sys
    import scripts.extract_scene_urdf_visual_mesh_index as mesh_index

    scenes_root = tmp_path / 'scenes'
    scene = scenes_root / 'lite_scene'
    (scene / 'urdf').mkdir(parents=True)
    (scene / 'urdf' / 'scene.urdf.xacro').write_text(
        '<robot xmlns:xacro="http://ros.org/wiki/xacro"><xacro:ur_robot name="ur5"/></robot>',
        encoding='utf-8',
    )
    monkeypatch.setattr(mesh_index, 'SCENES_ROOT', scenes_root)
    monkeypatch.setattr(mesh_index, 'discover_xacro_command', lambda: (None, False, 'xacro executable unavailable'))
    monkeypatch.setattr(
        mesh_index,
        'expand_xacro',
        lambda *args, **kwargs: ('<robot name="lite_scene"/>', True, 'skipped unresolved macros: ur_robot', ['xacro-lite', 'scene.urdf.xacro']),
    )

    original_argv = sys.argv
    try:
        sys.argv = ['extract_scene_urdf_visual_mesh_index.py', '--scene', 'lite_scene']
        rc = mesh_index.main()
    finally:
        sys.argv = original_argv

    assert rc == 0
    index = scene / 'generated' / 'scene_visual_mesh_index.json'
    payload = json.loads(index.read_text(encoding='utf-8'))
    assert payload['extraction_mode'] == 'xacro_lite_expanded'
    assert payload['xacro_real_command_succeeded'] is False
    assert payload['static_robot_primitive_fallback_count'] > 0
    assert payload['safe_for_preview'] is False
    expected = 'xacro-lite skipped robot macro ur_robot; preview uses degraded fallback geometry'
    assert expected in payload['blockers']
    assert expected in payload['warnings']

def test_synthetic_chain_transform_composition_and_primitives():
    import scripts.extract_scene_urdf_visual_mesh_index as mesh_index
    xml = '''<robot name="t"><link name="root"/><link name="link1"/><link name="link2"/>
    <joint name="j1" type="fixed"><parent link="root"/><child link="link1"/><origin xyz="1 0 0" rpy="0 0 0"/></joint>
    <joint name="j2" type="fixed"><parent link="link1"/><child link="link2"/><origin xyz="0 2 0" rpy="0 0 1.5708"/></joint>
    <link name="link2"><visual name="v"><origin xyz="0 0 0.3" rpy="0 0 0"/><geometry><box size="1 2 3"/></geometry></visual></link>
    </robot>'''
    items = mesh_index.extract_from_urdf(xml, {})
    assert len(items) == 1
    item = items[0]
    xyz = item['pose']['xyz']
    assert abs(xyz[0] - 1.0) < 1e-4
    assert abs(xyz[1] - 2.0) < 1e-4
    assert abs(xyz[2] - 0.3) < 1e-4
    assert item['geometry_type'] == 'box'
    assert item['size'] == [1.0, 2.0, 3.0]
    assert item['transform_status'] == 'resolved'
    assert len(item['transform_chain']) == 2
