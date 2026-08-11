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


def test_discover_xacro_command_prefers_resolved_path_executable(monkeypatch, tmp_path):
    import scripts.extract_scene_urdf_visual_mesh_index as mesh_index

    fake_bin = tmp_path / 'bin'
    fake_bin.mkdir()
    fake_xacro = fake_bin / 'xacro'
    fake_xacro.write_text('#!/bin/sh\necho xacro\n', encoding='utf-8')
    fake_xacro.chmod(0o755)

    monkeypatch.setenv('PATH', str(fake_bin))
    monkeypatch.setattr(mesh_index.shutil, 'which', lambda name: str(fake_xacro) if name == 'xacro' else None)

    command, available, reason = mesh_index.discover_xacro_command()

    assert available is True
    assert command == [str(fake_xacro.resolve())]
    assert reason == ''


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


def test_discover_package_map_prefers_repo_ur_description_mesh_assets(monkeypatch, tmp_path):
    import scripts.extract_scene_urdf_visual_mesh_index as mesh_index

    repo_root = tmp_path / 'repo'
    ur_pkg = repo_root / 'assets' / 'robots' / 'universal_robot' / 'ur_description'
    visual_dir = ur_pkg / 'meshes' / 'ur5' / 'visual'
    visual_dir.mkdir(parents=True)
    _write_package_xml(ur_pkg, 'ur_description')
    for mesh_name in ['base', 'shoulder', 'upperarm', 'forearm', 'wrist1', 'wrist2', 'wrist3']:
        (visual_dir / f'{mesh_name}.dae').write_text('', encoding='utf-8')

    installed_pkg = tmp_path / 'install' / 'share' / 'ur_description'
    _write_package_xml(installed_pkg, 'ur_description')

    def fake_resolve(package_name, workspace_root=None):
        if package_name == 'ur_description':
            return installed_pkg, 'ament_index', {'package_name': package_name, 'attempts': [{'source_tier': 'ament_index', 'status': 'resolved'}]}
        return None, '', {'package_name': package_name, 'attempts': []}

    monkeypatch.setattr(mesh_index, 'ROOT', repo_root)
    monkeypatch.setattr(mesh_index, 'resolve_ros_package_share', fake_resolve)

    package_map, diagnostics = mesh_index.discover_package_map(
        tmp_path / 'scene',
        package_names=['ur_description'],
    )
    items = []
    added = mesh_index.append_static_ur5_mesh_visuals(items, package_map)
    ur5_diag = mesh_index._ur5_visual_mesh_diagnostics(package_map, diagnostics, added, 0)

    assert package_map['ur_description'] == ur_pkg
    assert added == 7
    assert ur5_diag['source'] == 'repo_assets/ur_description'
    assert ur5_diag['mesh_files_found'] == 7
    assert ur5_diag['static_fallback_items_generated'] == 0
    assert all(item['transform_status'] == 'ur5_fk_fallback_resolved' for item in items)
    assert all(item['package_uri'].startswith('package://ur_description/meshes/ur5/visual/') for item in items)


def _write_minimal_ur_description_visual_assets(tmp_path: Path) -> Path:
    ur_pkg = tmp_path / 'repo_assets' / 'ur_description'
    visual_dir = ur_pkg / 'meshes' / 'ur5' / 'visual'
    visual_dir.mkdir(parents=True)
    _write_package_xml(ur_pkg, 'ur_description')
    for mesh_name in ['base', 'shoulder', 'upperarm', 'forearm', 'wrist1', 'wrist2', 'wrist3']:
        (visual_dir / f'{mesh_name}.dae').write_text('<COLLADA/>\n', encoding='utf-8')
    return ur_pkg


def test_append_static_ur5_mesh_visuals_emits_fk_baked_mesh_rows_from_temp_ur_description_assets(tmp_path):
    import scripts.extract_scene_urdf_visual_mesh_index as mesh_index

    ur_pkg = _write_minimal_ur_description_visual_assets(tmp_path)
    package_map = {'ur_description': ur_pkg}
    items = []

    added = mesh_index.append_static_ur5_mesh_visuals(items, package_map)

    assert added == 7
    assert len(items) == 7
    assert all(item['baked_world_visual_transform_source'] == 'ur5_fk_fallback_link_world_times_visual_origin' for item in items)
    assert all(item.get('transform_status') != 'legacy_static_fallback_resolved_ur5_mesh_pose' for item in items)
    assert all(item.get('link_transform_status') != 'legacy_static_fallback_resolved_ur5_mesh_pose' for item in items)
    assert all(item['mesh_uri'].startswith(mesh_index.UR5_VISUAL_MESH_URI_PREFIX) for item in items)
    assert all(item['package_uri'].startswith(mesh_index.UR5_VISUAL_MESH_URI_PREFIX) for item in items)
    assert any(item.get('parent_link') != 'world' for item in items)
    assert {item.get('parent_link') for item in items} != {'world'}

    rows_by_link = {item['link']: item for item in items}
    for link in ['upper_arm_link', 'forearm_link', 'wrist_1_link', 'wrist_2_link', 'wrist_3_link']:
        chain = rows_by_link[link].get('link_chain')
        assert isinstance(chain, list), link
        assert chain[-1] == link
        assert 'base_link' in chain
        assert link in chain

    for item in items:
        assert isinstance(item.get('baked_world_visual_matrix'), list)
        assert len(item['baked_world_visual_matrix']) == 4
        assert isinstance(item.get('baked_world_visual_quaternion'), dict)
        assert set(item['baked_world_visual_quaternion']) == {'x', 'y', 'z', 'w'}
        assert item['geometry_type'] == 'mesh'
        assert item['mesh_available'] is True
        assert item['primitive_fallback'] is False


def test_append_static_ur5_mesh_visuals_all_zero_initial_joints_use_preview_home_pose(monkeypatch, tmp_path):
    import scripts.extract_scene_urdf_visual_mesh_index as mesh_index

    ur_pkg = _write_minimal_ur_description_visual_assets(tmp_path)
    zero_initial_positions = tmp_path / 'initial_positions.yaml'
    zero_initial_positions.write_text(
        "initial_positions:\n"
        "  shoulder_pan_joint: 0.0\n"
        "  shoulder_lift_joint: 0.0\n"
        "  elbow_joint: 0.0\n"
        "  wrist_1_joint: 0.0\n"
        "  wrist_2_joint: 0.0\n"
        "  wrist_3_joint: 0.0\n",
        encoding='utf-8',
    )

    original_read_ur5_initial_joint_positions = mesh_index.read_ur5_initial_joint_positions

    def read_zero_initial_joint_positions():
        return original_read_ur5_initial_joint_positions(zero_initial_positions)

    monkeypatch.setattr(mesh_index, 'read_ur5_initial_joint_positions', read_zero_initial_joint_positions)

    items = []
    added = mesh_index.append_static_ur5_mesh_visuals(items, {'ur_description': ur_pkg})

    assert added == 7
    joint_values = {item['joint_name']: item['joint_value'] for item in items if item.get('joint_name')}
    for joint_name, expected_value in mesh_index.UR5_PREVIEW_HOME_JOINT_POSE.items():
        assert joint_values[joint_name] == expected_value
    movable_items = [item for item in items if item.get('joint_type') != 'fixed']
    assert movable_items
    assert all(
        item['joint_value_source'] == 'workcell_preview_home_pose_all_zero_initial_positions'
        for item in movable_items
    )


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
            if i.get('category') == 'frame' and i.get('render_expected') is False:
                continue
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


def test_committed_ur5_2f_mesh_index_uses_fk_fallback_artifact():
    import scripts.extract_scene_urdf_visual_mesh_index as mesh_index

    idx = ROOT / 'scenes' / 'ur5_2f_test' / 'generated' / 'scene_visual_mesh_index.json'
    data = json.loads(idx.read_text(encoding='utf-8'))
    items = data.get('visual_items') or data.get('items') or []
    required_links = {
        'base_link_inertia',
        'shoulder_link',
        'upper_arm_link',
        'forearm_link',
        'wrist_1_link',
        'wrist_2_link',
        'wrist_3_link',
    }
    ur5_rows = [
        item for item in items
        if item.get('link') in required_links
        and 'ur_description/meshes/ur5/visual' in str(
            item.get('mesh_uri') or item.get('package_uri') or item.get('source_path')
        )
    ]

    assert data.get('extractor_version') == mesh_index.EXTRACTOR_VERSION
    assert len(ur5_rows) >= len(required_links)
    assert {row.get('link') for row in ur5_rows} >= required_links
    assert all(
        row.get('baked_world_visual_transform_source') != 'legacy_static_fallback_resolved_ur5_mesh_pose'
        for row in ur5_rows
    )
    assert all(
        row.get('baked_world_visual_transform_source') in {'ur5_fk_fallback_link_world_times_visual_origin', 'urdf_link_world_times_visual_origin', 'urdf_fk_link_world_times_visual_origin'}
        for row in ur5_rows
    )
    assert {row.get('parent_link') for row in ur5_rows} != {'world'}
    rows_by_link = {row.get('link'): row for row in ur5_rows}
    assert rows_by_link['upper_arm_link'].get('parent_link') == 'shoulder_link'
    assert rows_by_link['forearm_link'].get('parent_link') == 'upper_arm_link'
    assert rows_by_link['wrist_1_link'].get('parent_link') == 'forearm_link'
    assert rows_by_link['wrist_2_link'].get('parent_link') == 'wrist_1_link'
    assert rows_by_link['wrist_3_link'].get('parent_link') == 'wrist_2_link'
    assert 'static_mesh_resolved' not in (data.get('transform_status_counts') or {})
    assert any(row.get('transform_status') in {'ur5_fk_fallback_resolved', 'resolved'} for row in ur5_rows)


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
    assert all(scene.get('extraction_mode') in {'real_xacro_expanded', 'xacro_expanded'} for scene in data.get('scenes', []))


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
    assert payload['renderable_mesh_count'] == 0
    assert payload['visual_count'] == payload['static_robot_primitive_fallback_count']
    expected = 'xacro-lite skipped robot macro ur_robot; preview uses degraded fallback geometry'
    assert expected in payload['blockers']
    assert expected in payload['warnings']
    combined_diagnostics = '\n'.join(payload['blockers'] + payload['warnings'] + [payload['fallback_reason']])
    assert 'skipped' in combined_diagnostics
    assert 'ur_robot' in combined_diagnostics


def test_main_records_successful_real_xacro_expansion(monkeypatch, tmp_path):
    import sys
    import scripts.extract_scene_urdf_visual_mesh_index as mesh_index

    scenes_root = tmp_path / 'scenes'
    scene = scenes_root / 'real_xacro_scene'
    (scene / 'urdf').mkdir(parents=True)
    (scene / 'urdf' / 'scene.urdf.xacro').write_text(
        '<robot xmlns:xacro="http://ros.org/wiki/xacro" name="real_xacro_scene"/>',
        encoding='utf-8',
    )
    fake_xacro = tmp_path / 'bin' / 'xacro'
    fake_xacro.parent.mkdir(parents=True)
    fake_xacro.write_text('#!/bin/sh\nexit 0\n', encoding='utf-8')
    fake_xacro.chmod(0o755)

    monkeypatch.setattr(mesh_index, 'SCENES_ROOT', scenes_root)
    monkeypatch.setattr(mesh_index.shutil, 'which', lambda name: str(fake_xacro) if name == 'xacro' else None)

    class Result:
        returncode = 0
        stdout = 'real xacro ok\n'
        stderr = ''

    def fake_run(cmd, **kwargs):
        assert cmd[:2] == [str(fake_xacro.resolve()), str(scene / 'urdf' / 'scene.urdf.xacro')]
        assert '-o' in cmd
        out_path = Path(cmd[cmd.index('-o') + 1])
        out_path.parent.mkdir(parents=True, exist_ok=True)
        out_path.write_text(
            '<robot name="expanded"><link name="world"><visual name="table"><geometry><box size="1 1 0.1"/></geometry></visual></link></robot>',
            encoding='utf-8',
        )
        return Result()

    monkeypatch.setattr(mesh_index.subprocess, 'run', fake_run)

    original_argv = sys.argv
    try:
        sys.argv = ['extract_scene_urdf_visual_mesh_index.py', '--scene', 'real_xacro_scene', '--prefer-xacro']
        rc = mesh_index.main()
    finally:
        sys.argv = original_argv

    assert rc == 0
    payload = json.loads((scene / 'generated' / 'scene_visual_mesh_index.json').read_text(encoding='utf-8'))
    assert payload['extraction_mode'] == 'real_xacro_expanded'
    assert payload['xacro_available'] is True
    assert payload['xacro_real_command_succeeded'] is True
    assert payload['source_expanded_urdf_path'] == 'generated/expanded_scene_preview.urdf'
    assert payload['xacro_command'][0] == str(fake_xacro.resolve())
    assert payload['xacro_command'][0] != 'xacro-lite'
    assert payload['xacro_diagnostics']['xacro_status'] == 'real_xacro_succeeded'


def test_real_xacro_expansion_filters_mesh_placeholders_and_does_not_add_ur5_fk_fallback(monkeypatch, tmp_path):
    import sys
    import scripts.extract_scene_urdf_visual_mesh_index as mesh_index

    scenes_root = tmp_path / 'scenes'
    scene = scenes_root / 'real_ur5_scene'
    (scene / 'urdf').mkdir(parents=True)
    (scene / 'urdf' / 'scene.urdf.xacro').write_text('<robot name="real_ur5_scene"/>', encoding='utf-8')
    fake_xacro = tmp_path / 'bin' / 'xacro'
    fake_xacro.parent.mkdir(parents=True)
    fake_xacro.write_text('#!/bin/sh\nexit 0\n', encoding='utf-8')
    fake_xacro.chmod(0o755)
    ur_pkg = tmp_path / 'ur_description'
    mesh_dir = ur_pkg / 'meshes' / 'ur5' / 'visual'
    mesh_dir.mkdir(parents=True)
    for name in ('base.dae', 'shoulder.dae'):
        (mesh_dir / name).write_text('mesh', encoding='utf-8')

    monkeypatch.setattr(mesh_index, 'SCENES_ROOT', scenes_root)
    monkeypatch.setattr(mesh_index.shutil, 'which', lambda name: str(fake_xacro) if name == 'xacro' else None)
    monkeypatch.setattr(mesh_index, 'discover_package_map', lambda *a, **k: ({'ur_description': ur_pkg}, {'resolved_packages': []}))

    class Result:
        returncode = 0
        stdout = ''
        stderr = ''

    def fake_run(cmd, **kwargs):
        out_path = Path(cmd[cmd.index('-o') + 1])
        out_path.parent.mkdir(parents=True, exist_ok=True)
        out_path.write_text(
            '''<robot name="expanded">
              <link name="base_link_inertia">
                <visual name="base"><geometry><mesh filename="package://ur_description/meshes/ur5/visual/base.dae"/></geometry></visual>
                <visual name="placeholder"><geometry><mesh filename="${mesh}"/></geometry></visual>
              </link>
              <joint name="shoulder_pan_joint" type="revolute"><parent link="base_link_inertia"/><child link="shoulder_link"/><origin xyz="0 0 0.2"/><axis xyz="0 0 1"/></joint>
              <link name="shoulder_link"><visual name="shoulder"><geometry><mesh filename="package://ur_description/meshes/ur5/visual/shoulder.dae"/></geometry></visual></link>
            </robot>''',
            encoding='utf-8',
        )
        return Result()

    monkeypatch.setattr(mesh_index.subprocess, 'run', fake_run)

    original_argv = sys.argv
    try:
        sys.argv = ['extract_scene_urdf_visual_mesh_index.py', '--scene', 'real_ur5_scene', '--prefer-xacro']
        rc = mesh_index.main()
    finally:
        sys.argv = original_argv

    assert rc == 0
    payload = json.loads((scene / 'generated' / 'scene_visual_mesh_index.json').read_text(encoding='utf-8'))
    items = payload['visual_items']
    assert payload['extraction_mode'] == 'real_xacro_expanded'
    assert payload['xacro_real_command_succeeded'] is True
    assert not any('${mesh}' in json.dumps(item) for item in items)
    assert not any(str(item.get('source')) == 'ur5_fk_fallback' for item in items)
    assert payload['static_robot_mesh_visual_count'] == 0


def test_main_records_real_xacro_failure_command_and_output(monkeypatch, tmp_path):
    import sys
    import scripts.extract_scene_urdf_visual_mesh_index as mesh_index

    scenes_root = tmp_path / 'scenes'
    scene = scenes_root / 'failed_xacro_scene'
    (scene / 'urdf').mkdir(parents=True)
    (scene / 'urdf' / 'scene.urdf.xacro').write_text(
        '<robot xmlns:xacro="http://ros.org/wiki/xacro"><link name="world"/></robot>',
        encoding='utf-8',
    )
    fake_xacro = tmp_path / 'bin' / 'xacro'
    fake_xacro.parent.mkdir(parents=True)
    fake_xacro.write_text('#!/bin/sh\nexit 7\n', encoding='utf-8')
    fake_xacro.chmod(0o755)

    monkeypatch.setattr(mesh_index, 'SCENES_ROOT', scenes_root)
    monkeypatch.setattr(mesh_index.shutil, 'which', lambda name: str(fake_xacro) if name == 'xacro' else None)

    class Result:
        returncode = 7
        stdout = 'partial xacro stdout'
        stderr = 'synthetic xacro failure'

    def fake_run(cmd, **kwargs):
        assert cmd[0] == str(fake_xacro.resolve())
        return Result()

    monkeypatch.setattr(mesh_index.subprocess, 'run', fake_run)

    original_argv = sys.argv
    try:
        sys.argv = ['extract_scene_urdf_visual_mesh_index.py', '--scene', 'failed_xacro_scene', '--prefer-xacro']
        rc = mesh_index.main()
    finally:
        sys.argv = original_argv

    assert rc == 0
    payload = json.loads((scene / 'generated' / 'scene_visual_mesh_index.json').read_text(encoding='utf-8'))
    expected_command = [
        str(fake_xacro.resolve()),
        str(scene / 'urdf' / 'scene.urdf.xacro'),
        '-o',
        str(scene / 'generated' / 'expanded_scene_preview.urdf'),
        'use_fake_hardware:=true',
        'robot_prefix:=',
        'tool_prefix:=',
    ]
    assert payload['extraction_mode'] == 'best_effort_recursive'
    assert payload['xacro_available'] is True
    assert payload['xacro_real_command_succeeded'] is False
    assert payload['xacro_command'] == expected_command
    assert payload['xacro_diagnostics']['xacro_command'] == expected_command
    assert payload['xacro_diagnostics']['xacro_returncode'] == 7
    assert payload['xacro_diagnostics']['xacro_stdout'] == 'partial xacro stdout'
    assert payload['xacro_diagnostics']['xacro_stderr'] == 'synthetic xacro failure'
    assert 'partial xacro stdout' in payload['fallback_reason']
    assert 'synthetic xacro failure' in payload['fallback_reason']


def test_fallback_placeholder_rows_are_not_ur5_sanity_blockers():
    import scripts.extract_scene_urdf_visual_mesh_index as mesh_index

    placeholder = {
        'id': 'placeholder_${mesh}',
        'link': 'base_link',
        'geometry_type': 'mesh',
        'mesh_uri': '${mesh}',
        'source_path': '${mesh}',
        'render_expected': False,
        'resolved': False,
        'link_world_pose': {'xyz': [0, 0, 0], 'rpy': [0, 0, 0]},
        'pose': {'xyz': [0, 0, 0], 'rpy': [0, 0, 0]},
    }
    fk_rows = []
    for index, link in enumerate(('base_link', 'shoulder_link', 'upper_arm_link', 'forearm_link')):
        fk_rows.append({
            'id': f'fk_{link}',
            'link': link,
            'render_expected': True,
            'resolved': True,
            'geometry_type': 'mesh',
            'link_world_pose': {'xyz': [index * 0.1, 0, 0.2], 'rpy': [0, 0, 0]},
            'pose': {'xyz': [index * 0.1, 0, 0.2], 'rpy': [0, 0, 0]},
            'visual_origin': {'xyz': [0, 0, 0], 'rpy': [0, 0, 0]},
            'expected_visual_pose': {'xyz': [index * 0.1, 0, 0.2], 'rpy': [0, 0, 0]},
        })

    unresolved = [
        item for item in [placeholder, *fk_rows]
        if mesh_index._is_renderable_visual_item(item)
        and any(mesh_index.contains_placeholder(item.get(k, '')) for k in ('id', 'link', 'parent_link', 'mesh_uri', 'source_path'))
    ]
    warnings, blockers = mesh_index.validate_ur5_transform_sanity(
        [placeholder, *fk_rows],
        [row for row in [placeholder, *fk_rows] if mesh_index._is_renderable_visual_item(row)],
    )

    assert unresolved == []
    assert warnings == []
    assert not any('collapsed' in blocker.lower() or '[0, 0, 0]' in blocker for blocker in blockers)

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
    assert abs((item.get('baked_world_visual_pose') or item.get('world_from_visual') or item.get('pose') or {}).get('xyz', xyz)[2] - 0.3) < 1e-4
    assert item['geometry_type'] == 'box'
    assert item['size'] == [1.0, 2.0, 3.0]
    assert item['transform_status'] == 'resolved'
    assert len(item['transform_chain']) == 2


def test_extract_from_urdf_reports_root_and_missing_parent_diagnostics():
    import scripts.extract_scene_urdf_visual_mesh_index as mesh_index

    xml = '''<robot name="t">
    <link name="world"/>
    <link name="tool0"><visual name="tool_box"><geometry><box size="1 1 1"/></geometry></visual></link>
    <link name="finger"><visual name="finger_box"><geometry><box size="0.1 0.1 0.2"/></geometry></visual></link>
    <joint name="tool_mount" type="fixed"><parent link="missing_base"/><child link="tool0"/></joint>
    <joint name="finger_mount" type="fixed"><parent link="tool0"/><child link="finger"/></joint>
    </robot>'''

    items, diagnostics = mesh_index.extract_from_urdf(xml, {}, include_diagnostics=True)

    assert len(items) == 2
    assert diagnostics['root_links'] == ['world']
    assert diagnostics['visual_parent_link_counts'] == {'missing_base': 2}
    assert diagnostics['missing_parent_links'] == [{'link': 'missing_base', 'visual_count': 2}]
    assert diagnostics['transform_chain_diagnostics'][0]['missing_parent_link'] == 'missing_base'
    assert diagnostics['transform_chain_diagnostics'][0]['missing_parent_joint'] == 'tool_mount'


def test_supported_robot_root_diagnostics_warns_for_tool0_collapsed_visuals():
    import scripts.extract_scene_urdf_visual_mesh_index as mesh_index

    items = [{'parent_link': 'tool0'}, {'parent_link': 'tool0'}, {'parent_link': 'world'}]
    warnings, blockers = mesh_index.supported_robot_root_diagnostics(
        'ur5_2f_test',
        items,
        {'root_links': ['tool0']},
    )

    assert any('tool0 (2/3)' in blocker for blocker in blockers)
    assert any('expected world/base root' in warning for warning in warnings)


def test_regenerate_visual_mesh_indexes_forwards_workspace_root(monkeypatch, tmp_path):
    import sys
    import scripts.regenerate_scene_visual_mesh_indexes as regenerate

    repo_root = tmp_path / 'repo'
    scene = repo_root / 'scenes' / 'demo_scene'
    (scene / 'generated').mkdir(parents=True)
    (repo_root / 'scripts').mkdir(parents=True)
    (scene / 'generated' / 'scene_visual_mesh_index.json').write_text(
        json.dumps(
            {
                'extraction_mode': 'xacro_expanded',
                'xacro_available': True,
                'safe_for_preview': True,
                'visual_items': [
                    {
                        'geometry_type': 'mesh',
                        'pose': {'xyz': [0.1, 0.2, 0.3]},
                    }
                ],
            }
        ),
        encoding='utf-8',
    )
    workspace_root = tmp_path / 'workcell_ws'
    calls = []

    def fake_run(cmd, check=False):
        calls.append((cmd, check))

    monkeypatch.setattr(regenerate.subprocess, 'run', fake_run)
    original_argv = sys.argv
    try:
        sys.argv = [
            'regenerate_scene_visual_mesh_indexes.py',
            '--repo-root',
            str(repo_root),
            '--workspace-root',
            str(workspace_root),
            '--scene',
            'demo_scene',
            '--fail-on-unexpanded',
            '--xacro-arg',
            'use_fake_hardware:=true',
        ]
        rc = regenerate.main()
    finally:
        sys.argv = original_argv

    assert rc == 0
    assert len(calls) == 1
    cmd, check = calls[0]
    assert check is False
    assert cmd[:4] == [
        'python3',
        str(repo_root.resolve() / 'scripts' / 'extract_scene_urdf_visual_mesh_index.py'),
        '--scene',
        'demo_scene',
    ]
    assert cmd[cmd.index('--workspace-root') + 1] == str(workspace_root)
    assert cmd[cmd.index('--xacro-arg') + 1] == 'use_fake_hardware:=true'
    assert '--fail-on-unexpanded' in cmd


def test_validate_ur5_transform_sanity_reports_collapsed_nonfinite_and_double_origin():
    import scripts.extract_scene_urdf_visual_mesh_index as mesh_index

    rows = []
    for link in ['base_link', 'shoulder_link', 'upper_arm_link', 'forearm_link']:
        rows.append({
            'id': f'{link}_visual',
            'link': link,
            'pose': {'xyz': [0.1, 0.0, 0.0], 'rpy': [0.0, 0.0, 0.0]},
            'world_pose': {'xyz': [0.0, 0.0, 0.0], 'rpy': [0.0, 0.0, 0.0]},
            'link_world_pose': {'xyz': [0.0, 0.0, 0.0], 'rpy': [0.0, 0.0, 0.0]},
            'visual_origin': {'xyz': [0.1, 0.0, 0.0], 'rpy': [0.0, 0.0, 0.0]},
            'expected_visual_pose': {'xyz': [0.1, 0.0, 0.0], 'rpy': [0.0, 0.0, 0.0]},
        })
    rows[0]['pose']['rpy'] = [float('nan'), 0.0, 0.0]
    rows[1]['pose']['xyz'] = [0.2, 0.0, 0.0]

    warnings, blockers = mesh_index.validate_ur5_transform_sanity(rows, rows)

    assert any('non-finite pose.rpy' in blocker for blocker in blockers)
    assert any('identical or nearly identical' in blocker for blocker in blockers)
    assert any('within epsilon of [0, 0, 0]' in blocker for blocker in blockers)
    assert any('visual origin may be double-applied' in warning for warning in warnings)


def test_validate_ur5_transform_sanity_reports_adjacent_distance():
    import scripts.extract_scene_urdf_visual_mesh_index as mesh_index

    rows = [
        {'id': 'base', 'link': 'base_link', 'pose': {'xyz': [0, 0, 0], 'rpy': [0, 0, 0]}, 'link_world_pose': {'xyz': [0, 0, 0], 'rpy': [0, 0, 0]}, 'visual_origin': {'xyz': [0, 0, 0], 'rpy': [0, 0, 0]}, 'expected_visual_pose': {'xyz': [0, 0, 0], 'rpy': [0, 0, 0]}},
        {'id': 'shoulder', 'link': 'shoulder_link', 'pose': {'xyz': [2, 0, 0], 'rpy': [0, 0, 0]}, 'link_world_pose': {'xyz': [2, 0, 0], 'rpy': [0, 0, 0]}, 'visual_origin': {'xyz': [0, 0, 0], 'rpy': [0, 0, 0]}, 'expected_visual_pose': {'xyz': [2, 0, 0], 'rpy': [0, 0, 0]}},
        {'id': 'upper', 'link': 'upper_arm_link', 'pose': {'xyz': [2.2, 0, 0], 'rpy': [0, 0, 0]}, 'link_world_pose': {'xyz': [2.2, 0, 0], 'rpy': [0, 0, 0]}, 'visual_origin': {'xyz': [0, 0, 0], 'rpy': [0, 0, 0]}, 'expected_visual_pose': {'xyz': [2.2, 0, 0], 'rpy': [0, 0, 0]}},
        {'id': 'forearm', 'link': 'forearm_link', 'pose': {'xyz': [2.4, 0, 0], 'rpy': [0, 0, 0]}, 'link_world_pose': {'xyz': [2.4, 0, 0], 'rpy': [0, 0, 0]}, 'visual_origin': {'xyz': [0, 0, 0], 'rpy': [0, 0, 0]}, 'expected_visual_pose': {'xyz': [2.4, 0, 0], 'rpy': [0, 0, 0]}},
    ]

    _warnings, blockers = mesh_index.validate_ur5_transform_sanity(rows, rows)

    assert any('base_link->shoulder_link' in blocker and 'exceeding' in blocker for blocker in blockers)


def _robotiq_empty_xml():
    import scripts.extract_scene_urdf_visual_mesh_index as mesh_index
    links = ''.join(f'<link name="{name}" />' for name in mesh_index.ROBOTIQ_85_VISUAL_MESHES)
    joints = '<joint name="tool0_to_gripper" type="fixed"><parent link="tool0"/><child link="gripper_base_link"/><origin xyz="0 0 0" rpy="0 0 0"/></joint>'
    return f'<robot name="demo"><link name="tool0" />{links}{joints}</robot>'


def test_robotiq_preview_repair_is_strict_idempotent_and_preserves_joints(tmp_path):
    import scripts.extract_scene_urdf_visual_mesh_index as mesh_index
    pkg = tmp_path / 'robotiq_85_description'
    mesh_dir = pkg / 'meshes' / 'visual'
    mesh_dir.mkdir(parents=True)
    for mesh in set(mesh_index.ROBOTIQ_85_VISUAL_MESHES.values()):
        (mesh_dir / mesh).write_text('dae', encoding='utf-8')
    package_map = {'robotiq_85_description': pkg}
    source = _robotiq_empty_xml()
    repaired, applied = mesh_index.inject_missing_robotiq_85_visuals(
        source, package_map, contract_identifies_robotiq_85=True, real_xacro_succeeded=True, strict_assets=True
    )
    repaired_twice, applied_twice = mesh_index.inject_missing_robotiq_85_visuals(
        repaired, package_map, contract_identifies_robotiq_85=True, real_xacro_succeeded=True, strict_assets=True
    )
    diag = mesh_index.validate_expanded_preview_visual_contract(repaired_twice, package_map)
    assert applied is True
    assert applied_twice is False
    assert diag['robotiq_85_final_visual_count'] == 9
    assert 'tool0_to_gripper' in repaired_twice
    assert '<origin xyz="0 0 0" rpy="0 0 0"' in repaired_twice


def test_robotiq_preview_repair_requires_contract_real_xacro_and_assets(tmp_path):
    import pytest
    import scripts.extract_scene_urdf_visual_mesh_index as mesh_index
    xml = _robotiq_empty_xml()
    assert mesh_index.inject_missing_robotiq_85_visuals(xml, {}, contract_identifies_robotiq_85=False, real_xacro_succeeded=True)[1] is False
    assert mesh_index.inject_missing_robotiq_85_visuals(xml, {}, contract_identifies_robotiq_85=True, real_xacro_succeeded=False)[1] is False
    with pytest.raises(RuntimeError, match='missing mesh assets'):
        mesh_index.inject_missing_robotiq_85_visuals(xml, {}, contract_identifies_robotiq_85=True, real_xacro_succeeded=True, strict_assets=True)


def test_non_robotiq_tools_are_not_modified():
    import scripts.extract_scene_urdf_visual_mesh_index as mesh_index
    xml = '<robot name="demo"><link name="tool0"/><link name="suction_cup_link"/></robot>'
    repaired, applied = mesh_index.inject_missing_robotiq_85_visuals(
        xml, {}, contract_identifies_robotiq_85=False, real_xacro_succeeded=True
    )
    assert repaired == xml
    assert applied is False

def test_launch_xacro_request_resolves_canonical_layout_pose_mappings():
    import scripts.extract_scene_urdf_visual_mesh_index as mesh_index

    scene = ROOT / "scenes" / "ur5_2f_test"
    request = mesh_index._extract_scene_launch_xacro_request(scene, {})
    assert request is not None

    layout = yaml.safe_load((scene / "layout" / "workcell_studio_layout.yaml").read_text(encoding="utf-8"))
    items = {item["id"]: item for item in layout["items"]}

    def fmt(values):
        return " ".join(format(float(value), ".17g") for value in values)

    mappings = request["mappings"]
    assert mappings["table_world_xyz"] == fmt(items["support_surface_table"]["pose"]["xyz"])
    assert mappings["table_world_rpy"] == fmt(items["support_surface_table"]["pose"]["rpy"])
    assert mappings["camera_world_xyz"] == fmt(items["realsense_overhead"]["pose"]["xyz"])
    assert mappings["camera_world_rpy"] == fmt(items["realsense_overhead"]["pose"]["rpy"])
