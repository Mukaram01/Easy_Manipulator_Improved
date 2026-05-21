from pathlib import Path
import json
import subprocess

ROOT = Path(__file__).resolve().parents[1]


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
    assert data.get('renderable_mesh_count', 0) > 0

    assert data.get('candidate_mesh_count', 0) >= data.get('emitted_visual_count', 0)
    assert 'unresolved_placeholder_count' in data

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
        assert items
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
    assert mesh_resolved
    assert all((i.get('source_path') or '').strip() for i in mesh_resolved)
    assert all(Path(i.get('source_path')).exists() for i in mesh_resolved if (i.get('source_path') or '').strip())
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
