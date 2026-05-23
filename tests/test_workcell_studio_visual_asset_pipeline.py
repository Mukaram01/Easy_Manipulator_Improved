from pathlib import Path
import json
import subprocess

ROOT = Path(__file__).resolve().parents[1]


def test_resolver_package_uri_fake_map(tmp_path):
    from scripts.workcell_visual_asset_resolver import resolve_mesh_uri
    pkg = tmp_path / 'my_pkg'; (pkg / 'meshes').mkdir(parents=True)
    mesh = pkg / 'meshes' / 'm.stl'; mesh.write_text('x')
    r = resolve_mesh_uri('package://my_pkg/meshes/m.stl', repo_root=ROOT, package_map={'my_pkg': pkg})
    assert r.exists and r.source_kind == 'package_uri'


def test_resolver_robotiq_package_uri_nested_assets_root(tmp_path):
    from scripts.workcell_visual_asset_resolver import resolve_mesh_uri
    repo_root = tmp_path / 'repo'
    pkg = repo_root / 'assets' / 'end_effectors' / 'robotiq_85' / 'robotiq_85_description'
    (pkg / 'meshes' / 'visual').mkdir(parents=True)
    mesh = pkg / 'meshes' / 'visual' / 'robotiq_85_base_link.dae'
    mesh.write_text('x')
    (pkg / 'package.xml').write_text('<package><name>robotiq_85_description</name></package>')
    resolved = resolve_mesh_uri(
        'package://robotiq_85_description/meshes/visual/robotiq_85_base_link.dae',
        repo_root=repo_root,
    )
    assert resolved.exists
    assert str(mesh) == str(resolved.resolved_path)


def test_resolver_relative_scene_and_repo_assets(tmp_path):
    from scripts.workcell_visual_asset_resolver import resolve_mesh_uri
    scene = tmp_path / 'scene'; (scene / 'urdf').mkdir(parents=True)
    m1 = scene / 'urdf' / 'a.obj'; m1.write_text('x')
    r1 = resolve_mesh_uri('a.obj', repo_root=ROOT, scene_dir=scene)
    assert r1.exists and r1.source_kind in ('relative_scene','asset_catalog','relative_repo')


def test_unresolved_returns_warning_not_exception():
    from scripts.workcell_visual_asset_resolver import resolve_mesh_uri
    r = resolve_mesh_uri('missing/path.stl', repo_root=ROOT)
    assert not r.exists and r.message


def test_inventory_schema_and_extensions_detected():
    proc = subprocess.run(['python3', str(ROOT / 'scripts/audit_workcell_studio_visual_assets.py')], capture_output=True, text=True)
    assert proc.returncode == 0, proc.stderr
    data = json.loads((ROOT / 'build/workcell_studio/visual_asset_inventory.json').read_text())
    assert data['schema'] == 'workcell_studio_visual_asset_inventory/v1'
    keys = set(k.lower() for k in data['mesh_files_by_extension'].keys())
    assert any(k in keys for k in ['.stl', '.dae', '.obj'])


def test_mesh_index_fields_and_fallback():
    subprocess.run(['python3', str(ROOT / 'scripts/extract_scene_urdf_visual_mesh_index.py'), '--all'], check=True)
    any_item = None
    for p in (ROOT / 'scenes').glob('*/generated/scene_visual_mesh_index.json'):
        payload = json.loads(p.read_text())
        if payload.get('visual_items'):
            any_item = payload['visual_items'][0]
            break
    assert any_item is not None
    for k in ['original_uri','resolved_source_path','exists','extension','render_expected']:
        assert k in any_item


def test_readiness_gate_include_visual_assets_flag():
    proc = subprocess.run(['python3', str(ROOT / 'scripts/run_workcell_studio_scene_readiness_gate.py'), '--dry-run-launches', '--include-visual-assets'], capture_output=True, text=True)
    assert proc.returncode in (0,1)
    payload = json.loads((ROOT / 'build/workcell_studio/workcell_studio_scene_readiness_gate.json').read_text())
    assert 'visual_asset_inventory' in payload


def test_canvas_logging_tokens_and_banned_tokens_absent():
    scan = (ROOT / 'workcell_builder/workcell_builder/src_workcell_studio_canvas_model.cpp').read_text(encoding='utf-8').lower()
    for tok in ['mesh-backed items loaded', 'primitive fallback items loaded', 'missing mesh items', 'unsupported mesh items']:
        assert tok in scan
    for banned in ['use_fake_hardware:=false','fake_hardware:=false','ur_robot_driver','ethercat','canopen']:
        assert banned not in scan
