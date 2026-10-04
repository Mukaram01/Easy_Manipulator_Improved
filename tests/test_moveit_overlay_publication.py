"""Publication must never admit a TEM-only or build-tree overlay."""
import importlib.util
import json
import hashlib
from pathlib import Path
import pytest

SCRIPT=Path(__file__).parents[1]/'scripts/moveit_teardown_repro/publication.py'

def publish():
    spec=importlib.util.spec_from_file_location('publication',SCRIPT)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    return module.publish

def test_incomplete_receipt_never_published(tmp_path):
    with pytest.raises(RuntimeError,match='incomplete'):
        publish()(tmp_path, {'library': {}})
    assert not (tmp_path/'provenance.json').exists()

def test_build_tree_executable_is_rejected(tmp_path):
    binary=tmp_path/'build/move_group';binary.parent.mkdir();binary.write_text('binary')
    lib=tmp_path/'install/moveit_ros_planning/lib/tem.so';lib.parent.mkdir(parents=True);lib.write_text('library')
    with pytest.raises(RuntimeError,match='installation'):
        publish()(tmp_path, {'library':{'path':str(lib)}, 'move_group':{'executable':{'path':str(binary)}}})
    assert not (tmp_path/'provenance.json').exists()

def add_active_proof(tmp_path,manifest):
    """Unit fixture only; never runtime qualification evidence."""
    artifacts={'binary':manifest['move_group']['executable'],'library':manifest['library'],
               'capabilities':manifest['move_group']['capabilities']}
    inputs=dict(artifacts)
    for name in ('request','scene','urdf','srdf','support_adapter','parameters','verifier'):
        path=tmp_path/name
        path.write_bytes(SCRIPT.with_name('verify_active_plan.py').read_bytes() if name=='verifier' else ('unit fixture '+name).encode())
        inputs[name]=dict(path=str(path),sha256=hashlib.sha256(path.read_bytes()).hexdigest())
    row=dict(passed=True,accepted=True,owned_sigint=True,pending_at_signal=True,
        shutdown_seconds=.2,interrupted_after_signal=False,
        signal_handler_observed=True,production_certificate_after_signal=True,plan_result_after_signal=True,returncode=0,
        remaining_processes=[],core_inventory_changed=False,mapped_artifacts=dict(artifacts,support_adapter=inputs['support_adapter']))
    proof=tmp_path/'active-results.json'
    proof.write_text(json.dumps(dict(method='real_saved_ur5_plan_owned_sigint/v2',passed=True,
        cycles=5,completed=5,results=[dict(row) for _ in range(5)],inputs=inputs,
        runtime_paths={'support_adapter':inputs['support_adapter']['path']})))
    manifest['move_group']['active_shutdown']=dict(results_path=str(proof),
        results_sha256=hashlib.sha256(proof.read_bytes()).hexdigest())


def complete_receipt(tmp_path):
    lib=tmp_path/'install/moveit_ros_planning/lib/tem.so';lib.parent.mkdir(parents=True);lib.write_text('library')
    binary=tmp_path/'install/moveit_ros_move_group/lib/moveit_ros_move_group/move_group';binary.parent.mkdir(parents=True);binary.write_text('binary')
    cap=binary.parents[1]/'libmoveit_move_group_default_capabilities.so.2.5.10';cap.write_text('capability')
    identity=lambda p:dict(path=str(p),sha256=hashlib.sha256(p.read_bytes()).hexdigest())
    manifest={'library':identity(lib),'move_group':{'executable':identity(binary),'capabilities':identity(cap)}}
    add_active_proof(tmp_path,manifest)
    return manifest


def test_complete_receipt_is_published_and_discoverable(tmp_path):
    manifest=complete_receipt(tmp_path)
    publish()(tmp_path,manifest)
    link=tmp_path/'install/moveit_ros_planning/share/moveit_ros_planning/workcell_teardown_overlay.json'
    assert json.loads(link.read_text())==manifest
    assert link.resolve()==tmp_path/'provenance.json'


def test_idle_only_receipt_cannot_be_published(tmp_path):
    manifest=complete_receipt(tmp_path)
    del manifest['move_group']['active_shutdown']
    with pytest.raises(RuntimeError,match='active planning shutdown'):
        publish()(tmp_path,manifest)
    assert not (tmp_path/'provenance.json').exists()


def test_real_tem_stage_writes_only_pending_receipt(tmp_path):
    """Run the real post-TEM publication statements with a completed TEM record.

    Changing that script back to publishing a TEM-only receipt must fail this
    test, even if the final-publication helper stays correct.
    """
    import ast
    import re
    source=(SCRIPT.parent.parent/'build_moveit_teardown_overlay.sh').read_text()
    first=re.findall(r"<<'PY'\n(.*?)\nPY(?:\n|$)",source,re.S)[0]
    tree=ast.parse(first)
    start=next(i for i,node in enumerate(tree.body) if isinstance(node,ast.Assign)
               and any(isinstance(t,ast.Name) and t.id=='manifest' for t in node.targets))
    tail=ast.Module(body=tree.body[start+1:],type_ignores=[])
    evidence=tmp_path/'evidence';evidence.mkdir()
    exec(compile(tail,'real-tem-stage','exec'),dict(overlay=tmp_path,evidence=evidence,
         manifest={'schema':'TEM_ONLY_SYNTHETIC'},json=json))
    assert not (tmp_path/'provenance.json').exists()
    assert json.loads((evidence/'pending-provenance.json').read_text())['schema']=='TEM_ONLY_SYNTHETIC'


def test_publication_binds_mapped_production_adapter_to_archived_input(tmp_path):
    manifest=complete_receipt(tmp_path)
    proof=Path(manifest['move_group']['active_shutdown']['results_path'])
    data=json.loads(proof.read_text())
    data['results'][0]['mapped_artifacts']['support_adapter']['sha256']='different'
    proof.write_text(json.dumps(data))
    manifest['move_group']['active_shutdown']['results_sha256']=hashlib.sha256(proof.read_bytes()).hexdigest()
    with pytest.raises(RuntimeError,match='active planning shutdown'):
        publish()(tmp_path,manifest)
    assert not (tmp_path/'provenance.json').exists()


def test_publication_rejects_unreviewed_active_probe(tmp_path):
    manifest=complete_receipt(tmp_path)
    proof=Path(manifest['move_group']['active_shutdown']['results_path'])
    data=json.loads(proof.read_text())
    verifier=Path(data['inputs']['verifier']['path']);verifier.write_text('alternate probe')
    data['inputs']['verifier']['sha256']=hashlib.sha256(verifier.read_bytes()).hexdigest()
    proof.write_text(json.dumps(data))
    manifest['move_group']['active_shutdown']['results_sha256']=hashlib.sha256(proof.read_bytes()).hexdigest()
    with pytest.raises(RuntimeError,match='active planning shutdown'):
        publish()(tmp_path,manifest)


def revise_active_proof(manifest, change):
    proof=Path(manifest['move_group']['active_shutdown']['results_path'])
    data=json.loads(proof.read_text());change(data)
    proof.write_text(json.dumps(data))
    manifest['move_group']['active_shutdown']['results_sha256']=hashlib.sha256(proof.read_bytes()).hexdigest()


def test_publication_rejects_shutdown_outside_launch_grace(tmp_path):
    manifest=complete_receipt(tmp_path)
    revise_active_proof(manifest,lambda data:data['results'][0].update(shutdown_seconds=5.001))
    with pytest.raises(RuntimeError,match='active planning shutdown'):
        publish()(tmp_path,manifest)


def test_publication_accepts_certification_interrupted_by_owned_shutdown(tmp_path):
    manifest=complete_receipt(tmp_path)
    def interrupt(data):
        for row in data['results']:
            row.update(production_certificate_after_signal=False,plan_result_after_signal=False,
                       interrupted_after_signal=True,shutdown_seconds=.2)
    revise_active_proof(manifest,interrupt)
    publish()(tmp_path,manifest)
