"""Publication must never admit a TEM-only or build-tree overlay."""
import importlib.util
import json
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

def test_complete_receipt_is_published_and_discoverable(tmp_path):
    lib=tmp_path/'install/moveit_ros_planning/lib/tem.so';lib.parent.mkdir(parents=True);lib.write_text('library')
    binary=tmp_path/'install/moveit_ros_move_group/lib/moveit_ros_move_group/move_group';binary.parent.mkdir(parents=True);binary.write_text('binary')
    manifest={'library':{'path':str(lib)},'move_group':{'executable':{'path':str(binary)}}}
    # Proof requirements are verified by the real build script before this
    # final publication operation; this helper enforces installation/location.
    publish()(tmp_path,manifest)
    link=tmp_path/'install/moveit_ros_planning/share/moveit_ros_planning/workcell_teardown_overlay.json'
    assert json.loads(link.read_text())==manifest
    assert link.resolve()==tmp_path/'provenance.json'


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
