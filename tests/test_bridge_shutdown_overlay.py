import importlib.util
from pathlib import Path
import subprocess

import pytest


SCRIPT=Path(__file__).parents[1]/'scripts/build_ros_gz_bridge_shutdown_overlay.py'
SPEC=importlib.util.spec_from_file_location('bridge_overlay_builder',SCRIPT)
MODULE=importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def source_fixture(tmp_path):
    source=tmp_path/'source';source.mkdir()
    def git(*args):return subprocess.check_output(['git','-C',str(source),*args])
    git('init','-q')
    (source/'main.cpp').write_text('spin();\nreturn 0;\n')
    (source/'header.hpp').write_text('reviewed header\n')
    git('add','.');git('-c','user.name=Test','-c','user.email=test@example.invalid','commit','-qm','fixture')
    (source/'main.cpp').write_text('spin();\nshutdown();\nreturn 0;\n')
    patch=tmp_path/'reviewed.patch';patch.write_bytes(git('diff','--binary'))
    git('restore','main.cpp')
    return source,patch,git


def test_bridge_source_guard_accepts_only_reviewed_patch_and_is_repeatable(tmp_path):
    source,patch,_=source_fixture(tmp_path)
    assert MODULE.apply_reviewed_patch(source,patch)==patch.read_bytes()
    assert MODULE.apply_reviewed_patch(source,patch)==patch.read_bytes()


@pytest.mark.parametrize('staged',[False,True])
def test_bridge_source_guard_rejects_extra_changes_even_in_index(tmp_path,staged):
    source,patch,git=source_fixture(tmp_path)
    (source/'header.hpp').write_text('unreviewed source change\n')
    if staged:git('add','header.hpp')
    with pytest.raises(RuntimeError,match='beyond the reviewed patch'):
        MODULE.apply_reviewed_patch(source,patch)
