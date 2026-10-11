"""Pure source-overlay tests. Native loader acceptance is explicitly bounded."""
import importlib.util
from pathlib import Path
import pytest
path=Path(__file__).resolve().parents[1]/'scripts/stage_a_rgbd/physics_owner/prepare.py'
spec=importlib.util.spec_from_file_location('owner_prepare',path)
api=importlib.util.module_from_spec(spec);spec.loader.exec_module(api)


def test_owner_registration_protects_only_its_exported_hook():
    original='physics implementation unchanged\nIGNITION_ADD_PLUGIN(WorkcellOwnerPhysics, System)\n'
    fixed=api.protect_owner_registration(original)
    assert fixed.count('.protected IgnitionPluginHook')==1
    assert fixed.endswith('IGNITION_ADD_PLUGIN(WorkcellOwnerPhysics, System)\n')
    assert fixed.startswith('physics implementation unchanged\n')


@pytest.mark.parametrize('source',['no registration','IGNITION_ADD_PLUGIN(WorkcellOwnerPhysics, X)\n'*2])
def test_missing_or_duplicate_owner_registration_fails_closed(source):
    with pytest.raises(ValueError):api.protect_owner_registration(source)
