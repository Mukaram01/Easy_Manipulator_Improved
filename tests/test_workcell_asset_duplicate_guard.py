from pathlib import Path
from scripts.audit_workcell_assets import audit

def test_duplicate_package_guard_field_present():
    payload=audit(Path('.').resolve())
    assert 'duplicated_package_names' in payload
    assert payload['rules']['no_duplicate_colcon_packages'] == (not bool(payload['duplicated_package_names']))
