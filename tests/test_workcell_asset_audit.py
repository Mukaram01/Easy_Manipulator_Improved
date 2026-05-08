from pathlib import Path
from scripts.audit_workcell_assets import audit

def test_asset_audit_runs_without_ros():
    payload=audit(Path('.').resolve())
    assert 'ros_packages' in payload
    assert payload['asset_roots']
