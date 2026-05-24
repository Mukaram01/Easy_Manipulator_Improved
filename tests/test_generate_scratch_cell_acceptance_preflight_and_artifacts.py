import importlib.util
import json
import sys
from pathlib import Path
import yaml

ROOT = Path(__file__).resolve().parents[1]
SCRIPTS = ROOT / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

spec = importlib.util.spec_from_file_location("scratch_acceptance", SCRIPTS / "generate_scratch_cell_acceptance.py")
target = importlib.util.module_from_spec(spec)
assert spec and spec.loader
spec.loader.exec_module(target)


def test_generated_scratch_schema_preflight_zero_errors_and_generation_attempt(tmp_path, monkeypatch):
    report = tmp_path / "acceptance.json"
    monkeypatch.setattr(target.sys, "argv", ["prog", "--scene-name", "scratchacceptancetest", "--output-root", str(tmp_path / "out"), "--json-out", str(report)])

    calls = []

    def fake_run(cmd):
        calls.append(" ".join(cmd))
        cmd_s = " ".join(cmd)
        if "validate_cell_definition.py" in cmd_s:
            return 0, json.dumps({"errors": [], "warnings": []}), ""
        if "generate_workcell_from_cell_definition.py" in cmd_s:
            scene_name = cmd[cmd.index("--package-name") + 1]
            scene_dir = (tmp_path / "out" / scene_name)
            for rel in ("scene_manifest.yaml", "config/scene3d_mesh_index.json", "urdf/scene.urdf.xacro", "launch/demo.launch.py", "package.xml", "CMakeLists.txt"):
                p = scene_dir / rel
                p.parent.mkdir(parents=True, exist_ok=True)
                p.write_text("x", encoding="utf-8")
            return 0, "ok", ""
        if "audit_new_cell_file_outputs.py" in cmd_s:
            return 0, "ok", ""
        raise AssertionError(cmd)

    monkeypatch.setattr(target, "_run", fake_run)
    rc = target.main()
    payload = json.loads(report.read_text(encoding="utf-8"))

    assert rc == 0
    assert payload["schema_preflight"]["schema_blockers"] == []
    assert payload["schema_preflight"]["validator_returncode"] == 0
    assert "validate_cell_definition.py" in payload["schema_preflight"]["validator_command"]
    assert payload["schema_preflight"]["cell_definition_path"].endswith("cell_definition.yaml")
    assert "objects" in payload["schema_preflight"]["cell_definition_top_level_keys"]
    assert payload["schema_preflight"]["object_count"] > 0
    assert payload["schema_preflight"]["next_failed_step"] == "generation"
    assert any("generate_workcell_from_cell_definition.py" in c for c in calls)
    assert "generate_workcell_from_cell_definition.py" in payload["generator"]["command"]


def test_generated_template_objects_pose_and_task_binding_integrity():
    doc = target.CELL_TMPL.format(scene="demo")
    assert "objects:" in doc
    assert "role: pick" in doc
    assert "role: destination" in doc
    assert "- id: pick_part" in doc
    assert "- id: place_bin" in doc
    assert "pose_xyz:" in doc and "pose_rpy:" in doc
    assert "source_object: pick_part" in doc
    assert "destination: place_bin" in doc


def test_intentionally_broken_definition_reports_multiple_missing_keys(tmp_path):
    broken = tmp_path / "broken_cell.yaml"
    broken.write_text("schema_version: cell_definition/v1\ncell:\n  id: broken\n  name: broken\n", encoding="utf-8")
    proc = __import__("subprocess").run(["python3", str(SCRIPTS / "validate_cell_definition.py"), str(broken), "--json"], capture_output=True, text=True, check=False)
    payload = json.loads((proc.stdout or proc.stderr).strip())
    assert proc.returncode != 0
    assert len(payload.get("errors", [])) >= 2


def test_acceptance_artifact_records_package_generation_command(tmp_path, monkeypatch):
    report = tmp_path / "acceptance.json"
    monkeypatch.setattr(target.sys, "argv", ["prog", "--scene-name", "scratchcmdtest", "--output-root", str(tmp_path / "out"), "--json-out", str(report)])

    def fake_run(cmd):
        cmd_s = " ".join(cmd)
        if "validate_cell_definition.py" in cmd_s:
            return 0, json.dumps({"errors": [], "warnings": []}), ""
        if "generate_workcell_from_cell_definition.py" in cmd_s:
            scene_name = cmd[cmd.index("--package-name") + 1]
            scene_dir = (tmp_path / "out" / scene_name)
            (scene_dir / "launch").mkdir(parents=True, exist_ok=True)
            for rel in target.REQUIRED:
                p = scene_dir / rel
                p.parent.mkdir(parents=True, exist_ok=True)
                p.write_text("x", encoding="utf-8")
            return 0, "ok", ""
        if "audit_new_cell_file_outputs.py" in cmd_s:
            return 0, "ok", ""
        raise AssertionError(cmd)

    monkeypatch.setattr(target, "_run", fake_run)
    rc = target.main()
    payload = json.loads(report.read_text(encoding="utf-8"))
    assert rc == 0
    assert "generator" in payload and "command" in payload["generator"]
    assert "--package-name" in payload["generator"]["command"]


def test_scratch_template_passes_real_validator(tmp_path):
    scene = tmp_path / 'cell_definition.yaml'
    scene.write_text(target.CELL_TMPL.format(scene='validatorpass'), encoding='utf-8')
    proc = __import__('subprocess').run(['python3', str(SCRIPTS / 'validate_cell_definition.py'), str(scene), '--json'], capture_output=True, text=True, check=False)
    payload = json.loads((proc.stdout or proc.stderr).strip())
    assert proc.returncode == 0
    assert payload.get('errors', []) == []


def test_generated_file_and_missing_file_lists_recorded(tmp_path, monkeypatch):
    report = tmp_path / 'acceptance.json'
    monkeypatch.setattr(target.sys, 'argv', ['prog', '--scene-name', 'scratchlisttest', '--output-root', str(tmp_path / 'out'), '--json-out', str(report)])

    def fake_run(cmd):
        cmd_s = ' '.join(cmd)
        if 'validate_cell_definition.py' in cmd_s:
            return 0, json.dumps({'errors': [], 'warnings': []}), ''
        if 'generate_workcell_from_cell_definition.py' in cmd_s:
            scene_name = cmd[cmd.index('--package-name') + 1]
            scene_dir = (tmp_path / 'out' / scene_name)
            for rel in ('scene_manifest.yaml', 'package.xml', 'CMakeLists.txt', 'launch/demo.launch.py'):
                p = scene_dir / rel
                p.parent.mkdir(parents=True, exist_ok=True)
                p.write_text('x', encoding='utf-8')
            return 0, 'ok', ''
        if 'audit_new_cell_file_outputs.py' in cmd_s:
            return 0, 'ok', ''
        raise AssertionError(cmd)

    monkeypatch.setattr(target, '_run', fake_run)
    target.main()
    payload = json.loads(report.read_text(encoding='utf-8'))
    assert isinstance(payload['generated_files'], list)
    assert isinstance(payload['missing_files'], list)
    assert payload['schema_preflight']['missing_package_files'] == payload['missing_files']


def test_audit_failure_recorded_not_crashed(tmp_path, monkeypatch):
    report = tmp_path / "acceptance.json"
    monkeypatch.setattr(target.sys, "argv", ["prog", "--scene-name", "scratchauditfail", "--output-root", str(tmp_path / "out"), "--json-out", str(report)])

    def fake_run(cmd):
        cmd_s = " ".join(cmd)
        if "validate_cell_definition.py" in cmd_s:
            return 0, json.dumps({"errors": [], "warnings": []}), ""
        if "generate_workcell_from_cell_definition.py" in cmd_s:
            scene_name = cmd[cmd.index("--package-name") + 1]
            scene_dir = (tmp_path / "out" / scene_name)
            for rel in target.REQUIRED:
                p = scene_dir / rel
                p.parent.mkdir(parents=True, exist_ok=True)
                p.write_text("x", encoding="utf-8")
            return 0, "ok", ""
        if "audit_new_cell_file_outputs.py" in cmd_s:
            return 9, "", "audit fail"
        raise AssertionError(cmd)

    monkeypatch.setattr(target, "_run", fake_run)
    rc = target.main()
    payload = json.loads(report.read_text(encoding="utf-8"))
    assert rc == 1
    assert payload["file_output_audit"]["returncode"] == 9
    assert payload["next_failed_step"] == "file_output_audit"


def test_cell_template_yaml_has_top_level_objects():
    doc = yaml.safe_load(target.CELL_TMPL.format(scene="demo"))
    assert "objects" in doc
