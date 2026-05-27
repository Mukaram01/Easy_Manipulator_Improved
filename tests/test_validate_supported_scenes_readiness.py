from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts" / "validate_supported_scenes_readiness.py"


def _write_scene(scene_dir: Path, name: str) -> None:
    scene_dir.mkdir(parents=True, exist_ok=True)
    (scene_dir / "package.xml").write_text(f"<package><name>{name}</name></package>\n", encoding="utf-8")
    (scene_dir / "CMakeLists.txt").write_text(f"project({name})\nament_package()\n", encoding="utf-8")
    (scene_dir / "environment.yaml").write_text("name: env\n", encoding="utf-8")
    (scene_dir / "cell_definition.yaml").write_text("robot: ur5\nend_effector: suction\nenvironment: demo\n", encoding="utf-8")
    (scene_dir / "launch").mkdir(exist_ok=True)
    (scene_dir / "launch/demo.launch.py").write_text("use_fake_hardware launch_rviz robot_state_publisher rviz xacro\n", encoding="utf-8")
    (scene_dir / "urdf").mkdir(exist_ok=True)
    (scene_dir / "urdf/scene.urdf.xacro").write_text("<robot name='x'></robot>\n", encoding="utf-8")
    (scene_dir / "layout").mkdir(exist_ok=True)
    (scene_dir / "layout/workcell_studio_layout.yaml").write_text("layout: ok\n", encoding="utf-8")
    (scene_dir / "generated").mkdir(exist_ok=True)
    (scene_dir / "generated/scene_package_readiness.json").write_text(json.dumps({"package_name": name}), encoding="utf-8")


def _run(tmp_path: Path, registry: Path, extra: list[str] | None = None) -> dict:
    cmd = [sys.executable, str(SCRIPT), "--repo-root", str(tmp_path), "--workspace-root", str(tmp_path), "--registry", str(registry), "--json", "--skip-build", "--skip-launch-smoke"]
    cmd.extend(extra or [])
    run = subprocess.run(cmd, capture_output=True, text=True, check=False)
    return json.loads(run.stdout)


def test_disabled_and_experimental_skipped(tmp_path: Path):
    _write_scene(tmp_path / "scenes/ok_scene", "ok_scene")
    reg = tmp_path / "registry.yaml"
    reg.write_text(yaml.safe_dump({"scenes": [
        {"scene_name": "disabled_scene", "scene_path": "scenes/disabled_scene", "enabled": False, "support_level": "supported", "expected_files": ["package.xml"]},
        {"scene_name": "exp_scene", "scene_path": "scenes/ok_scene", "enabled": True, "support_level": "experimental", "expected_files": ["package.xml"]},
    ]}), encoding="utf-8")
    payload = _run(tmp_path, reg)
    assert payload["summary"]["skipped"] == 2


def test_missing_scene_is_blocked(tmp_path: Path):
    reg = tmp_path / "registry.yaml"
    reg.write_text(yaml.safe_dump({"scenes": [{"scene_name": "missing", "scene_path": "scenes/missing", "enabled": True, "support_level": "supported", "expected_files": ["package.xml"]}]}), encoding="utf-8")
    payload = _run(tmp_path, reg)
    assert payload["per_scene"][0]["status"] == "BLOCKED"


def test_missing_required_file_is_fail(tmp_path: Path):
    (tmp_path / "scenes/a").mkdir(parents=True)
    reg = tmp_path / "registry.yaml"
    reg.write_text(yaml.safe_dump({"scenes": [{"scene_name": "a", "scene_path": "scenes/a", "enabled": True, "support_level": "supported", "expected_files": ["package.xml"]}]}), encoding="utf-8")
    payload = _run(tmp_path, reg)
    assert payload["per_scene"][0]["status"] == "FAIL"
    assert "missing_required_file: package.xml" in payload["per_scene"][0]["blockers"]


def test_experimental_included_with_flag(tmp_path: Path):
    _write_scene(tmp_path / "scenes/exp", "exp")
    reg = tmp_path / "registry.yaml"
    reg.write_text(yaml.safe_dump({"scenes": [{"scene_name": "exp", "scene_path": "scenes/exp", "enabled": True, "support_level": "experimental", "expected_files": ["package.xml", "CMakeLists.txt", "environment.yaml", "cell_definition.yaml", "launch/demo.launch.py", "urdf/scene.urdf.xacro", "generated/scene_package_readiness.json"]}]}), encoding="utf-8")
    payload = _run(tmp_path, reg, ["--include-experimental"])
    assert payload["per_scene"][0]["status"] in {"PASS", "PASS_WITH_WARNINGS"}


def test_summary_counts(tmp_path: Path):
    _write_scene(tmp_path / "scenes/pass_scene", "pass_scene")
    reg = tmp_path / "registry.yaml"
    reg.write_text(yaml.safe_dump({"scenes": [
        {"scene_name": "pass_scene", "scene_path": "scenes/pass_scene", "enabled": True, "support_level": "supported", "expected_files": ["package.xml", "CMakeLists.txt", "environment.yaml", "cell_definition.yaml", "launch/demo.launch.py", "urdf/scene.urdf.xacro", "generated/scene_package_readiness.json"]},
        {"scene_name": "missing_scene", "scene_path": "scenes/missing_scene", "enabled": True, "support_level": "supported", "expected_files": ["package.xml"]},
        {"scene_name": "disabled_scene", "scene_path": "scenes/disabled_scene", "enabled": False, "support_level": "supported", "expected_files": ["package.xml"]},
    ]}), encoding="utf-8")
    payload = _run(tmp_path, reg)
    assert payload["summary"] == {"total": 3, "pass": 1, "fail": 0, "blocked": 1, "warnings": 1, "skipped": 1}
