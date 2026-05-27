from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
VALIDATOR = REPO_ROOT / "scripts" / "validate_guided_generated_scene_build_readiness.py"


def _run_validator(scene_dir: Path) -> dict:
    proc = subprocess.run(
        [
            sys.executable,
            str(VALIDATOR),
            "--scene-dir",
            str(scene_dir),
            "--skip-build",
            "--skip-launch-smoke",
            "--json",
        ],
        cwd=REPO_ROOT,
        capture_output=True,
        text=True,
        check=False,
    )
    assert proc.stdout.strip(), f"validator produced empty stdout (stderr={proc.stderr!r})"
    payload = json.loads(proc.stdout)

    # Contract-level keys required by tests.
    for key in ("status", "blockers", "warnings", "package_xml_ok", "xacro_ok", "launch_contract_ok"):
        assert key in payload, f"missing expected contract key: {key}; got: {sorted(payload.keys())}"
    assert isinstance(payload["blockers"], list)
    assert isinstance(payload["warnings"], list)
    return payload


def _write(path: Path, content: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(content, encoding="utf-8")


def _make_minimal_valid_tree(scene_dir: Path) -> None:
    _write(
        scene_dir / "package.xml",
        """<package format=\"3\"><name>test_scene</name><version>0.0.1</version><description>test</description><maintainer email=\"test@example.com\">test</maintainer><license>Apache-2.0</license></package>""",
    )
    _write(scene_dir / "CMakeLists.txt", "cmake_minimum_required(VERSION 3.10)\nproject(test_scene)\n")
    _write(
        scene_dir / "urdf" / "scene.urdf.xacro",
        """<?xml version=\"1.0\"?><robot name=\"test_scene\" xmlns:xacro=\"http://www.ros.org/wiki/xacro\"><link name=\"base_link\"/></robot>""",
    )
    _write(scene_dir / "config" / "controllers.yaml", "controller_manager: {}\n")
    _write(
        scene_dir / "launch" / "demo.launch.py",
        """from launch import LaunchDescription\nfrom launch.actions import DeclareLaunchArgument\n\n\ndef generate_launch_description():\n    return LaunchDescription([\n        DeclareLaunchArgument('use_fake_hardware', default_value='true'),\n    ])\n""",
    )


def test_missing_required_files_fails_clearly(tmp_path: Path):
    scene_dir = tmp_path / "scene_pkg"
    # Intentionally partial to trigger missing-file blockers.
    _write(scene_dir / "package.xml", "<package></package>")

    payload = _run_validator(scene_dir)

    assert payload["status"] == "FAIL"
    missing_paths = payload.get("missing_required_paths") or payload.get("missing_files") or []
    assert missing_paths, f"expected missing paths list, got payload={payload}"
    blockers_text = "\n".join(payload["blockers"])
    for required_path in missing_paths:
        assert str(required_path) in blockers_text


def test_malformed_package_xml_fails_clearly(tmp_path: Path):
    scene_dir = tmp_path / "scene_pkg"
    _make_minimal_valid_tree(scene_dir)
    _write(scene_dir / "package.xml", "<package><name>broken</name>")

    payload = _run_validator(scene_dir)

    assert payload["status"] == "FAIL"
    assert payload["package_xml_ok"] is False
    blockers_text = "\n".join(payload["blockers"]).lower()
    assert "package.xml" in blockers_text
    assert any(tok in blockers_text for tok in ("parse", "xml", "malformed", "mismatched"))


def test_unresolved_xacro_placeholders_fail_clearly(tmp_path: Path):
    scene_dir = tmp_path / "scene_pkg"
    _make_minimal_valid_tree(scene_dir)
    _write(
        scene_dir / "urdf" / "scene.urdf.xacro",
        """<?xml version=\"1.0\"?><robot name=\"test_scene\" xmlns:xacro=\"http://www.ros.org/wiki/xacro\"><link name=\"${UNRESOLVED_PLACEHOLDER}\"/></robot>""",
    )

    payload = _run_validator(scene_dir)

    assert payload["status"] == "FAIL"
    assert payload["xacro_ok"] is False
    blockers_text = "\n".join(payload["blockers"])
    assert "unresolved" in blockers_text.lower()
    assert "placeholder" in blockers_text.lower() or "${" in blockers_text


def test_missing_use_fake_hardware_launch_contract_fails_clearly(tmp_path: Path):
    scene_dir = tmp_path / "scene_pkg"
    _make_minimal_valid_tree(scene_dir)
    _write(
        scene_dir / "launch" / "demo.launch.py",
        """from launch import LaunchDescription\n\n\ndef generate_launch_description():\n    return LaunchDescription([])\n""",
    )

    payload = _run_validator(scene_dir)

    assert payload["status"] == "FAIL"
    assert payload["launch_contract_ok"] is False
    blockers_text = "\n".join(payload["blockers"])
    assert "use_fake_hardware" in blockers_text


def test_valid_minimal_package_passes_static_validation(tmp_path: Path):
    scene_dir = tmp_path / "scene_pkg"
    _make_minimal_valid_tree(scene_dir)

    payload = _run_validator(scene_dir)

    assert payload["status"] in {"PASS", "PASS_WITH_WARNINGS"}
    assert payload["package_xml_ok"] is True
    assert payload["xacro_ok"] is True
    assert payload["launch_contract_ok"] is True
