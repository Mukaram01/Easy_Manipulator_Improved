from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts" / "validate_rviz_truth_preview_ui_path.py"


def _run(scene: str, workspace: Path, *extra: str) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        [sys.executable, str(SCRIPT), scene, "--workspace-root", str(workspace), "--repo-root", str(ROOT), "--json", *extra],
        cwd=ROOT,
        capture_output=True,
        text=True,
        check=False,
    )


def _run_scene_option(scene: str, workspace: Path, *extra: str) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        [
            sys.executable,
            str(SCRIPT),
            "--scene",
            scene,
            "--workspace-root",
            str(workspace),
            "--repo-root",
            str(ROOT),
            "--json",
            *extra,
        ],
        cwd=ROOT,
        capture_output=True,
        text=True,
        check=False,
    )


def test_scene_option_exact_command_has_no_false_non_determinism_blocker(tmp_path: Path):
    (tmp_path / "scenes" / "ur5_2f_test" / "launch").mkdir(parents=True)
    (tmp_path / "scenes" / "ur5_2f_test" / "package.xml").write_text("<package/>", encoding="utf-8")
    (tmp_path / "scenes" / "ur5_2f_test" / "launch" / "demo.launch.py").write_text("# launch", encoding="utf-8")
    (tmp_path / "install").mkdir()
    (tmp_path / "install" / "setup.bash").write_text("# setup", encoding="utf-8")

    proc = _run_scene_option("ur5_2f_test", tmp_path)
    assert proc.returncode == 0, proc.stdout + proc.stderr
    payload = json.loads(proc.stdout)

    expected = "ros2 launch ur5_2f_test demo.launch.py use_fake_hardware:=true launch_rviz:=true"
    assert payload["exact_launch_command"] == expected
    assert "launch command build is non-deterministic" not in payload["blockers"]


def test_exact_command_and_required_tokens_for_ur5_2f_test(tmp_path: Path):
    (tmp_path / "scenes" / "ur5_2f_test" / "launch").mkdir(parents=True)
    (tmp_path / "scenes" / "ur5_2f_test" / "package.xml").write_text("<package/>", encoding="utf-8")
    (tmp_path / "scenes" / "ur5_2f_test" / "launch" / "demo.launch.py").write_text("# launch", encoding="utf-8")
    (tmp_path / "install").mkdir()
    (tmp_path / "install" / "setup.bash").write_text("# setup", encoding="utf-8")

    proc = _run("ur5_2f_test", tmp_path)
    assert proc.returncode == 0, proc.stdout + proc.stderr
    payload = json.loads(proc.stdout)

    expected = "ros2 launch ur5_2f_test demo.launch.py use_fake_hardware:=true launch_rviz:=true"
    assert payload["exact_launch_command"] == expected
    assert "use_fake_hardware:=true" in payload["exact_launch_command"]
    assert "launch_rviz:=true" in payload["exact_launch_command"]
    assert "use_fake_hardware:=false" not in payload["exact_launch_command"]


def test_missing_launch_file_message(tmp_path: Path):
    (tmp_path / "scenes" / "ur5_2f_test").mkdir(parents=True)
    (tmp_path / "scenes" / "ur5_2f_test" / "package.xml").write_text("<package/>", encoding="utf-8")
    (tmp_path / "install").mkdir()
    (tmp_path / "install" / "setup.bash").write_text("# setup", encoding="utf-8")

    proc = _run("ur5_2f_test", tmp_path)
    assert proc.returncode != 0
    payload = json.loads(proc.stdout)
    assert "launch/demo.launch.py missing" in payload["blockers"]


def test_missing_workspace_setup_message(tmp_path: Path):
    (tmp_path / "scenes" / "ur5_2f_test" / "launch").mkdir(parents=True)
    (tmp_path / "scenes" / "ur5_2f_test" / "package.xml").write_text("<package/>", encoding="utf-8")
    (tmp_path / "scenes" / "ur5_2f_test" / "launch" / "demo.launch.py").write_text("# launch", encoding="utf-8")

    proc = _run("ur5_2f_test", tmp_path)
    assert proc.returncode != 0
    payload = json.loads(proc.stdout)
    assert "install/setup.bash missing under workspace root" in payload["blockers"]


def test_runner_uses_dry_run_and_run_paths_not_legacy_direct_launch(tmp_path: Path):
    (tmp_path / "scenes" / "ur5_2f_test" / "launch").mkdir(parents=True)
    (tmp_path / "scenes" / "ur5_2f_test" / "package.xml").write_text("<package/>", encoding="utf-8")
    (tmp_path / "scenes" / "ur5_2f_test" / "launch" / "demo.launch.py").write_text("# launch", encoding="utf-8")
    (tmp_path / "install").mkdir()
    (tmp_path / "install" / "setup.bash").write_text("# setup", encoding="utf-8")

    proc = _run("ur5_2f_test", tmp_path)
    payload = json.loads(proc.stdout)
    runner = payload["runner"]
    assert set(runner.keys()) == {"dry_run", "run"}
    assert "source install/setup.bash" in runner["dry_run"]
    assert "source install/setup.bash" in runner["run"]
    assert not runner["dry_run"].startswith("ros2 launch ")


def test_optional_ros2_pkg_prefix_check_reports_warning_when_unsourced(tmp_path: Path):
    (tmp_path / "scenes" / "ur5_2f_test" / "launch").mkdir(parents=True)
    (tmp_path / "scenes" / "ur5_2f_test" / "package.xml").write_text("<package/>", encoding="utf-8")
    (tmp_path / "scenes" / "ur5_2f_test" / "launch" / "demo.launch.py").write_text("# launch", encoding="utf-8")
    (tmp_path / "install").mkdir()
    (tmp_path / "install" / "setup.bash").write_text("# setup", encoding="utf-8")

    proc = _run("ur5_2f_test", tmp_path, "--check-ros2-prefix")
    payload = json.loads(proc.stdout)
    assert "ros2_pkg_prefix" in payload
    assert payload["ros2_pkg_prefix"]["checked"] is True
