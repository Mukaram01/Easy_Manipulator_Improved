from __future__ import annotations

import json
import subprocess
from pathlib import Path

from tools.workcell_studio_streamlit import backend

REPO_ROOT = Path(__file__).resolve().parents[1]
CLI = REPO_ROOT / "scripts" / "workcell_studio.py"


def _run(*args: str) -> subprocess.CompletedProcess[str]:
    return subprocess.run(["python3", str(CLI), *args], capture_output=True, text=True, check=False)


def test_demo_catalog_loads_with_expected_demo_types() -> None:
    payload = backend.load_demo_catalog()
    demos = payload["demos"]
    ids = {d.get("id", "") for d in demos}
    assert any("ur5" in i for i in ids)
    assert any("suction" in i for i in ids)
    assert any((d.get("runtime_mode") == "preview_only") for d in demos)


def test_demo_cell_definition_paths_exist_and_validate() -> None:
    for demo in backend.load_demo_catalog()["demos"]:
        cell = REPO_ROOT / demo["cell_definition"]
        assert cell.exists()
        proc = subprocess.run(["python3", str(REPO_ROOT / "scripts" / "validate_cell_definition.py"), str(cell), "--json"], capture_output=True, text=True, check=False)
        assert proc.returncode == 0, proc.stdout + proc.stderr


def test_cli_list_demos_text_and_json() -> None:
    txt = _run("list-demos")
    js = _run("list-demos", "--json")
    assert txt.returncode == 0
    assert "ur5_suction_sorting_demo" in txt.stdout
    assert js.returncode == 0
    payload = json.loads(js.stdout)
    assert "demos" in payload


def test_generate_demo_bundle_for_ur5_suction() -> None:
    out = Path("/tmp/workcell_studio_demos_test")
    proc = _run("generate-demo-bundle", "--demo-id", "ur5_suction_sorting_demo", "--output-dir", str(out), "--force")
    assert proc.returncode == 0, proc.stdout + proc.stderr
    bundle = out / "ur5_suction_sorting_demo"
    assert (bundle / "demo_bundle_summary.json").is_file()
    assert (bundle / "demo_bundle_summary.md").is_file()
    assert (bundle / "cell_definition.yaml").is_file()
    assert (bundle / "next_commands.md").is_file()
    assert list(bundle.glob("*/project_manifest.json"))


def test_generate_all_demo_bundles_and_preview_only_reported() -> None:
    out = Path("/tmp/workcell_studio_all_demos_test")
    proc = _run("generate-demo-bundle", "--all", "--output-dir", str(out), "--force", "--continue-on-error")
    assert proc.returncode in {0, 1}
    for demo in backend.load_demo_catalog()["demos"]:
        bundle = out / demo["id"]
        if bundle.exists() and (bundle / "demo_bundle_summary.json").exists():
            payload = json.loads((bundle / "demo_bundle_summary.json").read_text(encoding="utf-8"))
            if demo.get("runtime_mode") == "preview_only":
                assert payload.get("preview_only") is True


def test_backend_demo_wrappers_and_invalid_demo_id() -> None:
    listed = backend.list_demos()
    assert listed["ok"]
    out = Path("/tmp/workcell_studio_backend_demo")
    generated = backend.generate_demo_bundle(out, demo_id="ur5_suction_sorting_demo")
    assert generated["ok"]
    summary = backend.load_demo_bundle_summary(out / "ur5_suction_sorting_demo")
    assert summary["summary_json_path"]

    invalid = _run("generate-demo-bundle", "--demo-id", "missing_demo", "--output-dir", str(out), "--force")
    assert invalid.returncode != 0
    assert "Unknown or missing demo id" in (invalid.stdout + invalid.stderr)
