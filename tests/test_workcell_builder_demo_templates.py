from __future__ import annotations

from pathlib import Path

import yaml

REPO_ROOT = Path(__file__).resolve().parents[1]
CATALOG = REPO_ROOT / "catalog" / "workcell_studio_demos.yaml"


def _load() -> dict:
    return yaml.safe_load(CATALOG.read_text(encoding="utf-8")) or {}


def test_all_requested_templates_exist_and_load() -> None:
    demos = _load().get("demos", [])
    assert len(demos) == 10
    for demo in demos:
        assert demo.get("id")
        cell_path = REPO_ROOT / demo["cell_definition"]
        assert cell_path.exists(), demo["id"]


def test_templates_include_required_content_fields() -> None:
    for demo in _load().get("demos", []):
        payload = yaml.safe_load((REPO_ROOT / demo["cell_definition"]).read_text(encoding="utf-8"))
        assert payload.get("robot"), demo["id"]
        assert payload.get("end_effector"), demo["id"]
        assert payload.get("task"), demo["id"]
        assert payload.get("environment"), demo["id"]
        assert payload.get("grasp"), demo["id"]
        assert payload.get("commissioning", {}).get("fake_hardware_default") is True, demo["id"]
        assert payload.get("commissioning", {}).get("demo_template_id") == demo["id"], demo["id"]


def test_preview_only_templates_are_honest() -> None:
    for demo in _load().get("demos", []):
        if demo["runtime_mode"] == "preview_only":
            assert demo.get("preview_only") is True
            assert "warn" in str(demo.get("compatibility_status", "")).lower()


def test_ur5_templates_remain_supported_only() -> None:
    ur5_demos = [d for d in _load().get("demos", []) if d["id"].startswith("ur5_")]
    assert len(ur5_demos) == 4
    for demo in ur5_demos:
        assert demo.get("runtime_mode") == "fake_hardware_ready"
        assert demo.get("compatibility_status") == "supported"
