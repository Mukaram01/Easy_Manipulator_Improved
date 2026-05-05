#!/usr/bin/env python3
"""Tests for validate_static_sorting_scene_contract."""

import argparse
import importlib.util
import json
from pathlib import Path
import shutil
import sys
import tempfile
from unittest.mock import patch



def _load_module():
    scene_root = Path(__file__).resolve().parents[1]
    script_path = scene_root / "scripts" / "validate_static_sorting_scene_contract.py"
    spec = importlib.util.spec_from_file_location("contract", script_path)
    module = importlib.util.module_from_spec(spec)
    assert spec and spec.loader
    spec.loader.exec_module(module)
    return module


def _args(module, **overrides):
    base = dict(json=True, scene_package="ur5_2f_sorting_test", manifest=Path("scenes/ur5_2f_sorting_test/config/sorting_manifest.yaml"), payload_output=Path("/tmp/contract_payload_test.json"), strict=False, print_summary=False)
    base.update(overrides)
    return argparse.Namespace(**base)


def main() -> int:
    mod = _load_module()

    rep, rc = mod.build_report(_args(mod))
    assert rc == 0 and rep["result"]["status"] == "PASS"

    for key in ["schema", "scene_package", "manifest_path", "payload_path", "checks", "targets", "result"]:
        assert key in rep
    assert rep["schema"] == "static_sorting_scene_contract/v1"

    with tempfile.TemporaryDirectory() as td:
        src = Path(__file__).resolve().parents[1] / "sorting_manifest.yaml"
        mf = Path(td) / "sorting_manifest.yaml"
        shutil.copyfile(src, mf)
        text = mf.read_text(encoding="utf-8")
        mf.write_text(text.replace("    - id: item_blue", "    - id: item_red", 1), encoding="utf-8")
        rep, rc = mod.build_report(_args(mod, manifest=mf))
        assert rc == 2
        assert any("Duplicate target ids" in e for e in rep["errors"])

    with tempfile.TemporaryDirectory() as td:
        src = Path(__file__).resolve().parents[1] / "sorting_manifest.yaml"
        mf = Path(td) / "sorting_manifest.yaml"
        shutil.copyfile(src, mf)
        text = mf.read_text(encoding="utf-8")
        mf.write_text(text.replace("destination: bin_a", "destination: missing_bin", 1), encoding="utf-8")
        rep, rc = mod.build_report(_args(mod, manifest=mf))
        assert rc == 2
        assert any("missing destination" in e for e in rep["errors"])

    original_build_payload = mod.payload_generator.build_payload

    def bad_frame_payload(*args, **kwargs):
        payload = original_build_payload(*args, **kwargs)
        payload["grasp_task"]["grasp_targets"][0]["destination_pose"]["frame_id"] = "table"
        return payload

    with patch.object(mod.payload_generator, "build_payload", side_effect=bad_frame_payload):
        rep, rc = mod.build_report(_args(mod))
        assert rc == 2
        assert rep["checks"]["destination_poses_world_frame"] is False

    def bad_grasp_payload(*args, **kwargs):
        payload = original_build_payload(*args, **kwargs)
        payload["grasp_task"]["grasp_targets"][0]["grasp_methods"][0]["grasp_poses"][0]["xyz"][2] = 0.0
        return payload

    with patch.object(mod.payload_generator, "build_payload", side_effect=bad_grasp_payload):
        rep, rc = mod.build_report(_args(mod))
        assert rc == 2
        assert any("non-positive top grasp" in e for e in rep["errors"])

    with tempfile.TemporaryDirectory() as td:
        src = Path(__file__).resolve().parents[1] / "sorting_manifest.yaml"
        mf = Path(td) / "sorting_manifest.yaml"
        shutil.copyfile(src, mf)
        text = mf.read_text(encoding="utf-8")
        mf.write_text(text.replace("      pick_hint: top_grasp\n", "", 1), encoding="utf-8")
        rep, rc = mod.build_report(_args(mod, manifest=mf, strict=True))
        assert rep["result"]["status"] == "WARN"
        assert rc == 2

    with patch.object(mod.sequence_runner, "build_report", return_value=({"mode": "dry_run", "safety": {"robot_motion_requested": False}}, 0)):
        rep, rc = mod.build_report(_args(mod))
        assert rc == 0

    return 0


if __name__ == "__main__":
    sys.exit(main())
