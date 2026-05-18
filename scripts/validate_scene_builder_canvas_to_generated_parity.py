#!/usr/bin/env python3
"""Validate parity between authored Scene Builder canvas layout and generated outputs."""
from __future__ import annotations

import argparse
import json
import subprocess
import sys
import tempfile
from pathlib import Path
from typing import Any

SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parent

if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from capability_registry import load_structured_data


def _run(cmd: list[str]) -> tuple[int, str, str]:
    run = subprocess.run(cmd, capture_output=True, text=True, check=False)
    return run.returncode, run.stdout, run.stderr


def _load_structured(path: Path) -> dict[str, Any]:
    if not path.is_file():
        return {}
    loaded, _ = load_structured_data(path)
    return loaded if isinstance(loaded, dict) else {}


def _write_json(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def _fixture_layout() -> dict[str, Any]:
    return {
        "schema_version": "environment_layout/v1",
        "layout_id": "scene_builder_canvas_parity_fixture",
        "metadata": {"generated_defaults": False, "name": "Scene Builder Canvas Parity Fixture"},
        "assets": [
            {"id": "table_01", "asset_ref": "table_large", "frame": "world", "xyz": [0.65, 0.0, 0.36], "rpy": [0.0, 0.0, 0.0]},
            {"id": "bin_place_01", "asset_ref": "bin_large", "frame": "world", "xyz": [0.55, 0.28, 0.42], "rpy": [0.0, 0.0, 0.0]},
            {"id": "conveyor_01", "asset_ref": "conveyor_placeholder", "frame": "world", "xyz": [0.25, -0.35, 0.36], "rpy": [0.0, 0.0, 0.0]},
            {"id": "camera_01", "asset_ref": "camera_stand", "frame": "world", "xyz": [0.10, 0.40, 0.82], "rpy": [0.0, 0.0, 1.57]},
            {
                "id": "mesh_object_01",
                "asset_ref": "calibration_cube",
                "frame": "world",
                "xyz": [0.60, -0.15, 0.74],
                "rpy": [0.0, 0.0, 0.0],
                "mesh": "workcell_builder/workcell_builder/assets/environment/calibration_cube_description/meshes/calibration_cube.stl",
            },
        ],
        "zones": [
            {"id": "pick_zone_01", "type": "pick", "frame": "world", "bounds_xyz": {"min": [0.45, -0.25, 0.72], "max": [0.72, 0.0, 0.90]}}
        ],
        "targets": [
            {"id": "place_target_01", "type": "place", "frame": "world", "xyz": [0.55, 0.28, 0.47], "rpy": [0.0, 0.0, 0.0]}
        ],
        "camera": {"id": "camera_01", "frame": "world", "xyz": [0.10, 0.40, 0.82], "rpy": [0.0, 0.0, 1.57]},
        "objects": [
            {
                "id": "mesh_object_01",
                "type": "mesh",
                "mesh": "workcell_builder/workcell_builder/assets/environment/calibration_cube_description/meshes/calibration_cube.stl",
                "frame": "world",
                "xyz": [0.60, -0.15, 0.74],
                "rpy": [0.0, 0.0, 0.0],
            }
        ],
    }


def _asset_index(layout: dict[str, Any]) -> dict[str, dict[str, Any]]:
    out: dict[str, dict[str, Any]] = {}
    for asset in layout.get("assets", []):
        if isinstance(asset, dict) and isinstance(asset.get("id"), str):
            out[asset["id"]] = asset
    return out


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, default=REPO_ROOT / "generated" / "scene_builder_canvas_parity_report.json")
    args = parser.parse_args()

    warnings: list[str] = []
    blockers: list[str] = []

    with tempfile.TemporaryDirectory(prefix="scene_builder_canvas_parity_") as t:
        tmp = Path(t)
        scene_dir = tmp / "fixture_scene"
        generated_export_dir = scene_dir / "generated"
        out_dir = tmp / "generated_workspace"
        scene_dir.mkdir(parents=True, exist_ok=True)

        layout_path = scene_dir / "environment_layout.yaml"
        layout = _fixture_layout()
        _write_json(layout_path, layout)

        export_cmd = [sys.executable, str(SCRIPT_DIR / "export_builder_scene_to_cell_definition.py"), str(scene_dir), "--output-dir", str(generated_export_dir), "--validate"]
        rc_export, export_stdout, export_stderr = _run(export_cmd)
        if rc_export != 0:
            blockers.append("Failed to generate cell_definition.yaml via non-GUI export path.")

        cell_path = generated_export_dir / "cell_definition.yaml"
        if not cell_path.is_file():
            blockers.append("generated/cell_definition.yaml missing after export.")

        for script_name, input_path in [
            ("validate_environment_layout.py", layout_path),
            ("validate_cell_definition.py", cell_path),
        ]:
            rc, stdout, stderr = _run([sys.executable, str(SCRIPT_DIR / script_name), str(input_path), "--json"])
            if rc != 0:
                blockers.append(f"{script_name} failed")
            try:
                payload = json.loads(stdout) if stdout.strip() else {}
            except Exception:
                payload = {}
            if isinstance(payload, dict) and payload.get("warnings"):
                warnings.extend(str(w) for w in payload.get("warnings", []))

        rc_flow = subprocess.run([sys.executable, str(SCRIPT_DIR / "validate_scene_builder_mainline_flow.py")], capture_output=True, text=True, check=False).returncode
        if rc_flow != 0:
            blockers.append("validate_scene_builder_mainline_flow.py failed")

        pkg_name = "scene_builder_canvas_fixture_pkg"
        gen_cmd = [
            sys.executable,
            str(SCRIPT_DIR / "generate_workcell_from_cell_definition.py"),
            str(cell_path),
            "--output-dir",
            str(out_dir),
            "--package-name",
            pkg_name,
            "--force",
        ]
        rc_gen, gen_stdout, gen_stderr = _run(gen_cmd)
        if rc_gen != 0:
            blockers.append("generate_workcell_from_cell_definition.py failed")

        package_dir = out_dir / pkg_name
        if not package_dir.is_dir():
            blockers.append("Generated package directory missing.")
        generated_meta = _load_structured(package_dir / "generated" / "builder_export_summary.json") if (package_dir / "generated" / "builder_export_summary.json").is_file() else {}
        environment_yaml = _load_structured(package_dir / "environment.yaml")

        source_assets = _asset_index(layout)
        generated_assets_raw = environment_yaml.get("assets") if isinstance(environment_yaml.get("assets"), list) else []
        generated_assets = {a.get("id"): a for a in generated_assets_raw if isinstance(a, dict) and isinstance(a.get("id"), str)}

        missing = sorted([aid for aid in source_assets if aid not in generated_assets])
        transform_mismatches: list[dict[str, Any]] = []
        mesh_reference_mismatches: list[dict[str, Any]] = []
        for aid, src in sorted(source_assets.items()):
            gen = generated_assets.get(aid)
            if not gen:
                continue
            for key in ("xyz", "rpy"):
                if src.get(key) != gen.get(key):
                    transform_mismatches.append({"asset_id": aid, "field": key, "source": src.get(key), "generated": gen.get(key)})
            src_mesh = src.get("mesh")
            gen_mesh = gen.get("mesh") or gen.get("filepath")
            if src_mesh and src_mesh != gen_mesh:
                mesh_reference_mismatches.append({"asset_id": aid, "source_mesh": src_mesh, "generated_mesh": gen_mesh})

        launch_text = ""
        for launch_file in sorted((package_dir / "launch").glob("*.launch.py")):
            launch_text += launch_file.read_text(encoding="utf-8") + "\n"
        if "use_fake_hardware:=true" not in launch_text:
            blockers.append("Launch artifacts missing use_fake_hardware:=true token.")

        report = {
            "source_layout_path": str(layout_path),
            "generated_package_path": str(package_dir),
            "assets_in_canvas": sorted(source_assets.keys()),
            "assets_in_generated_metadata": sorted(generated_assets.keys()),
            "assets_missing_from_generated_output": missing,
            "transform_mismatches": transform_mismatches,
            "mesh_reference_mismatches": mesh_reference_mismatches,
            "warnings": sorted(set(warnings)),
            "blockers": sorted(set(blockers)),
            "debug": {"export_stdout": export_stdout.strip(), "export_stderr": export_stderr.strip(), "generation_stdout": gen_stdout.strip(), "generation_stderr": gen_stderr.strip(), "builder_export_summary": generated_meta},
        }

        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")

    print(json.dumps({"report": str(args.output)}, indent=2))
    return 1 if blockers else 0


if __name__ == "__main__":
    raise SystemExit(main())
