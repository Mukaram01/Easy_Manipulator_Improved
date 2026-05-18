#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
from pathlib import Path
import re
import sys
from typing import Any

ROOT = Path(__file__).resolve().parents[1]
CPP = ROOT / "workcell_builder/workcell_builder/gui/mainwindow.cpp"
CANVAS = ROOT / "workcell_builder/workcell_builder/src_workcell_studio_canvas_model.cpp"

CANONICAL_REQUIRED_REPORT_FIELDS = [
    "status",
    "source_layout_path",
    "generated_package_path",
    "scene_builder_parity_action_present",
    "parity_status_labels_present",
    "parity_report_filename_contract",
    "parity_report_required_fields",
    "layout_asset_ids",
    "generated_asset_ids",
    "layout_asset_id_set",
    "generated_asset_id_set",
    "assets_missing_from_generated_output",
    "generated_assets_not_in_layout",
    "transform_mismatches",
    "mesh_reference_mismatches",
    "unsupported_assets",
    "warning_count",
    "mismatch_count",
    "blocker_count",
    "fake_hardware_default",
    "fake_hardware_token_preserved",
    "unsupported_asset_warning_contract",
    "transform_mismatch_section",
    "mesh_reference_mismatch_section",
    "warnings",
    "errors",
]

def _contains(text: str, tokens: list[str]) -> bool:
    return all(token in text for token in tokens)

def _extract_asset_ids(text: str) -> list[str]:
    matches = re.findall(r'"([A-Za-z0-9_\-]+)"', text)
    filtered = {m for m in matches if any(c.isalpha() for c in m) and " " not in m and len(m) >= 3}
    return sorted(filtered)

def _safe_load_yaml(path: Path) -> Any:
    import yaml
    with path.open("r", encoding="utf-8") as f:
        return yaml.safe_load(f)

def _safe_load_json(path: Path) -> Any:
    with path.open("r", encoding="utf-8") as f:
        return json.load(f)

def _walk_assets(node: Any) -> list[dict[str, Any]]:
    out: list[dict[str, Any]] = []
    if isinstance(node, dict):
        has_id = any(k in node for k in ("id", "asset_id", "name"))
        has_pose_or_mesh = any(k in node for k in ("pose", "transform", "xyz", "rpy", "mesh", "mesh_path", "mesh_uri"))
        if has_id and has_pose_or_mesh:
            out.append(node)
        for v in node.values():
            out.extend(_walk_assets(v))
    elif isinstance(node, list):
        for v in node:
            out.extend(_walk_assets(v))
    return out


def _find_layout_path(scene_dir: Path, warnings: list[str]) -> Path:
    preferred = scene_dir / "layout" / "workcell_studio_layout.yaml"
    legacy = scene_dir / "environment_layout.yaml"
    if preferred.exists() and legacy.exists():
        warnings.append(
            f"both layout sources exist; using preferred layout file {preferred} and treating legacy {legacy} as secondary"
        )
    if preferred.exists():
        return preferred
    if legacy.exists():
        warnings.append(f"preferred layout source missing; falling back to legacy layout file {legacy}")
        return legacy
    return preferred

def _as_float_list(value: Any) -> list[float] | None:
    if not isinstance(value, list):
        return None
    out: list[float] = []
    for x in value:
        if isinstance(x, (int, float)):
            out.append(float(x))
        else:
            return None
    return out

def _extract_pose(asset: dict[str, Any]) -> dict[str, list[float]]:
    xyz = None
    rpy = None
    if isinstance(asset.get("pose"), dict):
        pose = asset["pose"]
        xyz = _as_float_list(pose.get("xyz"))
        rpy = _as_float_list(pose.get("rpy"))
    if xyz is None:
        xyz = _as_float_list(asset.get("xyz"))
    if rpy is None:
        rpy = _as_float_list(asset.get("rpy"))
    if isinstance(asset.get("transform"), dict):
        tf = asset["transform"]
        xyz = xyz if xyz is not None else _as_float_list(tf.get("xyz"))
        rpy = rpy if rpy is not None else _as_float_list(tf.get("rpy"))
    result: dict[str, list[float]] = {}
    if xyz is not None:
        result["xyz"] = xyz
    if rpy is not None:
        result["rpy"] = rpy
    return result

def _extract_mesh(asset: dict[str, Any]) -> str | None:
    for key in ("mesh", "mesh_path", "mesh_uri", "visual_mesh", "collision_mesh"):
        value = asset.get(key)
        if isinstance(value, str) and value.strip():
            return value.strip()
    if isinstance(asset.get("visual"), dict):
        value = asset["visual"].get("mesh")
        if isinstance(value, str) and value.strip():
            return value.strip()
    return None

def _asset_id(asset: dict[str, Any]) -> str | None:
    for key in ("id", "asset_id", "name"):
        value = asset.get(key)
        if isinstance(value, str) and value.strip():
            return value.strip()
    return None

def build_report(scene_dir: Path | None = None) -> dict[str, object]:
    warnings: list[str] = []
    errors: list[str] = []

    cpp_text = CPP.read_text(encoding="utf-8") if CPP.exists() else ""
    canvas_text = CANVAS.read_text(encoding="utf-8") if CANVAS.exists() else ""

    scene_builder_parity_action_present = _contains(cpp_text,["Validate Canvas/Generated Parity", "validate_scene_builder_canvas_generated_parity.py"])
    if not scene_builder_parity_action_present:
        errors.append("parity action missing from UI wiring")
    parity_status_labels_present = _contains(cpp_text, ["Parity: PASS", "Parity: WARN", "Parity: FAIL"])
    if not parity_status_labels_present:
        errors.append("parity status labels missing")
    parity_report_filename_contract = "scene_builder_canvas_generated_parity_report.json" in cpp_text
    if not parity_report_filename_contract:
        errors.append("parity report filename contract missing")

    fake_hardware_token_preserved = _contains(cpp_text, ["use_fake_hardware:=true", "# fake-hardware launch command"])
    if not fake_hardware_token_preserved:
        errors.append("fake-hardware token contract missing")
    fake_hardware_default = fake_hardware_token_preserved

    unsupported_asset_warning_contract = "unsupported asset" in cpp_text.lower() or "unsupported asset" in canvas_text.lower()
    transform_mismatch_section = _contains(cpp_text, ["transform_mismatch", "Transform Mismatch"])
    mesh_reference_mismatch_section = _contains(cpp_text, ["mesh_reference_mismatch", "Mesh Reference Mismatch"])

    layout_asset_ids = _extract_asset_ids(canvas_text)
    generated_asset_ids = _extract_asset_ids(cpp_text)
    source_layout_path = "layout/workcell_studio_layout.yaml"
    generated_package_path = "generated"

    transform_mismatches: list[dict[str, object]] = []
    mesh_reference_mismatches: list[dict[str, object]] = []
    unsupported_assets: list[str] = []
    assets_missing_from_generated_output: list[str] = []
    generated_assets_not_in_layout: list[str] = []

    if scene_dir is not None:
        scene_dir = scene_dir.resolve()
        source_layout = _find_layout_path(scene_dir, warnings)
        source_layout_path = str(source_layout)
        generated_package_path = str(scene_dir / "generated")
        layout_assets: dict[str, dict[str, Any]] = {}
        generated_assets: dict[str, dict[str, Any]] = {}
        layout_candidates = [source_layout]
        generated_candidates = [
            scene_dir / "generated" / "generated_workcell_summary.json",
            scene_dir / "generated" / "generated_environment_objects.yaml",
            scene_dir / "scene_manifest.yaml",
            scene_dir / "urdf" / "generated_asset_metadata.yaml",
        ]

        def _load_assets_for(paths: list[Path], target: dict[str, dict[str, Any]]) -> None:
            for path in paths:
                if not path.exists():
                    warnings.append(f"optional scene artifact missing: {path}")
                    continue
                try:
                    data = _safe_load_json(path) if path.suffix.lower() == ".json" else _safe_load_yaml(path)
                except Exception as exc:
                    errors.append(f"failed to parse {path}: {exc}")
                    continue
                if data is None:
                    warnings.append(f"empty scene artifact: {path}")
                    continue
                for asset in _walk_assets(data):
                    aid = _asset_id(asset)
                    if not aid:
                        continue
                    pose = _extract_pose(asset)
                    mesh = _extract_mesh(asset)
                    rec = target.setdefault(aid, {"poses": [], "meshes": [], "preview_only": False, "unsupported": False})
                    if pose:
                        rec["poses"].append(pose)
                    if mesh:
                        rec["meshes"].append(mesh)
                    flags = " ".join(str(asset.get(k, "")) for k in ("status", "type", "mode", "tags", "notes")).lower()
                    if "preview" in flags:
                        rec["preview_only"] = True
                    if "unsupported" in flags:
                        rec["unsupported"] = True

        _load_assets_for(layout_candidates, layout_assets)
        _load_assets_for(generated_candidates, generated_assets)

        layout_asset_ids = sorted(layout_assets.keys())
        generated_asset_ids = sorted(generated_assets.keys())
        assets_missing_from_generated_output = sorted(set(layout_asset_ids) - set(generated_asset_ids))
        generated_assets_not_in_layout = sorted(set(generated_asset_ids) - set(layout_asset_ids))

        tracked_asset_ids = sorted(set(layout_asset_ids) | set(generated_asset_ids))
        for aid in tracked_asset_ids:
            layout_rec = layout_assets.get(aid, {"poses": [], "meshes": [], "preview_only": False, "unsupported": False})
            generated_rec = generated_assets.get(aid, {"poses": [], "meshes": [], "preview_only": False, "unsupported": False})

            if layout_rec["preview_only"] or layout_rec["unsupported"] or generated_rec["preview_only"] or generated_rec["unsupported"]:
                unsupported_assets.append(aid)

            layout_poses = sorted({json.dumps(p, sort_keys=True) for p in layout_rec["poses"] if p})
            generated_poses = sorted({json.dumps(p, sort_keys=True) for p in generated_rec["poses"] if p})
            if layout_poses and generated_poses and layout_poses != generated_poses:
                transform_mismatches.append(
                    {
                        "asset_id": aid,
                        "layout_poses": [json.loads(p) for p in layout_poses],
                        "generated_poses": [json.loads(p) for p in generated_poses],
                    }
                )

            layout_meshes = sorted({m for m in layout_rec["meshes"] if m})
            generated_meshes = sorted({m for m in generated_rec["meshes"] if m})
            if layout_meshes and generated_meshes and layout_meshes != generated_meshes:
                mesh_reference_mismatches.append(
                    {"asset_id": aid, "layout_mesh_references": layout_meshes, "generated_mesh_references": generated_meshes}
                )

    mismatch_count = len(transform_mismatches) + len(mesh_reference_mismatches)
    warning_count = len(warnings) + len(unsupported_assets)
    blocker_count = len(errors)
    status = "PASS" if blocker_count == 0 and mismatch_count == 0 else ("WARN" if blocker_count == 0 else "FAIL")

    return {
        "status": status,
        "summary": {"warning_count": warning_count, "mismatch_count": mismatch_count, "blocker_count": blocker_count},
        "source_layout_path": source_layout_path,
        "generated_package_path": generated_package_path,
        "scene_builder_parity_action_present": scene_builder_parity_action_present,
        "parity_status_labels_present": parity_status_labels_present,
        "parity_report_filename_contract": parity_report_filename_contract,
        "parity_report_required_fields": list(CANONICAL_REQUIRED_REPORT_FIELDS),
        "layout_asset_ids": layout_asset_ids,
        "generated_asset_ids": generated_asset_ids,
        "layout_asset_id_set": sorted(set(layout_asset_ids)),
        "generated_asset_id_set": sorted(set(generated_asset_ids)),
        "assets_missing_from_generated_output": assets_missing_from_generated_output,
        "generated_assets_not_in_layout": generated_assets_not_in_layout,
        "transform_mismatches": transform_mismatches,
        "mesh_reference_mismatches": mesh_reference_mismatches,
        "unsupported_assets": sorted(set(unsupported_assets)),
        "warning_count": warning_count,
        "mismatch_count": mismatch_count,
        "blocker_count": blocker_count,
        "fake_hardware_default": fake_hardware_default,
        "fake_hardware_token_preserved": fake_hardware_token_preserved,
        "unsupported_asset_warning_contract": unsupported_asset_warning_contract,
        "transform_mismatch_section": transform_mismatch_section,
        "mesh_reference_mismatch_section": mesh_reference_mismatch_section,
        "warnings": warnings,
        "errors": errors,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description="Validate Scene Builder canvas/generated parity contract.")
    parser.add_argument("scene_dir", nargs="?", help="Optional scene directory to perform scene-aware parity checks.")
    parser.add_argument("--json", action="store_true", help="Emit machine-readable JSON report.")
    parser.add_argument("--output", help="Optional JSON report output path.")
    args = parser.parse_args()

    scene_dir = Path(args.scene_dir).expanduser() if args.scene_dir else None
    report = build_report(scene_dir)

    if args.output:
        out = Path(args.output).expanduser()
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text(json.dumps(report, indent=2), encoding="utf-8")

    if args.json:
        print(json.dumps(report, indent=2))
    else:
        print("Scene Builder Canvas/Generated Parity Validation")
        print(report["status"])

    return 1 if report["blocker_count"] > 0 else 0


if __name__ == "__main__":
    sys.exit(main())
