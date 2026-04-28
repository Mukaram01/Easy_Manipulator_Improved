#!/usr/bin/env python3
"""Validate detected_objects/v1 snapshots for offline task pipeline usage."""

from __future__ import annotations

import argparse
import hashlib
import json
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any

SCRIPTS_DIR = Path(__file__).resolve().parent
if str(SCRIPTS_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_DIR))

import validate_cell_definition as cell_yaml

KNOWN_COLOURS = {"red", "green", "blue", "yellow", "black", "white", "clear", "silver", "brown", "orange"}
KNOWN_SHAPES = {"box", "cylinder", "sphere", "irregular", "unknown"}
KNOWN_CLASSES = {"mouse", "box", "part", "bottle", "can", "paper", "plastic", "metal", "unknown"}


@dataclass
class ValidationResult:
    status: str
    warnings: list[str]
    errors: list[str]
    notes: list[str]
    payload: dict[str, Any]


def _load_yaml_or_json(path: Path) -> tuple[dict[str, Any], str, list[str]]:
    if path.suffix.lower() == ".json":
        loaded = json.loads(path.read_text(encoding="utf-8"))
        if loaded is None:
            loaded = {}
        if not isinstance(loaded, dict):
            raise ValueError("JSON root must be an object/mapping.")
        return loaded, "json", []
    return cell_yaml.load_yaml(path)


def _stable_object_id(obj: dict[str, Any], index: int) -> str:
    base = {
        "name": obj.get("name"),
        "class_id": obj.get("class_id"),
        "centroid": obj.get("centroid"),
        "pose": obj.get("pose"),
    }
    digest = hashlib.sha1(json.dumps(base, sort_keys=True, default=str).encode("utf-8")).hexdigest()[:8]
    return f"obj_{index:03d}_{digest}"


def _normalize_dimensions(raw: Any) -> tuple[dict[str, float] | None, str | None]:
    if raw is None:
        return None, None
    if isinstance(raw, list) and len(raw) == 3:
        dims = {"x": float(raw[0]), "y": float(raw[1]), "z": float(raw[2])}
        return dims, None
    if isinstance(raw, dict) and all(k in raw for k in ("x", "y", "z")):
        dims = {"x": float(raw["x"]), "y": float(raw["y"]), "z": float(raw["z"])}
        return dims, None
    return None, "dimensions must be a 3-list or mapping with x/y/z"


def validate_detected_objects(doc: dict[str, Any], strict: bool, allow_generate_ids: bool) -> ValidationResult:
    warnings: list[str] = []
    errors: list[str] = []
    notes: list[str] = []

    if doc.get("schema_version") != "detected_objects/v1":
        errors.append("schema_version must be detected_objects/v1")

    source = doc.get("source") if isinstance(doc.get("source"), dict) else {}
    if not source:
        warnings.append("source block is missing or empty")

    objects = doc.get("objects") if isinstance(doc.get("objects"), list) else None
    if objects is None:
        errors.append("objects must be a list")
        objects = []

    normalized: list[dict[str, Any]] = []
    seen: set[str] = set()
    for idx, raw in enumerate(objects, start=1):
        if not isinstance(raw, dict):
            errors.append(f"objects[{idx-1}] must be a mapping")
            continue

        obj = dict(raw)
        object_id = obj.get("object_id")
        if not isinstance(object_id, str) or not object_id.strip():
            if allow_generate_ids:
                object_id = _stable_object_id(obj, idx)
                notes.append(f"Generated stable object_id '{object_id}' for objects[{idx-1}].")
            else:
                errors.append(f"objects[{idx-1}] missing required object_id")
                continue
        object_id = object_id.strip()
        if object_id in seen:
            errors.append(f"Duplicate object_id '{object_id}'")
            continue
        seen.add(object_id)

        pose = obj.get("pose") if isinstance(obj.get("pose"), dict) else None
        if pose is None:
            errors.append(f"Object '{object_id}' is missing pose")
        else:
            xyz = pose.get("xyz")
            if not (isinstance(xyz, list) and len(xyz) == 3 and all(isinstance(v, (int, float)) for v in xyz)):
                errors.append(f"Object '{object_id}' pose.xyz must be numeric length-3 list")
            rpy = pose.get("rpy")
            if not (isinstance(rpy, list) and len(rpy) == 3 and all(isinstance(v, (int, float)) for v in rpy)):
                errors.append(f"Object '{object_id}' pose.rpy must be numeric length-3 list")
            if not isinstance(pose.get("frame_id"), str) or not str(pose.get("frame_id")).strip():
                warnings.append(f"Object '{object_id}' pose.frame_id is missing/empty")

        dims, dim_error = _normalize_dimensions(obj.get("dimensions"))
        if dim_error:
            errors.append(f"Object '{object_id}' {dim_error}")
        if dims is None:
            if strict:
                errors.append(f"Object '{object_id}' is missing dimensions in strict mode")
            else:
                warnings.append(f"Object '{object_id}' missing dimensions")
        else:
            for axis, value in dims.items():
                if value <= 0:
                    errors.append(f"Object '{object_id}' dimensions.{axis} must be > 0")

        conf = obj.get("confidence")
        if conf is None:
            warnings.append(f"Object '{object_id}' missing confidence")
        elif not isinstance(conf, (int, float)):
            errors.append(f"Object '{object_id}' confidence must be numeric")

        attributes = obj.get("attributes") if isinstance(obj.get("attributes"), dict) else {}
        colour = attributes.get("colour", obj.get("colour", "unknown"))
        shape = attributes.get("shape", obj.get("shape", "unknown"))
        class_id = obj.get("class_id") or obj.get("name")

        if isinstance(colour, str) and colour.lower() not in KNOWN_COLOURS:
            warnings.append(f"Object '{object_id}' has unknown colour '{colour}'")
        if isinstance(shape, str) and shape.lower() not in KNOWN_SHAPES:
            warnings.append(f"Object '{object_id}' has unknown shape '{shape}'")
        if isinstance(class_id, str) and class_id.lower() not in KNOWN_CLASSES:
            warnings.append(f"Object '{object_id}' has unknown class_id '{class_id}'")

        obj["object_id"] = object_id
        normalized.append(obj)

    status = "PASS"
    if errors:
        status = "FAIL"
    elif warnings:
        status = "WARN"

    return ValidationResult(
        status=status,
        warnings=warnings,
        errors=errors,
        notes=notes,
        payload={
            "schema_version": "detected_objects_validation/v1",
            "status": status,
            "summary": {
                "objects_seen": len(objects),
                "objects_validated": len(normalized),
                "warnings": len(warnings),
                "errors": len(errors),
            },
            "warnings": warnings,
            "errors": errors,
            "notes": notes,
            "normalized": {"schema_version": "detected_objects/v1", "source": source, "objects": normalized},
        },
    )


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path, help="Path to detected_objects/v1 YAML/JSON")
    parser.add_argument("--json", action="store_true", help="Print JSON output")
    parser.add_argument("--strict", action="store_true", help="Treat missing dimensions as failure")
    parser.add_argument(
        "--allow-generate-object-id",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Allow generating stable object ids when missing",
    )
    args = parser.parse_args(argv)

    try:
        loaded, parser_name, parser_notes = _load_yaml_or_json(args.input)
        result = validate_detected_objects(loaded, strict=args.strict, allow_generate_ids=bool(args.allow_generate_object_id))
        result.payload["input"] = str(args.input)
        result.payload["parser"] = parser_name
        result.payload.setdefault("notes", []).extend(parser_notes)
        rendered = json.dumps(result.payload, indent=2, sort_keys=True)
        print(rendered)
        return 1 if result.status == "FAIL" else 0
    except Exception as exc:
        print(json.dumps({"schema_version": "detected_objects_validation/v1", "status": "FAIL", "error": str(exc)}, indent=2))
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
