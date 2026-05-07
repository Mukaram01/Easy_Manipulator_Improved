from __future__ import annotations

from dataclasses import dataclass, asdict
from pathlib import Path
from typing import Any

import yaml

VALID_RUNTIME_STATUS = {"supported", "fake_hardware_only", "preview_only", "unsupported", "placeholder"}


def catalog_path(root: Path | None = None) -> Path:
    base = root or Path(__file__).resolve().parents[2]
    return base / "workcell_studio_catalog" / "catalog.yaml"


def load_catalog(root: Path | None = None) -> dict[str, Any]:
    payload = yaml.safe_load(catalog_path(root).read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        return {"items": []}
    return payload


def validate_catalog(payload: dict[str, Any], root: Path | None = None) -> list[str]:
    errors: list[str] = []
    base = root or Path(__file__).resolve().parents[2]
    for idx, item in enumerate(payload.get("items", [])):
        for req in ("id", "display_name", "category", "support_status", "runtime_status"):
            if not item.get(req):
                errors.append(f"item[{idx}] missing {req}")
        if item.get("runtime_status") not in VALID_RUNTIME_STATUS:
            errors.append(f"item[{idx}] invalid runtime_status")
        if item.get("preview_mesh_path") and not (base / item["preview_mesh_path"]).exists():
            errors.append(f"item[{idx}] preview mesh missing")
        if item.get("category") == "robots" and item.get("support_status") == "supported" and item.get("id") != "ur5":
            errors.append(f"item[{idx}] unsupported robot marked supported")
    return errors


def grouped_selection(payload: dict[str, Any]) -> dict[str, list[dict[str, Any]]]:
    groups = {
        "Robots": [], "End Effectors": [], "Tools": [], "Sensors": [], "Tables": [], "Bins": [],
        "Conveyors": [], "Fixtures": [], "Machines": [], "Safety": [], "Objects": [], "Custom STL": []
    }
    for item in payload.get("items", []):
        c = item.get("category", "")
        if c == "robots": groups["Robots"].append(item)
        elif c == "end_effectors": groups["End Effectors"].append(item)
        elif c == "tools": groups["Tools"].append(item)
        elif c == "sensors": groups["Sensors"].append(item)
        elif c == "conveyors": groups["Conveyors"].append(item)
        elif c == "fixtures": groups["Fixtures"].append(item)
        elif c == "machines": groups["Machines"].append(item)
        elif c == "safety": groups["Safety"].append(item)
        elif c == "objects": groups["Objects"].append(item)
        elif c == "environment":
            iid = item.get("id", "")
            if "table" in iid or "bench" in iid: groups["Tables"].append(item)
            elif "bin" in iid or "box" in iid or "tote" in iid: groups["Bins"].append(item)
    return groups


@dataclass
class CustomStlMetadata:
    file_path: str
    display_name: str
    category: str = "custom"
    scale: list[float] | None = None
    xyz: list[float] | None = None
    rpy: list[float] | None = None
    collision_enabled: bool = True
    visual_only: bool = False
    notes: str = ""

    def to_dict(self) -> dict[str, Any]:
        data = asdict(self)
        data.setdefault("scale", [1.0, 1.0, 1.0])
        data.setdefault("xyz", [0.0, 0.0, 0.0])
        data.setdefault("rpy", [0.0, 0.0, 0.0])
        return data

    @classmethod
    def from_dict(cls, payload: dict[str, Any]) -> "CustomStlMetadata":
        return cls(**payload)
