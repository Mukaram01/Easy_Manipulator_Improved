from __future__ import annotations

from dataclasses import dataclass, asdict, field
from pathlib import Path
from typing import Any

VALID_CUSTOM_STL_CATEGORIES = {"object", "environment_asset", "fixture", "machine", "bin", "conveyor_visual", "tool_visual", "safety_visual", "custom_visual"}
VALID_CUSTOM_STL_EXTENSIONS = {".stl", ".dae", ".obj"}
VALID_CUSTOM_STL_COLLISION_MODES = {"visual_only", "bounding_box_collision", "mesh_collision"}

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
        "Conveyors": [], "Fixtures": [], "Machines": [], "Safety / Fencing": [], "Camera Mounts": [], "Robot Bases": [], "Pick Objects": [], "Objects": [], "Custom STL": []
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
        elif c == "safety": groups["Safety / Fencing"].append(item)
        elif c == "objects": groups["Pick Objects"].append(item)
        elif c == "environment":
            iid = item.get("id", "")
            if "table" in iid or "bench" in iid: groups["Tables"].append(item)
            elif "bin" in iid or "box" in iid or "tote" in iid or "tray" in iid: groups["Bins"].append(item)
            elif "conveyor" in iid: groups["Conveyors"].append(item)
            elif "fixture" in iid or "pallet" in iid: groups["Fixtures"].append(item)
            elif "fence" in iid or "safety" in iid: groups["Safety / Fencing"].append(item)
            elif "camera" in iid: groups["Camera Mounts"].append(item)
            elif "robot_base" in iid or "pedestal" in iid: groups["Robot Bases"].append(item)
            elif "pick" in iid or "object" in iid or "cube" in iid or "cylinder" in iid: groups["Pick Objects"].append(item)
    return groups


@dataclass
class CustomStlMetadata:
    source_path: str
    display_name: str
    category: str = "custom_visual"
    copied_asset_path: str | None = None
    scale: list[float] = field(default_factory=lambda: [1.0, 1.0, 1.0])
    xyz: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    rpy: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    collision_mode: str = "bounding_box_collision"
    support_status: str = "supported"
    notes: str = ""

    def validate(self) -> list[str]:
        errors: list[str] = []
        path = Path(self.source_path)
        if not path.is_file():
            errors.append(f"source_path does not exist: {self.source_path}")
        if path.suffix.lower() not in VALID_CUSTOM_STL_EXTENSIONS:
            errors.append("source_path must have one of extensions: .stl, .dae, .obj")
        if self.category not in VALID_CUSTOM_STL_CATEGORIES:
            errors.append(f"invalid category: {self.category}")
        if self.collision_mode not in VALID_CUSTOM_STL_COLLISION_MODES:
            errors.append(f"invalid collision mode: {self.collision_mode}")
        if len(self.scale) != 3 or any(s <= 0 for s in self.scale):
            errors.append("scale must be a 3-element vector with values > 0")
        if len(self.xyz) != 3:
            errors.append("xyz must contain exactly 3 values")
        if len(self.rpy) != 3:
            errors.append("rpy must contain exactly 3 values")
        return errors

    def _effective_collision_mode(self) -> str:
        if self.collision_mode == "mesh_collision":
            return "bounding_box_collision"
        return self.collision_mode

    def to_dict(self) -> dict[str, Any]:
        data = asdict(self)
        if self.collision_mode == "mesh_collision":
            note = "mesh_collision requested but not supported in lightweight builder; fallback to bounding_box_collision"
            data["support_status"] = "warn"
            data["collision_mode"] = self._effective_collision_mode()
            data["notes"] = f"{self.notes}; {note}" if self.notes else note
        return data

    @classmethod
    def from_dict(cls, payload: dict[str, Any]) -> "CustomStlMetadata":
        remapped = dict(payload)
        # Backward compatibility
        if "source_path" not in remapped and "file_path" in remapped:
            remapped["source_path"] = remapped.pop("file_path")
        if "collision_mode" not in remapped:
            if remapped.pop("visual_only", False):
                remapped["collision_mode"] = "visual_only"
            elif remapped.pop("collision_enabled", True):
                remapped["collision_mode"] = "bounding_box_collision"
        remapped.pop("collision_enabled", None)
        remapped.pop("visual_only", None)
        return cls(**remapped)
