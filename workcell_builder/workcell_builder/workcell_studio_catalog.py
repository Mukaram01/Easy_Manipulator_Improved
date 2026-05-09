from __future__ import annotations

from pathlib import Path
from typing import Any
import yaml

REQUIRED = {"id", "label", "family"}
GROUPS = {
    "robots": ("catalog/capabilities/robots", "robot"),
    "end_effectors": ("catalog/capabilities/end_effectors", "end_effector"),
    "sensors": ("catalog/capabilities/sensors", "sensor"),
    "environment_assets": ("catalog/capabilities/environment_assets", "asset"),
    "tasks": ("catalog/capabilities/tasks", "task"),
}


def load_workcell_studio_catalog(root: Path | None = None) -> dict[str, Any]:
    base = (root or Path(__file__).resolve().parents[2]).resolve()
    out: dict[str, Any] = {"warnings": [], "catalog_root": str(base / "catalog" / "capabilities")}
    for group, (rel, key) in GROUPS.items():
        entries: list[dict[str, Any]] = []
        for path in sorted((base / rel).glob("*.yaml")):
            payload = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
            item = payload.get(key) if isinstance(payload.get(key), dict) else {}
            if not item:
                out["warnings"].append(f"{path.name}: missing {key}")
                continue
            missing = [f for f in REQUIRED if not item.get(f)]
            if missing:
                out["warnings"].append(f"{path.name}: missing required fields {','.join(missing)}")
                continue
            item = dict(item)
            item.setdefault("runtime_status", "supported")
            item.setdefault("support_status", "supported")
            item.setdefault("status_notes", [])
            item["source"] = str(path.relative_to(base))
            entries.append(item)
        out[group] = entries
    return out
