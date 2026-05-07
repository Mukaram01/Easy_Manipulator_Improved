from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

from scripts.capability_registry import DEFAULT_CAPABILITIES_DIR, load_structured_data


@dataclass
class CatalogCapability:
    capability_id: str
    display_name: str
    family: str
    group: str
    source_path: str
    runtime_supported: bool
    preview_only: bool
    warnings: list[str]


def _is_placeholder(item: dict[str, Any]) -> bool:
    cid = str(item.get("id", "")).lower()
    label = str(item.get("label", "")).lower()
    fam = str(item.get("family", "")).lower()
    return any(token in text for token in ("generic", "placeholder", "preview") for text in (cid, label, fam))


def _read_group(path: Path, payload_key: str, group: str) -> tuple[list[CatalogCapability], list[str]]:
    out: list[CatalogCapability] = []
    warnings: list[str] = []
    for doc_path in sorted(path.glob("*.yaml")):
        try:
            doc, _ = load_structured_data(doc_path)
        except Exception as exc:
            warnings.append(f"{doc_path.name}: failed to load ({exc})")
            continue
        item = doc.get(payload_key, {}) if isinstance(doc.get(payload_key), dict) else {}
        cap_id = item.get("id")
        if not isinstance(cap_id, str) and payload_key == "task":
            cap_id = item.get("task_family")
        if not isinstance(cap_id, str) or not cap_id.strip():
            warnings.append(f"{doc_path.name}: missing capability id")
            continue
        label = item.get("label") if isinstance(item.get("label"), str) else cap_id
        family = str(item.get("family") or item.get("task_family") or "unknown")
        item_warnings = [str(v) for v in (item.get("limitations_warnings") or []) if isinstance(v, str)]
        preview_only = _is_placeholder(item)
        if preview_only:
            item_warnings.append("Preview-only placeholder: runtime launch/motion support is not claimed.")
        out.append(CatalogCapability(capability_id=cap_id, display_name=label, family=family, group=group, source_path=str(doc_path), runtime_supported=not preview_only, preview_only=preview_only, warnings=item_warnings))
    return out, warnings


def load_workcell_capability_catalog(catalog_dir: Path | None = None) -> dict[str, Any]:
    base = (catalog_dir or DEFAULT_CAPABILITIES_DIR).resolve()
    grouped = {
        "robots": ("robots", "robot"),
        "end_effectors": ("end_effectors", "end_effector"),
        "sensors": ("sensors", "sensor"),
        "environment_assets": ("environment_assets", "asset"),
        "task_templates": ("tasks", "task"),
    }
    payload: dict[str, Any] = {"catalog_path": str(base), "warnings": []}
    for key, (folder, item_key) in grouped.items():
        entries, warns = _read_group(base / folder, item_key, key)
        payload[key] = [entry.__dict__ for entry in entries]
        payload["warnings"].extend(warns)
    return payload
