#!/usr/bin/env python3
"""Offline helper for loading and resolving capability contract fixtures."""

from __future__ import annotations

import json
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Any

REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CAPABILITIES_DIR = REPO_ROOT / "catalog" / "capabilities"
FIXTURE_CAPABILITIES_DIR = REPO_ROOT / "tests" / "fixtures" / "capabilities"

try:  # Optional dependency
    import yaml as _pyyaml
except Exception:  # pragma: no cover
    _pyyaml = None


class SimpleYamlError(ValueError):
    """Raised when fallback parser cannot decode a YAML file."""


def _strip_comment(value: str) -> str:
    quote: str | None = None
    for idx, ch in enumerate(value):
        if ch in {'"', "'"}:
            quote = ch if quote is None else (None if quote == ch else quote)
        if ch == "#" and quote is None and idx > 0 and value[idx - 1].isspace():
            return value[:idx].rstrip()
    return value.rstrip()


def _parse_scalar(value: str, line_no: int) -> Any:
    if (value.startswith('"') and value.endswith('"')) or (value.startswith("'") and value.endswith("'")):
        return value[1:-1]
    lowered = value.lower()
    if lowered == "true":
        return True
    if lowered == "false":
        return False
    if lowered == "null":
        return None
    if re.fullmatch(r"[+-]?\d+", value):
        return int(value)
    if re.fullmatch(r"[+-]?(?:\d+\.\d*|\d*\.\d+)(?:[eE][+-]?\d+)?", value) or re.fullmatch(
        r"[+-]?\d+[eE][+-]?\d+", value
    ):
        return float(value)
    if value.startswith("["):
        if not value.endswith("]"):
            raise SimpleYamlError(f"Line {line_no}: malformed inline list")
        inner = value[1:-1].strip()
        if not inner:
            return []
        return [_parse_scalar(part.strip(), line_no) for part in inner.split(",") if part.strip()]
    return value


def _tokenize_yaml(text: str) -> list[tuple[int, int, str]]:
    tokens: list[tuple[int, int, str]] = []
    for idx, raw in enumerate(text.splitlines(), start=1):
        if not raw.strip() or raw.lstrip().startswith("#"):
            continue
        if "\t" in raw:
            raise SimpleYamlError(f"Line {idx}: tabs are not supported")
        indent = len(raw) - len(raw.lstrip(" "))
        content = _strip_comment(raw[indent:]).strip()
        if content:
            tokens.append((idx, indent, content))
    return tokens


def _parse_mapping(tokens: list[tuple[int, int, str]], start: int, indent: int) -> tuple[dict[str, Any], int]:
    data: dict[str, Any] = {}
    i = start
    while i < len(tokens):
        line_no, current_indent, content = tokens[i]
        if current_indent < indent:
            break
        if current_indent > indent or content.startswith("- ") or ":" not in content:
            raise SimpleYamlError(f"Line {line_no}: invalid mapping structure")
        key, remainder = content.split(":", 1)
        key = key.strip()
        remainder = remainder.strip()
        i += 1
        if remainder:
            data[key] = _parse_scalar(remainder, line_no)
            continue
        if i >= len(tokens) or tokens[i][1] <= current_indent:
            data[key] = {}
            continue
        child_indent = tokens[i][1]
        if tokens[i][2].startswith("- "):
            parsed, i = _parse_list(tokens, i, child_indent)
        else:
            parsed, i = _parse_mapping(tokens, i, child_indent)
        data[key] = parsed
    return data, i


def _parse_list(tokens: list[tuple[int, int, str]], start: int, indent: int) -> tuple[list[Any], int]:
    out: list[Any] = []
    i = start
    while i < len(tokens):
        line_no, current_indent, content = tokens[i]
        if current_indent != indent or not content.startswith("- "):
            break
        payload = content[2:].strip()
        i += 1
        if not payload:
            raise SimpleYamlError(f"Line {line_no}: empty list item unsupported")
        if payload.endswith(":") or (":" in payload and not payload.startswith(("'", '"'))):
            end = i
            while end < len(tokens) and tokens[end][1] > current_indent:
                end += 1
            synthetic = [(line_no, indent + 2, payload)] + tokens[i:end]
            parsed, consumed = _parse_mapping(synthetic, 0, indent + 2)
            if consumed != len(synthetic):
                raise SimpleYamlError(f"Line {line_no}: unsupported list mapping structure")
            out.append(parsed)
            i = end
        else:
            out.append(_parse_scalar(payload, line_no))
    return out, i


def parse_yaml_fallback(text: str) -> dict[str, Any]:
    tokens = _tokenize_yaml(text)
    if not tokens:
        return {}
    root_indent = min(item[1] for item in tokens)
    parsed, consumed = _parse_mapping(tokens, 0, root_indent)
    if consumed != len(tokens):
        raise SimpleYamlError(f"Line {tokens[consumed][0]}: unsupported YAML structure")
    return parsed


def load_structured_data(path: Path) -> tuple[dict[str, Any], str]:
    text = path.read_text(encoding="utf-8")
    if _pyyaml is not None:
        loaded = _pyyaml.safe_load(text)
        if loaded is None:
            return {}, "pyyaml"
        if not isinstance(loaded, dict):
            raise ValueError("Top-level document must be a mapping/object")
        return loaded, "pyyaml"
    try:
        loaded = json.loads(text)
        if not isinstance(loaded, dict):
            raise ValueError("Top-level document must be a mapping/object")
        return loaded, "json"
    except Exception:
        loaded = parse_yaml_fallback(text)
        if not isinstance(loaded, dict):
            raise ValueError("Top-level document must be a mapping/object")
        return loaded, "fallback"


@dataclass
class CapabilityRecord:
    capability_id: str
    schema_version: str
    family: str | None
    metadata: dict[str, Any]
    payload: dict[str, Any]
    path: Path


class CapabilityRegistry:
    def __init__(self, records: dict[str, CapabilityRecord], parser_notes: list[str] | None = None) -> None:
        self._records = records
        self.parser_notes = parser_notes or []

    def get(self, capability_id: str | None) -> CapabilityRecord | None:
        if not capability_id:
            return None
        return self._records.get(capability_id)

    def ids(self) -> list[str]:
        return sorted(self._records.keys())


def _extract_capability_id_and_payload(doc: dict[str, Any]) -> tuple[str | None, dict[str, Any], str | None]:
    schema = str(doc.get("schema_version", ""))
    if "/" not in schema:
        return None, {}, None
    root_key = schema.split("/", 1)[0].replace("_capability", "")
    payload = doc.get(root_key)
    if not isinstance(payload, dict):
        payload = doc.get("asset") if root_key == "environment_asset" else {}
    if not isinstance(payload, dict):
        payload = {}
    cap_id = payload.get("id") if isinstance(payload.get("id"), str) else None
    if not cap_id and schema.startswith("task_capability/") and isinstance(payload.get("task_family"), str):
        cap_id = payload.get("task_family")
    family = payload.get("family") if isinstance(payload.get("family"), str) else None
    if not family and schema.startswith("task_capability/") and isinstance(payload.get("task_family"), str):
        family = payload.get("task_family")
    return cap_id, payload, family


def load_capability_registry(capabilities_dir: Path | None = None) -> CapabilityRegistry:
    cap_dir = (capabilities_dir or DEFAULT_CAPABILITIES_DIR).resolve()
    records: dict[str, CapabilityRecord] = {}
    notes: list[str] = []
    if not cap_dir.is_dir():
        notes.append(f"Capability directory not found: {cap_dir}")
        return CapabilityRegistry(records, parser_notes=notes)

    for path in sorted(cap_dir.rglob("*")):
        if path.suffix.lower() not in {".yaml", ".yml", ".json"} or not path.is_file():
            continue
        try:
            doc, parser = load_structured_data(path)
            if parser != "pyyaml":
                notes.append(f"{path.name}: loaded with {parser} parser")
            cap_id, payload, family = _extract_capability_id_and_payload(doc)
            if not cap_id:
                notes.append(f"{path.name}: skipped (missing capability id)")
                continue
            if cap_id in records:
                prev = records[cap_id].path.relative_to(cap_dir) if records[cap_id].path.is_relative_to(cap_dir) else records[cap_id].path
                curr = path.relative_to(cap_dir) if path.is_relative_to(cap_dir) else path
                notes.append(f"duplicate capability id '{cap_id}': {prev} overwritten by {curr}")

            records[cap_id] = CapabilityRecord(
                capability_id=cap_id,
                schema_version=str(doc.get("schema_version", "")),
                family=family,
                metadata={"schema_version": doc.get("schema_version"), "family": family},
                payload=payload,
                path=path,
            )
        except Exception as exc:
            notes.append(f"{path.name}: load failed ({exc})")

    return CapabilityRegistry(records, parser_notes=notes)
