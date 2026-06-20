#!/usr/bin/env python3
"""Audit conservative Workcell Studio cleanup candidates without deleting files."""
from __future__ import annotations

import argparse
import fnmatch
import json
import re
from collections import defaultdict
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Iterable

DEFAULT_OUTPUT_DIR = Path("build/workcell_cleanup_audit")
DEFAULT_JSON_NAME = "cleanup_candidates.json"
DEFAULT_MD_NAME = "cleanup_candidates.md"
SKIP_DIR_NAMES = {".git", ".mypy_cache", ".ruff_cache", "node_modules", "install", "log", "build"}
MAX_TEXT_BYTES = 1_000_000
TEXT_SUFFIXES = {".py", ".md", ".rst", ".txt", ".yaml", ".yml", ".json", ".xml", ".launch", ".html", ".ui", ".qss", ".js", ".ts", ".cpp", ".hpp", ".h", ".c", ".cmake"}
CACHE_DIR_NAMES = {"__pycache__", ".pytest_cache"}
LOG_SUFFIXES = (".log", ".stdout.log", ".stderr.log")
SMOKE_PATTERNS = ("scene3d_gui_smoke*.json", "scene3d_gui_smoke*.png", "scene3d_gui_smoke*.log")
BACKUP_PATTERNS = ("*.bak", "*.old", "*~")
DOC_INDEX_NAMES = {"readme.md", "readme.rst", "readme.txt", "index.md", "index.rst", "toc.md", "contents.md"}

DANGEROUS_NAME_PATTERNS = {
    "scene_manifest.yaml",
    "cell_definition.yaml",
    "environment.yaml",
    "workcell_studio_layout.yaml",
    "workcell_studio_layout.generated.yaml",
    "package.xml",
    "CMakeLists.txt",
}
DANGEROUS_SUFFIXES = {".urdf", ".xacro", ".stl", ".dae", ".obj", ".launch.py", ".rviz"}
DANGEROUS_PATH_TOKENS = {
    "robot", "robots", "mesh", "meshes", "urdf", "xacro", "launch", "safety", "gate",
    "controller", "controllers", "moveit", "runtime", "epd", "realsense", "perception", "layout",
}
UI_DEAD_ACTION_RE = re.compile(r"(?i)(todo|placeholder|not implemented|coming soon|no[-_ ]?op|pass\s*(?:#.*)?$|stub)")
UI_ACTION_CONTEXT_RE = re.compile(r"(?i)(button|action|slot|clicked|triggered|connect|qpushbutton|qaction)")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Audit cleanup candidates in a Workcell Studio repo. This tool never deletes files.")
    parser.add_argument("--repo-root", type=Path, default=None, help="Repository root. Defaults to the parent of this script directory.")
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR)
    parser.add_argument("--json-output", type=Path, default=None)
    parser.add_argument("--markdown-output", type=Path, default=None)
    return parser.parse_args()


def resolve_repo_root(repo_root_arg: Path | None) -> Path:
    if repo_root_arg is not None:
        return repo_root_arg.expanduser().resolve()
    return Path(__file__).resolve().parents[1]


def iter_paths(root: Path) -> Iterable[Path]:
    for path in root.rglob("*"):
        rel_parts = path.relative_to(root).parts
        if any(part in SKIP_DIR_NAMES for part in rel_parts[:-1]):
            continue
        yield path


def rel(path: Path, root: Path) -> str:
    return path.relative_to(root).as_posix()


def is_dangerous_required_asset(path: Path, root: Path) -> tuple[bool, str]:
    r = rel(path, root)
    lower = r.lower()
    name = path.name
    if name in DANGEROUS_NAME_PATTERNS:
        return True, "required Workcell Studio scene/package contract file"
    if any(lower.endswith(s) for s in DANGEROUS_SUFFIXES):
        return True, "robot, mesh, URDF/Xacro, launch, or RViz asset"
    if any(token in lower.split("/") for token in DANGEROUS_PATH_TOKENS) and path.suffix.lower() in {".yaml", ".yml", ".json", ".py"}:
        return True, "runtime, safety, launch, EPD bridge, layout, or robot metadata path"
    return False, ""


def record(path: Path, root: Path, category: str, reason: str, confidence: str, safe: bool, blocker: str | None = None) -> dict[str, Any]:
    data: dict[str, Any] = {
        "path": rel(path, root),
        "category": category,
        "reason": reason,
        "confidence": confidence,
        "safe_to_delete_now": safe,
    }
    if not safe:
        data["blocker_note"] = blocker or "Conservative audit requires human review before deletion."
    return data


def read_text(path: Path) -> str:
    try:
        if path.stat().st_size > MAX_TEXT_BYTES:
            return ""
        return path.read_text(encoding="utf-8", errors="ignore")
    except OSError:
        return ""


def build_reference_corpus(root: Path, files: list[Path], exclude: Path | None = None, docs_only: bool = False) -> str:
    chunks: list[str] = []
    for p in files:
        if p == exclude or p.suffix.lower() not in TEXT_SUFFIXES:
            continue
        r = rel(p, root)
        if docs_only and p.name.lower() not in DOC_INDEX_NAMES:
            continue
        chunks.append(read_text(p))
        chunks.append(r)
    return "\n".join(chunks).lower()


def referenced(path: Path, root: Path, corpus: str) -> bool:
    r = rel(path, root).lower()
    name = path.name.lower()
    # The corpus includes each file path once as metadata, so require a second hit.
    return corpus.count(r) > 1 or corpus.count(name) > 1


def generator_reference_found(path: Path, root: Path, corpus: str) -> bool:
    return "scene3d_gui_smoke" in corpus or "readiness" in corpus or path.name.lower() in corpus


def is_readiness_report(path: Path, root: Path) -> bool:
    r = rel(path, root).lower()
    if not (r.startswith("build/") or "/generated/" in r or r.startswith("generated/")):
        return False
    return "readiness" in r and path.suffix.lower() in {".json", ".md", ".html", ".txt", ".log"}


def scan_ui_dead_actions(root: Path) -> list[dict[str, Any]]:
    candidates: list[dict[str, Any]] = []
    for p in iter_paths(root):
        if not p.is_file() or p.suffix.lower() not in {".py", ".ui", ".qml", ".cpp", ".hpp"}:
            continue
        text = read_text(p)
        for i, line in enumerate(text.splitlines(), start=1):
            if UI_DEAD_ACTION_RE.search(line) and UI_ACTION_CONTEXT_RE.search(line):
                candidates.append({
                    "path": rel(p, root),
                    "category": "possible_dead_ui_actions",
                    "reason": f"Static search found placeholder/unconnected UI action marker on line {i}: {line.strip()[:160]}",
                    "confidence": "low",
                    "safe_to_delete_now": False,
                    "blocker_note": "Static UI string search is only a triage hint; inspect the action wiring and user flow before changing or deleting code.",
                })
    return candidates


def audit(root: Path) -> dict[str, Any]:
    all_paths = list(iter_paths(root))
    files = [p for p in all_paths if p.is_file()]
    dirs = [p for p in all_paths if p.is_dir()]
    corpus = build_reference_corpus(root, files)
    doc_index_corpus = build_reference_corpus(root, files, docs_only=True)
    seen: set[tuple[str, str]] = set()
    candidates: list[dict[str, Any]] = []

    def add(item: dict[str, Any]) -> None:
        key = (item["path"], item["category"])
        if key not in seen:
            seen.add(key)
            candidates.append(item)

    for p in dirs:
        if p.name in CACHE_DIR_NAMES:
            add(record(p, root, "python_cache", "Generated Python/test cache directory.", "high", True))

    for p in files:
        dangerous, note = is_dangerous_required_asset(p, root)
        if dangerous:
            add(record(p, root, "dangerous_do_not_delete", note, "low", False, "Required Workcell Studio assets are explicitly excluded from cleanup authorization."))
            continue
        lower_name = p.name.lower()
        if p.suffix == ".pyc":
            add(record(p, root, "python_cache", "Generated Python bytecode cache file.", "high", True))
        elif any(lower_name.endswith(s) for s in LOG_SUFFIXES):
            add(record(p, root, "generated_logs", "Generated log file; audit does not delete it.", "high", True))
        elif any(fnmatch.fnmatch(p.name, pat) for pat in SMOKE_PATTERNS):
            conf = "medium" if generator_reference_found(p, root, corpus) else "low"
            add(record(p, root, "generated_smoke_outputs", "Scene3D GUI smoke artifact that can be regenerated by smoke validation commands.", conf, conf == "medium", None if conf == "medium" else "Generator reference not found; verify provenance before deletion."))
        elif is_readiness_report(p, root):
            conf = "medium" if generator_reference_found(p, root, corpus) else "low"
            add(record(p, root, "readiness_reports", "Generated readiness report under build/generated output paths.", conf, conf == "medium", None if conf == "medium" else "Could not confirm generator reference; review provenance before deletion."))
        elif any(fnmatch.fnmatch(p.name, pat) for pat in BACKUP_PATTERNS):
            add(record(p, root, "temp_backup_files", "Temporary editor/backup file with no recognized runtime contract role.", "high", True))

    for p in root.joinpath("scripts").glob("*.py") if (root / "scripts").exists() else []:
        dangerous, note = is_dangerous_required_asset(p, root)
        if dangerous:
            add(record(p, root, "dangerous_do_not_delete", note, "low", False, "Safety/runtime scripts are not cleanup candidates."))
        elif not referenced(p, root, corpus):
            add(record(p, root, "possible_orphan_scripts", "Script is not referenced by docs, tests, or other text-searchable scripts.", "low", False, "Static reference analysis can miss dynamic CLI use, CI jobs, and external documentation."))

    docs_root = root / "docs"
    if docs_root.exists():
        for p in docs_root.rglob("*.md"):
            dangerous, note = is_dangerous_required_asset(p, root)
            if dangerous:
                add(record(p, root, "dangerous_do_not_delete", note, "low", False, "Required docs/metadata are not cleanup candidates."))
            elif p.name.lower() not in DOC_INDEX_NAMES and not referenced(p, root, doc_index_corpus):
                add(record(p, root, "possible_orphan_docs", "Doc is not referenced from README or docs index files.", "low", False, "Static documentation reference analysis can miss external links or generated navigation."))

    for item in scan_ui_dead_actions(root):
        add(item)

    candidates.sort(key=lambda x: (x["category"], x["path"], x["reason"]))
    return {
        "schema": "workcell_cleanup_candidates/v1",
        "repo_root": str(root),
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "deletes_anything": False,
        "warning": "This report is not authorization to delete low-confidence or dangerous files. It is a conservative triage list only.",
        "candidate_count": len(candidates),
        "candidates": candidates,
    }


def markdown(payload: dict[str, Any]) -> str:
    grouped: dict[str, list[dict[str, Any]]] = defaultdict(list)
    for c in payload["candidates"]:
        grouped[c["category"]].append(c)
    lines = [
        "# Workcell Cleanup Candidate Audit",
        "",
        "⚠️ This report is not authorization to delete low-confidence or dangerous files. The audit scans only and exposes no deletion option.",
        "",
        f"- Schema: `{payload['schema']}`",
        f"- Repository root: `{payload['repo_root']}`",
        f"- Generated at: `{payload['generated_at']}`",
        f"- Candidate records: {payload['candidate_count']}",
        "",
    ]
    for category in sorted(grouped):
        lines.extend([f"## {category}", "", "| Path | Confidence | Safe now | Reason | Blocker |", "| --- | --- | --- | --- | --- |"])
        for c in grouped[category]:
            blocker = c.get("blocker_note", "")
            lines.append(f"| `{c['path']}` | {c['confidence']} | {str(c['safe_to_delete_now']).lower()} | {c['reason'].replace('|','/')} | {blocker.replace('|','/')} |")
        lines.append("")
    return "\n".join(lines) + "\n"


def main() -> int:
    args = parse_args()
    root = resolve_repo_root(args.repo_root)
    output_dir = args.output_dir if args.output_dir.is_absolute() else root / args.output_dir
    json_output = args.json_output or output_dir / DEFAULT_JSON_NAME
    md_output = args.markdown_output or output_dir / DEFAULT_MD_NAME
    if not json_output.is_absolute():
        json_output = root / json_output
    if not md_output.is_absolute():
        md_output = root / md_output
    payload = audit(root)
    json_output.parent.mkdir(parents=True, exist_ok=True)
    md_output.parent.mkdir(parents=True, exist_ok=True)
    json_output.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
    md_output.write_text(markdown(payload), encoding="utf-8")
    print(json.dumps({"json_output": str(json_output), "markdown_output": str(md_output), "candidate_count": payload["candidate_count"], "deletes_anything": False}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
