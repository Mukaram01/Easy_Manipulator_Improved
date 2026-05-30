from __future__ import annotations

import re
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]

# Keep this list intentionally narrow: these are Workcell Builder production
# selection/editing and Scene3D candidate/payload assembly files where scene
# behavior must be capability/layer driven instead of catalog-scene-name driven.
PRODUCTION_SELECTION_AND_SCENE3D_PAYLOAD_FILES = [
    ROOT / "workcell_builder/workcell_builder/gui/mainwindow.cpp",
    ROOT / "workcell_builder/workcell_builder/gui/mainwindow.h",
    ROOT / "workcell_builder/workcell_builder/gui/scene_preview_widget.cpp",
    ROOT / "workcell_builder/workcell_builder/gui/scene_preview_widget.h",
    ROOT / "workcell_builder/workcell_builder/gui/scene3d_viewport_widget.cpp",
    ROOT / "workcell_builder/workcell_builder/gui/scene3d_viewport_widget.h",
    ROOT / "workcell_builder/workcell_builder/gui/scene3d_candidate_assembly.cpp",
    ROOT / "workcell_builder/workcell_builder/gui/scene3d_candidate_assembly.h",
    ROOT / "workcell_builder/workcell_builder/src_workcell_studio_canvas_model.cpp",
    ROOT / "workcell_builder/workcell_builder/include/workcell_studio_canvas_model.hpp",
    ROOT / "workcell_builder/workcell_builder/src_scene3d_visual_backend.cpp",
    ROOT / "workcell_builder/workcell_builder/include/scene3d_visual_backend.hpp",
]

SCENE_NAME_IDENTIFIER = r"\b\w*scene_name\w*\b"
QUOTED_LITERAL = r"(?P<quote>[\"'])(?P<literal>[^\"']+)(?P=quote)"

FORBIDDEN_PATTERNS = [
    (
        re.compile(
            rf"(?P<expr>{SCENE_NAME_IDENTIFIER}(?:\s*\.\s*\w+\s*\([^;\n]*?\))?)"
            rf"\s*(?P<op>==|!=)\s*{QUOTED_LITERAL}"
        ),
        False,
    ),
    (
        re.compile(
            rf"{QUOTED_LITERAL}\s*(?P<op>==|!=)\s*"
            rf"(?P<expr>{SCENE_NAME_IDENTIFIER}(?:\s*\.\s*\w+\s*\([^;\n]*?\))?)"
        ),
        False,
    ),
    (
        re.compile(
            rf"(?P<expr>{SCENE_NAME_IDENTIFIER}(?:\s*\.\s*\w+\s*\([^;\n]*?\))?"
            rf"\s*\.\s*contains\s*\(\s*){QUOTED_LITERAL}"
        ),
        True,
    ),
]


def _catalog_scene_names() -> set[str]:
    catalog = yaml.safe_load(
        (ROOT / "scenes/supported_scenes.yaml").read_text(encoding="utf-8")
    )
    return {
        str(entry["scene_name"])
        for entry in catalog.get("scenes", [])
        if "scene_name" in entry
    }


def _strip_cpp_comments_preserving_lines(text: str) -> str:
    """Remove C++ comments so the guard checks production logic, not examples."""
    result: list[str] = []
    i = 0
    in_block = False
    while i < len(text):
        if in_block:
            if text.startswith("*/", i):
                in_block = False
                result.append("  ")
                i += 2
            else:
                result.append("\n" if text[i] == "\n" else " ")
                i += 1
            continue
        if text.startswith("//", i):
            newline = text.find("\n", i)
            if newline == -1:
                result.append(" " * (len(text) - i))
                break
            result.append(" " * (newline - i))
            result.append("\n")
            i = newline + 1
            continue
        if text.startswith("/*", i):
            in_block = True
            result.append("  ")
            i += 2
            continue
        result.append(text[i])
        i += 1
    return "".join(result)


def _line_number(text: str, offset: int) -> int:
    return text.count("\n", 0, offset) + 1


def _is_scene_name_literal(literal: str, catalog_scene_names: set[str]) -> bool:
    if literal in catalog_scene_names:
        return True
    # Catch direct branches on non-catalog/customer scene names that look like
    # generated scene packages while still avoiding generic sentinels such as
    # "none" or UI text that can appear near scene-name variables.
    return bool(
        re.fullmatch(
            r"[A-Za-z][A-Za-z0-9]*(?:_[A-Za-z0-9]+)*_(?:test|demo|scene)",
            literal,
        )
    )


def test_production_scene3d_selection_logic_has_no_direct_scene_name_conditionals() -> None:
    catalog_scene_names = _catalog_scene_names()
    violations: list[str] = []

    for path in PRODUCTION_SELECTION_AND_SCENE3D_PAYLOAD_FILES:
        assert path.is_file(), f"guard target is missing: {path.relative_to(ROOT)}"
        raw_text = path.read_text(encoding="utf-8")
        code_text = _strip_cpp_comments_preserving_lines(raw_text)
        for pattern, fail_any_literal in FORBIDDEN_PATTERNS:
            for match in pattern.finditer(code_text):
                literal = match.group("literal")
                if not fail_any_literal and not _is_scene_name_literal(
                    literal, catalog_scene_names
                ):
                    continue
                line_no = _line_number(code_text, match.start())
                line = raw_text.splitlines()[line_no - 1].strip()
                violations.append(f"{path.relative_to(ROOT)}:{line_no}: {line}")

    assert not violations, (
        "Production Workcell Builder selection/editing and Scene3D payload assembly "
        "logic must not "
        "branch directly on catalog scene names. Move scene-specific knowledge to "
        "scenes/supported_scenes.yaml, fixtures, validation-command metadata, or capability/layer metadata.\n"
        + "\n".join(violations)
    )
