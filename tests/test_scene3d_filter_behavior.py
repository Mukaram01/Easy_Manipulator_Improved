from __future__ import annotations

from dataclasses import dataclass, field


@dataclass
class Item:
    id: str
    source_layer: str = ""
    active_visual_source: str = ""
    role: str = "object"
    category: str = "object"
    status: str = ""
    warnings: list[str] = field(default_factory=list)
    mesh_load_warning: str = ""
    locked: bool = False
    lock_reason: str = ""


def _tok(s: str) -> str:
    return s.strip().lower().replace("-", "_").replace(" ", "_")


def is_generated_robot_visual(item: Item) -> bool:
    source_layer = _tok(item.source_layer)
    visual_source = _tok(item.active_visual_source)
    combined = "|".join([_tok(item.role), _tok(item.category), _tok(item.id), _tok(item.lock_reason)])
    if source_layer in {"locked_generated_urdf_visual", "generated_urdf_visual"}:
        return True
    if visual_source in {
        "locked_generated_urdf_visual",
        "generated_urdf_visual",
        "generated_urdf_visual_fallback",
    }:
        return True
    if _tok(item.id).startswith("generated_urdf_fallback::"):
        return True
    return item.locked and any(token in combined for token in ("urdf", "generated", "robot_link", "robot_model"))


def include_item(item: Item, enabled_layers: set[str]) -> bool:
    source_layer = _tok(item.source_layer)
    visual_source = _tok(item.active_visual_source)
    combined = _tok(item.role) + "|" + _tok(item.category) + "|" + _tok(item.status) + "|" + "|".join(w.lower() for w in item.warnings)
    is_warning_or_missing = "warning" in combined or "missing" in combined or item.mesh_load_warning.strip() != ""
    is_overlay_or_helper = "overlay" in combined or "helper" in combined or "safety zone" in combined

    if is_generated_robot_visual(item):
        return "locked_generated_urdf_visual" in enabled_layers
    if source_layer == "editable_layout":
        return "editable_layout" in enabled_layers
    if source_layer == "primitive_fallback":
        return "primitive_fallback" in enabled_layers
    if visual_source == "mesh_preview":
        return "mesh_preview" in enabled_layers
    if is_overlay_or_helper:
        return "overlay" in enabled_layers
    if is_warning_or_missing:
        return "warning" in enabled_layers
    return True


def apply_filter(items: list[Item], enabled_layers: set[str]) -> tuple[list[str], list[str]]:
    visible = [it for it in items if include_item(it, enabled_layers)]
    logs: list[str] = []

    def looks_renderable(it: Item) -> bool:
        combined = "|".join([it.role, it.category, it.status, *it.warnings, it.mesh_load_warning]).lower()
        helper_or_overlay = any(x in combined for x in ("overlay", "helper", "safety zone"))
        warning_or_missing = any(x in combined for x in ("warning", "missing", "unsafe", "unrenderable"))
        return not helper_or_overlay and not warning_or_missing

    if not visible and items:
        restored = [it for it in items if looks_renderable(it)]
        if restored:
            visible = restored
            logs.append("default_filter_fallback_kept_renderable_items_visible")
    if not visible and items:
        logs.append("blocker")
    return [v.id for v in visible], logs


def _default_layers() -> set[str]:
    return {
        "editable_layout",
        "locked_generated_urdf_visual",
        "mesh_preview",
        "primitive_fallback",
        "overlay",
        "warning",
    }


def test_smoke_defaults_keep_mesh_editable_primitive_and_unknown_renderable_visible() -> None:
    ids, _ = apply_filter(
        [
            Item("mesh", active_visual_source="mesh_preview"),
            Item("layout", source_layer="editable_layout"),
            Item("prim", source_layer="primitive_fallback"),
            Item("unknown_renderable", source_layer="mystery_layer", role="object", category="fixture"),
        ],
        _default_layers(),
    )
    assert set(ids) == {"mesh", "layout", "prim", "unknown_renderable"}


def test_stale_unchecked_toggles_cannot_force_zero_visible_in_smoke_mode() -> None:
    ids, logs = apply_filter(
        [
            Item("mesh", active_visual_source="mesh_preview"),
            Item("layout", source_layer="editable_layout"),
        ],
        set(),
    )
    assert set(ids) == {"mesh", "layout"}
    assert "default_filter_fallback_kept_renderable_items_visible" in logs


def test_zero_visible_fallback_excludes_unsafe_or_unrenderable_items() -> None:
    ids, logs = apply_filter(
        [
            Item("ok", source_layer="editable_layout", role="object", category="fixture"),
            Item("unsafe", source_layer="editable_layout", warnings=["unsafe geometry"]),
            Item("broken", source_layer="editable_layout", status="unrenderable"),
        ],
        set(),
    )
    assert ids == ["ok"]
    assert "default_filter_fallback_kept_renderable_items_visible" in logs


def test_fixture_approx_failing_scene_does_not_end_at_zero_visible_items() -> None:
    ids, logs = apply_filter(
        [
            Item("robot", source_layer="editable_layout", role="robot"),
            Item("overlay", role="overlay helper", category="helper"),
            Item("warn", warnings=["missing mesh"]),
        ],
        set(),
    )
    assert "robot" in ids
    assert ids
    assert "blocker" not in logs


def test_generated_robot_layer_overrides_mesh_and_label_style_filters() -> None:
    ids, _ = apply_filter(
        [
            Item(
                "ur5_link_mesh",
                source_layer="locked_generated_urdf_visual",
                active_visual_source="mesh_preview",
                role="robot link",
                category="URDF Visual",
                locked=True,
            ),
            Item(
                "generated_urdf_fallback::forearm_link",
                source_layer="locked_generated_urdf_visual",
                active_visual_source="generated_urdf_visual_fallback",
                role="robot link",
                category="URDF Visual Fallback",
                locked=True,
            ),
            Item("loose_mesh_preview", active_visual_source="mesh_preview"),
        ],
        {"locked_generated_urdf_visual"},
    )
    assert "ur5_link_mesh" in ids
    assert "generated_urdf_fallback::forearm_link" in ids
    assert "loose_mesh_preview" not in ids
