#!/usr/bin/env python3
from pathlib import Path
import re

ROOT = Path(__file__).resolve().parents[1]
CPP = ROOT / "workcell_builder/workcell_builder/gui/scene3d_viewport_widget.cpp"
MAINWINDOW = ROOT / "workcell_builder/workcell_builder/gui/mainwindow.cpp"
MAINWINDOW_H = ROOT / "workcell_builder/workcell_builder/gui/mainwindow.h"

def must(cond, msg):
    if not cond:
        raise AssertionError(msg)

def fn_body(src: str, signature_prefix: str) -> str:
    m = re.search(rf"{re.escape(signature_prefix)}\s*\([^)]*\)\s*\{{(?P<body>.*?)\n\}}", src, re.DOTALL)
    must(m is not None, f"function not found: {signature_prefix}")
    return m.group("body")

def main():
    src = CPP.read_text(encoding="utf-8")
    diag_src = (ROOT / "scripts/validate_scene3d_visual_diagnostics.py").read_text(encoding="utf-8")
    body = fn_body(src, "bool Scene3DViewportWidget::draw_mesh_preview_if_available")
    # Lightweight guard: ensure paintGL closes before the next method definition.
    paint_sig = "void Scene3DViewportWidget::paintGL()"
    bounds_sig = "bool Scene3DViewportWidget::scene_bounds_from_visible_items("
    paint_pos = src.find(paint_sig)
    bounds_pos = src.find(bounds_sig)
    must(paint_pos != -1 and bounds_pos != -1 and bounds_pos > paint_pos, "paintGL or scene_bounds_from_visible_items signature missing/reordered")
    paint_open = src.find("{", paint_pos)
    must(paint_open != -1 and paint_open < bounds_pos, "paintGL opening brace not found")
    depth = 0
    paint_close = -1
    for i in range(paint_open, len(src)):
        ch = src[i]
        if ch == "{":
            depth += 1
        elif ch == "}":
            depth -= 1
            if depth == 0:
                paint_close = i
                break
    must(paint_close != -1, "paintGL closing brace not found")
    must(paint_close < bounds_pos, "paintGL does not close before scene_bounds_from_visible_items (possible unmatched brace)")

    for token in [
        "fit_include_overlays",
        "scene_bounds_from_visible_items",
        "include_in_fit_bounds",
        "FIT_PHYSICAL_ONLY_FILTER",
    ]:
        must(token in src, f"missing fit-default contract token: {token}")

    for token in [
        "MeshPreviewMode::Primitives",
        "MeshPreviewMode::Meshes",
        "MeshPreviewMode::Auto",
        "ensure_mesh_cached(",
        "for (const auto & tri : cache.mesh.triangles)",
        "glBegin(GL_TRIANGLES)",
        "compute_mesh_bounds_for_test",
    ]:
        must(token in src if token == "compute_mesh_bounds_for_test" else token in body, f"missing token: {token}")


    loop_pos = body.find("for (const auto & tri : cache.mesh.triangles)")
    cube_pos = body.find("draw_unit_cube_triangles(")
    must(loop_pos != -1, "triangle loop missing")
    must(cube_pos == -1 or cube_pos < loop_pos, "unit cube fallback must not be used in successful mesh path")

    inv_body = fn_body(src, "void Scene3DViewportWidget::invalidate_mesh_cache")
    must("mesh_cache_.clear();" in inv_body, "reload does not clear mesh_cache_")
    must("warned_mesh_fallbacks_.clear();" in inv_body, "reload does not clear warning-once set")


    extractor_src = (ROOT / "scripts/extract_scene_urdf_visual_mesh_index.py").read_text(encoding="utf-8")
    for token in [
        "def discover_package_map",
        "AMENT_PREFIX_PATH",
        "package://",
        "find\\s+",
        "gather_text_with_includes",
        "INCLUDE_RE",
        "best_effort_recursive",
    ]:
        must(token in extractor_src, f"missing extractor token: {token}")

    # Contract: URDF graph parsing (joint structures + parent/child link handling).
    for token in [
        "def parse_urdf_graph",
        "'inbound_joints'",
        "'outbound_joints'",
        "joint.find('parent')",
        "joint.find('child')",
        "links[parent]['outbound_joints'].append(j)",
        "links[child]['inbound_joints'].append(j)",
    ]:
        must(token in extractor_src, f"missing URDF graph token: {token}")

    # Contract: link->world transform computation path exists.
    for token in [
        "def compute_link_world_tfs",
        "matmul4(cur_tf, tf_from_xyz_rpy",
        "link_world_tfs, link_status, link_parent, link_parent_joint, link_chain_map, gw = compute_link_world_tfs",
    ]:
        must(token in extractor_src, f"missing link-world transform token: {token}")


    for token in [
        "parse_collada_bytes_for_test",
        'ext == QStringLiteral("dae")',
        "unsupported mesh format",
        "mesh_format_counts",
        "renderable_mesh_count",
        "non_renderable_mesh_count",
    ]:
        must(token in src if token.startswith("parse_collada") or token.startswith("ext ==") or token == "unsupported mesh format" else token in extractor_src, f"missing DAE/report token: {token}")

    # Contract: visual schema exposes local/link/world pose information + status.
    for token in [
        '"transform_status"',
        '"local_visual_pose"',
        '"link_world_pose"',
        '"pose"',
    ]:
        must(token in extractor_src, f"missing visual schema token: {token}")
    # extended report/placeholder contract
    for token in ["candidate_mesh_count","emitted_visual_count","skipped_duplicate_count","skipped_unresolved_macro_count","unresolved xacro substitution placeholder","fallback_asset_search"]:
        must(token in extractor_src or token in diag_src, f"missing extended contract token: {token}")

    # Contract: collapsed-pose detection + warning emission are implemented.
    for token in [
        "def has_collapsed_visual_poses",
        "collapse_warning = \"all visual poses collapsed; transform assembly likely failed\"",
        "has_transform_collapse_warning = has_collapsed_visual_poses(items)",
        "warnings.append(collapse_warning)",
    ]:
        must(token in extractor_src, f"missing collapsed-pose warning token: {token}")

    test_src = (ROOT / "tests/test_scene_urdf_visual_mesh_index.py").read_text(encoding="utf-8")
    for token in ["assert data['resolved'] > 0", "assert 'mesh_format_counts' in data", "assert data.get('renderable_mesh_count', 0) > 0"]:
        must(token in test_src, f"missing test assertion token: {token}")


    for token in [
        "workcell_studio_scene3d_visual_diagnostics.json",
        "loaded_mesh_count",
        "failed_mesh_count",
        "world_bounds",
        "largest_mesh",
        "smallest_mesh",
        "many mesh items share identical world positions",
        "extreme mesh scale detected",
    ]:
        must(token in diag_src, f"missing diagnostics token: {token}")

    mw_src = MAINWINDOW.read_text(encoding="utf-8")
    mw_h_src = MAINWINDOW_H.read_text(encoding="utf-8")
    mw_combined = mw_src + "\n" + mw_h_src
    diagnostics_tokens = [
        "model_items_count",
        "preview_items_count",
        "filtered_visible_count",
        "viewport_received_count",
        "render_cache_count",
        "rendered_count",
        "skipped_count",
        "paintGL cache-only guard: no YAML/file IO in paint path",
    ]
    for token in diagnostics_tokens:
        must(token in src or token in mw_src, f"missing Scene3D diagnostics token: {token}")

    for token in ["--scene", "--require-xacro", "--prefer-xacro", "safe_for_preview", "extractor_version", "generated_at"]:
        must(token in extractor_src, f"missing new extractor contract token: {token}")
    for token in ["Visual mesh index stale; regenerating", "Visual mesh index regenerated with xacro", "Visual mesh index unsafe/best-effort; preview may show placeholders"]:
        must(token in mw_src, f"missing mainwindow regen log token: {token}")

    # Contract: do not hardcode a specific workspace absolute helper path in mainwindow resolver/callsite.
    forbidden_abs = "/workcell_ws/scripts/extract_scene_urdf_visual_mesh_index.py"
    must(
        forbidden_abs not in mw_src,
        "mainwindow.cpp regression: hardcoded absolute extractor path found "
        f"({forbidden_abs}); use resolver helpers instead")

    # Contract: resolver helpers/tokens must exist in mainwindow.cpp/.h for robust script lookup.
    resolver_required_tokens = [
        ("helper_script_search_paths(", "missing script resolver helper declaration/definition"),
        ("helper_script_exists(", "missing script existence helper declaration/definition"),
        ("detect_workspace_root(", "missing workspace root resolver hook"),
        ("selected_scene_path()", "missing selected scene accessor used by resolver context"),
        ("QCoreApplication::applicationDirPath", "missing appDir fallback search path token"),
        ("AMENT_PREFIX_PATH", "missing AMENT_PREFIX_PATH resolver token/marker"),
    ]
    for token, explanation in resolver_required_tokens:
        must(token in mw_combined, f"{explanation}: expected token `{token}` in mainwindow.cpp/.h")

    # Contract: selected-scene parent-chain search token(s) should exist for scene/workspace resolution.
    parent_chain_tokens = ["parent_path()", "canonical_root.parent_path()"]
    must(
        any(t in mw_src for t in parent_chain_tokens),
        "missing selected-scene parent-chain search token(s): expected one of "
        + ", ".join(parent_chain_tokens))

    # Contract: regeneration command must include required flags and avoid raw scripts/... literal callsites.
    regen_call_pattern = re.search(r"regen_args\s*<<(.*?);", mw_src, re.DOTALL)
    must(regen_call_pattern is not None, "unable to locate regen_args construction")
    regen_expr = regen_call_pattern.group(1)
    must("--scene" in regen_expr, "regen args regression: missing required --scene flag")
    must("--prefer-xacro" in regen_expr, "regen args regression: missing required --prefer-xacro flag")
    must(
        "fs::path(\"scripts\")" not in regen_expr and "scripts/" not in regen_expr,
        "regen callsite regression: raw scripts/... literal used; expected resolved absolute script path variable")
    must(
        "regen_script" in regen_expr or "resolved_script" in regen_expr or "helper_script" in regen_expr or "script_path" in regen_expr,
        "regen callsite regression: extractor path should come from a resolved script-path variable")

    # Contract: extractor strict mode must fail hard when --require-xacro constraints are unmet.
    must("--require-xacro" in extractor_src,
         "extractor strict-mode regression: missing --require-xacro argument")
    must(
        "if a.require_xacro and not xacro_available" in extractor_src,
        "extractor strict-mode regression: missing require_xacro/xacro_available guard")
    must(
        "return 2" in extractor_src and "xacro executable unavailable (required)" in extractor_src,
        "extractor strict-mode regression: strict branch must return non-zero on unmet xacro requirement")
    must(
        "if a.require_xacro and mode != 'xacro_expanded'" in extractor_src,
        "extractor strict-mode regression: missing guard against silent best-effort fallback when require-xacro is set")

    for token in ["scene_visual_mesh_index.json", "urdf_visual", "Preview warning: URDF visual unresolved", "p.locked = true"]:
        must(token in mw_src, f"missing mainwindow token: {token}")
    must(
        "const auto connect_if = [](QObject * sender, QObject * receiver, auto signal, auto slot)" not in mw_src,
        "typed Qt connect regression: generic QObject* connect_if helper reintroduced")
    must(
        "QObject::connect(sender, signal, receiver, slot);" not in mw_src,
        "typed Qt connect regression: erased sender/receiver QObject::connect pattern found")

    # Contract: preview ingestion reads computed world pose for placement and keeps URDF locking behavior.
    for token in [
        'yaml_map_key(v, "pose")',
        'p.x =',
        'p.y =',
        'p.z =',
        'p.roll =',
        'p.pitch =',
        'p.yaw =',
        'p.locked = true',
        'p.editable = false',
        'p.lock_reason = "URDF visual preview-only item (locked)"',
    ]:
        must(token in mw_src, f"missing preview ingestion token: {token}")
    must("use_fake_hardware:=true" in mw_src, "fake hardware launch token missing")

    print("scene3d mesh preview contract: OK")

if __name__ == "__main__":
    main()
