#!/usr/bin/env python3
from pathlib import Path
import re

ROOT = Path(__file__).resolve().parents[1]
CPP = ROOT / "workcell_builder/workcell_builder/gui/scene3d_viewport_widget.cpp"
MAINWINDOW = ROOT / "workcell_builder/workcell_builder/gui/mainwindow.cpp"

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
    for token in ["scene_visual_mesh_index.json", "urdf_visual", "Preview warning: URDF visual unresolved", "p.locked = true"]:
        must(token in mw_src, f"missing mainwindow token: {token}")

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
        'p.lock_reason = "URDF visual preview item (locked)"',
    ]:
        must(token in mw_src, f"missing preview ingestion token: {token}")
    must("use_fake_hardware:=true" in mw_src, "fake hardware launch token missing")

    print("scene3d mesh preview contract: OK")

if __name__ == "__main__":
    main()
