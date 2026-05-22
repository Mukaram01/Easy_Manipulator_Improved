#!/usr/bin/env python3
from __future__ import annotations
import argparse, json, re
from collections import Counter
from pathlib import Path
import yaml

import sys

_REPO_ROOT = Path(__file__).resolve().parents[1]
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))

from scripts.workcell_studio_script_bootstrap import ensure_repo_root_on_sys_path

ensure_repo_root_on_sys_path(__file__)

from scripts.scene_root_resolver import resolve_scene_root

PLACEHOLDER_RE = re.compile(r"\$\{[^}]+\}")
CANONICAL_LAYERS = {"editable_layout", "mesh_preview", "locked_generated_urdf_visual", "primitive_fallback", "overlay"}


def normalize_layer_token(value: str | None) -> str:
    token = (value or "").strip().lower().replace("-", "_").replace(" ", "_")
    if token in {"generated_preview", "generated_urdf_visual", "locked_generated_urdf"}:
        return "locked_generated_urdf_visual"
    if token == "legacy_static_fallback":
        return "primitive_fallback"
    if token in {"overlays", "helper_overlay"}:
        return "overlay"
    return token


def _load_json(path: Path):
    try:
        return json.loads(path.read_text(encoding='utf-8'))
    except Exception:
        return None


def _load_yaml(path: Path):
    try:
        data = yaml.safe_load(path.read_text(encoding='utf-8'))
        return data if isinstance(data, dict) else {}
    except Exception:
        return {}


def _count_preview_items_from_generated_metadata(scene_dir: Path) -> int:
    generated_layout = scene_dir / 'layout' / 'workcell_studio_layout.generated.yaml'
    if not generated_layout.exists():
        return 0
    in_preview_items = False
    count = 0
    for raw in generated_layout.read_text(encoding='utf-8').splitlines():
        line = raw.rstrip()
        if line.strip() == 'preview_items:':
            in_preview_items = True
            continue
        if in_preview_items and line and not line.startswith(' '):
            break
        if in_preview_items and line.lstrip().startswith('- id:'):
            count += 1
    return count


def _runtime_scene_diagnostics(repo_root: Path, scene: str) -> dict:
    diag_path = repo_root / 'build' / 'workcell_studio_scene3d_visual_diagnostics.json'
    payload = _load_json(diag_path)
    if not isinstance(payload, dict):
        return {}
    scenes = payload.get('scenes', [])
    if not isinstance(scenes, list):
        return {}
    for item in scenes:
        if isinstance(item, dict) and item.get('scene') == scene:
            return item
    return {}


def check_scene(repo_root: Path, scene_dir: Path) -> dict:
    scene = scene_dir.name
    layout_path = scene_dir / 'layout' / 'workcell_studio_layout.yaml'
    has_layout = layout_path.exists()
    layout = _load_yaml(layout_path) if has_layout else {}
    idx = _load_json(scene_dir / 'generated' / 'scene_visual_mesh_index.json') or {}
    items = idx.get('items', []) if isinstance(idx, dict) else []
    preview_metadata_path = scene_dir / 'generated' / 'scene_preview_metadata.json'
    preview_metadata = _load_json(preview_metadata_path) if preview_metadata_path.exists() else {}

    overlay_sources = [
        scene_dir / 'generated/epd_snapshot.json',
        scene_dir / 'generated/detections_snapshot.json',
        scene_dir / 'config/epd_snapshot.json',
        scene_dir / 'config/readiness_overlay_metadata.json',
        scene_dir / 'generated/workcell_studio_runtime_acceptance.json',
        repo_root / 'build/workcell_studio/scene3d_runtime_acceptance.json',
    ]

    layer_counts = Counter()
    visual_counts = Counter()
    unresolved = 0
    unsafe_reasons = 0
    locked_generated = 0
    editable_layout = 0
    blockers, fixes = [], []

    for i in items:
        layer = normalize_layer_token(i.get('source_layer') or i.get('item_source') or ('primitive_fallback' if i.get('geometry_type') in {'box','cylinder','sphere'} else 'mesh_preview'))
        visual = i.get('active_visual_source') or ('expanded_urdf_mesh' if i.get('extraction_mode') == 'xacro_expanded' else ('mesh' if i.get('mesh_path') else 'primitive'))
        layer_counts[layer] += 1
        visual_counts[visual] += 1
        item_id = str(i.get('id', ''))
        unresolved += 1 if PLACEHOLDER_RE.search(item_id) else 0
        if i.get('unsafe_visual_reason'):
            unsafe_reasons += 1
        if layer == 'locked_generated_urdf_visual':
            if i.get('editable') is not False:
                blockers.append(f"locked item editable: {item_id}")
            locked_generated += 1
        if layer == 'editable_layout':
            if i.get('editable') is not True:
                blockers.append(f"editable layout locked: {item_id}")
            editable_layout += 1

    editable_from_layout = len(layout.get('items') or []) if isinstance(layout.get('items'), list) else 0
    primitive_from_layout = sum(1 for x in (layout.get('items') or []) if isinstance(x, dict) and x.get('geometry_type') in {'box', 'cylinder', 'sphere'})
    primitive_count = max(layer_counts['primitive_fallback'], primitive_from_layout)
    mesh_count = max(layer_counts['mesh_preview'], len(idx.get('visual_items') or []) if bool(idx.get('safe_for_preview', False)) else 0)
    urdf_count = layer_counts['locked_generated_urdf_visual']
    editable_layout = max(editable_layout, editable_from_layout)
    meaningful_total = primitive_count + mesh_count + urdf_count

    preview_items_count = _count_preview_items_from_generated_metadata(scene_dir)
    visible_mesh_index_items = sum(1 for i in items if i.get('hidden_by_filters') is not True and (i.get('mesh_path') or i.get('geometry_type') in {'box', 'cylinder', 'sphere'} or (i.get('active_visual_source') not in {None, '', 'missing'})))
    overlay_preview_count = len(preview_metadata.get('overlays') or []) if isinstance(preview_metadata, dict) else 0
    overlay_file_count = sum(1 for p in overlay_sources if p.exists())
    overlay_count = max(layer_counts['overlay'], overlay_preview_count, overlay_file_count)
    visible_after_filters_count = max(visible_mesh_index_items, editable_layout + mesh_count + primitive_count)
    filtered_hidden_count = max(0, len(items) - visible_after_filters_count)
    diag_scene = _runtime_scene_diagnostics(repo_root, scene)
    render_cache_received_count = None
    if diag_scene:
        render_cache_received_count = diag_scene.get('render_cache_received_count', diag_scene.get('render_cache_received'))

    if not has_layout:
        blockers.append('missing layout/workcell_studio_layout.yaml')
    if editable_layout == 0:
        blockers.append(f'editable_layout_count is zero (layout path={layout_path}, exists={has_layout})')
    if primitive_count == 0:
        blockers.append(f'primitive_fallback_count is zero (layout path={layout_path}, mesh_index path={scene_dir / "generated" / "scene_visual_mesh_index.json"})')
        fixes.append('retain primitive_fallback visuals even when mesh/urdf previews exist')
    if mesh_count == 0:
        blockers.append(f'mesh_preview_count is zero (mesh_index path={scene_dir / "generated" / "scene_visual_mesh_index.json"}, safe_for_preview={bool(idx.get("safe_for_preview", False))})')
        fixes.append('if safe mesh index exists, map items into mesh_preview layer')
    if urdf_count == 0:
        blockers.append(f'locked_generated_urdf_visual_count is zero (generated layout path={scene_dir / "layout" / "workcell_studio_layout.generated.yaml"})')
    if unresolved > 0 and bool(idx.get('safe_for_preview', False)):
        blockers.append('unresolved placeholders remain in IDs during safe preview')
    if meaningful_total == 0:
        blockers.append('meaningful visible-capable layer counts are all zero across layout + mesh index + preview metadata')
    if visible_after_filters_count == 0:
        blockers.append('visible_after_filters_count is zero by default (no concrete visible-capable items)')
    if mesh_count == 0 and primitive_count > 0 and visible_after_filters_count > 0:
        fixes.append('optional mesh assets missing; primitive fallback is present and visible')

    status = 'PASS'
    if blockers:
        status = 'FAIL'
    elif mesh_count == 0 or urdf_count == 0:
        status = 'WARN'

    return {
        'scene': scene,
        'editable_layout_count': editable_layout,
        'mesh_preview_count': mesh_count,
        'locked_generated_urdf_visual_count': urdf_count,
        'primitive_fallback_count': primitive_count,
        'overlay_count': overlay_count,
        'missing_count': sum(count for key, count in layer_counts.items() if key not in CANONICAL_LAYERS),
        'unsafe_visual_reason_count': unsafe_reasons,
        'unresolved_placeholder_count': unresolved,
        'preview_items_count': preview_items_count,
        'visible_after_filters_count': visible_after_filters_count,
        'filtered_hidden_count': filtered_hidden_count,
        'render_cache_received_count': render_cache_received_count,
        'contract_status': status,
        'blockers': blockers,
        'suggested_fixes': fixes,
        'source_layer_counts': dict(layer_counts),
        'overlay_sources_present': [str(p) for p in overlay_sources if p.exists()],
        'active_visual_source_counts': dict(visual_counts),
    }


def markdown(report: list[dict]) -> str:
    lines = ['# Scene3D Canvas Contract Report', '']
    for r in report:
        lines += [f"## {r['scene']} - {r['contract_status']}",
                  f"- editable_layout_count: {r['editable_layout_count']}",
                  f"- mesh_preview_count: {r['mesh_preview_count']}",
                  f"- locked_generated_urdf_visual_count: {r['locked_generated_urdf_visual_count']}",
                  f"- primitive_fallback_count: {r['primitive_fallback_count']}",
                  f"- overlay_count: {r['overlay_count']}",
                  f"- unsafe_visual_reason_count: {r['unsafe_visual_reason_count']}",
                  f"- unresolved_placeholder_count: {r['unresolved_placeholder_count']}",
                  f"- preview_items_count: {r['preview_items_count']}",
                  f"- visible_after_filters_count: {r['visible_after_filters_count']}",
                  f"- filtered_hidden_count: {r['filtered_hidden_count']}",
                  f"- render_cache_received_count: {r['render_cache_received_count']}"]
        if r['blockers']:
            lines.append('- blockers:'); lines += [f"  - {b}" for b in r['blockers']]
        if r['suggested_fixes']:
            lines.append('- suggested_fixes:'); lines += [f"  - {s}" for s in r['suggested_fixes']]
        lines.append('')
    return '\n'.join(lines)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--repo-root', default=str(Path(__file__).resolve().parents[1]))
    ap.add_argument('--scene', action='append')
    ap.add_argument('--all', action='store_true')
    ap.add_argument('--json')
    ap.add_argument('--markdown')
    args = ap.parse_args()

    repo_root = Path(args.repo_root).resolve()
    scenes_root = resolve_scene_root(repo_root)
    if not scenes_root.exists():
        ap.error(f"scenes root does not exist: {scenes_root}")

    scenes = []
    if args.all:
        scenes = sorted([p for p in scenes_root.iterdir() if p.is_dir()]) if scenes_root.exists() else []
    elif args.scene:
        scenes = [scenes_root / s for s in args.scene]
    else:
        ap.error('use --all or --scene <name>')

    missing = [str(s) for s in scenes if not s.exists()]
    if missing:
        ap.error(f"requested scene path is missing under scenes root {scenes_root}: {', '.join(missing)}")

    report = [check_scene(repo_root, s) for s in scenes]
    overall = 'PASS' if all(r['contract_status'] == 'PASS' for r in report) else ('FAIL' if any(r['contract_status']=='FAIL' for r in report) else 'WARN')
    payload = {'overall_status': overall, 'repo_root': str(repo_root), 'scenes_root': str(scenes_root), 'scenes': report}

    if args.json:
        Path(args.json).write_text(json.dumps(payload, indent=2), encoding='utf-8')
    if args.markdown:
        Path(args.markdown).write_text(markdown(report), encoding='utf-8')
    print(f"Scene3D Contract: {overall}")
    for r in report:
        print(f"Layers: editable_layout={r['editable_layout_count']} mesh_preview={r['mesh_preview_count']} locked_generated_urdf_visual={r['locked_generated_urdf_visual_count']} primitive_fallback={r['primitive_fallback_count']} overlay={r['overlay_count']}")
    raise SystemExit(1 if overall == 'FAIL' else 0)

if __name__ == '__main__':
    main()
