#!/usr/bin/env python3
"""Ensure generated Workcell Studio web-scene artifacts are fresh.

The web scene consumed by browser/Product View flows depends on both scene-local
source files and generator/configuration files.  This helper checks the expected
outputs and refreshes them when missing, stale, or when the mesh-index extractor
version changed.

``generated/scene_visual_mesh_index.json`` is generated cache/build output for
preview refresh, not canonical scene state, and should not be committed under
``scenes/*/generated/``.  Web scene JSON exports should be written under
``build/workcell_studio_web_scene/`` (or another ignored output location), while
tracked YAML/xacro/URDF/layout inputs remain the source of truth.
"""

from __future__ import annotations

import argparse
import ast
import json
import os
import shutil
import subprocess
import sys
from pathlib import Path
from typing import Iterable, List, Optional, Sequence, Tuple

REPO_ROOT = Path(__file__).resolve().parents[1]
SCENES_ROOT = REPO_ROOT / "scenes"
MESH_INDEX_REL = Path("generated/scene_visual_mesh_index.json")
WEB_BUILD_ROOT = REPO_ROOT / "build/workcell_studio_web_scene"
ASSET_BUILD_ROOT = WEB_BUILD_ROOT / "assets"

SCENE_INPUT_RELS = (
    "scene_manifest.yaml",
    "cell_definition.yaml",
    "environment.yaml",
    "environment_layout.yaml",
    "urdf/scene.urdf.xacro",
    "launch/demo.launch.py",
    "config/workcell_builder_task_intent.yaml",
    "layout/workcell_studio_layout.yaml",
)

GENERATOR_INPUT_RELS = (
    "scripts/extract_scene_urdf_visual_mesh_index.py",
    "scripts/export_workcell_studio_web_scene.py",
    "assets/robots/universal_robot/ur_description/config/ur5/default_kinematics.yaml",
    "assets/robots/universal_robot/ur_description/config/ur5/visual_parameters.yaml",
    "assets/robots/universal_robot/ur_description/config/ur5/physical_parameters.yaml",
    "assets/robots/universal_robot/ur5_moveit_config/config/initial_positions.yaml",
)

UR5_VISUAL_MESH_URI_TOKEN = "ur_description/meshes/ur5/visual/"
UR5_REQUIRED_LINKS = {
    "base_link_inertia",
    "shoulder_link",
    "upper_arm_link",
    "forearm_link",
    "wrist_1_link",
    "wrist_2_link",
    "wrist_3_link",
}


def _is_relative_to(path: Path, root: Path) -> bool:
    try:
        path.relative_to(root)
        return True
    except ValueError:
        return False


def normalize_scene(scene_arg: str) -> Tuple[str, Path]:
    """Return ``(scene_id, scene_dir)`` for scene id, relative path, or absolute path."""
    raw = Path(scene_arg).expanduser()
    candidates: List[Path] = []
    if raw.is_absolute():
        candidates.append(raw)
    else:
        candidates.extend((REPO_ROOT / raw, SCENES_ROOT / raw))
        if raw.parts[:1] == ("scenes",):
            candidates.insert(0, REPO_ROOT / raw)

    seen = set()
    for candidate in candidates:
        resolved = candidate.resolve()
        if resolved in seen:
            continue
        seen.add(resolved)
        if resolved.is_dir():
            scene_id = resolved.name
            return scene_id, resolved

    fallback = candidates[0].resolve() if candidates else (SCENES_ROOT / scene_arg).resolve()
    raise FileNotFoundError(f"scene directory does not exist: {scene_arg} (resolved candidate: {fallback})")


def normalize_output(output_arg: str) -> Path:
    output = Path(output_arg).expanduser()
    if not output.is_absolute():
        output = REPO_ROOT / output
    return output.resolve()


def existing_inputs(scene_dir: Path) -> List[Path]:
    inputs: List[Path] = []
    for rel in SCENE_INPUT_RELS:
        path = scene_dir / rel
        if path.exists():
            inputs.append(path)
    for rel in GENERATOR_INPUT_RELS:
        path = REPO_ROOT / rel
        if path.exists():
            inputs.append(path)
    return inputs


def newest_mtime(paths: Iterable[Path]) -> Optional[float]:
    mtimes = [path.stat().st_mtime for path in paths if path.exists()]
    return max(mtimes) if mtimes else None


def is_output_stale(output: Path, inputs: Sequence[Path]) -> Tuple[bool, str]:
    if not output.exists():
        return True, f"missing output: {output}"
    newest_input = newest_mtime(inputs)
    if newest_input is not None and output.stat().st_mtime < newest_input:
        return True, f"output is older than at least one input: {output}"
    return False, "fresh"


def are_staged_assets_stale(asset_dir: Path, inputs: Sequence[Path]) -> Tuple[bool, str]:
    if not asset_dir.exists() or not asset_dir.is_dir():
        return True, f"missing staged asset directory: {asset_dir}"
    if not any(asset_dir.rglob("*")):
        return True, f"staged asset directory is empty: {asset_dir}"
    newest_input = newest_mtime(inputs)
    if newest_input is not None and asset_dir.stat().st_mtime < newest_input:
        return True, f"staged asset directory is older than at least one input: {asset_dir}"
    return False, "fresh"


def read_extractor_version(script_path: Path) -> str:
    tree = ast.parse(script_path.read_text(encoding="utf-8"), filename=str(script_path))
    for node in tree.body:
        if isinstance(node, ast.Assign):
            for target in node.targets:
                if isinstance(target, ast.Name) and target.id == "EXTRACTOR_VERSION":
                    value = ast.literal_eval(node.value)
                    return str(value)
    raise RuntimeError(f"EXTRACTOR_VERSION not found in {script_path}")


def read_mesh_index_version(mesh_index: Path) -> Optional[str]:
    try:
        payload = json.loads(mesh_index.read_text(encoding="utf-8"))
    except Exception:
        return None
    version = payload.get("extractor_version") if isinstance(payload, dict) else None
    return str(version) if version is not None else None


def real_xacro_is_discoverable() -> bool:
    if shutil.which("xacro"):
        return True
    if Path("/opt/ros/humble/bin/xacro").exists():
        return True
    return any((Path(prefix) / "bin" / "xacro").exists() for prefix in os.environ.get("AMENT_PREFIX_PATH", "").split(os.pathsep) if prefix)


def require_real_xacro_mesh_index(mesh_index: Path) -> None:
    if not real_xacro_is_discoverable():
        return
    try:
        payload = json.loads(mesh_index.read_text(encoding="utf-8"))
    except Exception as exc:
        raise SystemExit(f"error: failed to inspect regenerated mesh index for real xacro status: {mesh_index}: {exc}") from exc
    if payload.get("xacro_real_command_succeeded") is True and payload.get("extraction_mode") in {"real_xacro_expanded", "xacro_expanded"}:
        return
    raise SystemExit(
        "error: real xacro is discoverable, but the regenerated mesh index did not use successful real xacro expansion "
        f"({mesh_index}; extraction_mode={payload.get('extraction_mode')!r}, xacro_status={payload.get('xacro_status')!r})"
    )


def _mesh_uri_values(item: dict) -> List[str]:
    values: List[str] = []
    for field in ("mesh_uri", "package_uri", "mesh_path", "source_path", "resolved_source_path", "repo_relative_source_path", "asset_relative_source_path"):
        value = item.get(field)
        if isinstance(value, str) and value:
            values.append(value)
    return values


def normalize_ur5_mesh_preview_rows(mesh_index: Path) -> bool:
    """Mark real-xacro UR5 mesh rows as browser robot preview rows.

    The real URDF extractor can emit correct UR5 mesh rows without the static
    fallback classification fields.  The web exporter buckets generated rows by
    role/category before staging assets, so unclassified real-xacro UR5 rows land
    in generic assets and the browser scene loses the required robot mesh contract.
    Normalize only resolved UR5 visual mesh rows; do not fabricate geometry.
    """
    if not mesh_index.exists():
        return False
    try:
        payload = json.loads(mesh_index.read_text(encoding="utf-8"))
    except Exception:
        return False
    if not isinstance(payload, dict):
        return False
    raw_items = payload.get("visual_items") or payload.get("items")
    if not isinstance(raw_items, list):
        return False

    changed = False
    normalized_links: set[str] = set()
    for item in raw_items:
        if not isinstance(item, dict):
            continue
        link = str(item.get("link") or item.get("link_name") or item.get("object_name") or "")
        if link not in UR5_REQUIRED_LINKS:
            continue
        if str(item.get("geometry_type") or "").lower() != "mesh":
            continue
        if not any(UR5_VISUAL_MESH_URI_TOKEN in value for value in _mesh_uri_values(item)):
            continue

        desired = {
            "category": "robot_static_mesh_visual",
            "role": "robot",
            "source_layer": "locked_generated_urdf_visual",
            "active_visual_source": "mesh_preview",
            "primitive_geometry_type": "mesh",
            "mesh_available": True,
            "render_expected": True,
            "primitive_fallback": False,
            "fallback_reason": "",
        }
        for key, value in desired.items():
            if item.get(key) != value:
                item[key] = value
                changed = True
        normalized_links.add(link)

    if normalized_links:
        payload["workcell_web_ur5_mesh_preview_normalized"] = True
        payload["workcell_web_ur5_mesh_preview_links"] = sorted(normalized_links)
        changed = True
    if changed:
        mesh_index.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        print(
            "[workcell_web_scene_fresh] normalized UR5 mesh preview rows: "
            + ",".join(sorted(normalized_links))
        )
    return changed


def run_checked(command: Sequence[str]) -> None:
    result = subprocess.run(command, cwd=REPO_ROOT, text=True, capture_output=True, check=False)
    if result.returncode != 0:
        rendered = " ".join(command)
        print(f"error: command failed with exit code {result.returncode}: {rendered}", file=sys.stderr)
        print("--- stdout ---", file=sys.stderr)
        print(result.stdout or "", file=sys.stderr, end="" if result.stdout.endswith("\n") else "\n")
        print("--- stderr ---", file=sys.stderr)
        print(result.stderr or "", file=sys.stderr, end="" if result.stderr.endswith("\n") else "\n")
        raise SystemExit(result.returncode)
    if result.stdout:
        print(result.stdout, end="")
    if result.stderr:
        print(result.stderr, end="", file=sys.stderr)


def repo_relative(path: Path) -> str:
    resolved = path.resolve()
    if _is_relative_to(resolved, REPO_ROOT):
        return str(resolved.relative_to(REPO_ROOT))
    return str(resolved)


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description="Refresh stale Workcell Studio web-scene outputs.")
    parser.add_argument("--scene", required=True, help="Scene id or path, e.g. ur5_2f_test or scenes/ur5_2f_test.")
    parser.add_argument("--output", required=True, help="Web scene JSON output path.")
    parser.add_argument("--stage-assets", action="store_true", help="Require and regenerate staged browser assets.")
    parser.add_argument("--force", action="store_true", help="Regenerate even when freshness checks pass.")
    args = parser.parse_args(argv)

    try:
        scene_id, scene_dir = normalize_scene(args.scene)
    except FileNotFoundError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2

    output_path = normalize_output(args.output)
    mesh_index = scene_dir / MESH_INDEX_REL
    asset_dir = ASSET_BUILD_ROOT / scene_id
    extractor_script = REPO_ROOT / "scripts/extract_scene_urdf_visual_mesh_index.py"
    exporter_script = REPO_ROOT / "scripts/export_workcell_studio_web_scene.py"
    inputs = existing_inputs(scene_dir)

    mesh_stale, mesh_reason = is_output_stale(mesh_index, inputs)
    expected_version = read_extractor_version(extractor_script)
    actual_version = read_mesh_index_version(mesh_index) if mesh_index.exists() else None
    if actual_version != expected_version:
        mesh_stale = True
        mesh_reason = f"mesh index extractor_version is {actual_version!r}, expected {expected_version!r}"

    web_inputs = list(inputs) + [mesh_index]
    web_stale, web_reason = is_output_stale(output_path, web_inputs)
    assets_stale = False
    assets_reason = "not requested"
    if args.stage_assets:
        assets_stale, assets_reason = are_staged_assets_stale(asset_dir, web_inputs)

    if args.force or mesh_stale:
        reason = "forced" if args.force else mesh_reason
        print(f"Refreshing mesh index for {scene_id}: {reason}")
        command = [sys.executable, repo_relative(extractor_script), "--scene", repo_relative(scene_dir), "--prefer-xacro"]
        if real_xacro_is_discoverable():
            command.append("--require-xacro")
        run_checked(command)
        require_real_xacro_mesh_index(mesh_index)
        web_stale = True
        web_reason = "mesh index was refreshed"

    if normalize_ur5_mesh_preview_rows(mesh_index):
        web_stale = True
        web_reason = "mesh index UR5 preview rows were normalized for web export"
        if args.stage_assets:
            assets_stale = True
            assets_reason = "mesh index UR5 preview rows were normalized for web export"

    if args.force or web_stale or assets_stale:
        reasons = []
        if args.force:
            reasons.append("forced")
        if web_stale:
            reasons.append(web_reason)
        if assets_stale:
            reasons.append(assets_reason)
        print(f"Refreshing web scene for {scene_id}: {'; '.join(reasons)}")
        command = [sys.executable, repo_relative(exporter_script), "--scene", repo_relative(scene_dir), "--output", repo_relative(output_path)]
        if args.stage_assets:
            command.append("--stage-assets")
        run_checked(command)
    else:
        print(f"Workcell Studio web scene is fresh for {scene_id}: {repo_relative(output_path)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
