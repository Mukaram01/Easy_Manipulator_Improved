#!/usr/bin/env python3
"""Ensure generated Workcell Studio web-scene artifacts are fresh.

The web scene consumed by browser/Product View flows depends on both scene-local
source files and generator/configuration files. This helper checks the expected
outputs and refreshes them when missing, stale, or when the mesh-index extractor
version changed.

``generated/scene_visual_mesh_index.json`` is generated cache/build output for
preview refresh, not canonical scene state, and should not be committed under
``scenes/*/generated/``. Web scene JSON exports should be written under
``build/workcell_studio_web_scene/`` (or another ignored output location), while
tracked YAML/xacro/URDF/layout inputs remain the source of truth.
"""

from __future__ import annotations

import argparse
import ast
import copy
import json
import os
import shutil
import subprocess
import tempfile
import hashlib
import sys
from pathlib import Path
from typing import Any, Iterable, List, Optional, Sequence, Tuple

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
    for field in (
        "mesh_uri",
        "package_uri",
        "mesh_path",
        "source_path",
        "resolved_source_path",
        "repo_relative_source_path",
        "asset_relative_source_path",
    ):
        value = item.get(field)
        if isinstance(value, str) and value:
            values.append(value)
    return values


def _pose_has_finite_xyz(pose: object) -> bool:
    if not isinstance(pose, dict):
        return False
    xyz = pose.get("xyz")
    if not isinstance(xyz, list) or len(xyz) < 3:
        return False
    try:
        return all(float(v) == float(v) for v in xyz[:3])
    except Exception:
        return False


def _visual_world_pose(item: dict) -> Optional[dict]:
    """Return the finite visible mesh-world pose baked by the URDF extractor.

    Generated URDF browser placement can use ``frame_world_pose``/``link_world_pose``
    as the link root plus a local ``visual_origin`` wrapper. Rows flagged with
    ``workcell_web_render_pose_mode=baked_visible_world_pose`` instead render from
    this baked link*visual-origin pose so Product View matches the visible mesh
    placement directly.
    """
    for field in ("baked_world_visual_pose", "expected_visual_pose"):
        pose = item.get(field)
        if _pose_has_finite_xyz(pose):
            return copy.deepcopy(pose)
    return None


def _is_generated_urdf_mesh_row(item: dict) -> bool:
    if str(item.get("geometry_type") or "").lower() != "mesh":
        return False
    source_text = " ".join(
        str(item.get(field) or "")
        for field in (
            "source",
            "source_kind",
            "source_layer",
            "active_visual_source",
            "transform_source",
            "baked_world_visual_transform_source",
            "primary_pose_source",
        )
    ).lower()
    return "urdf" in source_text


def _baked_visible_pose_fields(item: dict, visible_pose: dict) -> dict:
    fields = {
        "final_transform": copy.deepcopy(visible_pose),
        "world_from_visual": copy.deepcopy(visible_pose),
        "workcell_web_visible_mesh_pose_source": "baked_world_visual_pose",
        "workcell_web_render_pose_mode": "baked_visible_world_pose",
        "visual_origin_application": "baked_into_web_preview_pose",
    }
    if "link_world_pose" in item:
        fields["original_link_world_pose"] = copy.deepcopy(item.get("link_world_pose"))
    if "frame_world_pose" in item:
        fields["original_frame_world_pose"] = copy.deepcopy(item.get("frame_world_pose"))
    return fields


def normalize_ur5_mesh_preview_rows(mesh_index: Path) -> bool:
    """Mark real-xacro UR5 mesh rows as browser robot preview rows.

    The real URDF extractor can emit correct UR5 mesh rows without the static
    fallback classification fields. The web exporter buckets generated rows by
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
    visible_pose_links: set[str] = set()
    for item in raw_items:
        if not isinstance(item, dict):
            continue
        link = str(
            item.get("link") or item.get("link_name") or item.get("object_name") or ""
        )
        if not _is_generated_urdf_mesh_row(item):
            continue

        desired = {}
        if link in UR5_REQUIRED_LINKS and any(
            UR5_VISUAL_MESH_URI_TOKEN in value for value in _mesh_uri_values(item)
        ):
            desired.update(
                {
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
            )
            normalized_links.add(link)

        visible_pose = _visual_world_pose(item)
        if visible_pose is not None:
            # Flag generated URDF mesh rows to render from the baked visible
            # world pose instead of using link_world_pose/frame_world_pose as
            # the viewer's primary root pose plus a local visual-origin wrapper.
            desired.update(_baked_visible_pose_fields(item, visible_pose))
            visible_pose_links.add(link)

        for key, value in desired.items():
            if item.get(key) != value:
                item[key] = value
                changed = True

    if normalized_links:
        payload["workcell_web_ur5_mesh_preview_normalized"] = True
        payload["workcell_web_ur5_mesh_preview_links"] = sorted(normalized_links)
        changed = True
    if visible_pose_links:
        payload["workcell_web_ur5_visible_mesh_pose_links"] = sorted(visible_pose_links)
        changed = True
    if changed:
        mesh_index.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        print(
            "[workcell_web_scene_fresh] normalized mesh preview rows: "
            + ",".join(sorted(normalized_links | visible_pose_links)),
            file=sys.stderr,
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
        print(result.stdout, end="", file=sys.stderr)
    if result.stderr:
        print(result.stderr, end="", file=sys.stderr)


def repo_relative(path: Path) -> str:
    resolved = path.resolve()
    if _is_relative_to(resolved, REPO_ROOT):
        return str(resolved.relative_to(REPO_ROOT))
    return str(resolved)



FRESHENER_RESULT_SCHEMA_VERSION = "workcell_studio_web_scene_freshener/v1"
SUPPORTED_WEB_SCENE_SCHEMA_VERSIONS = {"workcell_studio_web_scene/v1"}


def file_fingerprint(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return "sha256:" + digest.hexdigest()


BROWSER_ASSET_REFERENCE_KEYS = {"mesh_uri", "mesh_url", "meshStagedPath", "mesh_staged_path", "urdf_url", "texture_uri", "texture_url"}


def _collect_browser_asset_refs(value: Any, key: str = "") -> Iterable[str]:
    if isinstance(value, str):
        if key in BROWSER_ASSET_REFERENCE_KEYS:
            yield value
    elif isinstance(value, dict):
        for child_key, child in value.items():
            yield from _collect_browser_asset_refs(child, str(child_key))
    elif isinstance(value, list):
        for child in value:
            yield from _collect_browser_asset_refs(child, key)


def staged_asset_diagnostics(payload: dict[str, Any], asset_dir: Path, stage_assets: bool) -> dict[str, Any]:
    refs = sorted({s for s in _collect_browser_asset_refs(payload) if isinstance(s, str) and (s.startswith("assets/") or "/assets/" in s)})
    missing: list[str] = []
    for ref in refs:
        candidate_text = ref.split("?", 1)[0].split("#", 1)[0]
        if candidate_text.startswith("assets/"):
            candidate = WEB_BUILD_ROOT / candidate_text
        else:
            candidate = WEB_BUILD_ROOT / candidate_text[candidate_text.find("assets/"):]
        if not candidate.exists():
            missing.append(ref)
    files = [p for p in asset_dir.rglob("*") if p.is_file()] if asset_dir.exists() else []
    return {
        "stage_assets_requested": bool(stage_assets),
        "asset_dir": repo_relative(asset_dir),
        "asset_dir_exists": asset_dir.is_dir(),
        "staged_file_count": len(files),
        "referenced_asset_count": len(refs),
        "missing_referenced_assets": missing,
        "ok": (not stage_assets or asset_dir.is_dir()) and not missing,
    }


def validate_web_scene_output(output: Path, scene_id: str, asset_dir: Path, stage_assets: bool, status: str) -> tuple[dict[str, Any], dict[str, Any]]:
    if status not in {"current", "rebuilt"}:
        raise RuntimeError(f"freshener status must be current or rebuilt, got {status!r}")
    if not output.exists():
        raise RuntimeError(f"web scene output does not exist: {output}")
    try:
        payload = json.loads(output.read_text(encoding="utf-8"))
    except Exception as exc:
        raise RuntimeError(f"web scene output is not valid JSON: {output}: {exc}") from exc
    if not isinstance(payload, dict):
        raise RuntimeError(f"web scene output must be a JSON object: {output}")
    schema_version = str(payload.get("schema_version") or "")
    if schema_version not in SUPPORTED_WEB_SCENE_SCHEMA_VERSIONS:
        raise RuntimeError(f"unsupported web scene schema_version {schema_version!r} in {output}")
    actual_scene = str(payload.get("scene_id") or payload.get("scene_name") or "")
    if actual_scene != scene_id:
        raise RuntimeError(f"web scene scene_id mismatch: expected {scene_id!r}, got {actual_scene!r} in {output}")
    diagnostics = staged_asset_diagnostics(payload, asset_dir, stage_assets)
    if diagnostics["missing_referenced_assets"]:
        raise RuntimeError(f"web scene references missing staged assets: {diagnostics['missing_referenced_assets']}")
    return payload, diagnostics


def atomic_export_web_scene(command: Sequence[str], output_path: Path) -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(prefix=output_path.name + ".", suffix=".tmp", dir=output_path.parent, delete=False) as handle:
        tmp_path = Path(handle.name)
    try:
        tmp_command = list(command)
        out_index = tmp_command.index("--output") + 1
        tmp_command[out_index] = repo_relative(tmp_path)
        run_checked(tmp_command)
        os.replace(tmp_path, output_path)
    finally:
        tmp_path.unlink(missing_ok=True)

def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description="Refresh stale Workcell Studio web-scene outputs.")
    parser.add_argument("--scene", required=True, help="Scene id or path, e.g. ur5_2f_test or scenes/ur5_2f_test.")
    parser.add_argument("--output", required=True, help="Web scene JSON output path.")
    parser.add_argument("--stage-assets", action="store_true", help="Require and regenerate staged browser assets.")
    parser.add_argument("--force", action="store_true", help="Regenerate even when freshness checks pass.")
    parser.add_argument("--allow-incomplete-preview", action="store_true", help="Allow the exporter to produce a read-only diagnostic preview for authoring blockers.")
    parser.add_argument("--authoring-session-overlay", help="Request-scoped PreviewItem overlay JSON to merge only into this export.")
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
        print(f"Refreshing mesh index for {scene_id}: {reason}", file=sys.stderr)
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

    if args.force or args.allow_incomplete_preview or args.authoring_session_overlay or web_stale or assets_stale:
        reasons = []
        if args.force:
            reasons.append("forced")
        if web_stale:
            reasons.append(web_reason)
        if assets_stale:
            reasons.append(assets_reason)
        print(f"Refreshing web scene for {scene_id}: {'; '.join(reasons)}", file=sys.stderr)
        command = [sys.executable, repo_relative(exporter_script), "--scene", repo_relative(scene_dir), "--output", repo_relative(output_path)]
        if args.stage_assets:
            command.append("--stage-assets")
        if args.allow_incomplete_preview:
            command.append("--allow-incomplete-preview")
        if args.authoring_session_overlay:
            overlay = Path(args.authoring_session_overlay).expanduser().resolve()
            if not overlay.is_file():
                print(f"error: authoring-session overlay does not exist: {overlay}", file=sys.stderr)
                return 2
            command.extend(["--authoring-session-overlay", str(overlay)])
        atomic_export_web_scene(command, output_path)
        status = "rebuilt"
    else:
        print(f"Workcell Studio web scene is fresh for {scene_id}: {repo_relative(output_path)}", file=sys.stderr)
        status = "current"
    try:
        payload, asset_diagnostics = validate_web_scene_output(output_path, scene_id, asset_dir, args.stage_assets, status)
    except RuntimeError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 3
    result = {
        "schema_version": FRESHENER_RESULT_SCHEMA_VERSION,
        "status": status,
        "scene_id": scene_id,
        "output": repo_relative(output_path),
        "fingerprint": file_fingerprint(output_path),
        "web_scene_schema_version": payload.get("schema_version"),
        "staged_asset_diagnostics": asset_diagnostics,
    }
    print(json.dumps(result, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
