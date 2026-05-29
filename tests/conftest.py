from __future__ import annotations

import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))


import json
from collections.abc import Callable, Iterable, Mapping
from typing import Any

import pytest
import yaml

SCENE_AUTHORING_FILES = ("environment.yaml", "layout/workcell_studio_layout.yaml")
SCENE_GENERATED_FILES = (
    "cell_definition.yaml",
    "scene_manifest.yaml",
    "urdf/scene.urdf.xacro",
    "launch/demo.launch.py",
    "generated/scene_visual_mesh_index.json",
)
SUPPORTED_SCENE_SCHEMA = "workcell_studio_supported_scenes/v1"


def _remove_relpath(root: Path, relpath: str) -> None:
    path = root / relpath
    if path.exists():
        path.unlink()


def write_minimal_scene_dir(
    scene_dir: Path,
    *,
    scene_name: str | None = None,
    package_xml_name: str | None = None,
    cmake_project_name: str | None = None,
    missing_authoring_files: Iterable[str] = (),
    missing_generated_files: Iterable[str] = (),
    mesh_index_payload: Mapping[str, Any] | str | None = None,
) -> Path:
    """Write a minimal Workcell Studio scene fixture for validator tests.

    Passing a string writes raw malformed JSON; mappings are JSON-serialized. Use
    ``missing_generated_files`` to omit generated/scene_visual_mesh_index.json.
    """
    name = scene_name or scene_dir.name
    package = package_xml_name or name
    project = cmake_project_name or package
    payload: Mapping[str, Any] | str | None = mesh_index_payload
    if payload is None:
        payload = {"items": [{"id": "fixture_mesh", "render_expected": True}]}

    scene_dir.mkdir(parents=True, exist_ok=True)
    (scene_dir / "package.xml").write_text(
        f"<package format='3'><name>{package}</name><version>0.0.0</version><description>fixture</description><maintainer email='test@example.com'>Test</maintainer><license>Apache-2.0</license></package>\n",
        encoding="utf-8",
    )
    (scene_dir / "CMakeLists.txt").write_text(
        f"cmake_minimum_required(VERSION 3.8)\nproject({project})\nfind_package(ament_cmake REQUIRED)\nament_package()\n",
        encoding="utf-8",
    )
    (scene_dir / "environment.yaml").write_text(
        "schema_version: workcell_studio_environment/v1\nsupport_surfaces: []\ntask_zones: []\n",
        encoding="utf-8",
    )
    (scene_dir / "cell_definition.yaml").write_text(
        f"schema_version: workcell_cell_definition/v1\npackage_name: {package}\nrobot: ur5\nend_effector: suction\nenvironment: fixture\n",
        encoding="utf-8",
    )
    (scene_dir / "scene_manifest.yaml").write_text(
        f"schema_version: workcell_scene_manifest/v1\nscene_name: {name}\npackage_name: {package}\n",
        encoding="utf-8",
    )
    (scene_dir / "layout").mkdir(exist_ok=True)
    (scene_dir / "layout/workcell_studio_layout.yaml").write_text(
        "schema_version: workcell_studio_layout/v1\nitems: [{id: fixture, type: marker}]\n",
        encoding="utf-8",
    )
    (scene_dir / "launch").mkdir(exist_ok=True)
    (scene_dir / "launch/demo.launch.py").write_text(
        "# fixture launch supports use_fake_hardware launch_rviz robot_state_publisher xacro\n", encoding="utf-8"
    )
    (scene_dir / "urdf").mkdir(exist_ok=True)
    (scene_dir / "urdf/scene.urdf.xacro").write_text("<robot name='fixture_scene'><link name='environment'/><link name='end_effector'/></robot>\n", encoding="utf-8")
    (scene_dir / "generated").mkdir(exist_ok=True)
    (scene_dir / "generated/scene_package_readiness.json").write_text(
        json.dumps({"package_name": package}), encoding="utf-8"
    )
    if payload is not None:
        index_path = scene_dir / "generated/scene_visual_mesh_index.json"
        if isinstance(payload, str):
            index_path.write_text(payload, encoding="utf-8")
        else:
            index_path.write_text(json.dumps(payload), encoding="utf-8")

    for relpath in [*missing_authoring_files, *missing_generated_files]:
        _remove_relpath(scene_dir, relpath)
    return scene_dir


def minimal_supported_scene_entry(
    scene_name: str,
    *,
    scene_path: str | None = None,
    package_name: str | None = None,
    build_package_name: str | None = None,
    support_level: str = "supported",
    status: str = "supported",
    known_blocker: str = "",
    authoring_files: Iterable[str] = SCENE_AUTHORING_FILES,
    generated_files: Iterable[str] = SCENE_GENERATED_FILES,
    fake_hardware_launch_command: str | None = None,
    **overrides: Any,
) -> dict[str, Any]:
    package = package_name or scene_name
    build_package = build_package_name or package
    entry: dict[str, Any] = {
        "scene_name": scene_name,
        "package_name": package,
        "scene_path": scene_path or f"scenes/{scene_name}",
        "support_level": support_level,
        "status": status,
        "known_blocker": known_blocker,
        "authoring_files": list(authoring_files),
        "generated_files": list(generated_files),
        "validation_command": f"python3 scripts/validate_builder_generated_scene.py scenes/{scene_name} --json",
        "build_package_name": build_package,
        "build_command": f"colcon build --symlink-install --packages-select {build_package}",
        "fake_hardware_launch_command": fake_hardware_launch_command
        or f"ros2 launch {build_package} demo.launch.py use_fake_hardware:=true launch_rviz:=true",
    }
    entry.update(overrides)
    return entry


def write_supported_scene_catalog(path: Path, entries: Iterable[dict[str, Any]]) -> Path:
    path.write_text(
        yaml.safe_dump({"schema_version": SUPPORTED_SCENE_SCHEMA, "scenes": list(entries)}),
        encoding="utf-8",
    )
    return path


@pytest.fixture
def make_minimal_scene() -> Callable[..., Path]:
    return write_minimal_scene_dir


@pytest.fixture
def make_supported_scene_entry() -> Callable[..., dict[str, Any]]:
    return minimal_supported_scene_entry


@pytest.fixture
def write_scene_catalog() -> Callable[[Path, Iterable[dict[str, Any]]], Path]:
    return write_supported_scene_catalog
