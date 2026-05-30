from __future__ import annotations

import json
import sys
from pathlib import Path
from typing import Any

import pytest

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

MINIMAL_AUTHORING_FILES = ["environment.yaml", "layout/workcell_studio_layout.yaml"]
MINIMAL_GENERATED_FILES = [
    "cell_definition.yaml",
    "launch/demo.launch.py",
    "urdf/scene.urdf.xacro",
    "generated/scene_visual_mesh_index.json",
]


def _write_text(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8")


@pytest.fixture
def minimal_scene_factory(tmp_path: Path):
    """Write configurable minimal scene directories and matching catalog entries."""

    def _make_scene(
        name: str,
        *,
        scene_path: str | None = None,
        package_xml_name: str | None = None,
        cmake_project_name: str | None = None,
        package_name: str | None = None,
        build_package_name: str | None = None,
        missing_authoring_files: list[str] | tuple[str, ...] = (),
        missing_generated_files: list[str] | tuple[str, ...] = (),
        mesh_index_payload: Any = None,
        catalog_status: str = "supported",
        support_level: str = "supported",
        known_blocker: str = "",
        fake_hardware_launch_command: str | None = None,
        build_command: str | None = None,
        validation_command: str | None = None,
        enabled: bool = True,
        repo_root: Path | None = None,
    ) -> tuple[Path, dict[str, Any]]:
        root = repo_root or tmp_path
        rel_scene_path = scene_path or f"scenes/{name}"
        scene_dir = root / rel_scene_path
        package_xml_name = package_xml_name or package_name or name
        cmake_project_name = cmake_project_name or package_xml_name
        package_name = package_name or package_xml_name
        build_package_name = build_package_name or package_name
        fake_hardware_launch_command = fake_hardware_launch_command or (
            f"ros2 launch {package_name} demo.launch.py use_fake_hardware:=true launch_rviz:=true"
        )
        build_command = build_command or f"colcon build --symlink-install --packages-select {build_package_name}"
        validation_command = validation_command or f"python3 scripts/validate_builder_generated_scene.py {rel_scene_path} --json"

        _write_text(scene_dir / "package.xml", f"<package><name>{package_xml_name}</name></package>\n")
        _write_text(scene_dir / "CMakeLists.txt", f"project({cmake_project_name})\nament_package()\n")

        file_payloads = {
            "environment.yaml": "name: env\n",
            "layout/workcell_studio_layout.yaml": "schema_version: workcell_studio_layout/v1\nitems: [{id: item_1, type: marker}]\n",
            "cell_definition.yaml": "robot: ur5\nend_effector: suction\nenvironment: demo\n",
            "launch/demo.launch.py": "use_fake_hardware launch_rviz robot_state_publisher rviz xacro\n",
            "urdf/scene.urdf.xacro": "<robot name='x'></robot>\n",
            "scene_manifest.yaml": "schema_version: workcell_studio_scene_manifest/v1\n",
            "generated/scene_package_readiness.json": json.dumps({"package_name": package_xml_name}),
        }
        for rel, text in file_payloads.items():
            _write_text(scene_dir / rel, text)

        if mesh_index_payload is None:
            mesh_index_payload = {"items": [{"render_expected": True}]}
        if isinstance(mesh_index_payload, str):
            _write_text(scene_dir / "generated/scene_visual_mesh_index.json", mesh_index_payload)
        else:
            _write_text(scene_dir / "generated/scene_visual_mesh_index.json", json.dumps(mesh_index_payload))

        for rel in [*missing_authoring_files, *missing_generated_files]:
            target = scene_dir / rel
            if target.exists():
                target.unlink()

        entry = {
            "scene_name": name,
            "package_name": package_name,
            "scene_path": rel_scene_path,
            "support_level": support_level,
            "status": catalog_status,
            "known_blocker": known_blocker,
            "authoring_files": list(MINIMAL_AUTHORING_FILES),
            "generated_files": list(MINIMAL_GENERATED_FILES),
            "validation_command": validation_command,
            "build_package_name": build_package_name,
            "build_command": build_command,
            "fake_hardware_launch_command": fake_hardware_launch_command,
            "enabled": enabled,
        }
        return scene_dir, entry

    return _make_scene
