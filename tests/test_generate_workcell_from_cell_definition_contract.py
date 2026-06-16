#!/usr/bin/env python3
from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path

import yaml

from scripts.generate_workcell_from_cell_definition import generate_package

REPO_ROOT = Path(__file__).resolve().parents[1]
SCRIPT = REPO_ROOT / "scripts" / "generate_workcell_from_cell_definition.py"

REQUIRED_GENERATED_FILES = [
    "package.xml",
    "CMakeLists.txt",
    "environment.yaml",
    "cell_definition.yaml",
    "scene_manifest.yaml",
    "layout/workcell_studio_layout.yaml",
    "launch/demo.launch.py",
    "urdf/scene.urdf.xacro",
    "generated/scene_visual_mesh_index.json",
]

READINESS_METADATA_FILES = [
    "generated/commissioning_summary.md",
    "generated/validation_report.md",
    "generated/generated_workcell_summary.json",
    "generated/generated_gated_dry_run_command.sh",
    "generated/generated_detected_objects_example.yaml",
    "generated/generated_environment_objects.yaml",
    "generated/generated_destinations.yaml",
]

REAL_HARDWARE_DRIVER_ACTIVATION_STRINGS = [
    "ur_robot_driver",
    "external_control",
    "dashboard_client",
    "ros2_control_node",
    "controller_manager",
    "scaled_joint_trajectory_controller",
]


def _write_minimal_scene(scene_dir: Path) -> Path:
    (scene_dir / "layout").mkdir(parents=True, exist_ok=True)
    (scene_dir / "environment.yaml").write_text(
        yaml.safe_dump(
            {
                "schema_version": "workcell_environment/v1",
                "robot": {"name": "ur5"},
                "end_effector": {"name": "robotiq_2f"},
                "objects": {"part": {"class": "box"}},
                "fake_hardware_first": True,
                "runtime_execution_enabled": False,
            },
            sort_keys=False,
        ),
        encoding="utf-8",
    )
    (scene_dir / "environment_layout.yaml").write_text(
        yaml.safe_dump(
            {
                "schema_version": "environment_layout/v1",
                "assets": [
                    {
                        "id": "robot_base",
                        "type": "robot_base",
                        "pose": {"xyz": [0.0, 0.0, 0.0], "rpy": [0.0, 0.0, 0.0]},
                    },
                    {
                        "id": "table_main",
                        "type": "table",
                        "pose": {"xyz": [0.45, 0.0, 0.0], "rpy": [0.0, 0.0, 0.0]},
                    },
                ],
                "zones": [
                    {
                        "id": "pick_zone",
                        "type": "pick_area",
                        "pose": {"xyz": [0.45, 0.0, 0.08], "rpy": [0.0, 0.0, 0.0]},
                    }
                ],
            },
            sort_keys=False,
        ),
        encoding="utf-8",
    )
    (scene_dir / "layout" / "workcell_studio_layout.yaml").write_text(
        yaml.safe_dump(
            {
                "schema_version": "workcell_studio_layout/v1",
                "items": [
                    {
                        "id": "editable_table_main",
                        "type": "table",
                        "pose": {"xyz": [0.45, 0.0, 0.0], "rpy": [0.0, 0.0, 0.0]},
                    }
                ],
                "test_sentinel": "source workcell studio layout preserved",
            },
            sort_keys=False,
        ),
        encoding="utf-8",
    )
    cell_definition = {
        "schema_version": "cell_definition/v1",
        "cell": {
            "id": "contract_cell",
            "name": "Contract Test Cell",
            "planning_frame": "world",
        },
        "robot": {
            "model": "ur5",
            "planning_group": "manipulator",
            "base_frame": "world",
            "tool_link": "tool0",
            "home_named_target": "home",
            "safe_joint_state": [],
        },
        "end_effector": {
            "id": "robotiq_2f",
            "type": "finger",
            "brand": "robotiq",
            "grasp_frame": "tool0",
            "allowed_touch_links": ["left_inner_finger", "right_inner_finger"],
        },
        "camera": {
            "id": "realsense_d435i",
            "type": "depth_camera",
            "frame": "camera_depth_optical_frame",
        },
        "environment": {
            "frame": "world",
            "layout": "environment_layout.yaml",
            "support_surfaces": [
                {
                    "id": "table_main",
                    "type": "table",
                    "frame": "world",
                    "pose_xyz": [0.0, 0.0, 0.0],
                    "pose_rpy": [0.0, 0.0, 0.0],
                    "dimensions": [1.0, 1.0, 0.05],
                }
            ],
        },
        "objects": [
            {
                "id": "part",
                "class": "box",
                "shape": "box",
                "color": "blue",
                "material": "plastic",
                "frame": "world",
                "dimensions": [0.05, 0.05, 0.05],
                "pose_xyz": [0.45, 0.0, 0.08],
                "pose_rpy": [0.0, 0.0, 0.0],
            }
        ],
        "task": {
            "id": "pick_place_task",
            "type": "pick_place",
            "source_object": "part",
            "destinations": [
                {
                    "id": "place_target",
                    "frame": "world",
                    "pose_xyz": [0.6, -0.2, 0.1],
                    "pose_rpy": [0, 0, 0],
                }
            ],
            "rules": [
                {
                    "id": "default_place",
                    "when": {"always": True},
                    "destination": "place_target",
                }
            ],
        },
        "commissioning": {
            "self_test_enabled": True,
            "export_bundle": False,
            "fake_hardware_default": True,
        },
    }
    cell_path = scene_dir / "cell_definition.yaml"
    cell_path.write_text(
        yaml.safe_dump(cell_definition, sort_keys=False), encoding="utf-8"
    )
    return cell_path


def _assert_generated_contract(package_dir: Path) -> None:
    for rel_path in REQUIRED_GENERATED_FILES + READINESS_METADATA_FILES:
        assert (
            package_dir / rel_path
        ).is_file(), f"missing generated contract file: {rel_path}"

    launch_text = (package_dir / "launch" / "demo.launch.py").read_text(
        encoding="utf-8"
    )
    assert '"use_fake_hardware"' in launch_text
    assert 'default_value="true"' in launch_text
    for forbidden in REAL_HARDWARE_DRIVER_ACTIVATION_STRINGS:
        assert forbidden not in launch_text

    mesh_index = json.loads(
        (package_dir / "generated" / "scene_visual_mesh_index.json").read_text(
            encoding="utf-8"
        )
    )
    assert mesh_index["safe_for_preview"] is False
    assert mesh_index["source_urdf_xacro_path"] == "urdf/scene.urdf.xacro"

    summary = json.loads(
        (package_dir / "generated" / "generated_workcell_summary.json").read_text(
            encoding="utf-8"
        )
    )
    assert summary["approval"]["status"] == "unapproved"
    assert summary["recommended_commands"]["gated_dry_run"]


def test_generate_package_preserves_authoring_contract_files(tmp_path: Path) -> None:
    source_scene = tmp_path / "source_scene"
    cell_path = _write_minimal_scene(source_scene)
    out_dir = tmp_path / "scenes"

    assert (
        generate_package(
            cell_path, out_dir, "contract_scene", force=True, dry_run=False
        )
        == 0
    )

    package_dir = out_dir / "contract_scene"
    _assert_generated_contract(package_dir)
    assert (package_dir / "environment.yaml").read_text(encoding="utf-8") == (
        source_scene / "environment.yaml"
    ).read_text(encoding="utf-8")
    layout = yaml.safe_load(
        (package_dir / "layout" / "workcell_studio_layout.yaml").read_text(
            encoding="utf-8"
        )
    )
    assert layout["test_sentinel"] == "source workcell studio layout preserved"


def test_cli_force_in_place_regeneration_keeps_source_layout_and_environment(
    tmp_path: Path,
) -> None:
    scenes_dir = tmp_path / "scenes"
    package_dir = scenes_dir / "contract_scene"
    cell_path = _write_minimal_scene(package_dir)
    expected_environment = (package_dir / "environment.yaml").read_text(
        encoding="utf-8"
    )
    expected_environment_layout = (package_dir / "environment_layout.yaml").read_text(
        encoding="utf-8"
    )
    expected_workcell_layout = (
        package_dir / "layout" / "workcell_studio_layout.yaml"
    ).read_text(encoding="utf-8")

    proc = subprocess.run(
        [
            sys.executable,
            str(SCRIPT),
            str(cell_path),
            "--output-dir",
            str(scenes_dir),
            "--package-name",
            "contract_scene",
            "--force",
        ],
        cwd=REPO_ROOT,
        capture_output=True,
        text=True,
        check=False,
    )

    assert proc.returncode == 0, proc.stdout + proc.stderr
    _assert_generated_contract(package_dir)
    assert (package_dir / "environment.yaml").read_text(
        encoding="utf-8"
    ) == expected_environment
    assert (package_dir / "environment_layout.yaml").read_text(
        encoding="utf-8"
    ) == expected_environment_layout
    assert (package_dir / "layout" / "workcell_studio_layout.yaml").read_text(
        encoding="utf-8"
    ) == expected_workcell_layout


def test_visual_mesh_index_passes_workspace_root_to_extractor(tmp_path: Path, monkeypatch) -> None:
    from scripts import generate_workcell_from_cell_definition as generator

    workspace_root = tmp_path / "ws"
    package_dir = (
        workspace_root / "src" / "easy_manipulation_deployment" / "scenes" / "demo_scene"
    )
    (package_dir / "urdf").mkdir(parents=True)
    (package_dir / "urdf" / "scene.urdf.xacro").write_text(
        '<robot name="demo"><link name="world"><visual><geometry>'
        '<mesh filename="package://demo_assets/meshes/part.stl"/>'
        "</geometry></visual></link></robot>",
        encoding="utf-8",
    )
    extractor = tmp_path / "fake_extractor.py"
    extractor.write_text(
        """
import os
from pathlib import Path
EXTRACTOR_VERSION = 'test'
EXPECTED_WORKSPACE_ROOT = Path(os.environ['EXPECTED_WORKSPACE_ROOT'])

def expand_xacro(urdf_path, scene_dir=None, xacro_args=None, workspace_root=None):
    assert xacro_args == {'use_fake_hardware': 'true', 'robot_prefix': '', 'tool_prefix': ''}
    assert Path(workspace_root) == EXPECTED_WORKSPACE_ROOT
    return (Path(urdf_path).read_text(encoding='utf-8'), '', '', ['xacro'])

def discover_package_map(scene_dir, workspace_root=None, package_names=None):
    assert Path(workspace_root) == EXPECTED_WORKSPACE_ROOT
    assert package_names == ['demo_assets']
    return {'demo_assets': EXPECTED_WORKSPACE_ROOT / 'src' / 'demo_assets'}, {'resolution_paths': []}

def extract_referenced_package_names(text):
    return ['demo_assets']

def extract_from_urdf(xml_text, package_map):
    return [{'id': 'mesh', 'link': 'world', 'parent_link': 'world', 'geometry_type': 'mesh', 'resolved': True, 'render_expected': True}]

def contains_placeholder(text):
    return False

def discover_xacro_command():
    return ('xacro', True, '')
""",
        encoding="utf-8",
    )
    monkeypatch.setattr(generator, "MESH_INDEX_EXTRACTOR_PATH", extractor)
    monkeypatch.setenv("EXPECTED_WORKSPACE_ROOT", str(workspace_root.resolve()))
    warnings: list[str] = []

    assert generator._write_scene_visual_mesh_index("demo_scene", package_dir, warnings) is None

    payload = json.loads(
        (package_dir / "generated" / "scene_visual_mesh_index.json").read_text(
            encoding="utf-8"
        )
    )
    assert payload["workspace_root"] == str(workspace_root.resolve())
    assert payload["xacro_command"] == ["xacro"]
    assert payload["visual_readiness"]["status"] == "PASS"
    assert warnings == []



def test_visual_mesh_index_marks_xacro_lite_ur_robot_macro_as_degraded(
    tmp_path: Path, monkeypatch
) -> None:
    from scripts import generate_workcell_from_cell_definition as generator

    package_dir = tmp_path / "scene"
    (package_dir / "urdf").mkdir(parents=True)
    (package_dir / "urdf" / "scene.urdf.xacro").write_text(
        '<robot name="demo"><link name="world"><visual><geometry>'
        '<mesh filename="package://demo_assets/meshes/part.stl"/>'
        "</geometry></visual></link></robot>",
        encoding="utf-8",
    )
    extractor = tmp_path / "lite_extractor.py"
    extractor.write_text(
        """
EXTRACTOR_VERSION = 'test'

def expand_xacro(urdf_path, scene_dir=None, xacro_args=None, workspace_root=None):
    return (open(urdf_path, encoding='utf-8').read(), True, 'skipped unresolved macros: ur_robot', ['xacro-lite', str(urdf_path)])

def discover_package_map(scene_dir, workspace_root=None, package_names=None):
    return {'demo_assets': scene_dir}, {'resolution_paths': []}

def extract_referenced_package_names(text):
    return ['demo_assets']

def extract_from_urdf(xml_text, package_map):
    return [{'id': 'mesh', 'link': 'world', 'parent_link': 'world', 'geometry_type': 'mesh', 'resolved': True, 'render_expected': True}]

def contains_placeholder(text):
    return False

def discover_xacro_command():
    return ('', False, 'xacro unavailable in test')
""",
        encoding="utf-8",
    )
    monkeypatch.setattr(generator, "MESH_INDEX_EXTRACTOR_PATH", extractor)
    warnings: list[str] = []

    assert generator._write_scene_visual_mesh_index("demo_scene", package_dir, warnings) is None

    payload = json.loads(
        (package_dir / "generated" / "scene_visual_mesh_index.json").read_text(
            encoding="utf-8"
        )
    )
    expected = "xacro-lite skipped robot macro ur_robot; preview uses degraded fallback geometry"
    assert payload["extraction_mode"] == "xacro_lite_expanded"
    assert payload["safe_for_preview"] is False
    assert expected in payload["blockers"]
    assert expected in payload["warnings"]
    assert expected in payload["visual_readiness"]["reasons"]

def test_visual_mesh_index_warns_when_xacro_expansion_falls_back(
    tmp_path: Path, monkeypatch
) -> None:
    from scripts import generate_workcell_from_cell_definition as generator

    package_dir = tmp_path / "scene"
    (package_dir / "urdf").mkdir(parents=True)
    (package_dir / "urdf" / "scene.urdf.xacro").write_text(
        '<robot name="demo"><link name="world"><visual><geometry>'
        '<mesh filename="package://missing_assets/meshes/part.stl"/>'
        "</geometry></visual></link></robot>",
        encoding="utf-8",
    )
    extractor = tmp_path / "fallback_extractor.py"
    extractor.write_text(
        """
EXTRACTOR_VERSION = 'test'

def expand_xacro(urdf_path, scene_dir=None, xacro_args=None, workspace_root=None):
    return ('', 'best_effort_recursive', 'xacro unavailable in test', [])

def discover_package_map(scene_dir, workspace_root=None, package_names=None):
    return {}, {'resolution_paths': []}

def extract_referenced_package_names(text):
    return ['missing_assets']

def extract_from_urdf(xml_text, package_map):
    return [{'id': 'mesh', 'link': 'world', 'parent_link': 'world', 'geometry_type': 'mesh', 'resolved': False, 'render_expected': False}]

def contains_placeholder(text):
    return False

def discover_xacro_command():
    return ('', False, 'xacro unavailable in test')
""",
        encoding="utf-8",
    )
    monkeypatch.setattr(generator, "MESH_INDEX_EXTRACTOR_PATH", extractor)
    warnings: list[str] = []

    assert generator._write_scene_visual_mesh_index("demo_scene", package_dir, warnings) is None

    payload = json.loads(
        (package_dir / "generated" / "scene_visual_mesh_index.json").read_text(
            encoding="utf-8"
        )
    )
    assert payload["safe_for_preview"] is False
    assert payload["visual_readiness"]["status"] == "WARN"
    assert any(
        "fully expanded xacro" in reason
        for reason in payload["visual_readiness"]["reasons"]
    )
    assert any(
        "primitive fallback alone" in reason
        for reason in payload["visual_readiness"]["reasons"]
    )
