import json
from dataclasses import dataclass
from pathlib import Path

CPP = Path("workcell_builder/workcell_builder/src_workcell_studio_template_instantiator.cpp").read_text(encoding="utf-8")
MAIN = Path("workcell_builder/workcell_builder/gui/mainwindow.cpp").read_text(encoding="utf-8")


@dataclass(frozen=True)
class GuidedScenario:
    name: str
    robot_xacro_resolvable: bool
    ee_xacro_resolvable: bool


SCENARIOS = [
    GuidedScenario("robot+ee_resolved", robot_xacro_resolvable=True, ee_xacro_resolvable=True),
    GuidedScenario("robot_missing", robot_xacro_resolvable=False, ee_xacro_resolvable=True),
    GuidedScenario("ee_missing", robot_xacro_resolvable=True, ee_xacro_resolvable=False),
]


def _expected_scene_xacro_contract(s: GuidedScenario) -> dict:
    robot_include = '<xacro:include filename="$(find robot_description_pkg)/urdf/robot.urdf.xacro"/>'
    ee_include = '<xacro:include filename="$(find end_effector_description_pkg)/urdf/end_effector.urdf.xacro"/>'
    robot_joint = '<joint name="world_to_robot_base" type="fixed">'
    ee_joint = '<joint name="robot_tool0_to_end_effector" type="fixed">'
    robot_placeholder = "robot_visual_placeholder_due_to_missing_robot_xacro"
    ee_placeholder = "missing end-effector xacro -> placeholder visual only"

    expected_present = [robot_joint]
    expected_absent = []
    if s.robot_xacro_resolvable:
        expected_present.append(robot_include)
        expected_absent.append(robot_placeholder)
    else:
        expected_present.append(robot_placeholder)
        expected_absent.append(robot_include)

    if s.ee_xacro_resolvable:
        expected_present += [ee_include, ee_joint]
        expected_absent.append(ee_placeholder)
    else:
        expected_present.append(ee_placeholder)
        expected_absent += [ee_include, ee_joint]

    return {"present": expected_present, "absent": expected_absent}


def _expected_readiness(s: GuidedScenario) -> dict:
    blockers = []
    warnings = []
    robot_visual_mode = "real"

    if not s.robot_xacro_resolvable:
        blockers.extend(["missing robot xacro", "missing robot package"])
        warnings.append("placeholder visual only")
        robot_visual_mode = "placeholder"

    if not s.ee_xacro_resolvable:
        blockers.append("missing end-effector xacro")
        warnings.append("end-effector fallback omitted")

    return {
        "placeholder_launch_only": False,
        "rviz_truth_preview_ready": not blockers,
        "robot_visual_mode": robot_visual_mode,
        "resolved": {
            "robot_xacro": s.robot_xacro_resolvable,
            "end_effector_xacro": s.ee_xacro_resolvable,
        },
        "blockers": blockers,
        "warnings": warnings,
    }


def test_generated_scene_xacro_contract_is_resolution_driven_not_static_placeholders():
    assert 'write_file(scene_dir / "urdf" / "scene.urdf.xacro"' in CPP
    for scenario in SCENARIOS:
        expected = _expected_scene_xacro_contract(scenario)
        for token in expected["present"]:
            assert token in CPP, f"{scenario.name}: expected scene xacro contract token missing: {token}"
        for token in expected["absent"]:
            assert token not in CPP, f"{scenario.name}: unresolved-only placeholder leaked: {token}"


def test_generated_readiness_json_contract_is_deterministic_by_resolution_state():
    assert 'write_file(scene_dir / "generated" / "scene_package_readiness.json"' in CPP
    assert 'json' in CPP.lower()
    for scenario in SCENARIOS:
        expected = _expected_readiness(scenario)
        serialized = json.dumps(expected, sort_keys=True)
        assert f'"placeholder_launch_only": {str(expected["placeholder_launch_only"]).lower()}' in CPP
        assert f'"rviz_truth_preview_ready": {str(expected["rviz_truth_preview_ready"]).lower()}' in CPP
        assert f'"robot_visual_mode": "{expected["robot_visual_mode"]}"' in CPP
        for blocker in expected["blockers"]:
            assert blocker in CPP, f"{scenario.name}: missing blocker in readiness output: {blocker}"
        for warning in expected["warnings"]:
            assert warning in CPP, f"{scenario.name}: missing warning in readiness output: {warning}"
        assert "\"resolved\"" in serialized


def test_generated_launch_contract_keeps_safe_preview_defaults_and_nodes():
    assert "DeclareLaunchArgument('use_fake_hardware', default_value='true')" in CPP
    assert "DeclareLaunchArgument('launch_rviz', default_value='true')" in CPP
    assert "Node(package='robot_state_publisher', executable='robot_state_publisher'" in CPP
    assert "Node(package='rviz2', executable='rviz2', condition=IfCondition(launch_rviz)" in CPP
    assert "ur_robot_driver" not in CPP
    assert "real_hardware:=true" not in CPP


def test_existing_readiness_path_still_checks_scene_xacro():
    assert 'const bool scene_xacro_ready = has("urdf/scene.urdf.xacro") || s.has_scene_urdf_xacro;' in MAIN
