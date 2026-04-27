#!/usr/bin/env python3
"""Guided wizard for generating Cell Definition v1 YAML files."""

from __future__ import annotations

import argparse
import importlib.util
import re
import sys
import tempfile
from pathlib import Path
from typing import Any

REPO_ROOT = Path(__file__).resolve().parents[1]
VALIDATOR_PATH = REPO_ROOT / "scripts" / "validate_cell_definition.py"
WORKCELL_GENERATOR_PATH = REPO_ROOT / "scripts" / "generate_workcell_from_cell_definition.py"

TEMPLATES = (
    "pick_place",
    "sort_by_colour",
    "sort_by_shape",
    "sort_by_class",
    "garbage_sorting",
    "inspection_then_place",
    "custom",
)

ROBOT_PRESETS: dict[str, dict[str, Any]] = {
    "ur5": {
        "model": "ur5",
        "planning_group": "manipulator",
        "base_frame": "world",
        "tool_link": "tool0",
        "home_named_target": "home",
        "safe_joint_state": [],
    },
    "generic_6dof": {
        "model": "generic_6dof",
        "planning_group": "manipulator",
        "base_frame": "world",
        "tool_link": "tool0",
        "home_named_target": "home",
        "safe_joint_state": [],
    },
}

END_EFFECTOR_PRESETS: dict[str, dict[str, Any]] = {
    "robotiq_2f": {
        "id": "robotiq_2f",
        "type": "finger",
        "brand": "robotiq",
        "grasp_frame": "ee_palm",
        "allowed_touch_links": [
            "gripper_finger1_finger_tip_link",
            "gripper_finger2_finger_tip_link",
        ],
    },
    "robotiq_3f": {
        "id": "robotiq_3f",
        "type": "finger",
        "brand": "robotiq",
        "grasp_frame": "ee_palm",
        "allowed_touch_links": ["finger_1_link", "finger_2_link", "finger_middle_link"],
    },
    "suction": {
        "id": "suction",
        "type": "suction",
        "brand": "generic",
        "grasp_frame": "suction_tip",
        "allowed_touch_links": [],
    },
    "generic": {
        "id": "generic_ee",
        "type": "custom",
        "brand": "generic",
        "grasp_frame": "tool0",
        "allowed_touch_links": [],
    },
}

CAMERA_PRESETS: dict[str, dict[str, Any]] = {
    "realsense_d435i": {
        "id": "realsense_d435i",
        "type": "depth_camera",
        "frame": "camera_depth_optical_frame",
        "pointcloud_topic": "/camera/camera/depth/color/points",
        "rgb_topic": "/camera/camera/color/image_raw",
        "depth_topic": "/camera/camera/aligned_depth_to_color/image_raw",
    },
    "generic_rgbd": {
        "id": "generic_rgbd",
        "type": "depth_camera",
        "frame": "camera_depth_optical_frame",
    },
    "none/offline": {
        "id": "offline_camera",
        "type": "none",
        "frame": "world",
    },
}


def _load_module(module_name: str, module_path: Path):
    spec = importlib.util.spec_from_file_location(module_name, module_path)
    if not spec or not spec.loader:
        raise RuntimeError(f"Unable to load module from {module_path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[module_name] = module
    spec.loader.exec_module(module)
    return module


def _to_yaml_scalar(value: Any) -> str:
    if isinstance(value, bool):
        return "true" if value else "false"
    if value is None:
        return "null"
    if isinstance(value, (int, float)):
        return str(value)
    if isinstance(value, str):
        if value == "" or any(ch in value for ch in [":", "#", "[", "]", "{", "}", ","]) or value.strip() != value:
            return f'"{value}"'
        return value
    return str(value)


def _yaml_lines(value: Any, indent: int = 0) -> list[str]:
    prefix = " " * indent
    if isinstance(value, dict):
        lines: list[str] = []
        for key, child in value.items():
            if isinstance(child, dict):
                lines.append(f"{prefix}{key}:")
                lines.extend(_yaml_lines(child, indent + 2))
            elif isinstance(child, list):
                if not child:
                    lines.append(f"{prefix}{key}: []")
                elif all(not isinstance(item, (dict, list)) for item in child):
                    rendered = ", ".join(_to_yaml_scalar(item) for item in child)
                    lines.append(f"{prefix}{key}: [{rendered}]")
                else:
                    lines.append(f"{prefix}{key}:")
                    for item in child:
                        if isinstance(item, dict):
                            first = True
                            for item_key, item_value in item.items():
                                item_prefix = " " * (indent + 2)
                                if first:
                                    if isinstance(item_value, dict):
                                        lines.append(f"{item_prefix}- {item_key}:")
                                        lines.extend(_yaml_lines(item_value, indent + 6))
                                    elif isinstance(item_value, list) and item_value and all(
                                        not isinstance(entry, (dict, list)) for entry in item_value
                                    ):
                                        rendered = ", ".join(_to_yaml_scalar(entry) for entry in item_value)
                                        lines.append(f"{item_prefix}- {item_key}: [{rendered}]")
                                    elif isinstance(item_value, list):
                                        lines.append(f"{item_prefix}- {item_key}:")
                                        lines.extend(_yaml_lines(item_value, indent + 6))
                                    else:
                                        lines.append(f"{item_prefix}- {item_key}: {_to_yaml_scalar(item_value)}")
                                    first = False
                                else:
                                    sub_prefix = " " * (indent + 4)
                                    if isinstance(item_value, dict):
                                        lines.append(f"{sub_prefix}{item_key}:")
                                        lines.extend(_yaml_lines(item_value, indent + 6))
                                    elif isinstance(item_value, list) and item_value and all(
                                        not isinstance(entry, (dict, list)) for entry in item_value
                                    ):
                                        rendered = ", ".join(_to_yaml_scalar(entry) for entry in item_value)
                                        lines.append(f"{sub_prefix}{item_key}: [{rendered}]")
                                    elif isinstance(item_value, list):
                                        lines.append(f"{sub_prefix}{item_key}:")
                                        lines.extend(_yaml_lines(item_value, indent + 6))
                                    else:
                                        lines.append(f"{sub_prefix}{item_key}: {_to_yaml_scalar(item_value)}")
                        else:
                            lines.append(f"{' ' * (indent + 2)}- {_to_yaml_scalar(item)}")
            else:
                lines.append(f"{prefix}{key}: {_to_yaml_scalar(child)}")
        return lines

    if isinstance(value, list):
        return [f"{prefix}- {_to_yaml_scalar(item)}" for item in value]

    return [f"{prefix}{_to_yaml_scalar(value)}"]


def to_yaml_text(data: dict[str, Any]) -> str:
    header = [
        "# GENERATED BY create_cell_definition_wizard.py",
        "# REVIEW REQUIRED: verify dimensions, poses, routing rules, and safety constraints before commissioning.",
        "",
    ]
    return "\n".join(header + _yaml_lines(data)) + "\n"


def _sanitize_id(value: str) -> str:
    lowered = value.strip().lower().replace(" ", "_")
    cleaned = re.sub(r"[^a-z0-9_]", "_", lowered)
    cleaned = re.sub(r"_+", "_", cleaned).strip("_")
    return cleaned or "generated_cell"


def _default_destinations(template: str) -> list[dict[str, Any]]:
    if template == "pick_place":
        ids = ["place_bin", "reject_bin"]
    elif template == "sort_by_colour":
        ids = ["red_bin", "green_bin", "blue_bin", "unknown_reject_bin"]
    elif template == "sort_by_shape":
        ids = ["box_bin", "cylinder_bin", "unknown_reject_bin"]
    elif template == "sort_by_class":
        ids = ["class_a_bin", "class_b_bin", "unknown_reject_bin"]
    elif template == "garbage_sorting":
        ids = ["recyclable_bin", "general_waste_bin", "reject_bin"]
    elif template == "inspection_then_place":
        ids = ["pass_bin", "fail_bin"]
    else:
        ids = ["default_destination", "reject_bin"]

    out = []
    for idx, destination_id in enumerate(ids):
        out.append(
            {
                "id": destination_id,
                "frame": "world",
                "pose_xyz": [0.3, -0.3 + (0.15 * idx), 0.1],
                "pose_rpy": [0.0, 0.0, 0.0],
            }
        )
    return out


def _default_rules(template: str, destinations: list[dict[str, Any]]) -> list[dict[str, Any]]:
    dmap = {item["id"]: item["id"] for item in destinations}
    if template == "pick_place":
        return [
            {"id": "default_place", "when": {"always": True}, "destination": next(iter(dmap))},
        ]
    if template == "sort_by_colour":
        return [
            {"id": "route_red", "when": {"color": "red"}, "destination": dmap.get("red_bin", next(iter(dmap)))},
            {
                "id": "route_green",
                "when": {"color": "green"},
                "destination": dmap.get("green_bin", next(iter(dmap))),
            },
            {"id": "route_blue", "when": {"color": "blue"}, "destination": dmap.get("blue_bin", next(iter(dmap)))},
            {
                "id": "fallback",
                "when": {"always": True},
                "destination": dmap.get("unknown_reject_bin", next(reversed(dmap))),
            },
        ]
    if template == "sort_by_shape":
        return [
            {"id": "route_box", "when": {"shape": "box"}, "destination": dmap.get("box_bin", next(iter(dmap)))},
            {
                "id": "route_cylinder",
                "when": {"shape": "cylinder"},
                "destination": dmap.get("cylinder_bin", next(iter(dmap))),
            },
            {
                "id": "fallback",
                "when": {"always": True},
                "destination": dmap.get("unknown_reject_bin", next(reversed(dmap))),
            },
        ]
    if template == "sort_by_class":
        return [
            {"id": "route_class_a", "when": {"class": "class_a"}, "destination": dmap.get("class_a_bin", next(iter(dmap)))},
            {"id": "route_class_b", "when": {"class": "class_b"}, "destination": dmap.get("class_b_bin", next(iter(dmap)))},
            {
                "id": "fallback",
                "when": {"always": True},
                "destination": dmap.get("unknown_reject_bin", next(reversed(dmap))),
            },
        ]
    if template == "garbage_sorting":
        return [
            {
                "id": "route_recyclable",
                "when": {"class": "recyclable"},
                "destination": dmap.get("recyclable_bin", next(iter(dmap))),
            },
            {
                "id": "route_general",
                "when": {"class": "general"},
                "destination": dmap.get("general_waste_bin", next(iter(dmap))),
            },
            {"id": "fallback", "when": {"always": True}, "destination": dmap.get("reject_bin", next(reversed(dmap)))},
        ]
    if template == "inspection_then_place":
        return [
            {"id": "route_pass", "when": {"class": "pass"}, "destination": dmap.get("pass_bin", next(iter(dmap)))},
            {"id": "fallback", "when": {"always": True}, "destination": dmap.get("fail_bin", next(reversed(dmap)))},
        ]
    return [
        {"id": "fallback", "when": {"always": True}, "destination": next(iter(dmap))},
    ]


def _parse_float_list(raw: str, expected: int | None = None) -> list[float]:
    values = [item.strip() for item in raw.split(",") if item.strip()]
    result = [float(item) for item in values]
    if expected is not None and len(result) != expected:
        raise ValueError(f"expected {expected} values")
    return result


def _choose(prompt: str, values: tuple[str, ...], default: str | None = None) -> str:
    print(f"{prompt}: {', '.join(values)}")
    while True:
        raw = input(f"> [{default or values[0]}] ").strip()
        choice = raw or (default or values[0])
        if choice in values:
            return choice
        print(f"Invalid choice '{choice}'.")


def _interactive_args(args: argparse.Namespace) -> argparse.Namespace:
    print("Cell Definition Wizard (interactive mode)")
    args.template = _choose("Task template", TEMPLATES, default="pick_place")
    args.cell_name = input("> Cell name [Demo Cell]: ").strip() or "Demo Cell"
    args.cell_id = _sanitize_id(input("> Cell ID / package-safe name [demo_cell]: ").strip() or "demo_cell")
    args.robot = _choose("Robot preset", tuple(ROBOT_PRESETS.keys()), default="ur5")
    args.end_effector = _choose("End-effector preset", tuple(END_EFFECTOR_PRESETS.keys()), default="robotiq_2f")
    args.camera = _choose("Camera preset", tuple(CAMERA_PRESETS.keys()), default="realsense_d435i")
    args.pick_source = input("> Pick source object id [commissioning_object]: ").strip() or "commissioning_object"
    args.destinations = input("> Destination IDs (comma-separated, blank for template defaults): ").strip() or None
    args.decision_rules = input("> Decision rule note (optional): ").strip() or "review_template_rules"
    args.self_test_shape = input("> Self-test shape [box]: ").strip() or "box"
    args.self_test_dimensions = input("> Self-test dimensions x,y,z [0.05,0.05,0.05]: ").strip() or "0.05,0.05,0.05"
    args.self_test_pose = input("> Self-test pose xyz [0.45,0.0,0.08]: ").strip() or "0.45,0.0,0.08"
    home_strategy = _choose(
        "Home return strategy",
        ("named_target", "safe_joint_state", "leave_empty_warn"),
        default="named_target",
    )
    args.home_strategy = home_strategy
    if home_strategy == "named_target":
        args.home_named_target = input("> Home named target [home]: ").strip() or "home"
    elif home_strategy == "safe_joint_state":
        args.safe_joint_state = input("> Safe joint state values (comma-separated) [0,0,0,0,0,0]: ").strip() or "0,0,0,0,0,0"
        args.home_named_target = ""
    else:
        args.home_named_target = ""
        args.safe_joint_state = ""
        print("WARN: home return left empty; update before runtime use.")
    if not args.output:
        args.output = str(Path.cwd() / f"{args.cell_id}.cell.yaml")
    return args


def _build_definition(args: argparse.Namespace) -> dict[str, Any]:
    destinations = _default_destinations(args.template)
    if args.destinations:
        destination_ids = [item.strip() for item in args.destinations.split(",") if item.strip()]
        if destination_ids:
            destinations = []
            for idx, destination_id in enumerate(destination_ids):
                destinations.append(
                    {
                        "id": destination_id,
                        "frame": "world",
                        "pose_xyz": [0.3, -0.3 + (0.15 * idx), 0.1],
                        "pose_rpy": [0.0, 0.0, 0.0],
                    }
                )

    rules = _default_rules(args.template, destinations)
    if args.template == "custom" and args.decision_rules:
        rules = [{"id": "fallback", "when": {"always": True}, "destination": destinations[0]["id"]}]

    safe_joint_state: list[float] = []
    home_named_target = args.home_named_target if args.home_named_target is not None else "home"
    if args.home_strategy == "safe_joint_state" and args.safe_joint_state:
        safe_joint_state = _parse_float_list(args.safe_joint_state)
        home_named_target = ""
    elif args.home_strategy == "leave_empty_warn":
        safe_joint_state = []
        home_named_target = ""

    dims = _parse_float_list(args.self_test_dimensions, expected=3)
    pose_xyz = _parse_float_list(args.self_test_pose, expected=3)

    robot = dict(ROBOT_PRESETS[args.robot])
    robot["home_named_target"] = home_named_target
    robot["safe_joint_state"] = safe_joint_state

    camera = dict(CAMERA_PRESETS[args.camera])
    perception_mode = "offline" if args.camera == "none/offline" else "online"

    return {
        "schema_version": "cell_definition/v1",
        "cell": {
            "id": _sanitize_id(args.cell_id),
            "name": args.cell_name,
            "description": "Generated by cell definition wizard. REVIEW_REQUIRED before commissioning.",
        },
        "robot": robot,
        "end_effector": dict(END_EFFECTOR_PRESETS[args.end_effector]),
        "camera": camera,
        "perception": {
            "mode": perception_mode,
            "notes": "Review sensor calibration and frame alignment.",
        },
        "environment": {
            "frame": "world",
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
                "id": args.pick_source,
                "class": "unknown",
                "shape": args.self_test_shape,
                "color": "unknown",
                "material": "unknown",
                "frame": "world",
                "dimensions": dims,
                "pose_xyz": pose_xyz,
                "pose_rpy": [0.0, 0.0, 0.0],
            }
        ],
        "task": {
            "id": f"{_sanitize_id(args.cell_id)}_task",
            "type": args.template,
            "source_object": args.pick_source,
            "destinations": destinations,
            "rules": rules,
        },
        "self_test": {
            "enabled": True,
            "object_shape": args.self_test_shape,
            "object_dimensions": dims,
            "object_pose_xyz": pose_xyz,
            "notes": "Update to match commissioning object before real robot tests.",
        },
        "home_return": {
            "strategy": args.home_strategy,
            "named_target": home_named_target,
            "safe_joint_state": safe_joint_state,
            "notes": "If empty, configure safe home before runtime use.",
        },
        "commissioning": {
            "self_test_enabled": True,
            "export_bundle": True,
            "require_operator_review": True,
            "operator_notes": args.decision_rules or "Review decision rules and destination poses.",
        },
    }


def _validate_data(data: dict[str, Any], path_hint: Path) -> tuple[bool, list[str], list[str]]:
    validator = _load_module("wizard_cell_validator", VALIDATOR_PATH)
    with tempfile.TemporaryDirectory(prefix="cell_wizard_validate_") as tmpdir:
        temp_path = Path(tmpdir) / path_hint.name
        temp_path.write_text(to_yaml_text(data), encoding="utf-8")
        loaded, parser_name, parser_notes = validator.load_yaml(temp_path)
        summary = validator.validate_cell_definition(loaded, temp_path, parser_name, parser_notes)
    return summary.ok, summary.errors, summary.warnings


def _validate_args(args: argparse.Namespace) -> str | None:
    if args.template and args.template not in TEMPLATES:
        return f"Invalid template '{args.template}'. Use --list-templates."
    if args.robot and args.robot not in ROBOT_PRESETS:
        return f"Invalid robot preset '{args.robot}'. Use --list-presets."
    if args.end_effector and args.end_effector not in END_EFFECTOR_PRESETS:
        return f"Invalid end-effector preset '{args.end_effector}'. Use --list-presets."
    if args.camera and args.camera not in CAMERA_PRESETS:
        return f"Invalid camera preset '{args.camera}'. Use --list-presets."
    return None


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--list-templates", action="store_true", help="List supported task templates and exit")
    parser.add_argument("--list-presets", action="store_true", help="List supported robot/EE/camera presets and exit")
    parser.add_argument("--template", type=str, help="Task template")
    parser.add_argument("--cell-name", type=str, help="Cell display name")
    parser.add_argument("--cell-id", type=str, help="Cell ID / package-safe name")
    parser.add_argument("--robot", type=str, help="Robot preset")
    parser.add_argument("--end-effector", type=str, help="End-effector preset")
    parser.add_argument("--camera", type=str, help="Camera preset")
    parser.add_argument("--pick-source", type=str, default="commissioning_object", help="Source object id")
    parser.add_argument("--destinations", type=str, help="Comma-separated destination IDs")
    parser.add_argument("--decision-rules", type=str, help="Operator note for rules")
    parser.add_argument("--self-test-shape", type=str, default="box")
    parser.add_argument("--self-test-dimensions", type=str, default="0.05,0.05,0.05")
    parser.add_argument("--self-test-pose", type=str, default="0.45,0.0,0.08")
    parser.add_argument(
        "--home-strategy",
        choices=("named_target", "safe_joint_state", "leave_empty_warn"),
        default="named_target",
    )
    parser.add_argument("--home-named-target", type=str, default="home")
    parser.add_argument("--safe-joint-state", type=str, default="")
    parser.add_argument("--output", type=str, help="Output YAML path")
    parser.add_argument("--generate-workcell", action="store_true")
    parser.add_argument("--workcell-output-dir", type=str, default="/tmp/generated_workcells")
    parser.add_argument("--package-name", type=str)
    parser.add_argument("--force", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args(argv)

    if not args.output and args.cell_id:
        args.output = str(Path.cwd() / f"{_sanitize_id(args.cell_id)}.cell.yaml")
    return args


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)

    if args.list_templates:
        print("Supported templates:")
        for item in TEMPLATES:
            print(f"- {item}")
        return 0

    if args.list_presets:
        print("Robot presets:")
        for key in ROBOT_PRESETS:
            print(f"- {key}")
        print("End-effector presets:")
        for key in END_EFFECTOR_PRESETS:
            print(f"- {key}")
        print("Camera presets:")
        for key in CAMERA_PRESETS:
            print(f"- {key}")
        return 0

    interactive = not all(
        [args.template, args.cell_name, args.cell_id, args.robot, args.end_effector, args.camera]
    )
    if interactive:
        args = _interactive_args(args)

    message = _validate_args(args)
    if message:
        print(f"FAIL: {message}")
        return 2

    if not args.output:
        args.output = str(Path.cwd() / f"{_sanitize_id(args.cell_id)}.cell.yaml")

    try:
        data = _build_definition(args)
    except Exception as exc:
        print(f"FAIL: Unable to build cell definition: {exc}")
        return 1

    output_path = Path(args.output)
    yaml_text = to_yaml_text(data)

    ok, errors, warnings = _validate_data(data, output_path)
    if not ok:
        print("FAIL: Generated YAML did not pass validation.")
        for error in errors:
            print(f" - {error}")
        return 1

    if args.dry_run:
        print(yaml_text)
        print("PASS: dry-run completed; no files were written.")
    else:
        if output_path.exists() and not args.force:
            print(f"FAIL: Refusing to overwrite existing file: {output_path} (use --force).")
            return 1
        output_path.parent.mkdir(parents=True, exist_ok=True)
        output_path.write_text(yaml_text, encoding="utf-8")
        print(f"PASS: wrote {output_path}")

    for warning in warnings:
        print(f"WARN: {warning}")

    if args.generate_workcell:
        workcell_generator = _load_module("wizard_workcell_generator", WORKCELL_GENERATOR_PATH)
        package_name = args.package_name or f"generated_{_sanitize_id(args.cell_id)}"
        rc = workcell_generator.generate_package(
            cell_definition_path=output_path.resolve(),
            output_dir=Path(args.workcell_output_dir).resolve(),
            package_name=package_name,
            force=args.force,
            dry_run=args.dry_run,
        )
        if rc != 0:
            print("FAIL: --generate-workcell step failed.")
            return rc

    print("Next steps:")
    print(f"  python3 scripts/validate_cell_definition.py {output_path}")
    print(
        "  python3 scripts/generate_workcell_from_cell_definition.py "
        f"{output_path} --output-dir {args.workcell_output_dir} --package-name "
        f"{args.package_name or f'generated_{_sanitize_id(args.cell_id)}'} --force"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
