#!/usr/bin/env python3
"""Convert saved Stage A telemetry evidence to an offline C++ regression fixture.

Run with the ROS/workcell environment sourced. No ROS node, controller, scene
service, simulator, or execution authority is created.
"""
import argparse
import hashlib
import json
from pathlib import Path

import xacro
from moveit_msgs.msg import PlanningScene, RobotTrajectory
from rclpy.serialization import serialize_message
from rosidl_runtime_py.set_message import set_message_fields


def decode_octets(value):
    if isinstance(value, dict):
        return {key: (item.encode("latin1") if key == "operation" and isinstance(item, str)
                      else decode_octets(item)) for key, item in value.items()}
    if isinstance(value, list):
        return [decode_octets(item) for item in value]
    return value


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("stage_directory", type=Path, help="Saved 03-telemetry directory")
    parser.add_argument("output_directory", type=Path)
    args = parser.parse_args()
    root, output = args.stage_directory.resolve(), args.output_directory.resolve()
    output.mkdir(parents=True, exist_ok=True)
    hashes = {}

    def read(path):
        raw = path.read_bytes()
        hashes[str(path)] = hashlib.sha256(raw).hexdigest()
        return raw

    summary = json.loads(read(root / "telemetry/summary.json"))
    goal = summary["owned_execution_goal"]
    if goal["stage"] != "EXECUTE_APPROACH" or not goal["accepted"]:
        raise ValueError("Fixture requires the original accepted Stage A approach")
    scene = json.loads(read(root / "telemetry/planning_scene_before.json"))
    for name, data, cls in (("scene", scene, PlanningScene),
                            ("trajectory", goal["trajectory"], RobotTrajectory)):
        message = cls()
        set_message_fields(message, decode_octets(data))
        (output / (name + ".cdr")).write_bytes(serialize_message(message))
    (output / "robot.urdf").write_bytes(read(root / "runtime/robot.urdf"))
    srdf = root / "ur5_2f_test/urdf/arm_hand.srdf.xacro"
    read(srdf)
    (output / "robot.srdf").write_text(xacro.process_file(str(srdf)).toxml())
    for included in xacro.all_includes:
        read(Path(included))
    (output / "provenance.json").write_text(json.dumps(hashes, indent=2) + "\n")


if __name__ == "__main__":
    main()
