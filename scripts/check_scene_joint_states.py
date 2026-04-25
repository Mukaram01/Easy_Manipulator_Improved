#!/usr/bin/env python3
"""Check JointState message shape for a given topic."""

import argparse
import sys
from threading import Event

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class JointStateChecker(Node):
    def __init__(self, topic: str) -> None:
        super().__init__("check_scene_joint_states")
        self._topic = topic
        self._event = Event()
        self._message = None
        self.create_subscription(JointState, topic, self._callback, 10)

    def _callback(self, msg: JointState) -> None:
        self._message = msg
        self._event.set()

    @property
    def message(self) -> JointState | None:
        return self._message

    def wait_for_message(self, timeout_sec: float) -> bool:
        return self._event.wait(timeout=timeout_sec)

    def publisher_count(self) -> int | None:
        try:
            return len(self.get_publishers_info_by_topic(self._topic))
        except Exception:
            return None


def _validate_message(msg: JointState) -> list[str]:
    errors = []
    name_len = len(msg.name)
    if len(msg.position) != name_len:
        errors.append("len(name) != len(position)")
    if msg.velocity and len(msg.velocity) != name_len:
        errors.append("velocity is non-empty and len(velocity) != len(name)")
    if msg.effort and len(msg.effort) != name_len:
        errors.append("effort is non-empty and len(effort) != len(name)")
    return errors


def main() -> int:
    parser = argparse.ArgumentParser(description="Check a JointState topic for malformed messages.")
    parser.add_argument("topic", nargs="?", default="/joint_states", help="JointState topic to inspect")
    parser.add_argument("--timeout", type=float, default=10.0, help="Seconds to wait for a message")
    args = parser.parse_args()

    rclpy.init()
    node = JointStateChecker(args.topic)

    try:
        end_time = node.get_clock().now().nanoseconds + int(args.timeout * 1e9)
        while rclpy.ok() and not node.wait_for_message(0.1):
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.get_clock().now().nanoseconds >= end_time:
                print(f"No JointState message received on {args.topic} within {args.timeout:.1f}s", file=sys.stderr)
                return 2

        msg = node.message
        if msg is None:
            print(f"No JointState message received on {args.topic}", file=sys.stderr)
            return 2

        print(f"topic: {args.topic}")
        publisher_count = node.publisher_count()
        print(f"publisher_count: {publisher_count if publisher_count is not None else 'unavailable'}")
        print(f"len(name): {len(msg.name)}")
        print(f"len(position): {len(msg.position)}")
        print(f"len(velocity): {len(msg.velocity)}")
        print(f"len(effort): {len(msg.effort)}")
        print(f"names: {list(msg.name)}")
        print(f"positions: {list(msg.position)}")

        errors = _validate_message(msg)
        if errors:
            for error in errors:
                print(f"ERROR: {error}", file=sys.stderr)
            return 1
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
