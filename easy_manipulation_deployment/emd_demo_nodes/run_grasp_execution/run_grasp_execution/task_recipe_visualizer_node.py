#!/usr/bin/env python3
from __future__ import annotations
import json
from pathlib import Path

import rclpy
from rclpy.node import Node

from .task_recipe import load_task_recipe, validate_task_recipe, build_offline_task_plan, write_task_plan_report

try:
    from visualization_msgs.msg import Marker, MarkerArray
    MARKERS_AVAILABLE = True
except Exception:
    MARKERS_AVAILABLE = False


class TaskRecipeVisualizerNode(Node):
    def __init__(self) -> None:
        super().__init__('task_recipe_visualizer_node')
        self.declare_parameter('task_recipe_path', '')
        self.declare_parameter('output_dir', '/tmp/workcell_task_preview')
        self.declare_parameter('publish_markers', True)
        self.declare_parameter('dry_run_only', True)
        self.declare_parameter('keep_alive', False)
        self.declare_parameter('frame_id', 'world')

        self._publisher = None
        if MARKERS_AVAILABLE and bool(self.get_parameter('publish_markers').value):
            self._publisher = self.create_publisher(MarkerArray, '/workcell_studio/task_plan_markers', 10)

        self._run_preview()

    def _run_preview(self) -> None:
        recipe_path = Path(str(self.get_parameter('task_recipe_path').value))
        output_dir = Path(str(self.get_parameter('output_dir').value))
        dry_run_only = bool(self.get_parameter('dry_run_only').value)
        frame_id = str(self.get_parameter('frame_id').value)

        try:
            if not dry_run_only:
                raise RuntimeError('dry_run_only must be true')
            recipe = load_task_recipe(recipe_path)
            validation = validate_task_recipe(recipe)
            plan = build_offline_task_plan(recipe, {'frame_id': frame_id, 'rviz_preview': True})
            report = write_task_plan_report(plan, output_dir)
            for idx, step in enumerate(plan.get('steps', []), start=1):
                self.get_logger().info(f"[{idx:02d}] {step.get('name')} -> {step.get('status')}")
            self.get_logger().info(f"Task preview reports: {report['json']} | {report['markdown']} (task_plan_preview.json/task_plan_preview.md)")
            if self._publisher is not None:
                self._publisher.publish(self._build_markers(recipe, validation, frame_id))
            if validation.get('valid'):
                self.get_logger().info('WORKCELL_TASK_RECIPE_RVIZ_PREVIEW: PASS')
            else:
                self.get_logger().info('WORKCELL_TASK_RECIPE_RVIZ_PREVIEW: FAIL')
        except Exception as exc:
            self.get_logger().error(f'WORKCELL_TASK_RECIPE_RVIZ_PREVIEW: FAIL ({exc})')

    def _build_markers(self, recipe: dict, validation: dict, frame_id: str) -> MarkerArray:
        arr = MarkerArray()
        labels = [
            'pick source','approach pick','grasp','retreat','transfer placeholder',
            'approach place','release','retreat','complete'
        ]
        for i, label in enumerate(labels):
            m = Marker()
            m.header.frame_id = frame_id
            m.id = i
            m.ns = 'task_plan'
            m.type = Marker.TEXT_VIEW_FACING if i in (2,6,8) else (Marker.ARROW if i in (1,3,4,5,7) else Marker.SPHERE)
            m.action = Marker.ADD
            m.pose.position.x = float(i) * 0.2
            m.pose.position.y = 0.0
            m.pose.position.z = 0.3
            m.scale.x = 0.06
            m.scale.y = 0.06
            m.scale.z = 0.06
            m.color.a = 1.0
            m.color.r = 0.2
            m.color.g = 0.8 if validation.get('valid') else 0.8
            m.color.b = 0.2 if validation.get('valid') else 0.0
            m.text = label
            arr.markers.append(m)
        safety = Marker()
        safety.header.frame_id = frame_id
        safety.id = 999
        safety.ns = 'task_plan_safety'
        safety.type = Marker.TEXT_VIEW_FACING
        safety.action = Marker.ADD
        safety.pose.position.x = 0.0
        safety.pose.position.y = -0.25
        safety.pose.position.z = 0.45
        safety.scale.z = 0.08
        safety.color.a = 1.0
        safety.color.r = 1.0
        safety.color.g = 1.0
        safety.color.b = 0.0
        grasp_strategy = recipe.get('grasp', {}).get('strategy', 'unknown')
        safety.text = f'Offline dry-run preview only — no robot motion | grasp={grasp_strategy}'
        arr.markers.append(safety)
        return arr


def main(args=None):
    rclpy.init(args=args)
    node = TaskRecipeVisualizerNode()
    keep_alive = bool(node.get_parameter('keep_alive').value)
    if keep_alive:
        rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
