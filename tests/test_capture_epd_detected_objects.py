#!/usr/bin/env python3
from __future__ import annotations

import unittest
from unittest.mock import patch

from scripts.capture_epd_detected_objects import (
    _normalize_pose_with_tf,
    convert_epd_message_to_detected_objects,
    create_qos_profile,
)
from scripts.validate_detected_objects import validate_detected_objects


class CaptureEPDDetectedObjectsTests(unittest.TestCase):
    def _msg(self, *, frame_id: str = "camera_depth_optical_frame"):
        return {
            "header": {"frame_id": frame_id},
            "objects": [
                {
                    "name": "red_box",
                    "class_id": "box",
                    "confidence": 0.92,
                    "centroid": {"x": 0.1, "y": 0.2, "z": 0.3},
                    "bounding_box": {"x": 0.04, "y": 0.05, "z": 0.06},
                    "colour": "red",
                    "shape": "box",
                }
            ],
        }

    def test_valid_detection_converts(self) -> None:
        payload, warnings = convert_epd_message_to_detected_objects(
            self._msg(), "/easy_perception_deployment/epd_localize_output", "ur5_2f_test", "camera_depth_optical_frame"
        )
        self.assertEqual(payload["schema_version"], "detected_objects/v1")
        self.assertEqual(len(payload["objects"]), 1)
        self.assertFalse(warnings)

    def test_missing_frame_uses_fallback_and_warns(self) -> None:
        payload, warnings = convert_epd_message_to_detected_objects(
            self._msg(frame_id=""), "/easy_perception_deployment/epd_localize_output", "ur5_2f_test", "fallback_frame"
        )
        self.assertEqual(payload["objects"][0]["pose"]["frame_id"], "fallback_frame")
        self.assertTrue(any("fallback" in w for w in warnings))

    def test_no_detections_results_empty(self) -> None:
        payload, _ = convert_epd_message_to_detected_objects(
            {"header": {"frame_id": "camera_depth_optical_frame"}, "objects": []}, "t", "ur5_2f_test", "fallback"
        )
        self.assertEqual(payload["objects"], [])

    def test_label_mapping_stable(self) -> None:
        msg = {"header": {"frame_id": "f"}, "objects": [{"label": "paper", "centroid": {"x": 1, "y": 2, "z": 3}}]}
        payload, _ = convert_epd_message_to_detected_objects(msg, "t", "ur5_2f_test", "f")
        obj = payload["objects"][0]
        self.assertEqual(obj["name"], "paper")
        self.assertEqual(obj["class_id"], "paper")

    def test_tracking_ids_pose_dimensions_and_source_stamp_are_preserved(self) -> None:
        msg = {
            "header": {"frame_id": "camera_color_optical_frame", "stamp": {"sec": 7, "nanosec": 42}},
            "object_ids": ["2"],
            "objects": [{
                "name": "person", "centroid": {"x": 0.1, "y": 0.2, "z": 1.5},
                "length": 0.4, "breadth": 0.3, "height": 1.2,
                "pose": {"orientation": {"x": 0.0, "y": 0.0, "z": 0.5, "w": 0.8660254}},
            }],
        }
        payload, _ = convert_epd_message_to_detected_objects(msg, "tracking", "ur5_2f_test", "fallback")
        obj = payload["objects"][0]
        self.assertEqual(obj["object_id"], "2")
        self.assertEqual(obj["tracking_id"], "2")
        self.assertEqual(obj["dimensions"], {"x": 0.4, "y": 0.3, "z": 1.2})
        self.assertEqual(obj["pose"]["orientation_xyzw"], [0.0, 0.0, 0.5, 0.8660254])
        self.assertEqual(payload["source"]["source_stamp_ns"], 7_000_000_042)
        self.assertNotIn("confidence", obj)

    def test_lost_tracking_ids_are_preserved_and_optional(self) -> None:
        msg = self._msg()
        msg["lost_track_ids"] = ["1", "2"]
        payload, _ = convert_epd_message_to_detected_objects(msg, "tracking", "ur5_2f_test", "fallback")
        self.assertEqual(payload["lost_object_ids"], ["1", "2"])
        payload, _ = convert_epd_message_to_detected_objects(self._msg(), "tracking", "ur5_2f_test", "fallback")
        self.assertNotIn("lost_object_ids", payload)

    def test_output_validates_with_existing_validator(self) -> None:
        payload, _ = convert_epd_message_to_detected_objects(
            self._msg(), "/easy_perception_deployment/epd_localize_output", "ur5_2f_test", "camera_depth_optical_frame"
        )
        result = validate_detected_objects(payload, strict=False, allow_generate_ids=True)
        self.assertIn(result.status, {"PASS", "WARN"})
        self.assertFalse(result.errors)

    def test_qos_profile_best_effort_and_reliable(self) -> None:
        class Dummy:
            BEST_EFFORT = "be"
            RELIABLE = "rel"
            VOLATILE = "vol"
            KEEP_LAST = "kl"

        class DummyQoS:
            def __init__(self, **kwargs):
                self.kwargs = kwargs

        with patch.dict("sys.modules", {"rclpy.qos": type("M", (), {"DurabilityPolicy": Dummy, "HistoryPolicy": Dummy, "QoSProfile": DummyQoS, "ReliabilityPolicy": Dummy})}):
            be, choice_be = create_qos_profile("best_effort", 7)
            rel, choice_rel = create_qos_profile("reliable", 3)
            auto, choice_auto = create_qos_profile("auto", 4)
        self.assertEqual(choice_be, "best_effort")
        self.assertEqual(choice_rel, "reliable")
        self.assertEqual(choice_auto, "best_effort")
        self.assertEqual(be.kwargs["reliability"], "be")
        self.assertEqual(rel.kwargs["reliability"], "rel")
        self.assertEqual(auto.kwargs["reliability"], "be")

    def test_normalize_pose_helper_pass(self) -> None:
        obj = {"pose": {"frame_id": "camera_depth_optical_frame", "xyz": [0.1, 0.2, 0.3], "rpy": [0.0, 0.0, 0.0]}}

        def fake_tf(src: str, target: str, xyz: list[float], rpy: list[float], timeout: float):
            self.assertEqual(src, "camera_depth_optical_frame")
            self.assertEqual(target, "world")
            self.assertEqual(timeout, 2.0)
            return [1.0, 2.0, 3.0], [0.1, 0.2, 0.3], "ok"

        status, message = _normalize_pose_with_tf(obj, "world", 2.0, fake_tf)
        self.assertEqual(status, "PASS")
        self.assertEqual(message, "ok")
        self.assertEqual(obj["pose"]["frame_id"], "world")


if __name__ == "__main__":
    unittest.main()
