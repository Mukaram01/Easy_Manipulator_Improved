#!/usr/bin/env python3
from __future__ import annotations

import unittest
from unittest.mock import patch

from scripts.capture_epd_detected_objects import convert_epd_message_to_detected_objects, create_qos_profile
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


if __name__ == "__main__":
    unittest.main()
