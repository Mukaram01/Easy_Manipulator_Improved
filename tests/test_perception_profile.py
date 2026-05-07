from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path


def test_validate_perception_profile_fixture(tmp_path: Path) -> None:
    profile = tmp_path / 'perception_profile.yaml'
    profile.write_text(
        """
schema: workcell_perception_profile/v1
sensor:
  type: realsense_d435i
  camera_frame: camera_color_optical_frame
topics:
  rgb: /camera/color/image_raw
  depth: /camera/aligned_depth_to_color/image_raw
  camera_info: /camera/color/camera_info
  epd_localization_output: /epd/localisation/detected_objects
  epd_tracking_output: /epd/tracking/detected_objects
frames:
  object_frame: world
  scene_frame: world
expected_snapshot_path: tests/fixtures/perception/detected_objects_snapshot_golden.yaml
safety_mode:
  perception_only: true
  no_robot_motion: true
  no_runtime_execution: true
  fake_hardware_default: true
""".strip()+"\n", encoding='utf-8')
    proc = subprocess.run([sys.executable, 'scripts/validate_perception_profile.py', str(profile), '--json'], capture_output=True, text=True, check=False)
    assert proc.returncode == 0
    payload = json.loads(proc.stdout)
    assert payload['status'] == 'PASS'
