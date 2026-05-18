import json
from pathlib import Path

from scripts.generate_workcell_studio_readiness_pack import _generate_perception_readiness


def test_generate_perception_readiness_skips_incomplete_pose_xyz(tmp_path: Path) -> None:
    profile = tmp_path / "profile.yaml"
    snapshot = tmp_path / "snapshot.yaml"
    output = tmp_path / "report.json"

    profile.write_text(
        """
schema: workcell_perception_profile/v1
frames:
  scene_frame: world
""".strip()
        + "\n",
        encoding="utf-8",
    )
    snapshot.write_text(
        """
schema_version: detected_objects/v1
objects:
  - id: bad_obj
    label: cube
    frame_id: world
    pose:
      xyz: [0.1, 0.2]
""".strip()
        + "\n",
        encoding="utf-8",
    )

    status, warnings, blockers = _generate_perception_readiness(profile, snapshot, output)

    assert status == "perception_partial"
    assert any("Skipping detected object 'bad_obj'" in item for item in warnings)
    assert any("No valid detected object" in item for item in blockers)

    payload = json.loads(output.read_text(encoding="utf-8"))
    assert payload["status"] == "perception_partial"
