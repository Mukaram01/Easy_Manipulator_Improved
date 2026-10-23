#!/usr/bin/env python3
"""Disposable reparented camera + table Web3D fixture for the combined acceptance.

Usage: build_fixture.py <repo_root> <work_dir>

1. Copies scenes/ur5_2f_test to <work_dir>/scene/ur5_2f_test and authors the
   compound camera pose (0, pi/2, 15 deg) and a 20 deg table yaw.
2. Exports it with the repository exporter (production code), giving the real
   reparented D435 camera visual.
3. ur5_2f_test has no generated table visual, so one is CONSTRUCTED: a locked
   generated_preview table mesh at owner * local offset, whose
   owner_relative_visual_transform comes from the exporter's _parent_to_child_pose.
"""
import copy
import json
import math
import shutil
import subprocess
import sys
from pathlib import Path

import yaml

repo, work = Path(sys.argv[1]).resolve(), Path(sys.argv[2]).resolve()
scene = work / "scene" / "ur5_2f_test"
if scene.exists():
    shutil.rmtree(scene)
shutil.copytree(repo / "scenes" / "ur5_2f_test", scene, ignore=shutil.ignore_patterns("generated", "*.bak.yaml"))
env_path = scene / "environment.yaml"
data = yaml.safe_load(env_path.read_text())
env = data["environment"]
CAMERA_RPY = [0.0, math.pi / 2, math.radians(15.0)]
TABLE_RPY = [0.0, 0.0, math.radians(20.0)]
for row in env["assets"]:
    if row["id"] == "realsense_overhead":
        row["pose_rpy"] = CAMERA_RPY
for row in env["support_surfaces"]:
    if row["id"] == "support_surface_table":
        row["pose_rpy"] = TABLE_RPY
if isinstance(data.get("camera"), dict) and isinstance(data["camera"].get("pose"), list):
    data["camera"]["pose"][3:6] = CAMERA_RPY
env_path.write_text(yaml.safe_dump(data, sort_keys=False))
# The editor layout carries the same authored poses (Save writes both files).
layout_path = scene / "layout" / "workcell_studio_layout.yaml"
layout = yaml.safe_load(layout_path.read_text())
for item in layout.get("items", []):
    if item.get("id") == "realsense_overhead":
        item["pose"]["rpy"] = CAMERA_RPY
    if item.get("id") == "support_surface_table":
        item["pose"]["rpy"] = TABLE_RPY
layout_path.write_text(yaml.safe_dump(layout, sort_keys=False))

# The generated visual index (URDF FK with the authored camera pose) is what the
# exporter turns into the reparented D435 visual.
subprocess.run([sys.executable, str(repo / "scripts/extract_scene_urdf_visual_mesh_index.py"), "--scene", str(scene),
                "--prefer-xacro-expanded"], cwd=repo, check=True, stdout=subprocess.DEVNULL)
payload_path = work / "fixture.web_scene.json"
subprocess.run([sys.executable, str(repo / "scripts/export_workcell_studio_web_scene.py"), "--scene", str(scene),
                "--output", str(payload_path), "--no-stage-assets"], cwd=repo, check=True)
payload = json.loads(payload_path.read_text())

sys.path.insert(0, str(repo / "scripts"))
import export_workcell_studio_web_scene_impl as exporter  # noqa: E402

owners = {o["id"]: o for o in payload["ui_selection_owners"]}
camera_visuals = [i for i in payload["sensors"] if i.get("camera_id") == "realsense_overhead" and "owner_relative_visual_transform" in i]
assert len(camera_visuals) == 1, "expected the exporter's reparented D435 visual"
assert owners["realsense_overhead"]["pose"]["rpy"] == CAMERA_RPY, owners["realsense_overhead"]["pose"]
# The D435 mesh sits at a fixed offset from camera_link: the relative transform must
# not depend on the owner pose (it would if the generated FK ignored the compound pose).
rel = camera_visuals[0]["owner_relative_visual_transform"]
assert max(abs(a - b) for a, b in zip(rel["xyz"], [0.0149, 0.0, 0.0125])) < 1e-6, rel
assert max(abs(a - b) for a, b in zip(rel["rpy"], [math.pi / 2, 0.0, math.pi / 2])) < 1e-6, rel

table_owner = owners["support_surface_table"]
authored_table = next(i for i in payload["assets"] if i.get("id") == "support_surface_table")
owner_pose = table_owner["pose"]
local = {"xyz": [0.03, -0.02, -0.005], "rpy": [0.0, 0.0, 0.1]}
R = exporter._ros_rpy_matrix(owner_pose["rpy"])
world_xyz = [owner_pose["xyz"][r] + sum(R[r][k] * local["xyz"][k] for k in range(3)) for r in range(3)]
world_rpy = exporter._ros_rpy_from_matrix(exporter._matrix_multiply(R, exporter._ros_rpy_matrix(local["rpy"])))
visual = copy.deepcopy(authored_table)
visual.update({
    "id": "fixture_generated_table_top_visual",
    "display_name": "Fixture generated table visual",
    "source_kind": "generated_preview",
    "source_layer": "locked_generated_urdf_visual",
    "locked": True,
    "editable": False,
    "mesh_contract_category": "table",
    "support_surface_ref": "support_surface_table",
    "pose": {"xyz": world_xyz, "rpy": world_rpy},
    "final_transform": {"xyz": world_xyz, "rpy": world_rpy},
    "mesh_uri": authored_table.get("mesh_uri") or "assets/environment/fixture_table.stl",
    "constructed_fixture": "disposable reparented table visual (ur5_2f_test exports none)",
})
for key in ("layout_item_ref", "canonical_scene_item_id"):
    visual.pop(key, None)
relative = exporter._parent_to_child_pose(owner_pose, visual["final_transform"])
relative["scale"] = [1.0, 1.0, 1.0]
visual["owner_relative_visual_transform"] = relative
visual.setdefault("provenance", {})["owner_relative_visual_transform"] = {
    "operation": "inverse(source_physical_owner_world_pose)_times_generated_visual_world_pose",
    "owner_id": "support_surface_table", "source_owner_pose": copy.deepcopy(owner_pose)}
payload["assets"].append(visual)
payload_path.write_text(json.dumps(payload, indent=1))
print(json.dumps({"payload": str(payload_path), "camera_rpy": owners["realsense_overhead"]["pose"]["rpy"],
                  "camera_visual": camera_visuals[0]["id"], "camera_relative": camera_visuals[0]["owner_relative_visual_transform"],
                  "table_owner_rpy": owner_pose["rpy"], "table_relative": relative}))
