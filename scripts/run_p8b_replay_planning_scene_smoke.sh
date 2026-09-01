#!/usr/bin/env bash
set -euo pipefail

summary="${1:-/tmp/p8b_integration_summary.json}"
scene_log=/tmp/p8b_scene.log
bridge_log=/tmp/p8b_bridge.log
replay_summary=/tmp/p8b_epd_replay_summary.json

source /opt/ros/humble/setup.bash
source /home/ubuntu/epd_ros2_ws/install/setup.bash
source /home/ubuntu/workcell_ws/install/setup.bash

scene_pid=""
tf_pid=""
bridge_pid=""
cleanup() {
  for pid in "$bridge_pid" "$tf_pid" "$scene_pid"; do
    if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then
      kill -INT "$pid" 2>/dev/null || true
    fi
  done
}
trap cleanup EXIT

ros2 launch ur5_2f_test demo.launch.py use_fake_hardware:=true launch_rviz:=false >"$scene_log" 2>&1 &
scene_pid=$!
for _ in $(seq 1 60); do
  ros2 service type /apply_planning_scene >/dev/null 2>&1 && break
  sleep 0.25
done
ros2 service type /apply_planning_scene >/dev/null

# The P8 fixture camera is the canonical scene D435i color optical frame.
ros2 run tf2_ros static_transform_publisher --x 0 --y 0 --z 0 --roll 0 --pitch 0 --yaw 0 \
  --frame-id camera_color_optical_frame --child-frame-id fixture_color_optical_frame >/tmp/p8b_fixture_tf.log 2>&1 &
tf_pid=$!

ros2 run workcell_builder epd_dynamic_planning_scene_node.py --summary-output "$summary" >"$bridge_log" 2>&1 &
bridge_pid=$!
ros2 launch easy_perception_deployment replay.launch.py mode:=fast summary_output:="$replay_summary"

python3 - "$summary" "$replay_summary" <<'PY'
import json, sys, time
summary_path, replay_path = sys.argv[1:]
deadline = time.monotonic() + 15.0
while time.monotonic() < deadline:
    try:
        integration = json.load(open(summary_path, encoding="utf-8"))
        replay = json.load(open(replay_path, encoding="utf-8"))
    except (FileNotFoundError, json.JSONDecodeError):
        time.sleep(0.1)
        continue
    if integration.get("result") == "PASS":
        assert replay.get("result") == "PASS"
        assert integration["planning_scene_verified_ids"] == ["1", "2"]
        assert integration["duplicate_ids"] == []
        assert integration["realsense_used"] is False
        print(json.dumps(integration, indent=2, sort_keys=True))
        raise SystemExit(0)
    time.sleep(0.1)
raise SystemExit("P8-B integration summary did not reach PASS")
PY
