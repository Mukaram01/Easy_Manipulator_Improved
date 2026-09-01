#!/usr/bin/env bash
set -eo pipefail

summary="${1:-/tmp/p8c2_integration_summary.json}"
scene_log=/tmp/p8c2_scene.log
bridge_log=/tmp/p8c2_bridge.log
replay_summary=/tmp/p8c2_epd_replay_summary.json

source /opt/ros/humble/setup.bash
source /home/user/epd_ros2_ws/install/setup.bash
source /home/user/workcell_ws/install/setup.bash
set -u

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
sleep "${P8C2_BRIDGE_READY_SECONDS:-2}"
env -i HOME="$HOME" USER="${USER:-}" DISPLAY="${DISPLAY:-}" \
  PATH="/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin" \
  ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" bash --noprofile --norc -c '
  set +u
  source /opt/ros/humble/setup.bash
  source /home/user/epd_ros2_ws/install/setup.bash
  set -u
  cd /home/user/epd_ros2_ws/src/easy_perception_deployment/easy_perception_deployment
  exec ros2 launch easy_perception_deployment replay.launch.py mode:=fast summary_output:="$1"
' _ "$replay_summary"

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
        assert integration["lost_ids_received"] == ["1", "2"]
        assert integration["removed_ids"] == ["1", "2"]
        assert integration["planning_scene_verified_ids"] == []
        assert integration["duplicate_ids"] == []
        assert integration["realsense_used"] is False
        print(json.dumps(integration, indent=2, sort_keys=True))
        raise SystemExit(0)
    time.sleep(0.1)
raise SystemExit("P8-C2 integration summary did not reach PASS")
PY
