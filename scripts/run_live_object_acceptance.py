#!/usr/bin/env python3
"""Owned real-camera commissioning through the canonical fake-hardware executor."""
import argparse
import json
import os
import re
import struct
import signal
from pathlib import Path
import subprocess
import sys
import time

from run_r14_plan_only_acceptance import stop, process_failures
from perceived_object_grasp_execute import fake_hardware_evidence
from capture_epd_detected_objects import topic_endpoints


MOTION_STAGES = ('APPROACH', 'GRASP', 'CLOSE_GRIPPER', 'LIFT', 'TRANSFER',
                 'PLACE', 'OPEN_GRIPPER', 'RETREAT', 'HOME')


def perception_ready(endpoints, health, expected_type):
    return (endpoints['actual_topic_type'] == [expected_type] and
        endpoints['publisher_count'] >= 1 and
        all(p['reliability'] == 'RELIABLE' for p in endpoints['publisher_qos']) and
        int(health.get('inference_completed', 0)) > 0 and
        int(health.get('geometry_valid_total', 0)) > 0)


def epd_capture_command(binary, mode):
    return [str(binary/'easy_perception_deployment'), '--ros-args',
        '-p', 'use_depth:=true', '-p', 'usecase_mode_override:='+('4' if mode == 'tracking' else '3'),
        '-p', 'rgb_topic:=/easy_perception_deployment/ingress/color/image_raw',
        '-p', 'depth_topic:=/easy_perception_deployment/ingress/aligned_depth/image_raw',
        '-p', 'camera_info_topic:=/easy_perception_deployment/ingress/color/camera_info',
        '-p', 'qos_overrides./easy_perception_deployment/epd_tracking_output.publisher.reliability:=reliable',
        '-p', 'qos_overrides./easy_perception_deployment/epd_localize_output.publisher.reliability:=reliable',
        '-r', '/easy_perception_deployment/image_input:=/easy_perception_deployment/ingress/color/image_raw']


def verify_execution_result(result):
    required = ('full_cycle_execution_success', 'attach_verified', 'detach_verified',
                'home_verified', 'destination_verified')
    missing = [key for key in required if result.get(key) is not True]
    completed = set(result.get('stages', []))
    missing.extend('EXECUTE_'+stage for stage in MOTION_STAGES if 'EXECUTE_'+stage not in completed)
    if result.get('result') != 'PASS' or missing:
        raise RuntimeError(f'Incomplete state-verified execution: {missing}')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--domain-id', type=int, default=180, help='Unused isolated ROS domain')
    parser.add_argument('--capture-timeout', type=float, default=60)
    parser.add_argument('--plan-only', action='store_true')
    parser.add_argument('--message-type', choices=('tracking', 'localization'), default='tracking')
    args = parser.parse_args()
    if not 0 <= args.domain_id <= 232 or not 0 < args.capture_timeout <= 180:
        parser.error('Invalid domain or capture timeout')
    # A complete scene, RViz, camera, ingress, EPD and capture exceed Cyclone's
    # default auto participant search range. Scope this configuration to owned children.
    if os.environ.get('RMW_IMPLEMENTATION') == 'rmw_cyclonedds_cpp':
        os.environ.setdefault('CYCLONEDDS_URI', '<CycloneDDS><Domain><Discovery>'
            '<ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>100</MaxAutoParticipantIndex>'
            '</Discovery></Domain></CycloneDDS>')
    os.environ.update(ROS_DOMAIN_ID=str(args.domain_id), ROS_LOCALHOST_ONLY='1')
    import rclpy
    from rcl_interfaces.srv import GetParameters
    from controller_manager_msgs.srv import ListHardwareComponents
    from ament_index_python.packages import get_package_share_directory, get_package_prefix
    from diagnostic_msgs.msg import DiagnosticArray
    import xml.etree.ElementTree as ET
    out = args.output_dir.resolve()
    out.mkdir(parents=True, exist_ok=True)
    if (out/'acceptance.json').exists():
        parser.error('Evidence directory already contains acceptance.json; choose a new directory')
    scripts = Path(__file__).resolve().parent
    owned, logs = [], []
    audit = dict(overall_result='FAIL', execution_attempted=False, clean_shutdown=False)
    rclpy.init()
    def interrupted(signum, frame):
        raise InterruptedError(f'Commissioning interrupted by signal {signum}')
    signal.signal(signal.SIGTERM, interrupted)
    node = rclpy.create_node('live_object_commissioning')
    def write(name, data):
        (out/name).write_text(json.dumps(data, indent=2)+'\n')
    def launch(name, command, cwd=None, env=None):
        log = (out/(name+'.log')).open('w')
        logs.append(log)
        process = subprocess.Popen(command, cwd=cwd, env=env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        owned.append((name, process))
        return process
    def call(service, kind, request):
        client = node.create_client(kind, service)
        if not client.wait_for_service(timeout_sec=45):
            raise RuntimeError(f'Missing required service {service}')
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=10)
        if not future.done() or future.result() is None:
            raise RuntimeError(f'No response from {service}')
        return future.result()
    def capture_rviz():
        # Xwayland cannot capture the root window; capture the mapped RViz window.
        from PIL import Image
        tree = subprocess.check_output(['xwininfo','-root','-tree'], text=True, timeout=5)
        match = re.search(r'(0x[0-9a-f]+) "[^"\n]+ - RViz"', tree)
        if not match:
            raise RuntimeError('No mapped RViz window for visual evidence')
        images = out/'rviz_frames'
        images.mkdir(exist_ok=True)
        data = subprocess.check_output(['xwd','-id',match[1],'-silent'], timeout=5)
        header = struct.unpack('>25I',data[:100])
        if header[2] != 2 or header[7] != 0 or header[11] not in (24,32):
            raise RuntimeError('Unsupported RViz XWD image format')
        pixels = data[header[0]+header[19]*12:]
        Image.frombytes('RGB',(header[4],header[5]),pixels,'raw',
            'BGR' if header[11] == 24 else 'BGRX',header[12],1).save(images/(str(time.time_ns())+'.png'))

    def run(name, command, timeout):
        process = launch(name, command)
        deadline = time.monotonic()+timeout
        last_image = 0.0
        while process.poll() is None:
            if time.monotonic() > deadline:
                raise RuntimeError(f'{name} exceeded {timeout}s')
            for owner, child in owned:
                if owner in ('scene','camera','epd') and child.poll() is not None:
                    raise RuntimeError(f'{owner} exited during {name}')
            if name == 'executor' and time.monotonic()-last_image > 1:
                capture_rviz()
                last_image = time.monotonic()
            rclpy.spin_once(node, timeout_sec=.1)
        if process.returncode:
            raise RuntimeError(f'{name} failed ({process.returncode}); see {name}.log')
    try:
        # Fixed launch arguments: this entry point has no real-hardware option.
        launch('scene', ['ros2','launch','ur5_2f_test','demo.launch.py',
            'use_fake_hardware:=true','allow_trajectory_execution:='+str(not args.plan_only).lower(),
            'launch_rviz:=true'])
        params = call('/move_group/get_parameters', GetParameters,
            GetParameters.Request(names=['use_fake_hardware','allow_trajectory_execution','robot_description'])).values
        hardware = call('/controller_manager/list_hardware_components', ListHardwareComponents,
            ListHardwareComponents.Request()).component
        guard = fake_hardware_evidence(params, hardware)
        plugins = [p.text for p in ET.fromstring(params[2].string_value).findall('./ros2_control/hardware/plugin')]
        if not plugins or any(p != 'mock_components/GenericSystem' for p in plugins):
            raise RuntimeError(f'Non-mock robot description hardware: {plugins}')
        if params[1].bool_value != (not args.plan_only):
            raise RuntimeError('Trajectory execution gate differs from requested fake mode')
        audit.update(fake_hardware=True, real_hardware=False, hardware_classes=guard['hardware_classes'],
            trajectory_execution_enabled=params[1].bool_value)
        write('safety_guards.json',dict(**guard,urdf_hardware_plugins=plugins,
            trajectory_execution_enabled=params[1].bool_value))
        deadline = time.monotonic()+30
        while 'authored collision objects; MoveIt is planning truth' not in (out/'scene.log').read_text():
            if time.monotonic() > deadline: raise RuntimeError('Authored collision scene not ready')
            rclpy.spin_once(node, timeout_sec=.1)
        if not any(name == 'rviz2' for name, namespace in node.get_node_names_and_namespaces()):
            raise RuntimeError('RViz is not running')
        deadline = time.monotonic()+20
        while True:
            try:
                capture_rviz()
                break
            except RuntimeError:
                if time.monotonic() > deadline: raise
                rclpy.spin_once(node, timeout_sec=.2)
        launch('camera',['ros2','launch','realsense2_camera','rs_launch.py',
            'align_depth.enable:=true','enable_color:=true','enable_depth:=true',
            'rgb_camera.color_profile:=640x480x15','depth_module.depth_profile:=640x480x15',
            'enable_sync:=true','pointcloud.enable:=false'])
        health = {}
        def health_update(msg):
            for status in msg.status:
                health.update({v.key:v.value for v in status.values})
        node.create_subscription(DiagnosticArray, '/easy_perception_deployment/inference_diagnostics', health_update, 10)
        binary = Path(get_package_prefix('easy_perception_deployment'))/'lib/easy_perception_deployment'
        launch('ingress', [str(binary/'epd_sensor_ingress')])
        launch('epd', epd_capture_command(binary, args.message_type),
            get_package_share_directory('easy_perception_deployment'),
            dict(os.environ, EPD_EXECUTION_BACKEND='cpu'))
        topic = '/easy_perception_deployment/epd_' + ('tracking' if args.message_type == 'tracking' else 'localize') + '_output'
        expected_type = 'epd_msgs/msg/EPDObject' + ('Tracking' if args.message_type == 'tracking' else 'Localization')
        deadline = time.monotonic()+args.capture_timeout
        while True:
            endpoints = topic_endpoints(node, topic)
            ready = perception_ready(endpoints, health, expected_type)
            if ready or time.monotonic() >= deadline:
                write('perception_readiness.json', dict(ready=ready, health=health, **endpoints))
                if not ready:
                    raise RuntimeError('EPD not ready: see perception_readiness.json')
                break
            for name, process in owned:
                if name in ('epd', 'ingress', 'camera') and process.poll() is not None:
                    raise RuntimeError(f'{name} exited before perception readiness')
            rclpy.spin_once(node, timeout_sec=.1)
        capture = out/'live_detection.json'
        capture.unlink(missing_ok=True)
        run('capture',[sys.executable,str(scripts/'capture_epd_detected_objects.py'),
            '--message-type',args.message_type,'--topic',topic,
            '--qos-reliability','reliable','--diagnostics-output',str(out/'capture_diagnostics.json'),
            '--scene-package','ur5_2f_test','--once','--timeout',str(args.capture_timeout),
            '--target-frame','world','--require-transform','--json','--output',str(capture)],args.capture_timeout+15)
        detection = json.loads(capture.read_text())
        if detection['source']['mode'] != 'live_epd' or detection['source']['type'] != 'epd_'+args.message_type:
            raise RuntimeError('Capture was not the requested live EPD mode')
        audit.update(real_camera_used=True,live_epd_used=True,epd_mode=args.message_type,
            detected_objects=detection['objects'],tf_valid=detection['source']['transform']['status']=='PASS')
        write('preflight.json',audit)
        executor_args=[sys.executable,str(scripts/'perceived_object_grasp_execute.py'),
            '--scene-package','ur5_2f_test','--task-request',str(scripts.parent/'config/runtime/live_object_task.yaml'),
            '--detections',str(capture),'--timeout','180','--summary-output',str(out/'execution_result.json')]
        if not args.plan_only: executor_args.append('--start')
        run('executor',executor_args,360)
        result = json.loads((out/'execution_result.json').read_text())
        if not args.plan_only:
            verify_execution_result(result)
        audit.update(result)
        audit['overall_result'] = 'PLAN_ONLY' if args.plan_only else result['result']
    except (Exception, KeyboardInterrupt) as exc:
        audit['failure'] = str(exc)
        if (out/'execution_result.json').exists():
            audit['executor_result'] = json.loads((out/'execution_result.json').read_text())
            audit['execution_attempted'] = audit['executor_result']['execution_attempted']
    finally:
        cleanup = {}
        for name, process in reversed(owned):
            cleanup[name] = dict(clean=stop(process),returncode=process.returncode,pid=process.pid)
        for log in logs: log.close()
        crashes = {name:process_failures((out/(name+'.log')).read_text()) for name,_ in owned}
        for name, process in owned:
            if process.returncode is not None and process.returncode < 0 and process.returncode not in (-signal.SIGINT, -signal.SIGTERM):
                crashes[name].append(f'Process terminated by signal {-process.returncode}')
        audit['clean_shutdown'] = all(v['clean'] for v in cleanup.values()) and not any(crashes.values())
        audit['cleanup'] = cleanup
        audit['child_crashes'] = crashes
        if not audit['clean_shutdown']: audit['overall_result'] = 'FAIL'
        result = audit.get('executor_result', audit)
        audit['motion_stages'] = {stage: ('PASS' if 'EXECUTE_'+stage in result.get('stages', []) else
            'FAIL' if result.get('failed_stage') == 'EXECUTE_'+stage else 'NOT_RUN') for stage in MOTION_STAGES}
        audit['real_camera_used'] = 'RealSense Node Is Up!' in ((out/'camera.log').read_text() if (out/'camera.log').exists() else '')
        epd_log = (out/'epd.log').read_text() if (out/'epd.log').exists() else ''
        audit['epd_mode'] = next((mode for mode in ('TRACKING_MODE','LOCALISATION_MODE') if mode in epd_log), None)
        write('acceptance.json',audit)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print(json.dumps(audit,indent=2))
    return 0 if audit['overall_result'] in ('PASS','PLAN_ONLY') else 1


if __name__ == '__main__':
    raise SystemExit(main())
