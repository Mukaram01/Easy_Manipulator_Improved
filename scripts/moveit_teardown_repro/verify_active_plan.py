#!/usr/bin/env python3
"""Observe owned SIGINT during a replayed real UR5 plan, using production adapters.

This starts only MoveGroup and a read-only planning client/joint-state publisher.
The saved request and scene are replayed without timing barriers or altered
planner/certificate code. No trajectory execution or hardware driver is started.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import shutil
import subprocess
import time
import yaml
from verify import core_inventory, group_members


def identity(path):
    path=Path(path).resolve()
    return dict(path=str(path),sha256=hashlib.sha256(path.read_bytes()).hexdigest())


def shutdown_completion(text):
    """Observe completion after the pinned Humble rclcpp signal-handler record."""
    marker='[rclcpp]: signal_handler(SIGINT/SIGTERM)'
    after_signal=text.split(marker,1)[1] if marker in text else ''
    return dict(signal_handler_observed=marker in text,
        production_certificate_after_signal=('[workcell.controller_certificate]' in after_signal and
            'result=0 reason=ALL_INTERVALS_CERTIFIED' in after_signal),
        plan_result_after_signal='Motion plan was computed successfully.' in after_signal)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    for name in ['binary','library','capabilities','request','scene','urdf','srdf','output']:
        p.add_argument('--'+name,type=Path,required=True)
    p.add_argument('--cycles',type=int,default=5)
    p.add_argument('--signal-after-seconds',type=float,default=1.)
    args=p.parse_args()
    if args.cycles<1 or args.signal_after_seconds<=0:p.error('positive cycles and signal delay required')
    args.output.mkdir(parents=True,exist_ok=False)
    import rclpy
    from rclpy.action import ActionClient
    from rclpy.serialization import deserialize_message
    from ament_index_python.packages import get_package_share_directory,get_package_prefix
    from moveit_msgs.action import MoveGroup
    from moveit_msgs.msg import MotionPlanRequest,PlanningScene
    from sensor_msgs.msg import JointState
    request=deserialize_message(args.request.read_bytes(),MotionPlanRequest)
    scene=deserialize_message(args.scene.read_bytes(),PlanningScene)
    assert request.group_name=='manipulator','real UR5 manipulator request required'
    share=Path(get_package_share_directory('ur5_moveit_config'))
    adapters=' '.join(['workcell/InitialSupportContact','workcell/StraightCartesianPath',
        'default_planner_request_adapters/AddTimeOptimalParameterization',
        'default_planner_request_adapters/FixWorkspaceBounds','default_planner_request_adapters/FixStartStateBounds',
        'default_planner_request_adapters/FixStartStateCollision','default_planner_request_adapters/FixStartStatePathConstraints'])
    ompl=dict(planning_plugin='ompl_interface/OMPLPlanner',request_adapters=adapters,start_state_max_bounds_error=.1)
    ompl.update(yaml.safe_load((share/'config/ompl_planning.yaml').read_text()) or {})
    params=dict(robot_description=args.urdf.read_text(),robot_description_semantic=args.srdf.read_text(),
        robot_description_kinematics=yaml.safe_load((share/'config/kinematics.yaml').read_text()),
        planning_pipelines=['ompl'],default_planning_pipeline='ompl',ompl=ompl,use_sim_time=False,
        allow_trajectory_execution=False,moveit_manage_controllers=False,
        moveit_controller_manager='moveit_simple_controller_manager/MoveItSimpleControllerManager')
    parameter_file=args.output/'parameters.yaml'
    parameter_file.write_text(yaml.safe_dump({'/**':{'ros__parameters':params}},sort_keys=False))
    support=Path(get_package_prefix('workcell_builder'))/'lib/libworkcell_support_contact.so'
    inputs={name:identity(getattr(args,name)) for name in ['binary','library','capabilities','request','scene','urdf','srdf']}
    # Snapshot mutable workspace artifacts before any runtime is started.
    # Mappings still must name the actual loaded original path/device/inode.
    archived=args.output/'inputs';archived.mkdir()
    archived_support=archived/support.name;shutil.copy2(support,archived_support)
    archived_verifier=archived/'verify_active_plan.py';shutil.copy2(__file__,archived_verifier)
    inputs.update(support_adapter=identity(archived_support),parameters=identity(parameter_file),verifier=identity(archived_verifier))
    runtime_paths={name:Path(inputs[name]['path']) for name in ['binary','library','capabilities']}
    runtime_paths['support_adapter']=support.resolve()
    rows=[]
    for index in range(args.cycles):
        directory=args.output/f'{index+1:02d}';directory.mkdir()
        env=dict(os.environ,ROS_LOCALHOST_ONLY='1')
        before=core_inventory(directory)
        log=(directory/'move-group.log').open('w')
        process=subprocess.Popen([str(args.binary.resolve()),'--ros-args','--params-file',str(parameter_file)],
            stdout=log,stderr=subprocess.STDOUT,env=env,start_new_session=True,cwd=directory)
        rclpy.init()
        node=rclpy.create_node('workcell_active_plan_shutdown_probe')
        publisher=node.create_publisher(JointState,'/joint_states',10)
        joint_state=scene.robot_state.joint_state
        def publish_state():
            joint_state.header.stamp=node.get_clock().now().to_msg();publisher.publish(joint_state)
        timer=node.create_timer(.02,publish_state)
        client=ActionClient(node,MoveGroup,'move_action')
        row=dict(cycle=index+1,pid=process.pid,owned_sigint=False,passed=False)
        try:
            deadline=time.monotonic()+30
            while time.monotonic()<deadline and process.poll() is None and not client.server_is_ready():
                rclpy.spin_once(node,timeout_sec=.05)
            if not client.server_is_ready():raise RuntimeError('MoveGroup action unavailable')
            # Use the captured effective request and complete frozen scene.
            goal=MoveGroup.Goal();goal.request=request
            goal.planning_options.plan_only=True
            goal.planning_options.planning_scene_diff=scene
            sent=client.send_goal_async(goal)
            rclpy.spin_until_future_complete(node,sent,timeout_sec=10.)
            handle=sent.result() if sent.done() else None
            if handle is None or not handle.accepted:raise RuntimeError('real planning goal not accepted')
            row.update(goal_uuid=bytes(handle.goal_id.uuid).hex(),accepted=True,accepted_monotonic=time.monotonic())
            result=handle.get_result_async()
            deadline=time.monotonic()+args.signal_after_seconds
            while time.monotonic()<deadline and not result.done():rclpy.spin_once(node,timeout_sec=.02)
            if result.done():raise RuntimeError('plan completed before owned SIGINT; no active-shutdown evidence')
            text=(directory/'move-group.log').read_text()
            if 'Forwarding to planning pipeline.' not in text:raise RuntimeError('planning pipeline not entered before signal')
            maps=Path(f'/proc/{process.pid}/maps').read_text();(directory/'move-group.maps').write_text(maps)
            row['loaded_executable']=str(Path(f'/proc/{process.pid}/exe').resolve())
            if row['loaded_executable']!=inputs['binary']['path']:raise RuntimeError('different executable mapped')
            row['mapped_artifacts']={}
            for name in ['binary','library','capabilities','support_adapter']:
                path=runtime_paths[name];st=path.stat()
                if identity(path)['sha256']!=inputs[name]['sha256']:raise RuntimeError('mapped content changed: '+name)
                lines=[ln.split() for ln in maps.splitlines() if ln.split()[-1]==str(path)]
                if not lines or any(int(ln[4])!=st.st_ino or tuple(int(x,16) for x in ln[3].split(':'))!=(os.major(st.st_dev),os.minor(st.st_dev)) for ln in lines):
                    raise RuntimeError('mapped path/device/inode differs: '+name)
                row['mapped_artifacts'][name]=dict(identity(path),inode=st.st_ino,device=st.st_dev)
            rclpy.spin_once(node,timeout_sec=0.)
            if result.done():raise RuntimeError('plan completed during mapping capture; no active-shutdown evidence')
            row.update(pending_at_signal=True,signal_monotonic=time.monotonic())
            os.killpg(process.pid,signal.SIGINT);row['owned_sigint']=True
            try:process.wait(timeout=60)
            except subprocess.TimeoutExpired:row['timed_out']=True;raise
            log.flush();text=(directory/'move-group.log').read_text()
            row.update(returncode=process.returncode,remaining_processes=group_members(process.pid),
                core_inventory_changed=core_inventory(directory)!=before,
                **shutdown_completion(text))
            row['passed']=(process.returncode==0 and not row['remaining_processes'] and not row['core_inventory_changed']
                           and row['signal_handler_observed'] and row['production_certificate_after_signal'] and row['plan_result_after_signal'])
        except Exception as exc:row['error']=str(exc)
        finally:
            if process.poll() is None:
                row['forced_cleanup']=True;os.killpg(process.pid,signal.SIGKILL);process.wait(timeout=10)
            client.destroy();node.destroy_node();rclpy.shutdown();log.close()
        rows.append(row)
        report=dict(method='real_saved_ur5_plan_owned_sigint/v1',cycles=args.cycles,completed=len(rows),
            passed=all(r['passed'] for r in rows),inputs=inputs,results=rows,
            runtime_paths={name:str(path) for name,path in runtime_paths.items()},
            scope='Actual saved UR5 plan-only request and production certificate; no timing barriers, execution goals or hardware drivers',
            environment={k:os.environ.get(k) for k in ['ROS_DOMAIN_ID','ROS_LOCALHOST_ONLY','RMW_IMPLEMENTATION','AMENT_PREFIX_PATH','LD_LIBRARY_PATH']})
        (args.output/'results.json').write_text(json.dumps(report,indent=2)+'\n')
        print(json.dumps(row),flush=True)
        if not row['passed']:return 1
    return 0


if __name__=='__main__':raise SystemExit(main())
