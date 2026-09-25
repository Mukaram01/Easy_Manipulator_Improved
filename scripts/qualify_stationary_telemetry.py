#!/usr/bin/env python3
"""No-motion qualification of the executor's telemetry, waits, logs and stop window."""
import argparse,json,time
from pathlib import Path
import rclpy
from rclpy.task import Future
from simulator_execution import Measurements,StopWindow

def run(receipt,output,duration):
 output.mkdir(parents=True,exist_ok=False)
 rclpy.init();node=rclpy.create_node('stationary_telemetry_qualification')
 m=Measurements(node,receipt,output/'measurements.jsonl');report={'result':'FAIL','motion_commands':0}
 try:
  until=time.monotonic()+5
  while m.latest is None and time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.005)
  m.fresh();m.arm()
  names={j.get('name') for control in m.robot.findall('ros2_control') for j in control.findall('joint')}
  window=StopWindow(names);stop=None;count=0;max_age=0.;started=time.monotonic()
  while time.monotonic()-started<duration:
   # Exercise a ROS wait with acquisition running, then the same batch evidence
   # writing used during cancellation. No execution or controller goal is sent.
   f=Future();timer=node.create_timer(.6,lambda: f.set_result(True) if not f.done() else None)
   rclpy.spin_until_future_complete(node,f,timeout_sec=1);node.destroy_timer(timer)
   sample=m.fresh();max_age=max(max_age,time.time()-sample['wall_ns']/1e9)
   pending=m.drain()
   for s in pending:
    m.record('stationary_wait',s);count+=1
    if stop is None:
     began=time.monotonic_ns();stationary=window.observe(s,m.joints(s))
     m.timing.append(dict(event='stop_check',iteration=s['iteration'],duration_ns=time.monotonic_ns()-began))
     if stationary:stop=window.evidence
   # This is intentionally the existing whole-window evidence-write workload.
   if stop is not None:
    (output/'stop-window.json').write_text(json.dumps(stop,indent=2))
   m.fresh()
  if stop is None:raise RuntimeError('no consecutive 300ms stationary window')
  report.update(result='PASS',samples=count,wall_seconds=time.monotonic()-started,max_age_seconds=max_age,
      stop_duration_sim_ns=stop['duration_sim_ns'],stop_max_velocity=stop['max_velocity_rad_s'])
 except Exception as e:
  report.update(error=str(e),latest_iteration=m.latest['iteration'] if m.latest else None,
      latest_age_seconds=time.time()-m.latest['wall_ns']/1e9 if m.latest else None)
 finally:
  m.close();node.destroy_node();rclpy.shutdown();(output/'summary.json').write_text(json.dumps(report,indent=2));print(json.dumps(report))
 return 0 if report['result']=='PASS' else 1
if __name__=='__main__':
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--receipt',required=True,type=Path);p.add_argument('--output',required=True,type=Path);p.add_argument('--duration',type=float,default=10.)
 a=p.parse_args();raise SystemExit(run(a.receipt,a.output,a.duration))
