"""No-motion live acceptance: SCENE SCENE_SOURCE_DIR EVIDENCE_DIR.

Run from the repository root after sourcing ROS, Builder and generated packages.
Launches fake hardware/RViz, reads services/TF/markers, then shuts down its launch.
"""
import os,sys,time,json,signal,subprocess,math
from pathlib import Path
import rclpy,yaml
import xml.etree.ElementTree as ET
sys.path.insert(0,str(Path.cwd()/"scripts"))
from asset_mounting import compose, rotation
from rcl_interfaces.srv import GetParameters
from controller_manager_msgs.srv import ListControllers
from moveit_msgs.srv import GetPlanningScene
from moveit_msgs.msg import PlanningSceneComponents
from sensor_msgs.msg import JointState
from visualization_msgs.msg import MarkerArray
from tf2_ros import Buffer,TransformListener
scene=sys.argv[1];root=Path(sys.argv[2]);out=Path(sys.argv[3]);out.mkdir(parents=True,exist_ok=True)
f=open(out/'launch.log','w');p=subprocess.Popen(['ros2','launch',scene,'demo.launch.py','use_fake_hardware:=true','launch_rviz:=true'],stdout=f,stderr=subprocess.STDOUT,start_new_session=True)
rclpy.init();n=rclpy.create_node('mounting_acceptance_probe');buf=Buffer();listener=TransformListener(buf,n);states=[]
n.create_subscription(JointState,'/joint_states',lambda m:states.append(m),10)
markers=[]
n.create_subscription(MarkerArray,'/'+scene+'/canonical_mesh_markers',lambda m:markers.append(m),10)
result={'scene':scene,'fake_hardware_only':True,'execution_goals_sent':0,'result':'FAIL'}
def call(typ,name,req,limit=35):
 c=n.create_client(typ,name)
 if not c.wait_for_service(timeout_sec=limit):raise RuntimeError('service missing: '+name)
 future=c.call_async(req);rclpy.spin_until_future_complete(n,future,timeout_sec=limit)
 if not future.done():raise RuntimeError('service timeout: '+name)
 return future.result()
try:
 desc=call(GetParameters,'/move_group/get_parameters',GetParameters.Request(names=['robot_description','robot_description_semantic']))
 assert all(v.string_value for v in desc.values),'empty robot description/semantic'
 assert 'mock_components/GenericSystem' in desc.values[0].string_value,'not exclusively fake hardware'
 result['description_and_semantic_loaded']=True
 deadline=time.monotonic()+40
 while time.monotonic()<deadline:
  controllers=call(ListControllers,'/controller_manager/list_controllers',ListControllers.Request(),10)
  active={c.name:c.state for c in controllers.controller};rclpy.spin_once(n,timeout_sec=.3)
  if len(active)>=3 and all(v=='active' for v in active.values()) and states:break
 assert len(active)>=3 and all(v=='active' for v in active.values()),active
 assert states,'no joint-state messages'
 result['controllers']=active;result['joint_states_received']=len(states)
 req=GetPlanningScene.Request();req.components.components=PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
 env=yaml.safe_load((root/'environment.yaml').read_text())
 expected=yaml.safe_load((root/'config/moveit_collision_objects.yaml').read_text())
 deadline=time.monotonic()+35
 while time.monotonic()<deadline:
  response=call(GetPlanningScene,'/get_planning_scene',req,10)
  actual={x.id:x for x in response.scene.world.collision_objects}
  if len(actual)>=len(expected['objects']):break
  rclpy.spin_once(n,timeout_sec=.5)
 result['collision_object_ids']=list(actual)
 for obj in expected['objects']:
  assert obj['id'] in actual,'missing collision '+obj['id']
  item=actual[obj['id']];poses=item.primitive_poses or item.mesh_poses
  assert poses,'missing collision pose '+obj['id']
  q=item.pose.orientation;position=item.pose.position;local=poses[0].position
  v=[local.x,local.y,local.z];u=[q.x,q.y,q.z]
  def cross(a,b):return [a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]]
  t=[2*x for x in cross(u,v)];c=cross(u,t)
  world=[p+x+q.w*y+z for p,x,y,z in zip([position.x,position.y,position.z],v,t,c)]
  result.setdefault('collision_world_xyz',{})[obj['id']]=world
  assert math.dist(world,obj['pose']['xyz'])<1e-7,'collision transform mismatch '+obj['id']
  a=q;b=poses[0].orientation
  quaternion=[a.w*b.x+a.x*b.w+a.y*b.z-a.z*b.y,a.w*b.y-a.x*b.z+a.y*b.w+a.z*b.x,a.w*b.z+a.x*b.y-a.y*b.x+a.z*b.w,a.w*b.w-a.x*b.x-a.y*b.y-a.z*b.z]
  expected_q=obj['pose']['quaternion_xyzw']
  assert min(math.dist(quaternion,expected_q),math.dist(quaternion,[-v for v in expected_q]))<1e-7,'collision rotation mismatch '+obj['id']
 result['collision_manifest_applied']=True
 deadline=time.monotonic()+12
 while time.monotonic()<deadline and not buf.can_transform('world','camera_link',rclpy.time.Time()):rclpy.spin_once(n,timeout_sec=.2)
 t=buf.lookup_transform('world','camera_link',rclpy.time.Time()).transform
 xyz=[t.translation.x,t.translation.y,t.translation.z];result['camera_world_xyz']=xyz
 expanded=subprocess.check_output(['xacro',str(root/'urdf/scene.urdf.xacro'),'use_fake_hardware:=true'],text=True)
 model=ET.fromstring(expanded);joints={j.find('child').get('link'):j for j in model.findall('joint')}
 def frame(name):
  if name=='world':return ([0.,0.,0.],rotation([0.,0.,0.]))
  j=joints[name];o=j.find('origin')
  return compose(frame(j.find('parent').get('link')),([float(v) for v in (o.get('xyz') or '0 0 0').split()],rotation([float(v) for v in (o.get('rpy') or '0 0 0').split()])))
 expected_xyz,expected_rotation=frame('camera_link');result['generated_camera_world_xyz']=expected_xyz
 assert math.dist(xyz,expected_xyz)<1e-7,(xyz,expected_xyz)
 q=t.rotation;x,y,z,w=q.x,q.y,q.z,q.w
 actual_rotation=[[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]]
 assert max(abs(a-b) for row,other in zip(expected_rotation,actual_rotation) for a,b in zip(row,other))<1e-7,'camera rotation mismatch'
 result['camera_tf_matches_authored']=True
 log=(out/'launch.log').read_text();assert 'rviz2' in log,'RViz not started'
 assert markers and markers[-1].markers,'no canonical mesh markers received'
 assert '[ERROR]' not in log,'runtime error before shutdown; inspect launch.log'
 result['mesh_markers_received']=len(markers[-1].markers)
 result['rviz_started']=True;result['result']='PASS'
except Exception as e:result['failure']=str(e)
finally:
 n.destroy_node();rclpy.shutdown();os.killpg(p.pid,signal.SIGINT)
 try:p.wait(timeout=12)
 except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGTERM);p.wait(timeout=5)
 f.close();result['launch_exit_code']=p.returncode
 (out/'acceptance.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
 sys.exit(0 if result['result']=='PASS' else 1)
