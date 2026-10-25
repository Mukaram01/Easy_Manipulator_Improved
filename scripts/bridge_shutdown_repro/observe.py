"""Observe real bridge shutdown ordering; never alter inferior state or publish admission."""
import argparse, hashlib, importlib.util, json, os, signal, subprocess, time
from pathlib import Path
script_root=Path(__file__).resolve().parent
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--binary',type=Path,required=True)
parser.add_argument('--publisher',type=Path,required=True)
parser.add_argument('--output',type=Path,required=True)
parser.add_argument('--cycles',type=int,default=5)
args=parser.parse_args()
if not 5 <= args.cycles <= 10:parser.error('observation requires 5–10 bounded cycles')
root=args.output.resolve();root.mkdir(parents=True,exist_ok=False)
proof=script_root
spec=importlib.util.spec_from_file_location('existing_verify',proof/'verify.py')
verify=importlib.util.module_from_spec(spec);spec.loader.exec_module(verify)
os.environ.update(ROS_DOMAIN_ID='196',ROS_LOCALHOST_ONLY='1',
                  IGN_PARTITION='workcell_bridge_shutdown_proof',RMW_IMPLEMENTATION='rmw_fastrtps_cpp')
import rclpy
from std_msgs.msg import String
binary=args.binary.resolve(strict=True)
library=Path('/opt/ros/humble/lib/libros_gz_bridge.so').resolve()
publisher=args.publisher.resolve(strict=True)
def identity(p):
    p=Path(p).resolve();s=p.stat()
    return dict(path=str(p),sha256=verify.digest(p),device=s.st_dev,inode=s.st_ino)
metadata=dict(binary=identity(binary),library=identity(library),publisher=identity(publisher),
              config=identity(proof/'bridges.yaml'),verifier=identity(proof/'verify.py'),
              instrumentation=[identity(__file__),identity(script_root/'observe.gdb.py')],
              environment={k:v for k,v in os.environ.items() if k.startswith(('ROS_','RMW_','IGN_','AMENT_')) or k in ('LD_LIBRARY_PATH','LD_PRELOAD','LD_AUDIT')},
              cycles_limit=args.cycles,traffic_threshold=300,traffic_timeout_seconds=30,shutdown_timeout_seconds=30,
              purpose='Natural shutdown observation. GDB affects timing; this is not a historical crash reproduction.')
(root/'experiment.json').write_text(json.dumps(metadata,indent=2)+'\n')
rclpy.init();node=rclpy.create_node('bridge_shutdown_observation');count=[0]
sub=node.create_subscription(String,'/world/a0/workcell_measurements',lambda msg:count.__setitem__(0,count[0]+1),1000)
rows=[]
try:
    for index in range(1,args.cycles+1):
        out=root/f'{index:02}';out.mkdir(exist_ok=False)
        before=count[0];core_before=verify.cores();procs=[];logs=[];pid=None;groups=set();row=dict(cycle=index)
        try:
            commands=[('publisher',[str(publisher)]),('debugger',['/usr/bin/gdb','-q','-nx','-batch','-x',str(script_root/'observe.gdb.py'),'--args',str(binary),'--ros-args','-p','config_file:='+str(proof/'bridges.yaml')])]
            row['commands']=commands
            for name,command in commands:
                log=(out/(name+'.log')).open('w');logs.append(log)
                p=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,start_new_session=True,cwd=out,
                                   env=dict(os.environ,BRIDGE_OBSERVATION_CYCLE=str(out)))
                procs.append(p);groups.add(p.pid)
            pub,debug=procs
            until=time.monotonic()+30
            while count[0]-before<300 and time.monotonic()<until and debug.poll() is None:
                rclpy.spin_once(node,timeout_sec=.02)
            row['received']=count[0]-before
            pid=int((out/'inferior.pid').read_text());groups.add(os.getpgid(pid))
            maps=Path(f'/proc/{pid}/maps').read_text();(out/'bridge.maps').write_text(maps)
            exe=str(Path(f'/proc/{pid}/exe').resolve())
            row.update(pids=dict(publisher=pub.pid,debugger=debug.pid,bridge=pid),groups=sorted(groups),loaded_executable=exe)
            mapped=sorted({line.split()[-1] for line in maps.splitlines() if len(line.split())==6 and line.split()[-1].startswith('/')})
            libs=[p for p in mapped if any(s in p for s in ('libros_gz_bridge','librclcpp','librcl.','librcutils','librcl_logging','libspdlog','librmw','libfastcdr','libfastrtps','libddsc','libc.so.','libstdc++','libignition','libsdformat'))]
            row['loaded_identities']=[identity(p) for p in libs]
            (out/'task_inventory.json').write_text(json.dumps({p.name:(p/'comm').read_text().strip() for p in Path(f'/proc/{pid}/task').iterdir()},indent=2))
            if row['received']<300 or exe!=str(binary) or str(library) not in mapped:
                raise RuntimeError('traffic or identity prerequisite failed')
            row['signal_sent_monotonic_ns']=time.monotonic_ns()
            os.kill(pid,signal.SIGINT)
            time.sleep(.003)
            pub.send_signal(signal.SIGINT)
            row['publisher_returncode']=pub.wait(timeout=30)
            row['debugger_returncode']=debug.wait(timeout=30)
            events=[json.loads(s) for s in (out/'events.jsonl').read_text().splitlines()]
            exits=[e for e in events if e['kind']=='inferior_exited']
            row['bridge_returncode']=exits[-1]['exit_code'] if exits else None
            row['observer_errors']=[e for e in events if e['kind']=='observer_error']
            row['debugger_terminated_inferior']=any(e['kind']=='observer_finished' and e['pid'] for e in events)
            if row['debugger_terminated_inferior']:row['observer_errors'].append({'message':'observer returned before inferior exit'})
            destroyed=[e for e in events if e['kind']=='return' and e.get('function')=='spdlog::details::registry::~registry()']
            accesses=[e for e in events if e['kind']=='entry' and 'registry::drop(' in e.get('function','')]
            row['access_after_destruction']=any(a['seq']>d['seq'] and a['object']==d['object'] for a in accesses for d in destroyed)
        except Exception as exc:row['exception']=repr(exc)
        finally:
            for group in sorted(groups):
                if verify.members(group):
                    row['forced_cleanup']=True
                    try:os.killpg(group,signal.SIGKILL)
                    except ProcessLookupError:pass
            for p in procs:
                if p.poll() is None:p.wait(timeout=5)
            for log in logs:log.close()
            row['remaining_processes']=sorted({p for group in groups for p in verify.members(group)})
            row['core_inventory_changed']=verify.cores()!=core_before
        row['clean_observation']=bool(row.get('received',0)>=300 and row.get('publisher_returncode')==0 and row.get('debugger_returncode')==0 and row.get('bridge_returncode')==0 and not row.get('forced_cleanup') and not row['remaining_processes'] and not row['core_inventory_changed'] and 'exception' not in row and not row.get('observer_errors'))
        rows.append(row);(root/'results.json').write_text(json.dumps(rows,indent=2)+'\n')
        print(json.dumps({k:v for k,v in row.items() if k not in ('commands','loaded_identities')}),flush=True)
        if row.get('access_after_destruction') or not row['clean_observation']:break
finally:
    node.destroy_node();rclpy.shutdown()

raise SystemExit(0 if len(rows)==args.cycles and all(r['clean_observation'] for r in rows) else 1)
