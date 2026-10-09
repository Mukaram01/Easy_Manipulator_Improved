#!/usr/bin/env python3
"""Owned active-traffic bridge teardown proof, without simulation or robot motion."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import signal
import subprocess
import time


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def members(pgid):
    found=[]
    for path in Path('/proc').iterdir():
        if not path.name.isdigit():continue
        try:
            fields=(path/'stat').read_text().rsplit(')',1)[1].split()
            if fields[0]!='Z' and int(fields[2])==pgid:found.append(int(path.name))
        except (OSError,ValueError,IndexError):pass
    return found


def cores():
    return {str(p):(p.stat().st_size,p.stat().st_mtime_ns)
            for p in Path('/var/crash').glob('*parameter_bridge*.crash')}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--binary',type=Path,required=True)
    parser.add_argument('--publisher',type=Path,required=True)
    parser.add_argument('--library',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--cycles',type=int,default=20)
    parser.add_argument('--valgrind',type=Path)
    parser.add_argument('--expect-abort',action='store_true')
    args=parser.parse_args()
    if args.cycles<1 or (args.valgrind and args.expect_abort):parser.error('invalid proof options')
    args.output.mkdir(parents=True,exist_ok=False)
    # Keep all proof processes isolated from workstation commissioning domains.
    os.environ.update(ROS_DOMAIN_ID='196',ROS_LOCALHOST_ONLY='1',IGN_PARTITION='workcell_bridge_shutdown_proof')
    import rclpy
    from std_msgs.msg import String
    rclpy.init();node=rclpy.create_node('bridge_shutdown_proof');count=[0]
    subscriber=node.create_subscription(String,'/world/a0/workcell_measurements',
        lambda message:count.__setitem__(0,count[0]+1),1000)
    binary=args.binary.resolve();library=args.library.resolve()
    rows=[];success=False
    try:
        for index in range(1,args.cycles+1):
            before=count[0];core_before=cores();processes=[];logs=[];row=dict(cycle=index)
            bridge_command=[str(binary),'--ros-args','-p',
                'config_file:='+str(Path(__file__).with_name('bridges.yaml').resolve())]
            checker_log=args.output/f'{index:02}-memcheck.log'
            if args.valgrind:
                bridge_command=[str(args.valgrind.resolve()),'--tool=memcheck','--error-exitcode=99',
                    '--num-callers=30','--log-file='+str(checker_log),*bridge_command]
            try:
                for name,command in [('publisher',[str(args.publisher.resolve())]),('bridge',bridge_command)]:
                    log=(args.output/f'{index:02}-{name}.log').open('w');logs.append(log)
                    processes.append(subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,
                        start_new_session=True,cwd=args.output))
                publisher,bridge=processes
                until=time.monotonic()+(30 if args.valgrind else 6)
                while count[0]-before<300 and time.monotonic()<until and bridge.poll() is None:
                    rclpy.spin_once(node,timeout_sec=.02)
                maps=Path(f'/proc/{bridge.pid}/maps').read_text()
                (args.output/f'{index:02}-bridge.maps').write_text(maps)
                loaded=sorted({line.split()[-1] for line in maps.splitlines() if '/libros_gz_bridge.so' in line})
                exe=str(Path(f'/proc/{bridge.pid}/exe').resolve())
                mapped_executable=any(line.split()[-1]==str(binary) for line in maps.splitlines())
                bridge.send_signal(signal.SIGINT)
                time.sleep(.003)
                publisher.send_signal(signal.SIGINT)
                codes=[p.wait(timeout=30 if args.valgrind else 8) for p in processes]
                errors=0
                if args.valgrind:
                    matches=re.findall(r'ERROR SUMMARY: ([\d,]+) errors',checker_log.read_text())
                    errors=int(matches[-1].replace(',','')) if matches else -1
                row.update(pids=[p.pid for p in processes],received=count[0]-before,
                    returncodes=codes,loaded_executable=exe,mapped_executable=mapped_executable,
                    loaded_libraries=loaded,memcheck_errors=errors,core_inventory_changed=cores()!=core_before)
            except Exception as exc:
                row['exception']=repr(exc)
            finally:
                for process in processes:
                    if process.poll() is None:
                        os.killpg(process.pid,signal.SIGKILL);process.wait(timeout=5)
                        row['forced_kill']=True
                for log in logs:log.close()
            row['remaining_processes']=[pid for p in processes for pid in members(p.pid)]
            healthy=(row.get('received',0)>=300 and row.get('loaded_libraries')==[str(library)]
                and row.get('mapped_executable') and (args.valgrind or row.get('loaded_executable')==str(binary))
                and not row.get('forced_kill') and not row['remaining_processes'] and 'exception' not in row)
            if args.expect_abort:
                row['passed']=bool(healthy and row.get('returncodes')==[0,-signal.SIGABRT])
            else:
                row['passed']=bool(healthy and row.get('returncodes')==[0,0]
                    and row.get('memcheck_errors')==0 and not row.get('core_inventory_changed'))
            rows.append(row)
            success=any(r['passed'] for r in rows) if args.expect_abort else len(rows)==args.cycles and all(r['passed'] for r in rows)
            result=dict(cycles=args.cycles,completed=len(rows),passed=sum(r['passed'] for r in rows),
                all_success=success,expected_abort=args.expect_abort,binary=str(binary),binary_sha256=digest(binary),
                library=str(library),library_sha256=digest(library),errors=sum(r.get('memcheck_errors',0) for r in rows),results=rows)
            (args.output/'results.json').write_text(json.dumps(result,indent=2)+'\n')
            print(json.dumps(row),flush=True)
            if (args.expect_abort and row['passed']) or (not args.expect_abort and not row['passed']):break
    finally:
        node.destroy_node();rclpy.shutdown()
    return 0 if success else 1


if __name__=='__main__':raise SystemExit(main())
