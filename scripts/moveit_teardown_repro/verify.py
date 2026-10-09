#!/usr/bin/env python3
"""Exercise actual TEM or MoveGroup through readiness, owned SIGINT and exit."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import selectors
import signal
import subprocess
import time


def group_members(pgid):
    members=[]
    for item in Path('/proc').iterdir():
        if not item.name.isdigit():continue
        try:
            fields=(item/'stat').read_text().rsplit(')',1)[1].split()
            if fields[0]!='Z' and int(fields[2])==pgid:members.append(int(item.name))
        except (OSError,ValueError,IndexError):pass
    return members


def core_inventory(output):
    paths=(list(Path('/var/crash').glob('*workcell_tem*'))+
           list(Path('/var/crash').glob('*move_group*'))+list(output.glob('core*')))
    return {str(p):(p.stat().st_size,p.stat().st_mtime_ns) for p in paths if p.is_file()}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--binary',type=Path,required=True)
    parser.add_argument('--library',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--cycles',type=int,default=20)
    parser.add_argument('--expect-crash',action='store_true')
    parser.add_argument('--move-group-parameters',type=Path)
    args=parser.parse_args()
    if args.cycles<1:parser.error('cycles must be positive')
    args.output.mkdir(parents=True,exist_ok=False)
    expected=args.library.resolve();rows=[]
    binary=args.binary.resolve()
    command=[str(binary)]
    if args.move_group_parameters:command+=['--ros-args','--params-file',str(args.move_group_parameters.resolve())]
    ready_marker=b'You can start planning now!' if args.move_group_parameters else b'TEM_READY\n'
    cores=core_inventory(args.output)
    for index in range(args.cycles):
        prefix=args.output/f'{index+1:02d}'
        process=subprocess.Popen(command,stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,start_new_session=True,cwd=args.output,
            env=dict(os.environ,ROS_DOMAIN_ID='193',ROS_LOCALHOST_ONLY='1'))
        output=b'';ready=False;timed_out=False;loaded=[];loaded_binary=None
        try:
            selector=selectors.DefaultSelector();selector.register(process.stdout,selectors.EVENT_READ)
            deadline=time.monotonic()+15
            while time.monotonic()<deadline and process.poll() is None:
                if not selector.select(.1):continue
                chunk=os.read(process.stdout.fileno(),65536);output+=chunk
                if ready_marker in output:ready=True;break
            selector.close()
            if ready:
                loaded_binary=str(Path(f'/proc/{process.pid}/exe').resolve())
                maps=Path(f'/proc/{process.pid}/maps').read_text()
                prefix.with_suffix('.maps').write_text(maps)
                loaded=sorted({line.split()[-1] for line in maps.splitlines()
                    if '/libmoveit_trajectory_execution_manager.so' in line})
                os.killpg(process.pid,signal.SIGINT)
            else:
                timed_out=True;os.killpg(process.pid,signal.SIGTERM)
            try:tail,_=process.communicate(timeout=15)
            except subprocess.TimeoutExpired:
                timed_out=True;os.killpg(process.pid,signal.SIGKILL);tail,_=process.communicate(timeout=5)
            output+=tail
        finally:
            if process.poll() is None:
                os.killpg(process.pid,signal.SIGKILL);process.wait(timeout=5)
        prefix.with_suffix('.log').write_bytes(output)
        remaining=group_members(process.pid)
        new_cores=core_inventory(args.output)
        core_changed=new_cores!=cores;cores=new_cores
        correct_library=loaded==[str(expected)]
        passed=ready and not timed_out and correct_library and loaded_binary==str(binary) and not remaining and (
            process.returncode==-signal.SIGSEGV if args.expect_crash else
            process.returncode==0 and (args.move_group_parameters or b'TEM_DESTROYED\n' in output) and not core_changed)
        rows.append(dict(cycle=index+1,pid=process.pid,ready=ready,returncode=process.returncode,
            loaded_libraries=loaded,loaded_executable=loaded_binary,correct_library=correct_library,remaining_processes=remaining,
            core_inventory_changed=core_changed,timed_out=timed_out,passed=passed))
        (args.output/'results.json').write_text(json.dumps(dict(
            cycles=args.cycles,completed=len(rows),passed=all(row['passed'] for row in rows),
            expected_crash=args.expect_crash,library=str(expected),executable=str(binary),
            executable_sha256=hashlib.sha256(binary.read_bytes()).hexdigest(),command=command,
            library_sha256=hashlib.sha256(expected.read_bytes()).hexdigest(),results=rows),indent=2)+'\n')
        if not passed:return 1
    return 0


if __name__=='__main__':raise SystemExit(main())
