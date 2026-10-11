#!/usr/bin/env python3
"""One isolated Fortress/EPD measurement capture. Never starts ROS or MoveIt."""
import argparse
import hashlib
import json
import os
import signal
import subprocess
import time
import uuid
import xml.etree.ElementTree as ET
from pathlib import Path


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--world',type=Path,required=True)
    parser.add_argument('--provider-library',type=Path,required=True)
    parser.add_argument('--capture-binary',type=Path,required=True)
    parser.add_argument('--model',type=Path,required=True)
    parser.add_argument('--labels',type=Path,required=True)
    parser.add_argument('--support-collision',required=True)
    parser.add_argument('--workpieces',nargs='+',required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    for path in (args.world,args.provider_library,args.capture_binary,args.model,args.labels):
        if not path.is_file():parser.error('missing input: '+str(path))
    tree=ET.parse(args.world);world=tree.getroot().find('world')
    if world is None:parser.error('one explicit world required')
    run_id=str(uuid.uuid4())
    plugin=ET.SubElement(world,'plugin',filename=str(args.provider_library.resolve()),
                         name='WorkcellPhysicsContactMeasurement')
    ET.SubElement(plugin,'support_collision').text=args.support_collision
    ET.SubElement(plugin,'workpieces').text=' '.join(args.workpieces)
    ET.SubElement(plugin,'run_id').text=run_id
    args.output.mkdir(exist_ok=False)
    derived=args.output/'world.sdf';tree.write(derived,encoding='utf-8',xml_declaration=True)
    env=dict(os.environ,IGN_PARTITION='workcell-contact-'+run_id)
    result=dict(run_id=run_id,partition=env['IGN_PARTITION'],execution_goals_sent=0,
        ros_bridge_started=False,moveit_started=False,
        hashes={str(p.resolve()):hashlib.sha256(p.read_bytes()).hexdigest()
            for p in (args.world,derived,args.provider_library,args.capture_binary,args.model,args.labels)})
    with (args.output/'gazebo.log').open('w') as log:
        server=subprocess.Popen(['ign','gazebo','-s','-r',str(derived),'-v','3'],env=env,
            stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        try:
            time.sleep(30)
            if server.poll() is not None:raise RuntimeError('owned Gazebo exited before capture')
            with (args.output/'capture.log').open('w') as capture_log:
                capture=subprocess.run([str(args.capture_binary.resolve()),str(args.model.resolve()),
                    str(args.labels.resolve()),str(args.output.resolve()/'capture'),world.get('name'),run_id],
                    env=env,stdout=capture_log,stderr=subprocess.STDOUT,timeout=120)
                result['capture_exit']=capture.returncode
                if capture.returncode!=0:raise RuntimeError('capture failed: '+str(capture.returncode))
        finally:
            if server.poll() is None:
                os.killpg(server.pid,signal.SIGINT)
                try:server.wait(timeout=15)
                except subprocess.TimeoutExpired:
                    os.killpg(server.pid,signal.SIGTERM)
                    try:server.wait(timeout=10)
                    except subprocess.TimeoutExpired:
                        os.killpg(server.pid,signal.SIGKILL);server.wait(timeout=5)
            result['owned_server_exit']=server.returncode
            (args.output/'run.json').write_text(json.dumps(result,indent=2)+'\n')
    print('captured isolated simulation-only measurement at '+str(args.output))


if __name__=='__main__':
    main()
