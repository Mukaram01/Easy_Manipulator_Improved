#!/usr/bin/env python3
"""Offline: rebuild one traced certificate's planning scene as the runtime now plans that stage.

Copies ``<trace prefix>.trajectory.cdr`` and writes ``<out prefix>.scene.cdr``: the traced frozen scene passed through
``simulation_collision.physics_pad_scene`` with the session's receipt-bound physics boxes and the stage's allowance
(``PHYSICS_PAD_ALLOWANCE_M``, or ``--allowance-m``). Replay it with ``certificate_replay`` (same header-only
certificate). Diagnostic only: nothing here changes the certificate or any runtime scene.

usage: physics_pad_scene.py <trace prefix> <session dir> <stage> <out prefix> [--allowance-m M]
"""
import argparse
import json
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from simulation_collision import PHYSICS_PAD_ALLOWANCE_M, physics_pad_scene  # noqa: E402


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('prefix')
    parser.add_argument('session', type=Path)
    parser.add_argument('stage', choices=sorted(PHYSICS_PAD_ALLOWANCE_M))
    parser.add_argument('out')
    parser.add_argument('--allowance-m', type=float)
    args = parser.parse_args(argv)
    from moveit_msgs.msg import PlanningScene
    from rclpy.serialization import deserialize_message, serialize_message
    receipt = json.loads((args.session / 'runtime/receipt.json').read_text())
    links = {link.get('name') for link in ET.parse(args.session / 'runtime/robot.urdf').getroot().findall('link')}
    allowance = PHYSICS_PAD_ALLOWANCE_M[args.stage] if args.allowance_m is None else args.allowance_m
    view = deserialize_message(Path(args.prefix + '.scene.cdr').read_bytes(), PlanningScene)
    scene = physics_pad_scene(view, receipt['simulator_collision_primitives'], links, allowance)
    Path(args.out + '.scene.cdr').write_bytes(serialize_message(scene))
    Path(args.out + '.trajectory.cdr').write_bytes(Path(args.prefix + '.trajectory.cdr').read_bytes())
    print(json.dumps(dict(stage=args.stage, allowance_m=allowance,
                          pads=[b.object.id for b in scene.robot_state.attached_collision_objects
                                if b.object.id.startswith('physics_pad::')])))


if __name__ == '__main__':
    main()
