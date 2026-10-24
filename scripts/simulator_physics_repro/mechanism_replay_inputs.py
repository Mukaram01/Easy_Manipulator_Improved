"""Inputs for mechanism_replay from a recorded Stage-A full-cycle session (read-only).

usage: mechanism_replay_inputs.py <session gate dir, e.g. .../06-full-cycle> <first iteration> <last iteration> <out dir>
Writes mechanism.txt (gripper joints, inertias, pad boxes from runtime/robot.sdf), replay.txt (start state,
hold command, recorded wrist poses) and arm.txt (UR5 chain and recorded arm joint states).
"""
import json, math, sys, xml.etree.ElementTree as ET
import numpy as np
run, start, stop, out = sys.argv[1], int(sys.argv[2]), int(sys.argv[3]), sys.argv[4]
root = ET.parse(f'{run}/runtime/robot.sdf').getroot()
links = {l.get('name'): l for l in root.iter('link')}
joints = [j for j in root.iter('joint') if j.get('name').startswith('gripper_')]
with open(out + '/mechanism.txt', 'w') as f:
    for j in joints:
        ax = j.find('axis'); lim = ax.find('limit')
        child = links[j.findtext('child')]; inert = child.find('inertial'); I = inert.find('inertia')
        assert j.find('pose').get('relative_to') == j.findtext('parent') and ax.findtext('xyz').split() == ['0', '0', '1']
        assert (child.findtext('pose') or '0 0 0 0 0 0').split() == ['0'] * 6
        ip = (inert.findtext('pose') or '0 0 0 0 0 0').split(); assert ip[3:] == ['0', '0', '0']
        mm = j.find('.//mimic')
        f.write(' '.join(['J', j.get('name'), j.findtext('parent'), j.findtext('child'), j.findtext('pose'),
                          lim.findtext('lower'), lim.findtext('upper'), lim.findtext('effort'),
                          mm.get('joint') if mm is not None else '-', mm.findtext('multiplier') if mm is not None else '0',
                          inert.findtext('mass'), *ip[:3], *(I.findtext(k) for k in ('ixx', 'iyy', 'izz', 'ixy', 'ixz', 'iyz'))]) + '\n')
        for c in child.findall('collision'):
            b = c.find('geometry/box')
            if b is None: continue
            mu = c.findtext('surface/friction/ode/mu')
            f.write(' '.join(['B', child.get('name'), c.findtext('pose'), b.findtext('size'), mu,
                              c.findtext('surface/contact/ode/min_depth') or '0']) + '\n')
    world = ET.parse(f'{run}/runtime/world.sdf').getroot()
    part = world.find(".//model[@name='part_04']/link/collision")
    f.write(f"P {part.findtext('surface/contact/ode/min_depth') or '0'}\n")
rows = []
with open(f'{run}/full-cycle/summary.measurements.jsonl') as fh:
    for line in fh:
        r = json.loads(line); s = r['measurement']
        if start <= s['iteration'] <= stop:
            rows.append(s)
rows.sort(key=lambda s: s['iteration'])
with open(out + '/replay.txt', 'w') as f:
    s0 = rows[0]; P = s0['poses']; J = {k.split('::')[-1]: v for k, v in s0['joints'].items()}
    f.write('S ' + ' '.join(map(repr, P['a0::workcell_robot::wrist_3_link'] + P['a0::part_04'])) + '\n')
    f.write(f"H {json.load(open(f'{run}/full-cycle/summary.json'))['grasp_hold']['hold_command_rad']!r}\n")
    for n, v in J.items():
        if n.startswith('gripper_'): f.write(f'Q {n} {v[0]!r} {v[1]!r}\n')
    for s in rows:
        f.write(f"T {s['iteration']} " + ' '.join(map(repr, s['poses']['a0::workcell_robot::wrist_3_link'])) + '\n')
print(len(rows), rows[0]['iteration'], rows[-1]['iteration'])
# Arm chain (UR5) for the dynamic-arm variant: same line format, tag A; measured arm joint positions per sample.
ARM = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
with open(out + '/arm.txt', 'w') as f:
    base = root.find(".//joint[@name='base_joint']")
    f.write('BASE ' + base.findtext('pose') + '\n')
    for name in ARM:
        j = root.find(f".//joint[@name='{name}']"); lim = j.find('axis/limit'); child = links[j.findtext('child')]
        inert = child.find('inertial'); I = inert.find('inertia'); ip = (inert.findtext('pose') or '0 0 0 0 0 0').split()
        assert j.findtext('axis/xyz').split() == ['0', '0', '1']
        r_, p_, y_ = map(float, ip[3:])
        Rx = np.array([[1, 0, 0], [0, math.cos(r_), -math.sin(r_)], [0, math.sin(r_), math.cos(r_)]])
        Ry = np.array([[math.cos(p_), 0, math.sin(p_)], [0, 1, 0], [-math.sin(p_), 0, math.cos(p_)]])
        Rz = np.array([[math.cos(y_), -math.sin(y_), 0], [math.sin(y_), math.cos(y_), 0], [0, 0, 1]])
        g = lambda k: float(I.findtext(k))
        M = np.array([[g('ixx'), g('ixy'), g('ixz')], [g('ixy'), g('iyy'), g('iyz')], [g('ixz'), g('iyz'), g('izz')]])
        R = Rz @ Ry @ Rx; M = R @ M @ R.T
        f.write(' '.join(['A', name, j.findtext('parent'), j.findtext('child'), j.findtext('pose'), lim.findtext('lower'), lim.findtext('upper'),
                          lim.findtext('effort'), '-', '0', inert.findtext('mass'), *ip[:3],
                          *map(repr, (M[0, 0], M[1, 1], M[2, 2], M[0, 1], M[0, 2], M[1, 2]))]) + '\n')
    for s in rows:
        J = {k.split('::')[-1]: v for k, v in s['joints'].items()}
        f.write(f"R {s['iteration']} " + ' '.join(repr(J[n][0]) + ' ' + repr(J[n][1]) for n in ARM) + '\n')
