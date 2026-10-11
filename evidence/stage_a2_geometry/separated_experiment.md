# Controlled separated-cube experiment

Source commit `892c9b43`; raw evidence `/tmp/workcell_stage_a2_separated`.
The compact machine-readable record is `separated_experiment.json`.

## Physical experiment and results

A disposable clone of canonical `stage_a0.sdf` retains `part_00` and `part_01` at
`(0.35, -0.217, 0.0125)` and `(0.45, -0.217, 0.0125)` metres, with yaw 0.15 and
−0.20 rad. Initial bottoms lie exactly on the original z=0 support plane.
All other cube fields are byte-equivalent XML subtrees: 25 mm dimensions, 20 g
mass, inertia, collision/contact/friction and visual properties unchanged.
Support, destination, light and physics subtrees are unchanged. The original
capture world has no robot model; no robot/gripper is started or altered. The
existing MoveIt robot/gripper definitions and authored scene are unchanged.
The canonical piled world remains byte-identical.

After 30 s of real physics settling, both centres have z=0.012499420027 m.
The numerical support contact depth is 0.000580 mm; no initial penetration or
floating placement was authored. Bracketing translation change is zero and
angular change is 2.98e-8 rad. Independent world-x projection proves at least
71.038 mm physical cube separation. Both cubes are visible (376 pixels each).

Genuine Ogre RGB-D → external EPD Mask R-CNN → segmented depth produced **2
EPD detections, 2 valid surfaces and 2 collision-ready objects**, with zero
geometry rejections. The existing explicit 25 mm workpiece profile is unchanged.
RGB/depth/calibration timestamps match; live camera-pose readback is verified.
Simulator poses enter only the independent validator. Centre errors are
0.947 and 1.215 mm, both within existing uncertainty; true corners lie inside
both reconstructed envelopes. Envelope edges remain 40.616 and 42.348 mm.
The native geometry predicate reports 51.960 mm inter-envelope gap; an
independent world-x projection proves at least 51.926 mm separation.

## Admission blocker

Running the **existing** `pile_extraction.extraction_intents` and its native
geometry predicate with the same table BOX consumed by the full-cycle preplanner
rejects both targets:

| EPD object | Table-envelope overlap | Existing limit | Result |
|---|---:|---:|---|
| `epd_27402000000_0` | 7.808697 mm | 0.1 mm | BLOCKED |
| `epd_27402000000_1` | 8.678595 mm | 0.1 mm | BLOCKED |

Exact reason: `NO_VALID_EXTRACTION / EXTRACTION_INITIAL_DEPTH`, neighbor
`workcell::support_surface_table`. The table top and Gazebo support plane both
remain z=0 in world coordinates. This is **uncertainty-envelope overlap**, not
physical cube penetration or a failed IK claim. A resting 25 mm cube has a
12.5 mm centre height; its current symmetric 40–42 mm collision envelope extends
below that support. Inter-cube separation therefore does not remove this blocker.

The required extraction prerequisite failed, so the expensive full-cycle runner
was **NOT RUN**. Approach, descent, closing, lift, retreat, transfer, placement,
release and home are **NOT RUN for this capture**. Earlier piled-capture approach/
descent/closing successes remain historical evidence, not separated-cube PASSes.
Zero robot/controller goals were sent; neither MoveIt nor a bridge was launched.
The owned Gazebo server was interrupted and waited cleanly.

Capture target build PASS, existing `masked_depth_geometry` CTest PASS, and
38 focused Python regressions PASS (`test_stage_a_rgbd`, `test_stage_a2_geometry`,
`test_pile_extraction`). No production implementation changed, so no unrelated
Builder/native test campaign was repeated. Protected Stage-A1 HEAD, all file
statuses and tracked diff match the saved fingerprint.

## Reproduce

From the isolated worktree, derive a fresh disposable physical world:

```bash
cd /home/user/workcell_ws_stage_a2
export SEP_RUN=$(mktemp -d /tmp/workcell-a2-separated.XXXXXX)
python3 - <<'PY'
import os, copy, xml.etree.ElementTree as E
from pathlib import Path
root=E.fromstring(Path('scenes/ur5_2f_test/worlds/stage_a0.sdf').read_bytes())
world=root.find('world')
poses=['0.35 -0.217 0.0125 0 0 0.15', '0.45 -0.217 0.0125 0 0 -0.2']
parts=[m for m in world.findall('model') if m.get('name','').startswith('part_')]
for i,m in enumerate(parts):
    if i>=2: world.remove(m)
    else:
        before=copy.deepcopy(m)
        m.find('pose').text=poses[i]
        after=copy.deepcopy(m)
        before.remove(before.find('pose')); after.remove(after.find('pose'))
        assert E.tostring(before)==E.tostring(after)
Path(os.environ['SEP_RUN'],'physical_world.sdf').write_bytes(E.tostring(root))
PY
python3 scripts/stage_a_rgbd_world.py --world "$SEP_RUN/physical_world.sdf" \
  --output "$SEP_RUN/world.sdf" --render-engine ogre \
  --camera-pose 0.4 -0.217 0.614 0 1.5707963267948966 0
```

Use the existing capture commands in `docs/manuals/STAGE_A_RGBD_PERCEPTION.md`
with `RGBD_RUN="$SEP_RUN"`, this checkout, and its existing built capture tool
(`/tmp/workcell_stage_a2_build/stage_a_rgbd_capture` on this workstation).
Skip world derivation there because the disposable world above already exists;
wait **30 s** after starting Gazebo before bracketing poses. Use the unchanged
workpiece profile and explicit replay export from the adjacent Stage-A2 README.
Do not start a bridge, MoveIt or an executor for capture.

To reproduce the exact extraction precondition using the generated snapshot:

```bash
source /opt/ros/humble/setup.bash
source /tmp/pr3175_verify/install/setup.bash
export AMENT_PREFIX_PATH=/tmp/workcell_stage_a2_runtime_prefix:$AMENT_PREFIX_PATH
export PYTHONPATH="$PWD/scripts:$PYTHONPATH"
python3 - <<'PY'
import os,json,yaml
from pathlib import Path
from pile_extraction import extraction_intents, ExtractionFailure
s=json.loads(Path(os.environ['SEP_RUN'],'capture/world_snapshot.json').read_text())
objects=[dict(id='runtime::'+o['object_id'],shape='BOX',dimensions=o['dimensions_xyz'],
              pose=o['pose']['position']+o['pose']['orientation_xyzw'])
         for o in s['objects'] if 'pose' in o]
m=yaml.safe_load(Path('scenes/ur5_2f_test/config/moveit_collision_objects.yaml').read_text())
static=[dict(id=o['id'],shape='BOX',dimensions=o['collision_geometry']['dimensions_m'],
             pose=o['pose']['xyz']+o['pose']['quaternion_xyzw'])
        for o in m['objects'] if o['collision_geometry']['type']=='box']
for o in objects:
    try:
        print(o['id'],extraction_intents(o,[n for n in objects+static
              if n['id']!=o['id']],'prerequisite_only',.15))
    except ExtractionFailure as error:
        print(o['id'],'BLOCKED',error)
PY
```

Next product action: review support-conditioned uncertainty/contact handling
within the existing collision contract and certificate. Preserve these envelopes
and the 0.1 mm rule until a defensible proof is available. Rerun this prerequisite
before admitting the existing bounded full-cycle command. No unchanged-replay
planning campaign is justified by this result.

This experiment accounts for both known physical cubes. It does not certify
full scene completeness, the original pile, live perception, bridge commissioning
or physical execution. Frozen snapshots remain replay-only and existing
incomplete-scene execution restrictions remain in force.
