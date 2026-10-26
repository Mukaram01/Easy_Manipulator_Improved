#!/usr/bin/env python3
"""Can the Workcell Builder run this saved cell in the qualified physics simulation? (READY or BLOCKED)

Read-only. It applies, in order, the checks the qualified runner applies before anything starts:
the saved task selects the physics simulator; the generated files are current with the saved task and
environment; the cell is inside the qualified physics envelope (its world derives); and the workspace has
a registered, still-valid qualified runtime. The first failure is the blocker, with the action that fixes
it. Nothing is launched, generated or written.
"""
import argparse
import hashlib
import json
import sys
from pathlib import Path

SCRIPTS = Path(__file__).resolve().parent
sys.path.insert(0, str(SCRIPTS))

BACKEND = 'Qualified physics simulation (Gazebo Fortress / DART, simulated robot only)'


def _sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def check_task(scene):
    import yaml
    intent = yaml.safe_load((scene / 'config/workcell_builder_task_intent.yaml').read_text())
    if not isinstance(intent, dict) or intent.get('schema') != 'workcell_builder_task_intent/v2':
        return 'Save the task in the Task Editor (no saved v2 task).'
    safety = intent.get('safety') or {}
    if (safety.get('execution_backend') != 'simulator' or safety.get('require_fake_hardware') is not False or
            safety.get('real_hardware_enabled') is not False):
        return 'Select Task Editor > Run > Qualified physics simulation, then Save and Generate.'
    return None


def check_generated(scene):
    import yaml
    from task_intent_v2 import normalized_intent_hash
    definition = yaml.safe_load((scene / 'cell_definition.yaml').read_text()) if (scene / 'cell_definition.yaml').is_file() else None
    if not definition:
        return 'Generate the cell (no generated cell definition).'
    intent = yaml.safe_load((scene / 'config/workcell_builder_task_intent.yaml').read_text())
    if definition.get('normalized_intent_sha256') != normalized_intent_hash(intent):
        return 'The saved task changed after the last Generate: Generate the cell again.'
    manifest = scene / 'config/moveit_collision_objects.yaml'
    if not manifest.is_file():
        return 'Generate the cell (no generated planning scene).'
    if (yaml.safe_load(manifest.read_text()).get('source') or {}).get('physical_state_sha256') != _sha256(scene / 'environment.yaml'):
        return 'The saved environment changed after the last Generate: Generate the cell again.'
    return None


def check_world(scene):
    from simulator_world import SimulatorWorldError, derive_world
    try:
        _, report = derive_world(scene)
    except SimulatorWorldError as exc:
        return f'Not runnable in the qualified physics simulation: {exc}', None
    return None, report


def check_perception(scene, world):
    """Stage-B: a replayed-perception task needs its replayed EPD session to be valid for this cell and world."""
    import yaml
    from perception_replay import PerceptionBlocked, validate_replay
    from task_intent_v2 import freshness_source_type
    intent = yaml.safe_load((scene / 'config/workcell_builder_task_intent.yaml').read_text())
    if freshness_source_type(intent) != 'replayed_perception_in_simulation':
        return None
    replay = Path(intent['perception'].get('replay_path') or '')
    replay = replay if replay.is_absolute() else scene / replay
    if not replay.is_file():
        return f'PERCEPTION_REPLAY_MISSING: {replay}; record and detect the cell camera session (perception_replay.py).'
    try:
        validate_replay(yaml.safe_load(replay.read_text()), scene,
                        physics_world_sha256=world['world_sha256'] if world else None)
    except PerceptionBlocked as exc:
        return str(exc)
    return None


def check_runtime(workspace):
    from qualified_physics_runtime import RuntimeUnavailable, load
    try:
        load(workspace)
    except RuntimeUnavailable as exc:
        return str(exc)
    return None


def preflight(scene, workspace):
    scene = Path(scene).resolve()
    checks, world = [], None
    for name, check in (('saved_task', lambda: check_task(scene)), ('generated_current', lambda: check_generated(scene)),
                        ('physics_envelope', lambda: check_world(scene)), ('perception_replay', lambda: check_perception(scene, world)),
                        ('qualified_runtime', lambda: check_runtime(workspace))):
        try:
            result = check()
        except (OSError, ValueError, KeyError, TypeError) as exc:
            result = f'{name} could not be checked: {exc}'
        if isinstance(result, tuple):
            result, world = result
        checks.append(dict(name=name, status='PASS' if result is None else 'BLOCKED', detail=result))
        if result is not None:
            break
    blocked = next((c for c in checks if c['status'] == 'BLOCKED'), None)
    summary = None
    if world is not None:
        destination = world['destination']
        summary = dict(robot=world['robot_model'], end_effector=world['end_effector_capability'],
                       objects=[f"{o['object_id']} ({o.get('class_id') or 'unclassified'}, {o['material']})" for o in world['objects']],
                       destination=f"{destination['asset_ref']} floor z {destination['floor_top_z']:.3f} m",
                       world_sha256=world['world_sha256'])
    return dict(status='BLOCKED' if blocked else 'READY', backend=BACKEND, scene=str(scene),
                blocker=blocked['detail'] if blocked else None, checks=checks, cell=summary)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('scene', type=Path)
    parser.add_argument('--workspace', type=Path, default=SCRIPTS.parents[2])
    args = parser.parse_args(argv)
    report = preflight(args.scene, args.workspace)
    print(json.dumps(report))
    return 0 if report['status'] == 'READY' else 2


if __name__ == '__main__':
    sys.exit(main())
