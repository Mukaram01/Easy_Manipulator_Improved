#!/usr/bin/env python3
"""Configuration-driven end-effector selection and replacement for a Workcell Studio cell.

Selecting or replacing a tool is a change to the authored ``environment.yaml``
(``end_effector.capability`` and ``hold_variant``); everything else the pipeline
needs is derived from the capability catalog. Before a change is applied, the
selection is checked at three levels, each reported separately:

1. Catalog compatibility -- the robot family, the task's required grasp
   capability and grasp strategy, a valid interface and hold profile.
2. Execution readiness -- whether the planner can express this tool's engagement
   today (a joint closure that is commissioned; IO engagement is not yet planned).
3. Description revalidation -- the interface against a robot description and
   SRDF (joints, limits, TCP link, collision on every touch link, planning group).
   The executor repeats this check against the live MoveIt description on every run.

A tool whose model is not the one the scene's description is generated from is
refused: changing the model needs scene generation, not a binding change.

usage: end_effector_selection.py --scene DIR [--capability ID] [--hold-variant NAME]
                                 [--urdf FILE --srdf FILE] [--apply] [--json]
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import yaml  # noqa: E402

from capability_registry import load_capability_registry  # noqa: E402
from end_effector_hold import HoldProfileError, load_hold_profile  # noqa: E402
from task_intent_v2 import strategy_supports_tool  # noqa: E402
from tool_interface import (  # noqa: E402
    ToolEngagementUnsupported, ToolInterfaceError, load_tool_interface, revalidate_against_description,
)


def _finding(code, detail):
    return dict(code=code, detail=detail)


def evaluate_selection(cell, *, environment=None, capability_id=None, hold_variant=None, capabilities_dir=None,
                       urdf_xml=None, srdf_xml=None):
    """Evaluate installing ``capability_id`` (default: the cell's own) in ``cell``.

    ``cell`` is a cell definition mapping; ``environment`` the authored scene, which
    names the tool model the description is generated from. Nothing is written.
    """
    end_effector = cell.get('end_effector') or {}
    capability_id = capability_id or end_effector.get('capability')
    hold_variant = hold_variant if hold_variant is not None else (
        end_effector.get('hold_variant') if capability_id == end_effector.get('capability') else None)
    registry = load_capability_registry(Path(capabilities_dir) if capabilities_dir else None)
    report = dict(capability_id=capability_id, hold_variant=hold_variant, blockers=[], warnings=[],
                  compatibility='INCOMPATIBLE', execution_readiness='BLOCKED', description_revalidation='NOT_RUN')
    blockers, warnings = report['blockers'], report['warnings']
    record = registry.get(capability_id) if capability_id else None
    if record is None:
        blockers.append(_finding('TOOL_CAPABILITY_UNKNOWN', f'no end-effector capability {capability_id!r} in the catalog'))
        return report
    tool = record.payload

    robot = cell.get('robot') or {}
    robot_id = robot.get('capability') or robot.get('model')
    robot_record = registry.get(robot_id)
    report['robot'] = dict(id=robot_id, family=robot_record.family if robot_record else None)
    families = tool.get('compatible_robot_families') or []
    if robot_record is None:
        warnings.append(_finding('ROBOT_CAPABILITY_UNKNOWN', f'robot {robot_id!r} has no catalog record; family unchecked'))
    elif families and robot_record.family not in families:
        blockers.append(_finding('ROBOT_FAMILY_INCOMPATIBLE', f'{capability_id} supports {families}, robot is {robot_record.family}'))

    try:
        interface = load_tool_interface(capability_id, capabilities_dir)
        report['interface'] = interface.evidence()
    except ToolInterfaceError as exc:
        blockers.append(_finding('TOOL_INTERFACE_INVALID', str(exc)))
        return report
    try:
        report['hold_profile'] = load_hold_profile(capability_id, capabilities_dir, hold_variant).evidence()
    except HoldProfileError as exc:
        blockers.append(_finding('HOLD_PROFILE_INVALID', str(exc)))

    intent = cell.get('builder_task_intent') or {}
    grasp = (intent.get('pick') or {}).get('grasp') or {}
    required = grasp.get('required_capability')
    if required and required not in interface.grasp_capabilities:
        blockers.append(_finding('REQUIRED_CAPABILITY_UNAVAILABLE',
                                 f'task requires {required}; {capability_id} provides {list(interface.grasp_capabilities)}'))
    strategy = grasp.get('strategy_ref')
    if strategy and grasp.get('policy') in ('EXACT', 'PREFERRED') and \
            not strategy_supports_tool(strategy, {'capability': capability_id}):
        (blockers if grasp.get('policy') == 'EXACT' else warnings).append(
            _finding('STRATEGY_TOOL_INCOMPATIBLE', f'strategy {strategy} does not list {capability_id} as compatible'))

    # The scene's description is generated from one tool model; a different model is a generation
    # change. Link names cannot prove the model (2F-85 and AirPick both name a gripper_base_link).
    authored = ((environment or {}).get('end_effector') or {}).get('description') or end_effector.get('description') or {}
    current = authored.get('package')
    target = tool.get('asset_package')
    report['tool_model'] = dict(scene_description_package=current, tool_asset_package=target)
    if not current or not target:
        blockers.append(_finding('TOOL_MODEL_UNVERIFIED',
                                 'the scene description package or the tool asset package is not declared'))
    elif current != target:
        blockers.append(_finding('TOOL_MODEL_CHANGE_REQUIRES_SCENE_GENERATION',
                                 f'the scene description is generated from {current}; {capability_id} needs {target}'))
    authored_touch = end_effector.get('allowed_touch_links')
    if authored_touch and capability_id == end_effector.get('capability') and \
            sorted(authored_touch) != sorted(interface.touch_links):
        warnings.append(_finding('TOUCH_LINKS_DIFFER_FROM_TOOL',
                                 f'cell authors {sorted(authored_touch)}; the tool declares {sorted(interface.touch_links)}'))

    report['compatibility'] = 'COMPATIBLE' if not blockers else 'INCOMPATIBLE'
    try:
        interface.closure_positions()
        report['execution_readiness'] = 'READY' if not blockers else 'BLOCKED'
    except ToolEngagementUnsupported as exc:
        blockers.append(_finding(str(exc).split(':', 1)[0], str(exc)))
        report['execution_readiness'] = 'BLOCKED'

    if urdf_xml is not None:
        revalidation = revalidate_against_description(
            interface, urdf_xml, touch_links=list(interface.touch_links), tcp_link=interface.tcp_link, srdf_xml=srdf_xml)
        report['revalidation'] = revalidation
        report['description_revalidation'] = revalidation['status']
        if revalidation['status'] != 'VALID':
            failed = sorted({c['code'] for c in revalidation['checks'] if c['status'] != 'PASS'})
            blockers.append(_finding('DESCRIPTION_REVALIDATION_FAILED', ', '.join(failed)))
            report['execution_readiness'] = 'BLOCKED'
    return report


def apply_selection(scene_dir, report):
    """Write an accepted selection into the scene's authored ``environment.yaml``.

    Only the end-effector binding changes: capability, hold variant, and the TCP
    link and touch links derived from the tool's interface. The generated cell
    definition is refreshed by the existing exporter afterwards; any saved task
    resolution becomes stale because the cell context hash changes.
    """
    if report.get('compatibility') != 'COMPATIBLE' or report.get('description_revalidation') == 'INVALID':
        raise RuntimeError('SELECTION_REJECTED: ' + '; '.join(b['code'] for b in report.get('blockers', [])))
    path = Path(scene_dir) / 'environment.yaml'
    environment = yaml.safe_load(path.read_text(encoding='utf-8'))
    end_effector = environment.setdefault('end_effector', {})
    interface = report['interface']
    end_effector['capability'] = report['capability_id']
    if report.get('hold_variant') is not None:
        end_effector['hold_variant'] = report['hold_variant']
    else:
        end_effector.pop('hold_variant', None)
    end_effector['grasp_frame'] = interface['tcp_link']
    end_effector['allowed_touch_links'] = list(interface['touch_links'])
    path.write_text(yaml.safe_dump(environment, sort_keys=False), encoding='utf-8')
    return dict(environment=str(path), end_effector={k: end_effector[k] for k in
                ('capability', 'grasp_frame', 'allowed_touch_links') if k in end_effector} |
                ({'hold_variant': end_effector['hold_variant']} if 'hold_variant' in end_effector else {}))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    parser.add_argument('--scene', type=Path, required=True, help='scene package directory with cell_definition.yaml')
    parser.add_argument('--capability', help='end-effector capability id to evaluate (default: the cell\'s own)')
    parser.add_argument('--hold-variant', help='named force setting of the tool\'s hold profile')
    parser.add_argument('--urdf', type=Path, help='robot description to revalidate against')
    parser.add_argument('--srdf', type=Path, help='semantic description (planning groups)')
    parser.add_argument('--apply', action='store_true', help='write an accepted selection into environment.yaml')
    parser.add_argument('--json', action='store_true')
    args = parser.parse_args(argv)
    if args.srdf and not args.urdf:
        parser.error('--srdf needs --urdf')
    cell = yaml.safe_load((args.scene / 'cell_definition.yaml').read_text(encoding='utf-8')) or {}
    environment_path = args.scene / 'environment.yaml'
    environment = yaml.safe_load(environment_path.read_text(encoding='utf-8')) if environment_path.is_file() else None
    report = evaluate_selection(cell, environment=environment, capability_id=args.capability, hold_variant=args.hold_variant,
                                urdf_xml=args.urdf.read_text() if args.urdf else None,
                                srdf_xml=args.srdf.read_text() if args.srdf else None)
    if args.apply:
        try:
            report['applied'] = apply_selection(args.scene, report)
        except RuntimeError as exc:
            report['applied'] = dict(error=str(exc))
    if args.json:
        print(json.dumps(report, indent=2))
    else:
        print(f"{report['capability_id']}: compatibility {report['compatibility']}, execution {report['execution_readiness']}, "
              f"description {report['description_revalidation']}")
        for kind in ('blockers', 'warnings'):
            for item in report[kind]:
                print(f"  {kind[:-1]}: {item['code']} - {item['detail']}")
    ok = report['compatibility'] == 'COMPATIBLE' and report['description_revalidation'] != 'INVALID' and \
        not (args.apply and 'error' in report.get('applied', {}))
    return 0 if ok else 1


if __name__ == '__main__':
    raise SystemExit(main())
