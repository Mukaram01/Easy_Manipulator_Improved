"""Capability-driven end-effector interface for the task pipeline.

A capability record (``catalog/capabilities/end_effectors/*.yaml``) may carry an
``end_effector.interface`` block (schema ``end_effector_interface/v1``). It states,
as data, how the pipeline drives one tool through the grasp lifecycle:

* engagement  -- ``joint_position``: the commanded joints, their open positions and
  the closure range searched for first opposing contact; ``io_signal``: the
  engage/release signals (suction, vacuum arrays).
* holding     -- method, reserve and actuator force come from the hold profile
  (``end_effector_hold``); this module does not duplicate them.
* feedback    -- what proves the hold: measured contact links or a vacuum signal.
* release     -- the open goal, or the release signal.
* recovery    -- the declared policy after a lost hold.

The preplanner and the executor ask this interface for joint names, positions,
the planning group and the aperture instead of naming a vendor or a joint. When
a cell binds no interface, :data:`LEGACY_TWO_FINGER` reproduces the values the
pipeline used before this module existed, exactly.

``revalidate_against_description`` checks an interface against the robot
description actually loaded by MoveIt (joints, limits, collision links, TCP link,
planning group), so replacing a tool is revalidated on every run.
"""
from __future__ import annotations

import hashlib
import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from pathlib import Path

SCHEMA = 'end_effector_interface/v1'
KINDS = ('joint_position', 'io_signal')
FEEDBACK = {'joint_position': 'measured_contacts', 'io_signal': 'vacuum_signal'}
RECOVERY_STRATEGIES = ('stop_and_require_operator',)
# deassert: release by turning the engage signal off; pulse: release by pulsing a separate signal.
RELEASE_MODES = ('deassert', 'pulse')


class ToolInterfaceError(ValueError):
    """The capability record cannot bind a usable tool interface (fail closed)."""


class ToolEngagementUnsupported(RuntimeError):
    """The pipeline has no planned engagement step for this interface kind yet."""


def _finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def _names(value):
    return isinstance(value, list) and bool(value) and all(isinstance(v, str) and v for v in value) and \
        len(set(value)) == len(value)


def interface_block_errors(end_effector):
    """Schema errors of ``end_effector.interface`` (empty when valid or absent)."""
    block = end_effector.get('interface')
    if block is None:
        return []
    if not isinstance(block, dict):
        return ["Field 'end_effector.interface' must be a mapping/object."]
    errors = []
    if block.get('schema') != SCHEMA:
        errors.append(f"Field 'end_effector.interface.schema' must be '{SCHEMA}'.")
    kind = block.get('kind')
    if kind not in KINDS:
        errors.append(f"Field 'end_effector.interface.kind' must be one of {list(KINDS)}.")
    if not _names(block.get('grasp_capabilities')):
        errors.append("Field 'end_effector.interface.grasp_capabilities' must be a non-empty list of names.")
    if not _names(block.get('touch_links')):
        errors.append("Field 'end_effector.interface.touch_links' must be a non-empty list of link names.")
    if not isinstance(block.get('tcp_link'), str) or not block['tcp_link']:
        errors.append("Field 'end_effector.interface.tcp_link' must name the grasp frame link.")
    recovery = block.get('recovery')
    if not isinstance(recovery, dict) or recovery.get('strategy') not in RECOVERY_STRATEGIES or \
            recovery.get('release_outside_destination') is not False:
        errors.append("Field 'interface.recovery' must declare strategy stop_and_require_operator and "
                      "release_outside_destination: false.")
    if kind == 'joint_position':
        errors.extend(_joint_errors(block))
    elif kind == 'io_signal':
        signals = block.get('signals')
        io = end_effector.get('required_io_signals')
        if not isinstance(signals, dict) or not all(isinstance(signals.get(k), str) and signals[k]
                                                    for k in ('engage', 'release', 'feedback')):
            errors.append("Field 'interface.signals' must name engage, release and feedback signals.")
        elif isinstance(io, list) and any(signals[k] not in io for k in ('engage', 'release')):
            errors.append("Field 'interface.signals' engage/release must be listed in required_io_signals.")
        if isinstance(signals, dict) and signals.get('release_mode') not in RELEASE_MODES:
            errors.append(f"Field 'interface.signals.release_mode' must be one of {list(RELEASE_MODES)}.")
    return errors


def _joint_errors(block):
    errors = []
    if not isinstance(block.get('planning_group'), str) or not block['planning_group']:
        errors.append("Field 'interface.planning_group' must name the tool's MoveIt group.")
    joints = block.get('command_joints')
    if not _names(joints):
        return errors + ["Field 'interface.command_joints' must be a non-empty list of joint names."]
    opened = block.get('open_positions_rad')
    if not isinstance(opened, dict) or set(opened) != set(joints) or not all(_finite(v) for v in opened.values()):
        errors.append("Field 'interface.open_positions_rad' must give one finite position per command joint.")
    closure = block.get('closure')
    if not isinstance(closure, dict):
        return errors + ["Field 'interface.closure' must be a mapping/object."]
    closed = closure.get('closed_position_rad')
    steps = closure.get('search_steps')
    if closed is not None and (not _finite(closed) or (isinstance(opened, dict) and joints[0] in opened and
                                                       _finite(opened[joints[0]]) and closed <= opened[joints[0]])):
        errors.append("Field 'interface.closure.closed_position_rad' must be finite and beyond the open position, "
                      "or null when the closure range is not yet commissioned.")
    if closed is None and not (isinstance(closure.get('commissioning_note'), str) and closure['commissioning_note']):
        errors.append("An uncommissioned closure (null closed_position_rad) must carry a commissioning_note.")
    if not (isinstance(steps, int) and not isinstance(steps, bool) and 2 <= steps <= 1000):
        errors.append("Field 'interface.closure.search_steps' must be an integer in [2, 1000].")
    aperture = block.get('max_aperture_m')
    if not (_finite(aperture) and aperture > 0):
        errors.append("Field 'interface.max_aperture_m' must be a positive number of meters.")
    return errors


@dataclass(frozen=True)
class ToolInterface:
    capability_id: str
    family: str
    kind: str
    grasp_capabilities: tuple
    touch_links: tuple
    tcp_link: str
    recovery: dict
    planning_group: str | None = None
    command_joints: tuple = ()
    open_positions_rad: dict = field(default_factory=dict)
    closed_position_rad: float | None = None
    search_steps: int | None = None
    max_aperture_m: float | None = None
    signals: dict = field(default_factory=dict)
    path: str | None = None
    sha256: str | None = None

    # --- engagement ---------------------------------------------------------
    @property
    def closure_joint(self):
        """The joint the closure search is expressed in (the first command joint)."""
        self._require_motion_engagement()
        return self.command_joints[0]

    def _require_motion_engagement(self):
        if self.kind != 'joint_position':
            raise ToolEngagementUnsupported(
                f'TOOL_ENGAGEMENT_UNSUPPORTED: {self.capability_id} engages by {self.kind}; the planner has no '
                'IO engagement step yet')
        if self.closed_position_rad is None:
            raise ToolEngagementUnsupported(
                f'TOOL_CLOSURE_UNCOMMISSIONED: {self.capability_id} has no commissioned closure range')

    def closure_positions(self):
        """Ordered closure search positions, first step to fully closed (open position excluded)."""
        self._require_motion_engagement()
        start = self.open_positions_rad[self.closure_joint]
        return [start + (self.closed_position_rad - start) * i / self.search_steps
                for i in range(1, self.search_steps + 1)]

    def closure_step(self):
        self._require_motion_engagement()
        return (self.closed_position_rad - self.open_positions_rad[self.closure_joint]) / self.search_steps

    def engage_goal(self, position):
        """Joint goal closing every command joint to ``position`` (expressed in the closure joint)."""
        self._require_motion_engagement()
        if not _finite(position):
            raise ToolInterfaceError('engagement position must be finite')
        offset = position - self.open_positions_rad[self.closure_joint]
        return {joint: self.open_positions_rad[joint] + offset for joint in self.command_joints}

    # --- release ------------------------------------------------------------
    def release_goal(self):
        """Joint goal that opens the tool; an IO tool releases by signal and has no joint goal."""
        return dict(self.open_positions_rad) if self.kind == 'joint_position' else {}

    # --- feedback / evidence --------------------------------------------------
    @property
    def feedback(self):
        if self.kind == 'io_signal':
            return dict(method=FEEDBACK[self.kind], signal=self.signals.get('feedback'))
        return dict(method=FEEDBACK[self.kind], contact_links=list(self.touch_links))

    def as_contract(self):
        """Plain, copy- and JSON-safe form carried in the grasp contract."""
        return dict(schema=SCHEMA, capability_id=self.capability_id, family=self.family, kind=self.kind,
                    grasp_capabilities=list(self.grasp_capabilities), touch_links=list(self.touch_links),
                    tcp_link=self.tcp_link, recovery=dict(self.recovery), planning_group=self.planning_group,
                    command_joints=list(self.command_joints), open_positions_rad=dict(self.open_positions_rad),
                    closed_position_rad=self.closed_position_rad, search_steps=self.search_steps,
                    max_aperture_m=self.max_aperture_m, signals=dict(self.signals), path=self.path, sha256=self.sha256)

    def evidence(self):
        record = self.as_contract()
        record['feedback'] = self.feedback
        return record


# The values the pipeline hardcoded for the Robotiq 2F-85 before this module. Used only
# when a cell binds no interface, so existing cells, tests and evidence keep their meaning.
LEGACY_TWO_FINGER = ToolInterface(
    capability_id='legacy_two_finger', family='finger_gripper', kind='joint_position',
    grasp_capabilities=('two_finger_parallel',),
    touch_links=('gripper_finger1_finger_tip_link', 'gripper_finger2_finger_tip_link'),
    tcp_link='ee_palm', recovery=dict(strategy='stop_and_require_operator', release_outside_destination=False),
    planning_group='gripper', command_joints=('gripper_finger1_joint',),
    open_positions_rad={'gripper_finger1_joint': 0.0}, closed_position_rad=0.804, search_steps=80,
    max_aperture_m=0.085)


def interface_from_record(capability_id, payload, path=None, raw=b''):
    errors = interface_block_errors(payload)
    if errors:
        raise ToolInterfaceError(f'{capability_id}: ' + ' '.join(errors))
    block = payload.get('interface')
    if block is None:
        raise ToolInterfaceError(f'TOOL_INTERFACE_REQUIRED: {capability_id} declares no end_effector.interface')
    common = dict(capability_id=capability_id, family=payload.get('family'), kind=block['kind'],
                  grasp_capabilities=tuple(block['grasp_capabilities']), touch_links=tuple(block['touch_links']),
                  tcp_link=block['tcp_link'], recovery=dict(block['recovery']),
                  path=str(path) if path else None, sha256=hashlib.sha256(raw).hexdigest() if raw else None)
    if block['kind'] == 'io_signal':
        return ToolInterface(**common, signals=dict(block['signals']))
    closure = block['closure']
    return ToolInterface(**common, planning_group=block['planning_group'], command_joints=tuple(block['command_joints']),
                         open_positions_rad={k: float(v) for k, v in block['open_positions_rad'].items()},
                         closed_position_rad=None if closure.get('closed_position_rad') is None
                         else float(closure['closed_position_rad']),
                         search_steps=int(closure['search_steps']), max_aperture_m=float(block['max_aperture_m']))


def load_tool_interface(capability_id, capabilities_dir=None):
    """The interface of one catalog capability id; fails closed when unbound."""
    if not isinstance(capability_id, str) or not capability_id.strip():
        raise ToolInterfaceError('TOOL_INTERFACE_REQUIRED: the cell binds no end-effector capability')
    try:
        from capability_registry import load_capability_registry
    except ImportError:  # imported as scripts.tool_interface
        from scripts.capability_registry import load_capability_registry
    record = load_capability_registry(Path(capabilities_dir) if capabilities_dir else None).get(capability_id)
    if record is None:
        raise ToolInterfaceError(f'TOOL_INTERFACE_REQUIRED: unknown end-effector capability {capability_id!r}')
    return interface_from_record(capability_id, record.payload, record.path, Path(record.path).read_bytes())


def interface_from_contract(contract):
    """The interface carried by a grasp contract, or the legacy two-finger values."""
    data = (contract or {}).get('tool_interface')
    if data is None:
        return LEGACY_TWO_FINGER
    if data.get('schema') != SCHEMA:
        raise ToolInterfaceError('grasp contract carries an unknown tool interface schema')
    fields = {k: v for k, v in data.items() if k != 'schema'}
    for key in ('grasp_capabilities', 'touch_links', 'command_joints'):
        fields[key] = tuple(fields.get(key) or ())
    return ToolInterface(**fields)


def revalidate_against_description(interface, urdf_xml, *, touch_links=None, tcp_link=None, srdf_xml=None):
    """Check a tool interface against the robot description MoveIt actually loaded.

    ``touch_links`` and ``tcp_link`` default to the interface's own; a cell that
    authors its own values is checked with those. Returns a report; ``status`` is
    VALID only when every check passes. Nothing here changes any description.
    """
    checks = []

    def check(code, ok, detail):
        checks.append(dict(code=code, status='PASS' if ok else 'FAIL', detail=detail))

    try:
        robot = ET.fromstring(urdf_xml)
    except ET.ParseError as exc:
        return dict(status='INVALID', capability_id=interface.capability_id,
                    checks=[dict(code='DESCRIPTION_PARSE', status='FAIL', detail=str(exc))])
    links = {link.get('name'): link for link in robot.findall('link')}
    joints = {joint.get('name'): joint for joint in robot.findall('joint')}
    for name in touch_links or interface.touch_links:
        link = links.get(name)
        check('TOUCH_LINK_PRESENT', link is not None, name)
        if link is not None:
            check('TOUCH_LINK_HAS_COLLISION', link.find('collision') is not None, name)
    tcp = tcp_link or interface.tcp_link
    check('TCP_LINK_PRESENT', tcp in links, tcp)
    if interface.kind == 'joint_position':
        for name in interface.command_joints:
            joint = joints.get(name)
            check('COMMAND_JOINT_PRESENT', joint is not None, name)
            if joint is None:
                continue
            check('COMMAND_JOINT_ACTUATED', joint.find('mimic') is None and joint.get('type') in ('revolute', 'prismatic'),
                  f"{name} ({joint.get('type')})")
            limit = joint.find('limit')
            lower = float(limit.get('lower', 'nan')) if limit is not None else math.nan
            upper = float(limit.get('upper', 'nan')) if limit is not None else math.nan
            opened = interface.open_positions_rad[name]
            check('OPEN_POSITION_WITHIN_LIMITS', lower <= opened <= upper, f'{name}: {opened} in [{lower}, {upper}]')
            if interface.closed_position_rad is not None:
                closed = interface.engage_goal(interface.closed_position_rad)[name]
                check('CLOSED_POSITION_WITHIN_LIMITS', lower <= closed <= upper, f'{name}: {closed} in [{lower}, {upper}]')
        if srdf_xml is not None:
            try:
                srdf = ET.fromstring(srdf_xml)
            except ET.ParseError as exc:
                check('SRDF_PARSE', False, str(exc))
            else:
                group = next((g for g in srdf.findall('group') if g.get('name') == interface.planning_group), None)
                check('PLANNING_GROUP_PRESENT', group is not None, str(interface.planning_group))
                if group is not None:
                    listed = {j.get('name') for j in group.findall('joint')}
                    check('PLANNING_GROUP_HAS_COMMAND_JOINTS', set(interface.command_joints) <= listed,
                          f'{sorted(interface.command_joints)} in group {interface.planning_group}')
    status = 'VALID' if all(c['status'] == 'PASS' for c in checks) else 'INVALID'
    return dict(status=status, capability_id=interface.capability_id, kind=interface.kind, checks=checks)
