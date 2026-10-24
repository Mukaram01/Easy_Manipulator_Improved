"""End-effector hold profiles: how a tool proves and maintains a physical hold.

A capability record (``catalog/capabilities/end_effectors/*.yaml``) may carry an
``end_effector.hold`` block (schema ``end_effector_hold/v1``). It states, per
tool family, the physical hold-verification method, the holding parameters and
the actuation model the simulator derivation and the executor consume:

* ``finger_gripper`` -> ``opposing_contact``: every contact link of the cell is
  measured against the target; one actuated joint drives the pads through a
  rigid linkage, so its torque cap sets the saturated grasp force.
* ``three_finger_gripper`` -> ``multi_contact``: at least ``min_contact_links``
  contact links are measured against the target; same actuation model.
* ``suction`` / ``vacuum_array`` -> ``vacuum_seal``: every seal link is measured
  against the target and the vacuum signal is proven; hold force is pressure
  times effective area. The current simulator has no vacuum model, so this
  method fails closed there instead of pretending a seal.

The executor, the contact guard and the simulator derivation read only this
profile; none of them names a vendor, a joint or a finger count. The cell
definition binds the tool's capability id and its measured contact links.
"""
from __future__ import annotations

import hashlib
import json
import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path

SCHEMA = 'end_effector_hold/v1'
FAMILY_METHODS = {'finger_gripper': 'opposing_contact', 'three_finger_gripper': 'multi_contact',
                  'suction': 'vacuum_seal', 'vacuum_array': 'vacuum_seal'}
CONTACT_METHODS = ('opposing_contact', 'multi_contact')
ACTUATION_KINDS = ('single_actuated_joint',)
MAX_HOLD_RESERVE_RAD = 0.01


class HoldProfileError(ValueError):
    """The capability record cannot bind a hold method (fail closed)."""


class HoldVerificationUnsupported(RuntimeError):
    """The measurement source cannot prove this hold method."""


def _finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def _positive(value):
    return _finite(value) and value > 0


def hold_block_errors(end_effector):
    """Schema errors of ``end_effector.hold`` (empty when valid or absent)."""
    hold = end_effector.get('hold')
    if hold is None:
        return []
    errors = []
    if not isinstance(hold, dict):
        return ["Field 'end_effector.hold' must be a mapping/object."]
    if hold.get('schema') != SCHEMA:
        errors.append(f"Field 'end_effector.hold.schema' must be '{SCHEMA}'.")
    family = end_effector.get('family')
    expected = FAMILY_METHODS.get(family)
    method = hold.get('method')
    if expected is None:
        errors.append(f"Family '{family}' has no hold method.")
    elif method != expected:
        errors.append(f"Field 'end_effector.hold.method' must be '{expected}' for family '{family}'.")
    if not isinstance(hold.get('release'), str) or not hold['release'].strip():
        errors.append("Field 'end_effector.hold.release' is required and must be a non-empty string.")
    if method in CONTACT_METHODS:
        verification = hold.get('contact_verification')
        minimum = verification.get('min_contact_links') if isinstance(verification, dict) else None
        if method == 'opposing_contact' and minimum != 'all':
            errors.append("opposing_contact requires contact_verification.min_contact_links: all.")
        if method == 'multi_contact' and not (isinstance(minimum, int) and not isinstance(minimum, bool) and minimum >= 2):
            errors.append("multi_contact requires an integer contact_verification.min_contact_links >= 2.")
        reserve = hold.get('hold_reserve_rad')
        if not _positive(reserve) or reserve > MAX_HOLD_RESERVE_RAD:
            errors.append(f"Field 'end_effector.hold.hold_reserve_rad' must be in (0, {MAX_HOLD_RESERVE_RAD}].")
        errors.extend(_actuation_errors(hold.get('actuation')))
    elif method == 'vacuum_seal':
        errors.extend(_vacuum_errors(hold.get('vacuum'), end_effector))
    return errors


def _actuation_errors(actuation):
    if not isinstance(actuation, dict):
        return ["Field 'end_effector.hold.actuation' must be a mapping/object."]
    errors = []
    if actuation.get('kind') not in ACTUATION_KINDS:
        errors.append(f"Field 'end_effector.hold.actuation.kind' must be one of {list(ACTUATION_KINDS)}.")
    span = actuation.get('grasp_force_range_n')
    if not (isinstance(span, list) and len(span) == 2 and all(_positive(v) for v in span) and span[0] < span[1]):
        errors.append("Field 'actuation.grasp_force_range_n' must be [min, max] with 0 < min < max.")
        span = None
    commanded = actuation.get('commanded_grasp_force_n')
    if not _positive(commanded):
        errors.append("Field 'actuation.commanded_grasp_force_n' must be a positive number.")
    elif span is not None and not span[0] <= commanded <= span[1]:
        errors.append("Field 'actuation.commanded_grasp_force_n' must lie within grasp_force_range_n.")
    mapping = actuation.get('pad_force_per_actuator_torque')
    if not isinstance(mapping, dict):
        errors.append("Field 'actuation.pad_force_per_actuator_torque' must be a mapping/object.")
    else:
        if not _positive(mapping.get('n_per_nm')):
            errors.append("Field 'pad_force_per_actuator_torque.n_per_nm' must be a positive number.")
        if not _finite(mapping.get('measured_closure_rad')):
            errors.append("Field 'pad_force_per_actuator_torque.measured_closure_rad' must be numeric.")
        if not isinstance(mapping.get('evidence'), str) or not mapping['evidence'].strip():
            errors.append("Field 'pad_force_per_actuator_torque.evidence' must name the measurement.")
    return errors


def _vacuum_errors(vacuum, end_effector):
    if not isinstance(vacuum, dict):
        return ["Field 'end_effector.hold.vacuum' must be a mapping/object."]
    errors = []
    signals = end_effector.get('required_io_signals')
    signal = vacuum.get('vacuum_signal')
    if not isinstance(signal, str) or not signal or (isinstance(signals, list) and signal not in signals):
        errors.append("Field 'hold.vacuum.vacuum_signal' must name one of required_io_signals.")
    if vacuum.get('seal_contact_links') != 'all':
        errors.append("vacuum_seal requires hold.vacuum.seal_contact_links: all.")
    if not _positive(vacuum.get('supply_vacuum_kpa')) or not _positive(vacuum.get('effective_area_mm2')):
        errors.append("Fields 'hold.vacuum.supply_vacuum_kpa' and 'effective_area_mm2' must be positive numbers.")
    if vacuum.get('simulator_support') != 'none':
        errors.append("vacuum_seal must state simulator_support: none until a measured vacuum model exists.")
    return errors


@dataclass(frozen=True)
class HoldProfile:
    capability_id: str
    family: str
    method: str
    path: str
    sha256: str
    release: str
    hold_reserve_rad: float | None = None
    min_contact_links: int | None = None      # None: every contact link of the cell
    actuation: dict | None = None
    vacuum: dict | None = None

    def required_contact_links(self, touch_links, measured):
        """Contact links a verified hold must keep, given the measured set."""
        touch = list(dict.fromkeys(touch_links))
        if self.method == 'vacuum_seal':
            raise HoldVerificationUnsupported(
                'HOLD_VERIFICATION_UNSUPPORTED: no measured vacuum seal source for ' + self.capability_id)
        if len(touch) < 2:
            raise HoldProfileError('contact hold verification needs at least two contact links')
        if self.min_contact_links is None:
            return touch
        held = [link for link in touch if link in set(measured)]
        if len(held) < self.min_contact_links:
            return touch[:self.min_contact_links]       # not satisfiable by the measured set; callers fail
        return held

    def contact_verified(self, touch_links, measured):
        required = self.required_contact_links(touch_links, measured)
        return set(required) <= set(measured)

    def simulated_actuator_effort_nm(self):
        if not self.actuation:
            return None
        return self.actuation['commanded_grasp_force_n'] / self.actuation['pad_force_per_actuator_torque']['n_per_nm']

    def expected_pad_force_n(self):
        return self.actuation['commanded_grasp_force_n'] if self.actuation else None

    def evidence(self):
        record = dict(capability_id=self.capability_id, family=self.family, method=self.method,
                      path=self.path, sha256=self.sha256, release=self.release)
        if self.method in CONTACT_METHODS:
            record.update(hold_reserve_rad=self.hold_reserve_rad, min_contact_links=self.min_contact_links or 'all',
                          commanded_grasp_force_n=self.expected_pad_force_n(),
                          grasp_force_range_n=list(self.actuation['grasp_force_range_n']),
                          pad_force_per_actuator_torque=dict(self.actuation['pad_force_per_actuator_torque']),
                          simulated_actuator_effort_nm=self.simulated_actuator_effort_nm())
        else:
            record.update(vacuum=dict(self.vacuum))
        return record


def profile_from_record(capability_id, payload, path, raw):
    errors = hold_block_errors(payload)
    if errors:
        raise HoldProfileError(f'{capability_id}: ' + ' '.join(errors))
    hold = payload.get('hold')
    if hold is None:
        raise HoldProfileError(f'{capability_id}: capability record declares no hold profile')
    minimum = hold.get('contact_verification', {}).get('min_contact_links') if hold['method'] in CONTACT_METHODS else None
    return HoldProfile(capability_id=capability_id, family=payload['family'], method=hold['method'], path=str(path),
                       sha256=hashlib.sha256(raw).hexdigest(), release=hold['release'],
                       hold_reserve_rad=float(hold['hold_reserve_rad']) if hold['method'] in CONTACT_METHODS else None,
                       min_contact_links=None if minimum in (None, 'all') else int(minimum),
                       actuation=dict(hold['actuation']) if hold['method'] in CONTACT_METHODS else None,
                       vacuum=dict(hold['vacuum']) if hold['method'] == 'vacuum_seal' else None)


def load_hold_profile(capability_id, capabilities_dir=None):
    """The hold profile of one catalog capability id; fails closed when unbound."""
    if not isinstance(capability_id, str) or not capability_id.strip():
        raise HoldProfileError('HOLD_PROFILE_REQUIRED: the cell binds no end-effector capability')
    from capability_registry import load_capability_registry
    record = load_capability_registry(Path(capabilities_dir) if capabilities_dir else None).get(capability_id)
    if record is None:
        raise HoldProfileError(f'HOLD_PROFILE_REQUIRED: unknown end-effector capability {capability_id!r}')
    return profile_from_record(capability_id, record.payload, record.path, Path(record.path).read_bytes())


def load_cell_hold_profile(cell_definition_path, capabilities_dir=None):
    """The hold profile bound by a cell definition, or None when the cell binds no capability."""
    import yaml
    cell = yaml.safe_load(Path(cell_definition_path).read_text(encoding='utf-8')) or {}
    end_effector = cell.get('end_effector') if isinstance(cell.get('end_effector'), dict) else {}
    capability = end_effector.get('capability')
    if capability is None:
        return None
    return load_hold_profile(capability, capabilities_dir)


def bind_actuation(root, command_joints, profile):
    """Apply the profile's actuator torque cap to the derived simulator description.

    The tool's actuated joint is the one commanded joint that drives mimic
    followers; its authored ``<limit effort>`` is the actuator's own limit and is
    only ever lowered to the profile's commanded grasp force. Returns the binding
    record for the receipt.
    """
    if profile.method not in CONTACT_METHODS:
        raise HoldProfileError('HOLD_ACTUATION_UNBOUND: ' + profile.method + ' has no simulated actuator')
    if profile.actuation.get('kind') != 'single_actuated_joint':
        raise HoldProfileError('HOLD_ACTUATION_UNBOUND: unsupported actuation kind')
    drivers = {j.find('mimic').get('joint') for j in root.findall('joint') if j.find('mimic') is not None}
    actuated = sorted(drivers & set(command_joints))
    if len(actuated) != 1:
        raise HoldProfileError('HOLD_ACTUATION_UNBOUND: expected exactly one commanded joint driving the tool linkage, '
                               'found ' + str(actuated))
    joint = root.find(f"joint[@name='{actuated[0]}']")
    limit = joint.find('limit') if joint is not None else None
    source = float(limit.get('effort', 'nan')) if limit is not None else float('nan')
    if not _positive(source):
        raise HoldProfileError('HOLD_ACTUATION_UNBOUND: actuated joint lacks a finite authored effort limit')
    derived = profile.simulated_actuator_effort_nm()
    if not _positive(derived) or derived > source:
        raise HoldProfileError(f'HOLD_ACTUATION_EXCEEDS_AUTHORED_LIMIT: {derived} N m requested, {source} N m authored')
    limit.set('effort', repr(derived))
    return dict(capability_id=profile.capability_id, sha256=profile.sha256, method=profile.method,
                actuated_joint=actuated[0], source_effort_nm=source, derived_effort_nm=derived,
                commanded_grasp_force_n=profile.expected_pad_force_n(),
                pad_force_per_actuator_torque_n_per_nm=profile.actuation['pad_force_per_actuator_torque']['n_per_nm'],
                hold_reserve_rad=profile.hold_reserve_rad)


def bind_source_description(xml, profile):
    """Bind the profile's actuator cap into the source description before simulator derivation.

    The evidence-bound simulator backend derives its model from this description
    unchanged, so the cap flows into the derived URDF/SDF and the receipt's
    description hash. Returns the bound description and the receipt record.
    """
    root = ET.fromstring(xml)
    commanded = [j.get('name') for c in root.findall('ros2_control') for j in c.findall('joint')
                 if j.find('command_interface') is not None]
    binding = bind_actuation(root, commanded, profile)
    return ET.tostring(root, encoding='unicode'), binding


def augment_spec(output, binding):
    """Record the hold binding in the prepared simulator spec (read by the receipt)."""
    spec_path = Path(output) / 'spec.json'
    spec = json.loads(spec_path.read_text())
    spec['end_effector_hold'] = dict(binding)
    spec_path.write_text(json.dumps(spec, indent=2))
    return spec['end_effector_hold']


def verify_hold_binding(profile, receipt):
    """The running simulator must have been derived from this exact profile."""
    binding = receipt.get('end_effector_hold') if isinstance(receipt, dict) else None
    if not isinstance(binding, dict):
        raise RuntimeError('HOLD_PROFILE_BINDING_MISSING: simulator receipt records no end-effector hold binding')
    expected = profile.simulated_actuator_effort_nm()
    derived = binding.get('derived_effort_nm')
    if (binding.get('capability_id') != profile.capability_id or binding.get('sha256') != profile.sha256 or
            binding.get('method') != profile.method or binding.get('hold_reserve_rad') != profile.hold_reserve_rad or
            not _positive(derived) or expected is None or abs(derived - expected) > 1e-9 * max(1., abs(expected))):
        raise RuntimeError('HOLD_PROFILE_BINDING_MISMATCH: simulator was derived from a different hold profile')
    return dict(profile=profile.evidence(), receipt_binding=dict(binding))
