"""Predictable carried-payload transport, authored per cell (``transport_policy/v1``).

A cell may author ``motion_policy.transport`` in its environment; the cell
definition carries it to the grasp contract. Without one, transport is the
unconstrained free-space plan the pipeline has always used.

``orientation: keep_level`` turns PREPLAN_TRANSFER into a bounded sequence of
attempts, each an orientation path constraint on the tool link that holds the
payload level (tight roll/pitch tolerance) while leaving yaw free. Tolerances
widen only along the authored ladder (at most three steps); an unconstrained
attempt is added only when ``allow_unconstrained_fallback`` is true. MoveIt plans
each attempt against the constraint, and the existing controller-interval
certificate re-checks the constraint and collisions on the interpolated
controller spline, so a constrained transfer is collision- and constraint-certified.

``destination_approach_height_m`` places the transfer's end waypoint that high
above the placement pose (never lower than the authored place approach); the
straight, certified PLACE descent covers the difference. No joint path is named,
so the same policy applies to any robot.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

SCHEMA = 'transport_policy/v1'
ORIENTATION_MODES = ('free', 'keep_level')
MAX_ATTEMPTS = 3
MAX_TILT_TOLERANCE_RAD = 0.5


class TransportPolicyError(ValueError):
    """The authored transport policy is invalid (fail closed)."""


def _finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


@dataclass(frozen=True)
class TransportAttempt:
    label: str
    tilt_tolerance_rad: float | None        # None: unconstrained free-space plan

    def as_dict(self):
        return dict(label=self.label, tilt_tolerance_rad=self.tilt_tolerance_rad)


@dataclass(frozen=True)
class TransportPolicy:
    orientation: str
    tilt_tolerances_rad: tuple = ()
    allow_unconstrained_fallback: bool = False
    destination_approach_height_m: float | None = None

    def attempts(self):
        """The bounded, ordered planning attempts for PREPLAN_TRANSFER."""
        if self.orientation == 'free':
            return [TransportAttempt('unconstrained', None)]
        ladder = [TransportAttempt(f'keep_level_{i}', tolerance)
                  for i, tolerance in enumerate(self.tilt_tolerances_rad)]
        if self.allow_unconstrained_fallback:
            ladder.append(TransportAttempt('unconstrained_fallback', None))
        return ladder

    def place_approach_m(self, authored):
        """Height of the transfer end waypoint above the placement pose."""
        if self.destination_approach_height_m is None:
            return authored
        return max(authored, self.destination_approach_height_m)

    def as_contract(self):
        return dict(schema=SCHEMA, orientation=self.orientation, tilt_tolerances_rad=list(self.tilt_tolerances_rad),
                    allow_unconstrained_fallback=self.allow_unconstrained_fallback,
                    destination_approach_height_m=self.destination_approach_height_m)


def parse_transport_policy(block):
    """Validate an authored ``motion_policy.transport`` block; None keeps legacy transport."""
    if block is None:
        return None
    if not isinstance(block, dict):
        raise TransportPolicyError('motion_policy.transport must be a mapping')
    if block.get('schema') != SCHEMA:
        raise TransportPolicyError(f"motion_policy.transport.schema must be '{SCHEMA}'")
    unknown = set(block) - {'schema', 'orientation', 'tilt_tolerances_rad', 'allow_unconstrained_fallback',
                            'destination_approach_height_m'}
    if unknown:
        raise TransportPolicyError(f'unknown transport policy fields: {sorted(unknown)}')
    orientation = block.get('orientation')
    if orientation not in ORIENTATION_MODES:
        raise TransportPolicyError(f'transport orientation must be one of {list(ORIENTATION_MODES)}')
    ladder = block.get('tilt_tolerances_rad', [])
    fallback = block.get('allow_unconstrained_fallback', False)
    if not isinstance(fallback, bool):
        raise TransportPolicyError('allow_unconstrained_fallback must be true or false')
    if orientation == 'keep_level':
        if (not isinstance(ladder, list) or not 1 <= len(ladder) <= MAX_ATTEMPTS - int(fallback) or
                not all(_finite(v) and 0 < v <= MAX_TILT_TOLERANCE_RAD for v in ladder) or
                any(b <= a for a, b in zip(ladder, ladder[1:]))):
            raise TransportPolicyError(
                f'keep_level needs 1..{MAX_ATTEMPTS} strictly increasing tilt tolerances in (0, {MAX_TILT_TOLERANCE_RAD}] rad, '
                f'counting an unconstrained fallback as one attempt')
    elif ladder or fallback:
        raise TransportPolicyError('free transport takes no tolerance ladder or fallback')
    height = block.get('destination_approach_height_m')
    if height is not None and not (_finite(height) and 0 < height <= 1.0):
        raise TransportPolicyError('destination_approach_height_m must be in (0, 1] m')
    return TransportPolicy(orientation, tuple(float(v) for v in ladder), fallback,
                           None if height is None else float(height))


def policy_from_contract(contract):
    data = (contract or {}).get('transport_policy')
    if data is None:
        return None
    return parse_transport_policy(data)


def keep_level_constraint(tool_link, frame_id, goal_orientation_xyzw, tilt_tolerance_rad):
    """An OrientationConstraint holding the tool's tilt to the goal's, yaw free.

    Tolerances are about the tool link's own axes: x/y bound the tilt; z (the
    approach axis of a top-down tool) is free.
    """
    from moveit_msgs.msg import OrientationConstraint
    constraint = OrientationConstraint()
    constraint.header.frame_id = frame_id
    constraint.link_name = tool_link
    (constraint.orientation.x, constraint.orientation.y,
     constraint.orientation.z, constraint.orientation.w) = goal_orientation_xyzw
    constraint.absolute_x_axis_tolerance = tilt_tolerance_rad
    constraint.absolute_y_axis_tolerance = tilt_tolerance_rad
    constraint.absolute_z_axis_tolerance = math.pi
    constraint.weight = 1.0
    return constraint
