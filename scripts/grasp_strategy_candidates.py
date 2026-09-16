#!/usr/bin/env python3
"""Deterministic 2F grasp geometry shared by readiness and runtime planning.

Top candidates preserve the legacy eight-pose order. Side candidates implement
the catalog's world-X-positive approach and horizontal grasp orientation. Poses
refer to the grasp frame; the preplanner applies the installed TCP exactly once.
"""
import math
from dataclasses import dataclass
from pathlib import Path

from perceived_object_grasp_plan import (
    build_grasp_target,
    generate_box_grasp_candidates,
    oriented_box_extents,
    quaternion_from_rpy,
)


@dataclass(frozen=True)
class GraspCandidate:
    strategy_ref: str
    candidate_id: str
    object_id: str
    grasp_pose: tuple[float, float, float, float, float, float, float]
    approach_pose: tuple[float, float, float, float, float, float, float]
    effective: dict


def _default_catalog_dir() -> Path:
    script = Path(__file__).resolve()
    candidates = [
        script.parents[1] / 'catalog' / 'grasp_strategies',
        script.parents[2] / 'share' / 'workcell_builder' / 'catalog' / 'grasp_strategies',
    ]
    return next((path for path in candidates if path.is_dir()), candidates[0])


def generate_strategy_candidates(
    strategy_ref: str, observation: dict, grasp_intent: dict,
    catalog_dir: Path | None = None,
) -> list[GraspCandidate]:
    if strategy_ref not in {'top_2f', 'side_grip_basic'}:
        raise ValueError(f'unsupported grasp strategy: {strategy_ref}')
    allowed = ({'approach_distance_m'} if strategy_ref == 'top_2f' else
               {'approach_distance_m', 'approach_axis', 'orientation_mode'})
    if set(grasp_intent) - allowed:
        raise ValueError(f'unsupported grasp constraints for {strategy_ref}')
    catalog = None
    if catalog_dir is not None or strategy_ref == 'side_grip_basic':
        import yaml
        root = Path(catalog_dir) if catalog_dir is not None else _default_catalog_dir()
        entry = yaml.safe_load((root / f'{strategy_ref}.yaml').read_text()) or {}
        catalog = entry.get('grasp_strategy') or {}
        if catalog.get('id') != strategy_ref:
            raise ValueError('grasp catalog identity mismatch')
    approach_distance = grasp_intent['approach_distance_m']
    if (not isinstance(approach_distance, (int, float)) or isinstance(approach_distance, bool) or
            not math.isfinite(approach_distance) or approach_distance < 0):
        raise ValueError('approach distance must be finite and nonnegative')
    geometry = build_grasp_target(observation)
    if strategy_ref == 'side_grip_basic':
        if (catalog.get('approach_axis') != 'x_plus' or
                catalog.get('orientation_mode') != 'horizontal'):
            raise ValueError('side grip catalog geometry must be x_plus/horizontal')
        for field in ('approach_axis', 'orientation_mode'):
            if field in grasp_intent and grasp_intent[field] != catalog[field]:
                raise ValueError(f'incompatible authored {field} for {strategy_ref}')
        x, y, z = geometry['target_pose'][:3]
        contact_x = x + oriented_box_extents(geometry)[0] / 2.0
        orientation = quaternion_from_rpy([0.0, -math.pi / 2.0, 0.0])
        contact = tuple([contact_x, y, z] + orientation)
        pregrasp = tuple([contact_x + approach_distance, y, z] + orientation)
        effective = {
            'approach_axis': 'x_plus',
            'orientation_mode': 'horizontal',
            'approach_distance_m': approach_distance,
        }
        return [GraspCandidate(strategy_ref, f'{strategy_ref}::000', observation['id'],
                               contact, pregrasp, effective)]
    grasp = generate_box_grasp_candidates(geometry, 0.0)
    approach = generate_box_grasp_candidates(geometry, approach_distance)
    return [GraspCandidate(strategy_ref, f'{strategy_ref}::{index:03}', observation['id'],
                           tuple(contact), tuple(pregrasp),
                           {'approach_distance_m': approach_distance})
            for index, (contact, pregrasp) in enumerate(zip(grasp, approach))]
