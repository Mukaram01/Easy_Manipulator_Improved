#!/usr/bin/env python3
"""Deterministic top 2F geometry shared by readiness and the legacy runtime.

This extraction accepts only the legacy approach distance. Additional authored
v2 constraints and strategies must be implemented before callers can use them.
Poses refer to the grasp frame; the preplanner applies the installed TCP.
"""
import math
from dataclasses import dataclass
from pathlib import Path

from perceived_object_grasp_plan import build_grasp_target, generate_box_grasp_candidates


@dataclass(frozen=True)
class GraspCandidate:
    strategy_ref: str
    candidate_id: str
    object_id: str
    grasp_pose: tuple[float, float, float, float, float, float, float]
    approach_pose: tuple[float, float, float, float, float, float, float]
    effective: dict


def generate_strategy_candidates(
    strategy_ref: str, observation: dict, grasp_intent: dict,
    catalog_dir: Path | None = None,
) -> list[GraspCandidate]:
    if strategy_ref != 'top_2f':
        raise ValueError(f'unsupported grasp strategy: {strategy_ref}')
    if set(grasp_intent) - {'approach_distance_m'}:
        raise ValueError('unsupported grasp constraints in legacy top_2f extraction')
    if catalog_dir is not None:
        import yaml
        entry = yaml.safe_load((Path(catalog_dir) / f'{strategy_ref}.yaml').read_text())
        if (entry.get('grasp_strategy') or {}).get('id') != strategy_ref:
            raise ValueError('grasp catalog identity mismatch')
    approach_distance = grasp_intent['approach_distance_m']
    if (not isinstance(approach_distance, (int, float)) or
            not math.isfinite(approach_distance) or approach_distance < 0):
        raise ValueError('approach distance must be finite and nonnegative')
    geometry = build_grasp_target(observation)
    grasp = generate_box_grasp_candidates(geometry, 0.0)
    approach = generate_box_grasp_candidates(geometry, approach_distance)
    return [GraspCandidate(strategy_ref, f'{strategy_ref}::{index:03}', observation['id'],
                           tuple(contact), tuple(pregrasp),
                           {'approach_distance_m': approach_distance})
            for index, (contact, pregrasp) in enumerate(zip(grasp, approach))]
