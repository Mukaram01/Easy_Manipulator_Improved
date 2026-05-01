#!/usr/bin/env python3
from __future__ import annotations

ROLE_DEVELOPER = "developer"
ROLE_OPERATOR = "operator"

CAPABILITIES = (
    "validate_cell_definition",
    "generate_workcell",
    "load_bundle",
    "preview_bundle",
    "preview_task_flow",
    "run_preflight",
    "run_gated_dry_run",
    "open_reports",
    "edit_runtime_config",
    "execute_motion",
)

ROLE_CAPABILITIES = {
    ROLE_DEVELOPER: {
        "validate_cell_definition": True,
        "generate_workcell": True,
        "load_bundle": True,
        "preview_bundle": True,
        "preview_task_flow": True,
        "run_preflight": True,
        "run_gated_dry_run": True,
        "open_reports": True,
        "edit_runtime_config": True,
        "execute_motion": False,
    },
    ROLE_OPERATOR: {
        "validate_cell_definition": False,
        "generate_workcell": False,
        "load_bundle": True,
        "preview_bundle": True,
        "preview_task_flow": True,
        "run_preflight": True,
        "run_gated_dry_run": True,
        "open_reports": True,
        "edit_runtime_config": False,
        "execute_motion": False,
    },
}


def can_role_do(role: str, capability: str) -> bool:
    return bool(ROLE_CAPABILITIES.get(role, {}).get(capability, False))


def list_capabilities(role: str) -> dict[str, bool]:
    defaults = {cap: False for cap in CAPABILITIES}
    defaults.update(ROLE_CAPABILITIES.get(role, {}))
    return defaults
