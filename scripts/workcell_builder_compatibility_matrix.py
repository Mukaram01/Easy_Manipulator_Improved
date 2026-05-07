from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Any


class CompatibilityStatus(str, Enum):
    SUPPORTED = "SUPPORTED"
    FAKE_HARDWARE_ONLY = "FAKE_HARDWARE_ONLY"
    PREVIEW_ONLY = "PREVIEW_ONLY"
    WARN = "WARN"
    INVALID = "INVALID"


@dataclass
class CompatibilityResult:
    status: CompatibilityStatus
    reasons: list[str]

    def to_dict(self) -> dict[str, Any]:
        return {"status": self.status.value, "reasons": self.reasons}


def _n(value: str | None) -> str:
    return (value or "").strip().lower().replace("_", " ")


def evaluate_compatibility(*, robot: str | None, gripper_tool: str | None, task_template: str | None,
                           grasp_strategy: str | None, environment_assets: list[str] | None = None,
                           custom_stl_assets: list[dict[str, Any]] | None = None,
                           hardware_mode: str | None = "fake") -> CompatibilityResult:
    robot_n = _n(robot)
    tool_n = _n(gripper_tool)
    task_n = _n(task_template)
    grasp_n = _n(grasp_strategy)
    assets = [_n(a) for a in (environment_assets or [])]
    stls = custom_stl_assets or []
    mode_n = _n(hardware_mode)

    reasons: list[str] = []
    status = CompatibilityStatus.SUPPORTED

    placeholder_robot = any(x in robot_n for x in ("fanuc", "abb", "kuka", "yaskawa"))
    if placeholder_robot or "generic delta" in robot_n or "generic gantry" in robot_n or "cartesian" in robot_n:
        status = CompatibilityStatus.PREVIEW_ONLY
        reasons.append("Selected robot is currently treated as placeholder/preview-only in Workcell Studio.")

    if ("delta" in robot_n or "gantry" in robot_n or "cartesian" in robot_n) and "suction" in tool_n:
        status = CompatibilityStatus.PREVIEW_ONLY
        reasons.append("Delta/gantry + suction combinations are preview-only.")

    if "conveyor" in assets:
        status = CompatibilityStatus.PREVIEW_ONLY if status != CompatibilityStatus.INVALID else status
        reasons.append("Conveyor assets are preview-only without a physics backend.")

    if any(a in assets for a in ("cnc", "machine tending", "machine_fixture")) or "machine tending" in task_n:
        status = CompatibilityStatus.PREVIEW_ONLY if status != CompatibilityStatus.INVALID else status
        reasons.append("CNC/machine-tending scenarios are currently placeholders (preview-only).")

    if "suction" in tool_n and "finger" in grasp_n:
        return CompatibilityResult(CompatibilityStatus.INVALID, ["Finger pinch grasp strategy is invalid for suction tools."])
    if ("robotiq" in tool_n or "2f" in tool_n) and "suction" in grasp_n:
        return CompatibilityResult(CompatibilityStatus.INVALID, ["Suction grasp strategy is invalid for finger grippers."])

    if "ur5" in robot_n and ("robotiq" in tool_n or "2f" in tool_n) and "pick" in task_n and "place" in task_n:
        status = CompatibilityStatus.FAKE_HARDWARE_ONLY
        reasons.append("UR5 + Robotiq 2F + pick_place is supported in fake hardware mode.")

    if "ur5" in robot_n and ("robotiq" in tool_n or "2f" in tool_n) and "sorting" in task_n:
        reasons.append("UR5 + Robotiq 2F + sorting depends on scene-template coverage.")
        status = CompatibilityStatus.WARN if status == CompatibilityStatus.SUPPORTED else status

    if "ur5" in robot_n and "suction" in tool_n and "suction top" in grasp_n:
        reasons.append("UR5 + suction + suction_top depends on AirPick package coverage.")
        status = CompatibilityStatus.WARN if status == CompatibilityStatus.SUPPORTED else status

    if mode_n in ("real", "real hardware", "real_hardware"):
        if status == CompatibilityStatus.SUPPORTED:
            status = CompatibilityStatus.WARN
        reasons.append("Real hardware mode is guarded and requires explicit runtime support.")

    unknown_collision = any(not bool(item.get("collision_known")) for item in stls if isinstance(item, dict))
    if unknown_collision:
        if status == CompatibilityStatus.SUPPORTED:
            status = CompatibilityStatus.WARN
        reasons.append("Custom STL assets with unknown collision quality require manual validation.")

    return CompatibilityResult(status, reasons)
