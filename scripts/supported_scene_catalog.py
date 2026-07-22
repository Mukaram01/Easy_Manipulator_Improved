from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml  # type: ignore

DEFAULT_SUPPORTED_SCENES_CATALOG = Path("scenes/supported_scenes.yaml")
CATALOG_SCHEMA_VERSION = "workcell_studio_supported_scenes/v1"
ACCEPTED_CATALOG_STATUSES = frozenset({"supported", "blocked", "disabled"})
ACCEPTED_SUPPORT_LEVELS = frozenset({"supported", "experimental"})

REQUIRED_SCENE_FIELDS = (
    "scene_name",
    "package_name",
    "authoring_files",
    "generated_files",
    "validation_command",
    "build_package_name",
    "fake_hardware_launch_command",
    "moveit_required",
    "status",
    "known_blocker",
)


@dataclass(frozen=True)
class SupportedSceneEntry:
    scene_name: str
    package_name: str
    scene_path: str
    support_level: str
    status: str
    known_blocker: str
    robot: str
    tool: str
    required_capabilities: tuple[str, ...]
    authoring_files: tuple[str, ...]
    generated_files: tuple[str, ...]
    validation_command: str
    build_package_name: str
    build_command: str
    fake_hardware_launch_command: str
    moveit_required: bool
    fake_hardware_acceptance: dict[str, Any]
    task_smoke: dict[str, Any]
    enabled: bool
    raw: dict[str, Any]

    @property
    def required_files(self) -> list[str]:
        return [*self.authoring_files, *self.generated_files]


def default_catalog_path(repo_root: Path) -> Path:
    return repo_root / DEFAULT_SUPPORTED_SCENES_CATALOG


def _load_yaml(path: Path) -> dict[str, Any]:
    payload = yaml.safe_load(path.read_text(encoding="utf-8"))
    return payload if isinstance(payload, dict) else {}


def _string_list(value: Any) -> tuple[str, ...]:
    if not isinstance(value, list):
        return tuple()
    return tuple(str(item).strip() for item in value if str(item).strip())


def load_supported_scene_catalog(path: Path) -> tuple[dict[str, Any], list[SupportedSceneEntry], list[str]]:
    catalog = _load_yaml(path)
    errors: list[str] = []
    if catalog.get("schema_version") != CATALOG_SCHEMA_VERSION:
        errors.append(
            f"catalog schema_version must be {CATALOG_SCHEMA_VERSION!r}; got {catalog.get('schema_version')!r}"
        )

    raw_entries = catalog.get("scenes")
    if not isinstance(raw_entries, list):
        errors.append("catalog field 'scenes' must be a list")
        raw_entries = []

    entries: list[SupportedSceneEntry] = []
    seen: set[str] = set()
    for idx, raw in enumerate(raw_entries):
        if not isinstance(raw, dict):
            errors.append(f"scenes[{idx}] must be a map")
            continue
        for field in REQUIRED_SCENE_FIELDS:
            if field not in raw:
                errors.append(f"scenes[{idx}] missing required field: {field}")

        scene_name = str(raw.get("scene_name", "")).strip()
        if not scene_name:
            errors.append(f"scenes[{idx}] scene_name must be non-empty")
            continue
        if scene_name in seen:
            errors.append(f"duplicate scene_name in catalog: {scene_name}")
        seen.add(scene_name)

        package_name = str(raw.get("package_name", scene_name)).strip() or scene_name
        build_package_name = str(raw.get("build_package_name", package_name)).strip() or package_name
        scene_path = str(raw.get("scene_path", f"scenes/{scene_name}")).strip() or f"scenes/{scene_name}"
        support_level = str(raw.get("support_level", "supported")).strip() or "supported"
        status = str(raw.get("status", "supported")).strip() or "supported"
        known_blocker = str(raw.get("known_blocker", "")).strip()
        robot = str(raw.get("robot", "")).strip() or scene_name.split("_", 1)[0]
        tool = str(raw.get("tool", "")).strip() or "unspecified_tool"
        required_capabilities = _string_list(raw.get("required_capabilities")) or ("fake_hardware_launch",)
        authoring_files = _string_list(raw.get("authoring_files"))
        generated_files = _string_list(raw.get("generated_files"))
        validation_command = str(raw.get("validation_command", "")).strip()
        build_command = str(raw.get("build_command", f"colcon build --symlink-install --packages-select {build_package_name}")).strip()
        fake_hardware_launch_command = str(raw.get("fake_hardware_launch_command", "")).strip()
        enabled = bool(raw.get("enabled", True))
        moveit_required = bool(raw.get("moveit_required", "fake_hardware_launch" in required_capabilities))
        acceptance = raw.get("fake_hardware_acceptance") if isinstance(raw.get("fake_hardware_acceptance"), dict) else {}
        task_smoke = raw.get("task_smoke") if isinstance(raw.get("task_smoke"), dict) else {}

        if status not in ACCEPTED_CATALOG_STATUSES:
            accepted = ", ".join(sorted(ACCEPTED_CATALOG_STATUSES))
            errors.append(f"{scene_name}: status must be one of [{accepted}]; got {status!r}")
        if support_level not in ACCEPTED_SUPPORT_LEVELS:
            accepted = ", ".join(sorted(ACCEPTED_SUPPORT_LEVELS))
            errors.append(f"{scene_name}: support_level must be one of [{accepted}]; got {support_level!r}")
        if status == "blocked" and not known_blocker:
            errors.append(f"{scene_name}: known_blocker must be non-empty when status is 'blocked'")
        if status == "supported" and known_blocker:
            errors.append(f"{scene_name}: known_blocker must be empty when status is 'supported'")
        if not authoring_files:
            errors.append(f"{scene_name}: authoring_files must list at least one file")
        if not generated_files:
            errors.append(f"{scene_name}: generated_files must list at least one file")
        if scene_name not in validation_command:
            errors.append(f"{scene_name}: validation_command must name the scene explicitly")
        if "use_fake_hardware:=true" not in fake_hardware_launch_command:
            errors.append(f"{scene_name}: fake_hardware_launch_command must explicitly set use_fake_hardware:=true")
        if "ros2 launch" not in fake_hardware_launch_command:
            errors.append(f"{scene_name}: fake_hardware_launch_command must be a ros2 launch command")
        if moveit_required and acceptance:
            if acceptance.get("motion_allowed") is not False:
                errors.append(f"{scene_name}: fake_hardware_acceptance.motion_allowed must be false")
            if not acceptance.get("require_fake_controllers", False):
                errors.append(f"{scene_name}: fake_hardware_acceptance.require_fake_controllers must be true")

        entries.append(
            SupportedSceneEntry(
                scene_name=scene_name,
                package_name=package_name,
                scene_path=scene_path,
                support_level=support_level,
                status=status,
                known_blocker=known_blocker,
                robot=robot,
                tool=tool,
                required_capabilities=required_capabilities,
                authoring_files=authoring_files,
                generated_files=generated_files,
                validation_command=validation_command,
                build_package_name=build_package_name,
                build_command=build_command,
                fake_hardware_launch_command=fake_hardware_launch_command,
                moveit_required=moveit_required,
                fake_hardware_acceptance=dict(acceptance),
                task_smoke=dict(task_smoke),
                enabled=enabled,
                raw=dict(raw),
            )
        )
    return catalog, entries, errors
