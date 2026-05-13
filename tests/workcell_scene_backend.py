from __future__ import annotations
from dataclasses import dataclass, field
from pathlib import Path
from datetime import datetime
import shutil
import yaml

@dataclass
class SceneModel:
    name: str
    robot: dict = field(default_factory=dict)
    end_effector: dict = field(default_factory=dict)
    objects: dict = field(default_factory=dict)
    external_joints: dict = field(default_factory=dict)
    metadata: dict = field(default_factory=dict)

@dataclass
class SceneOpenResult:
    status: str
    model: SceneModel | None = None
    reason: str = ""


def open_existing_scene(scene_dir: Path) -> SceneOpenResult:
    env = scene_dir / 'environment.yaml'
    if not env.exists():
        return SceneOpenResult('YAML_MISSING', reason='environment.yaml missing')
    try:
        root = yaml.safe_load(env.read_text()) or {}
    except Exception as exc:
        return SceneOpenResult('YAML_INVALID_REPAIRABLE', reason=str(exc))
    if not isinstance(root, dict):
        return SceneOpenResult('YAML_INVALID_BLOCKED', reason='root must be map')
    model = SceneModel(
        name=scene_dir.name,
        robot=root.get('robot') or {},
        end_effector=root.get('end_effector') or root.get('endeffector') or {},
        objects=root.get('objects') or {},
        external_joints=root.get('external joints') or {},
        metadata={k: v for k, v in root.items() if k not in {'robot','end_effector','endeffector','objects','external joints'}},
    )
    return SceneOpenResult('YAML_READY', model=model)


def save_scene(scene_dir: Path, model: SceneModel) -> Path:
    env = scene_dir / 'environment.yaml'
    if env.exists():
        backup = scene_dir / f"environment.yaml.{datetime.utcnow().strftime('%Y%m%d%H%M%S')}.bak"
        shutil.copy2(env, backup)
    else:
        backup = scene_dir / 'environment.yaml.bak'
    root = {
        'robot': model.robot,
        'end_effector': model.end_effector,
        'objects': model.objects,
        'external joints': model.external_joints,
        **model.metadata,
    }
    root.setdefault('fake_hardware_first', True)
    root.setdefault('runtime_execution_enabled', False)
    env.write_text(yaml.safe_dump(root, sort_keys=False))
    return backup


def duplicate_scene(src: Path, dst_name: str) -> Path:
    dst = src.parent / dst_name
    shutil.copytree(src, dst)
    return dst


def regenerate_scene(scene_dir: Path) -> str:
    launch = scene_dir / 'launch'
    urdf = scene_dir / 'urdf'
    launch.mkdir(parents=True, exist_ok=True)
    urdf.mkdir(parents=True, exist_ok=True)
    (scene_dir / 'README.generated.md').write_text('# generated\n')
    (launch / 'demo.launch.py').write_text(f"# launch for {scene_dir.name}\n")
    (launch / 'demo.rviz').write_text('Panels: []\n')
    (urdf / 'scene.urdf.xacro').write_text('<robot name="scene"/>\n')
    (urdf / 'arm_hand.srdf.xacro').write_text('<robot name="arm_hand"/>\n')
    return f"ros2 launch {scene_dir.name} demo.launch.py"


def canvas_items(model: SceneModel) -> list[dict]:
    items = []
    if model.robot:
        items.append({'type': 'robot', 'name': model.robot.get('name','robot')})
    names = list(model.objects.keys()) if isinstance(model.objects, dict) else []
    for n in names:
        t = 'table' if 'table' in n else 'bin' if 'bin' in n else 'object'
        items.append({'type': t, 'name': n})
    return items
