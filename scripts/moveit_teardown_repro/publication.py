"""Final publication of the already proved, fully installed MoveIt overlay."""
import json
from pathlib import Path


def publish(overlay, manifest):
    overlay = Path(overlay).resolve()
    try:
        library = Path(manifest['library']['path']).resolve(strict=True)
        executable = Path(manifest['move_group']['executable']['path']).resolve(strict=True)
    except (KeyError, OSError) as exc:
        raise RuntimeError('incomplete MoveIt qualification; no publication') from exc
    for path, package in ((library, 'moveit_ros_planning'), (executable, 'moveit_ros_move_group')):
        if not path.is_relative_to(overlay / 'install' / package):
            raise RuntimeError('qualified binary resolves outside its installation: ' + str(path))
    link = overlay / 'install/moveit_ros_planning/share/moveit_ros_planning/workcell_teardown_overlay.json'
    if link.exists() and not link.is_symlink():
        raise RuntimeError('refusing to replace unrelated provenance file')
    link.parent.mkdir(parents=True, exist_ok=True)
    target = overlay / 'provenance.json'
    pending = overlay / '.provenance.json.tmp'
    pending.write_text(json.dumps(manifest, indent=2) + '\n')
    pending.replace(target)
    if link.is_symlink() and link.resolve()==target:
        return
    temporary_link=link.with_name('.workcell_teardown_overlay.json.tmp')
    temporary_link.unlink(missing_ok=True)
    temporary_link.symlink_to(target)
    temporary_link.replace(link)
