"""Final publication of the already proved, fully installed MoveIt overlay."""
import hashlib
import json
from pathlib import Path


def validate_active_results(manifest):
    """Check the bound real-request proof for this exact capability artifact."""
    proof=manifest['move_group']['active_shutdown']
    path=Path(proof['results_path'])
    if hashlib.sha256(path.read_bytes()).hexdigest()!=proof['results_sha256']:
        raise ValueError('active planning shutdown evidence hash changed')
    results=json.loads(path.read_text())
    if (results.get('method')!='real_saved_ur5_plan_owned_sigint/v1' or
        not results.get('passed') or results.get('cycles',0)<5 or
        results.get('completed')!=results.get('cycles') or
        len(results.get('results',[]))!=results.get('completed')):
        raise ValueError('five completed real active planning shutdown cycles required')
    required={'binary','library','capabilities','request','scene','urdf','srdf','support_adapter','parameters','verifier'}
    if not required.issubset(results['inputs']):
        raise ValueError('active planning shutdown inputs are incomplete')
    reviewed_verifier=Path(__file__).with_name('verify_active_plan.py')
    if results['inputs']['verifier']['sha256']!=hashlib.sha256(reviewed_verifier.read_bytes()).hexdigest():
        raise ValueError('active planning shutdown used a different verifier')
    for item in results['inputs'].values():
        if hashlib.sha256(Path(item['path']).read_bytes()).hexdigest()!=item['sha256']:
            raise ValueError('active planning shutdown input changed')
    artifacts={'binary':manifest['move_group']['executable'],'library':manifest['library'],
               'capabilities':manifest['move_group']['capabilities']}
    for name,item in artifacts.items():
        if results['inputs'][name]!=item:
            # TEM receipts also include their SONAME; identity is path/hash.
            if any(results['inputs'][name].get(k)!=item.get(k) for k in ('path','sha256')):
                raise ValueError('active planning shutdown used different '+name)
    for row in results['results']:
        if (not all(row.get(key) for key in ('passed','accepted','owned_sigint','pending_at_signal',
                'signal_handler_observed','production_certificate_after_signal','plan_result_after_signal')) or
            row.get('returncode')!=0 or row.get('forced_cleanup') or row.get('timed_out') or
            row.get('remaining_processes') or row.get('core_inventory_changed')):
            raise ValueError('active planning shutdown cycle failed')
        support=row['mapped_artifacts']['support_adapter']
        if (support['sha256']!=results['inputs']['support_adapter']['sha256'] or
            support['path']!=results['runtime_paths']['support_adapter']):
            raise ValueError('active planning shutdown mapped different support adapter')
        for name,item in artifacts.items():
            if any(row['mapped_artifacts'][name].get(k)!=item.get(k) for k in ('path','sha256')):
                raise ValueError('active planning shutdown mapped different '+name)
    return results


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
    try:
        capabilities=Path(manifest['move_group']['capabilities']['path']).resolve(strict=True)
        if not capabilities.is_relative_to(overlay/'install/moveit_ros_move_group'):
            raise ValueError('capability resolves outside its installation')
        validate_active_results(manifest)
    except (KeyError, ValueError, TypeError, OSError) as exc:
        raise RuntimeError('incomplete active planning shutdown qualification; no publication') from exc
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
