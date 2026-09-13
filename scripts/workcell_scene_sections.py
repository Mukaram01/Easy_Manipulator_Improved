"""Resolve workcell_scene/v1 sections without merging competing representations.

Nested sections win, root sections fill omissions. Older scenes store their
single physical sensor in camera (six-value world pose), rather than sensors.
"""

def physical_sections(data):
    environment = data.get('environment', {})
    result = {name: environment[name] if name in environment else data.get(name, [])
              for name in ('assets', 'sensors', 'support_surfaces')}
    camera = data.get('camera', {})
    if ('sensors' not in environment and 'sensors' not in data and
            camera.get('enabled') is True and not any(
                item.get('id') == camera.get('camera_id') or item.get('role') == 'camera'
                for item in result['assets'])):
        pose = camera.get('pose')
        if not isinstance(pose, list) or len(pose) != 6:
            raise ValueError('Legacy camera.pose must contain six world pose values')
        result['sensors'] = [dict(id=camera['camera_id'], role='camera', type='camera',
                                  pose_xyz=pose[:3], pose_rpy=pose[3:],
                                  layout_item_ref=camera.get('layout_item_ref'))]
    return result
