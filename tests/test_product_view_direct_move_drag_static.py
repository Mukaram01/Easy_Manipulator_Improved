from pathlib import Path
VIEWER = Path('workcell_studio_web/viewer/viewer.js').read_text(encoding='utf-8')
def test_direct_move_drag_contract():
    for token in ["state.editorMode !== 'move'", 'canEditItem(rendered.item)', 'raycaster.setFromCamera(state.three.pointer, state.three.camera)', 'new THREE.Plane(new THREE.Vector3(0, 0, 1), -z)', 'offset: { x: start.pose.xyz.x - hit.x, y: start.pose.xyz.y - hit.y }', 'next.pose.xyz.z = drag.start.pose.xyz.z', "snapTransform(transform, { translationAxes: ['x', 'y'], rotationAxes: [] })", 'controlsWasEnabled: controls ? controls.enabled : true', 'controls.enabled = false', 'markDirtyTransform(rendered, finalTransform, { pushHistory: true, oldTransform: drag.start })', "pushEditorEvent('status', { message: `Moved ${itemLabel(rendered.item)}` })", "pushEditorEvent('status', { message: message || 'Move cancelled' })"]:
        assert token in VIEWER
    assert VIEWER.count("addEventListener('pointerdown', onCanvasPointerDown)") == 1
    assert VIEWER.count("addEventListener('pointermove', onCanvasPointerMove)") == 1
    assert VIEWER.count("addEventListener('pointerup', onCanvasPointerUp)") == 1
    move_body = VIEWER.split('function updateDirectMoveDrag', 1)[1].split('function finishDirectMoveDrag', 1)[0]
    assert all(token not in move_body for token in ['buildEditPatch', 'exportEditPatch', 'fetch(', 'loadSceneUrl', 'loadFile('])
