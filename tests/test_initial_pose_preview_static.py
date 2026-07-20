from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
VIEWER = ROOT / "workcell_studio_web/viewer/viewer.js"
RENDERER = ROOT / "workcell_studio_web/viewer/urdf_robot_renderer.js"
INDEX = ROOT / "workcell_studio_web/viewer/index.html"


def test_show_initial_pose_action_is_checkable_and_visual_only():
    html = INDEX.read_text(encoding="utf-8")
    js = VIEWER.read_text(encoding="utf-8")
    assert 'id="show-initial-pose" type="checkbox" disabled' in html
    assert 'Show Initial Pose' in html
    assert 'Preview the selected robot at its configured initial joint pose. This does not command the robot.' in html
    assert 'Initial pose preview' in html
    forbidden = ['/joint_states', 'FollowJointTrajectory', 'MoveGroup', 'controller_manager', 'create_publisher', 'create_client']
    preview_body = js.split('function toggleInitialPosePreview', 1)[1].split('function sceneDisplayName', 1)[0]
    assert not any(token in preview_body for token in forbidden)


def test_initial_pose_resolves_selected_robot_and_validates_joint_names():
    js = VIEWER.read_text(encoding="utf-8")
    assert 'function selectedRobotRecord()' in js
    assert 'robots.length === 1' in js
    assert 'robots.length > 1 && !selectedRobotRecord()' in js
    assert 'function configuredInitialJointValues(robot)' in js
    assert 'initial_joint_values' in js and 'configured_initial_positions' in js and 'initial_positions' in js
    assert 'Initial pose has unknown joint' in js
    assert 'Initial pose is missing joint' in js
    assert 'Initial pose contains an invalid value' in js
    assert '!joint?.isURDFMimicJoint && !joint?.mimicJoint' in js


def test_initial_pose_uses_existing_renderer_fk_and_restores_normal_pose():
    js = VIEWER.read_text(encoding="utf-8")
    renderer = RENDERER.read_text(encoding="utf-8")
    assert 'export function applyRobotJointPreview' in renderer
    assert 'robot.setJointValues(jointValues)' in renderer
    assert 'collectLinkMatrixDiagnostics(robot, links)' in renderer
    assert 'collectVisualWrapperMatrixDiagnostics(links)' in renderer
    assert 'collectDescendantRenderMeshDiagnostics(links)' in renderer
    assert 'applyRobotJointPreview(state.robotPreviewResult, jointMap)' in js
    assert 'applyRobotJointPreview?.(result, normalJointValues(selected.robot))' in js
    assert 'state.dirtyTransforms' not in js.split('function toggleInitialPosePreview', 1)[1].split('function sceneDisplayName', 1)[0]
    assert 'state.undoStack' not in js.split('function toggleInitialPosePreview', 1)[1].split('function sceneDisplayName', 1)[0]


def test_initial_pose_preview_clears_on_scene_reload_and_keeps_camera_fit_out():
    js = VIEWER.read_text(encoding="utf-8")
    clear_body = js.split('function clearSceneObjects()', 1)[1].split('function renderScene(items)', 1)[0]
    toggle_body = js.split('function toggleInitialPosePreview', 1)[1].split('function sceneDisplayName', 1)[0]
    assert 'state.initialPosePreview = { active: false' in clear_body
    assert 'el.showInitialPose.checked = false' in clear_body
    assert 'attemptInitialCameraFit' not in toggle_body
    assert 'resetView' not in toggle_body
