from pathlib import Path


def test_visualizer_node_and_launch_template_safety_contract():
    node = Path("easy_manipulation_deployment/emd_demo_nodes/run_grasp_execution/run_grasp_execution/task_recipe_visualizer_node.py")
    assert node.exists()
    txt = node.read_text(encoding="utf-8")
    for needle in [
        "task_recipe_path", "output_dir", "publish_markers", "dry_run_only", "keep_alive",
        "load_task_recipe", "validate_task_recipe", "build_offline_task_plan",
        "task_plan_preview.json", "task_plan_preview.md",
        "WORKCELL_TASK_RECIPE_RVIZ_PREVIEW: PASS",
        "/workcell_studio/task_plan_markers", "MarkerArray", "Offline dry-run preview only",
    ]:
        assert needle in txt

    launch = Path("workcell_builder/workcell_builder/templates/ros2/humble/launch/demo.launch.py").read_text(encoding="utf-8")
    for needle in ["launch_task_preview", "task_preview_markers", "task_preview_output_dir", "task_recipe_visualizer_node", "condition=IfCondition"]:
        assert needle in launch

    lowered = txt.lower()
    for forbidden in ["getmotionplan", "execute_trajectory", "followjointtrajectory", "actionclient", "hardware_interface"]:
        assert forbidden not in lowered
