#include <gtest/gtest.h>
#include <QDir>
#include <QTemporaryDir>

#include <boost/filesystem.hpp>

#include <fstream>

#include "rviz_preview_runner.hpp"
#include "workcell_studio_scene_browser.hpp"

namespace fs = boost::filesystem;

namespace {

workcell_builder::WorkcellStudioSceneInfo runnable_scene()
{
  workcell_builder::WorkcellStudioSceneInfo scene;
  scene.scene_name = "ur5_2f_test";
  scene.has_package_xml = true;
  scene.has_launch_demo = true;
  scene.has_task_intent = true;
  scene.status = "READY";
  return scene;
}

}  // namespace

TEST(RvizPreviewMetadataCommandTest, UsesMetadataDerivedLaunchPackageAndFile)
{
  auto scene = runnable_scene();
  scene.launch_metadata_present = true;
  scene.launch_package = "custom_generated_pkg";
  scene.launch_file = "custom_preview.launch.py";
  scene.launch_metadata_file_exists = true;

  const QString command = workcell_builder::build_command(scene);

  EXPECT_EQ(
    command,
    "ros2 launch custom_generated_pkg custom_preview.launch.py use_fake_hardware:=true launch_rviz:=true");
}

TEST(RvizPreviewMetadataCommandTest, FallsBackToSceneNameAndDemoLaunch)
{
  const auto scene = runnable_scene();

  const QString command = workcell_builder::build_command(scene);

  EXPECT_EQ(command, "ros2 launch ur5_2f_test demo.launch.py use_fake_hardware:=true launch_rviz:=true");
}

TEST(RvizPreviewMetadataCommandTest, BlocksMissingMetadataLaunchFileClearly)
{
  auto scene = runnable_scene();
  scene.launch_metadata_present = true;
  scene.launch_package = "custom_generated_pkg";
  scene.launch_file = "missing_preview.launch.py";
  scene.launch_metadata_file_exists = false;
  scene.launch_metadata_warning = "scene_manifest.yaml launch file missing: /tmp/scene/launch/missing_preview.launch.py";

  const auto status = workcell_builder::validate_readiness(scene, fs::current_path());

  EXPECT_FALSE(status.ready);
  EXPECT_NE(status.blocker_reason.indexOf("scene_manifest.yaml launch file missing"), -1);
  EXPECT_NE(status.blocker_reason.indexOf("missing_preview.launch.py"), -1);
}

TEST(RvizPreviewMetadataCommandTest, DiscoversManifestLaunchMetadata)
{
  const fs::path root = fs::temp_directory_path() / fs::unique_path("workcell_scene_metadata_test_%%%%%%");
  const fs::path scene_dir = root / "src" / "easy_manipulation_deployment" / "scenes" / "metadata_scene";
  fs::create_directories(scene_dir / "launch");
  std::ofstream(scene_dir / "environment.yaml") << "robot: {name: ur5}\n";
  std::ofstream(scene_dir / "package.xml") << "<package format=\"3\"><name>metadata_pkg</name></package>\n";
  std::ofstream(scene_dir / "launch" / "custom.launch.py") << "# launch\n";
  std::ofstream(scene_dir / "scene_manifest.yaml")
    << "scene:\n"
    << "  name: metadata_scene\n"
    << "  package: metadata_pkg\n"
    << "files:\n"
    << "  launch: launch/custom.launch.py\n";

  const auto result = workcell_builder::discover_workcell_studio_scenes(root);

  ASSERT_EQ(result.scenes.size(), 1U);
  EXPECT_EQ(result.scenes[0].launch_package, "metadata_pkg");
  EXPECT_EQ(result.scenes[0].launch_file, "custom.launch.py");
  EXPECT_TRUE(result.scenes[0].launch_metadata_file_exists);

  fs::remove_all(root);
}

TEST(RvizPreviewMetadataCommandTest, FakeHardwareAndRvizRemainDefaultWithoutRealHardwareFlags)
{
  auto scene = runnable_scene();
  scene.launch_package = "custom_generated_pkg";
  scene.launch_file = "custom_preview.launch.py";

  const QString command = workcell_builder::build_command(scene);

  EXPECT_NE(command.indexOf("use_fake_hardware:=true"), -1);
  EXPECT_NE(command.indexOf("launch_rviz:=true"), -1);
  EXPECT_EQ(command.indexOf("use_fake_hardware:=false"), -1);
  EXPECT_EQ(command.indexOf("real_hardware:=true"), -1);
  EXPECT_EQ(command.indexOf("runtime_execution_enabled:=true"), -1);
}

TEST(RvizPreviewMetadataCommandTest, RebuildsOnlySelectedGeneratedPackageInCleanOverlay)
{
  auto scene = runnable_scene();
  scene.launch_package = "metadata_selected_scene";
  const QString command = workcell_builder::build_selected_package_command(scene, "/home/user/workcell_ws");

  EXPECT_NE(command.indexOf("env -u AMENT_PREFIX_PATH -u CMAKE_PREFIX_PATH -u COLCON_PREFIX_PATH"), -1);
  EXPECT_NE(command.indexOf("source /opt/ros/humble/setup.bash"), -1);
  EXPECT_NE(command.indexOf("source '/home/user/workcell_ws/install/setup.bash'"), -1);
  EXPECT_NE(command.indexOf("colcon build --symlink-install --packages-select 'metadata_selected_scene'"), -1);
  EXPECT_NE(command.indexOf("--allow-overriding 'metadata_selected_scene'"), -1);
  EXPECT_EQ(command.indexOf("--packages-up-to"), -1);
  EXPECT_EQ(command.indexOf("ur5_2f_test"), -1);
}

TEST(RvizPreviewMetadataCommandTest, PackageDiscoveryStaysInsideTheWorkspaceSource)
{
  // A workspace root that also holds another view of the same source must not find the scene twice.
  QTemporaryDir workspace;
  ASSERT_TRUE(workspace.isValid());
  ASSERT_TRUE(QDir(workspace.path()).mkpath("src"));
  auto scene = runnable_scene();
  scene.launch_package = "metadata_selected_scene";
  const QString command = workcell_builder::build_selected_package_command(scene, workspace.path().toStdString());
  EXPECT_NE(command.indexOf("colcon build --base-paths src --symlink-install --packages-select 'metadata_selected_scene'"), -1);
}

TEST(RvizPreviewMetadataCommandTest, DiscoveryAndLaunchSourceBuiltOverlay)
{
  const auto scene = runnable_scene();

  EXPECT_EQ(
    workcell_builder::package_prefix_check_command(scene, "/home/user/workcell_ws"),
    "source /opt/ros/humble/setup.bash && source '/home/user/workcell_ws/install/setup.bash' && "
    "ros2 pkg prefix 'ur5_2f_test'");
  const QString launch = workcell_builder::build_launch_shell_command(scene, "/home/user/workcell_ws");
  EXPECT_NE(launch.indexOf("source /opt/ros/humble/setup.bash"), -1);
  EXPECT_NE(launch.indexOf("source '/home/user/workcell_ws/install/setup.bash'"), -1);
  EXPECT_NE(launch.indexOf("pgrep -f"), -1);
  EXPECT_NE(launch.indexOf("another ros2_control preview is already running"), -1);
  EXPECT_NE(launch.indexOf("RCUTILS_COLORIZED_OUTPUT=0"), -1);
  EXPECT_NE(launch.indexOf("workcell_preview_process_group.py"), -1);
  EXPECT_NE(launch.indexOf("exec python3"), -1);
  EXPECT_NE(launch.indexOf("-- ros2 launch ur5_2f_test demo.launch.py use_fake_hardware:=true launch_rviz:=true"), -1);
  EXPECT_TRUE(workcell_builder::launch_command_is_safe(launch));
}

TEST(RvizPreviewMetadataCommandTest, RejectsRealHardwareLaunchTokens)
{
  QString reason;
  EXPECT_FALSE(workcell_builder::launch_command_is_safe(
    "ros2 launch ur5_2f_test demo.launch.py use_fake_hardware:=false launch_rviz:=true", &reason));
  EXPECT_FALSE(workcell_builder::launch_command_is_safe(
    "ros2 launch ur5_2f_test demo.launch.py use_fake_hardware:=true launch_rviz:=true ur_robot_driver:=true",
    &reason));
}

TEST(RvizPreviewMetadataCommandTest, SavedTaskUsesOneExistingSupervisorAndPlanOnlyResolution)
{
  auto scene = runnable_scene();
  const auto command = workcell_builder::build_full_cycle_command(scene, "/home/user/workcell_ws", "/tmp/cycle evidence", 201, true);
  EXPECT_TRUE(command.contains("run_r14_plan_only_acceptance.py"));
  // The registered MoveIt teardown overlay precedes the workspace; no record sources nothing.
  EXPECT_TRUE(command.startsWith("eval \"$(python3 '"));
  EXPECT_TRUE(command.contains("qualified_physics_runtime.py' moveit-env)\" && source '/home/user/workcell_ws/install/local_setup.bash'"));
  EXPECT_TRUE(command.contains("--resolve-if-needed --stream-status"));
  EXPECT_FALSE(command.contains("--execute"));
  EXPECT_TRUE(command.contains("--scene-dir"));
  EXPECT_TRUE(command.contains("--launch-rviz"));
  EXPECT_TRUE(command.contains("--output-dir '/tmp/cycle evidence'"));
  EXPECT_FALSE(command.contains("ros2 launch"));  // Supervisor owns the sole scene stack.
  EXPECT_FALSE(command.contains("use_fake_hardware:=false"));
  EXPECT_FALSE(workcell_builder::build_full_cycle_command(scene, "/tmp/ws", "/tmp/out", 201, false).contains("--launch-rviz"));
  scene.scene_name = "uncommissioned";
  EXPECT_FALSE(workcell_builder::build_full_cycle_command(scene, "/tmp/ws", "/tmp/out", 201, true).isEmpty());
}

TEST(RvizPreviewMetadataCommandTest, SortingSessionUsesTheSameSupervisorOnFakeHardwareWithAControlFile)
{
  auto scene = runnable_scene();
  scene.scene_dir = "/home/user/workcell_ws/src/easy_manipulation_deployment/scenes/ur5_2f_test";
  const auto command = workcell_builder::build_sorting_session_command(scene, "/home/user/workcell_ws", "/tmp/sort out",
                                                                       77, "/tmp/sort out/control.json");
  EXPECT_TRUE(command.contains("run_r14_plan_only_acceptance.py' --sorting-session --stream-status"));
  EXPECT_TRUE(command.contains("--control-file '/tmp/sort out/control.json'"));
  EXPECT_TRUE(command.contains("--domain-id 77"));
  EXPECT_FALSE(command.contains("--execute"));
  EXPECT_FALSE(command.contains("ros2 launch"));
  EXPECT_FALSE(command.contains("use_fake_hardware:=false"));
  EXPECT_TRUE(command.contains("--scene-dir '/home/user/workcell_ws/src/easy_manipulation_deployment/scenes/ur5_2f_test'"));
  EXPECT_TRUE(workcell_builder::build_sorting_session_command(scene, "/tmp/ws", "/tmp/out", 77, "").isEmpty());
  scene.scene_dir.clear();
  EXPECT_TRUE(workcell_builder::build_sorting_session_command(scene, "/tmp/ws", "/tmp/out", 77, "/tmp/c.json").isEmpty());
}

TEST(RvizPreviewMetadataCommandTest, PhysicsSortingSessionIsTheG4RunnerPlanOnlyNeverATrial)
{
  auto scene = runnable_scene();
  scene.scene_name = "g6_cups";
  scene.scene_dir = "/home/user/cells/g6_cups";
  const auto command = workcell_builder::build_physics_plan_session_command(scene, "/home/user/workcell_ws",
    "/tmp/studio sorting/run", 226, "/tmp/studio sorting/run.control.json");
  EXPECT_TRUE(command.startsWith("eval \"$(python3 '"));
  EXPECT_TRUE(command.contains("qualified_physics_runtime.py' env)\" && "));
  EXPECT_TRUE(command.contains("run_g4_sorting.py' --scene-package '/home/user/cells/g6_cups' --output '/tmp/studio sorting/run'"));
  EXPECT_TRUE(command.contains("--domain 226 --plan-every-target --stream-status --control-file '/tmp/studio sorting/run.control.json'"));
  // No trial authority can be expressed from the Builder: the session record is G4_PLAN_ONLY.
  EXPECT_FALSE(command.contains("--authorise-g4-trial"));
  EXPECT_FALSE(command.contains("--prior-evidence"));
  EXPECT_FALSE(command.contains("--max-cycles"));
  EXPECT_FALSE(command.contains("use_fake_hardware:=false"));
  // The runner creates the output itself (the control file may not live in it) and owns domains outside 170..219.
  EXPECT_TRUE(workcell_builder::build_physics_plan_session_command(scene, "/tmp/ws", "/tmp/out", 226, "/tmp/out/c.json").isEmpty());
  EXPECT_TRUE(workcell_builder::build_physics_plan_session_command(scene, "/tmp/ws", "/tmp/out", 180, "/tmp/c.json").isEmpty());
  EXPECT_TRUE(workcell_builder::build_physics_plan_session_command(scene, "/tmp/ws", "/tmp/out", 233, "/tmp/c.json").isEmpty());
  EXPECT_TRUE(workcell_builder::build_physics_plan_session_command(scene, "/tmp/ws", "/tmp/out", 226, "").isEmpty());
  scene.scene_dir.clear();
  EXPECT_TRUE(workcell_builder::build_physics_plan_session_command(scene, "/tmp/ws", "/tmp/out", 226, "/tmp/c.json").isEmpty());
}

TEST(RvizPreviewMetadataCommandTest, BoundedMotionSessionStartsTheG4TrialOnlyAfterReadinessPasses)
{
  auto scene = runnable_scene();
  scene.scene_name = "g4_cups";
  scene.scene_dir = "/home/user/cells/g4_cups";
  const QStringList runs{"--session", "/q/q_s1", "--session", "/q/q_s2", "--session", "/q/q_s3", "--stop", "/q/q_stop2",
                         "--cancel", "/q/q_cancel", "--other", "/q/q_stop"};
  const auto command = workcell_builder::build_physics_trial_session_command(scene, "/home/user/workcell_ws",
    "/tmp/studio sorting/run", 226, 2, runs, "/tmp/studio sorting/run.admission.json");
  // Readiness of this exact cell is recomputed in the same shell; the runner is exec'd only after it exits 0.
  const auto readiness = command.indexOf("g4_qualification.py' readiness --cell '/home/user/cells/g4_cups' --session '/q/q_s1' "
    "--session '/q/q_s2' --session '/q/q_s3' --stop '/q/q_stop2' --cancel '/q/q_cancel' --other '/q/q_stop' > "
    "'/tmp/studio sorting/run.admission.json' && exec python3 ");
  const auto runner = command.indexOf("run_g4_sorting.py' --scene-package '/home/user/cells/g4_cups' --output '/tmp/studio sorting/run' "
    "--domain 226 --stream-status --authorise-g4-trial --prior-evidence \"$WORKCELL_STAGE_A_PRIOR_EVIDENCE\" --max-cycles 2");
  EXPECT_GT(readiness, 0);
  EXPECT_GT(runner, readiness);
  EXPECT_TRUE(command.contains("test -n \"$WORKCELL_STAGE_A_PRIOR_EVIDENCE\" && "));
  EXPECT_TRUE(command.contains("WORKCELL_PLANNING_TRACE_DIR='/tmp/studio sorting/run.trace' && "));
  EXPECT_FALSE(command.contains("--plan-every-target"));
  EXPECT_FALSE(command.contains("--control-file"));
  EXPECT_FALSE(command.contains("use_fake_hardware:=false"));
  const auto build = [&](int domain, int cycles, const QStringList & r, const QString & admission) {
    return workcell_builder::build_physics_trial_session_command(scene, "/tmp/ws", "/tmp/out", domain, cycles, r, admission);
  };
  EXPECT_FALSE(build(226, 1, runs, "/tmp/a.json").isEmpty());
  // Bounded: one or two cycles, never a continuous session.
  EXPECT_TRUE(build(226, 0, runs, "/tmp/a.json").isEmpty());
  EXPECT_TRUE(build(226, 3, runs, "/tmp/a.json").isEmpty());
  // The contract's five counted sessions, and nothing else, may be named.
  EXPECT_TRUE(build(226, 2, runs.mid(2), "/tmp/a.json").isEmpty());
  EXPECT_TRUE(build(226, 2, QStringList(runs) << "--session" << "/q/q_s4", "/tmp/a.json").isEmpty());
  EXPECT_TRUE(build(226, 2, QStringList(runs) << "--workspace" << "/elsewhere", "/tmp/a.json").isEmpty());
  EXPECT_TRUE(build(226, 2, QStringList(runs) << "--other", "/tmp/a.json").isEmpty());
  EXPECT_TRUE(build(180, 2, runs, "/tmp/a.json").isEmpty());
  EXPECT_TRUE(build(226, 2, runs, "/tmp/out/a.json").isEmpty());
  EXPECT_TRUE(build(226, 2, runs, "").isEmpty());
}

TEST(RvizPreviewMetadataCommandTest, PhysicsCycleRunsTheQualifiedRunnerOnTheSavedCellOnly)
{
  auto scene = runnable_scene();
  scene.scene_name = "beginner_cell";
  scene.scene_dir = "/home/user/cells/beginner_cell";
  const auto command = workcell_builder::build_physics_cycle_command(scene, "/home/user/workcell_ws", "/tmp/physics evidence");
  // The runtime is the workspace's registered, re-validated record: never a hand-edited script.
  EXPECT_TRUE(command.startsWith("eval \"$(python3 '"));
  EXPECT_TRUE(command.contains("qualified_physics_runtime.py' env)\" && "));
  EXPECT_FALSE(command.contains("workcell_qualified_physics.bash"));
  EXPECT_TRUE(command.contains("run_stage_a1_finish.py' --mode targeted --gate full-cycle --scene-package '/home/user/cells/beginner_cell'"));
  EXPECT_TRUE(command.contains("--stream-status --output '/tmp/physics evidence'"));
  EXPECT_TRUE(command.contains("--prior-evidence \"$WORKCELL_STAGE_A_PRIOR_EVIDENCE\""));
  EXPECT_TRUE(command.contains("exec python3"));             // Cancel's SIGTERM reaches the runner itself
  EXPECT_FALSE(command.contains("ros2 launch"));             // the qualified runner owns the one simulator session
  EXPECT_FALSE(command.contains("--class-id"));              // the saved task decides what to pick
  EXPECT_FALSE(command.contains("--source-world"));          // the world is derived from the saved cell
  EXPECT_FALSE(command.contains("real_hardware"));
  scene.scene_dir.clear();
  EXPECT_TRUE(workcell_builder::build_physics_cycle_command(scene, "/tmp/ws", "/tmp/out").isEmpty());
}
