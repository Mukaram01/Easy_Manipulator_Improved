// Copyright 2020 ROS Industrial Consortium Asia Pacific
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include "emd/grasp_execution/moveit2/moveit_cpp_if.hpp"

#include "rclcpp/rclcpp.hpp"

#include "moveit/moveit_cpp/moveit_cpp.h"
#include "moveit/moveit_cpp/planning_component.h"
#include "moveit/robot_state/cartesian_interpolator.h"
#include "moveit/macros/console_colors.h"
#include "moveit/utils/message_checks.h"
#include "moveit/trajectory_processing/iterative_time_parameterization.h"

#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2/impl/utils.h"
#include "tf2_eigen/tf2_eigen.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "shape_msgs/msg/mesh.hpp"
#include "shape_msgs/msg/solid_primitive.hpp"
#include "geometric_shapes/shapes.h"
#include "geometric_shapes/mesh_operations.h"
#include "geometric_shapes/shape_operations.h"

#include "moveit_msgs/msg/attached_collision_object.hpp"
#include "moveit_msgs/msg/collision_object.hpp"
#include "moveit_msgs/msg/constraints.hpp"
#include "moveit_msgs/msg/robot_trajectory.hpp"

namespace grasp_execution
{

namespace moveit2
{

namespace detail
{

geometry_msgs::msg::Pose get_object_pose_from_world_object(
  const collision_detection::World::ObjectConstPtr & object,
  const std::string & object_id,
  const rclcpp::Logger & logger)
{
  geometry_msgs::msg::Pose object_pose{};
  if (!object) {
    RCLCPP_ERROR(logger, "Object with id '%s' not found in planning scene", object_id.c_str());
    return object_pose;
  }

  if (object->shape_poses_.empty()) {
    RCLCPP_ERROR(
      logger,
      "Object with id '%s' has no shape poses in planning scene",
      object_id.c_str());
    return object_pose;
  }

  const auto & pose = object->shape_poses_.front();
  Eigen::Quaterniond q(pose.linear());
  object_pose.position.x = pose.translation().x();
  object_pose.position.y = pose.translation().y();
  object_pose.position.z = pose.translation().z();
  object_pose.orientation.x = q.x();
  object_pose.orientation.y = q.y();
  object_pose.orientation.z = q.z();
  object_pose.orientation.w = q.w();
  return object_pose;
}

}  // namespace detail

static const rclcpp::Logger LOGGER = rclcpp::get_logger("grasp_execution");

MoveitCppGraspExecution::MoveitCppGraspExecution(
  const rclcpp::Node::SharedPtr & node,
  size_t planning_concurrency,
  size_t execution_concurrency)
: GraspExecutionInterface(
    node, planning_concurrency, execution_concurrency),
  moveit_cpp_(std::make_shared<moveit_cpp::MoveItCpp>(node_))
{
  // let RViz display query PlanningScene
  moveit_cpp_->getPlanningSceneMonitor()->providePlanningSceneService();
  moveit_cpp_->getPlanningSceneMonitor()->setPlanningScenePublishingFrequency(100);

  // Set Robot Root link
  robot_frame_ = moveit_cpp_->getRobotModel()->getRootLinkName();
  RCLCPP_INFO(
    LOGGER,
    "Robot Root Link Name: %s", robot_frame_.c_str());

  // Initialize grasp_execution executor loader
  executor_loader_ = std::make_shared<pluginlib::ClassLoader<
        grasp_execution::moveit2::Executor>>(
    "emd_grasp_execution",
    "grasp_execution::moveit2::Executor");

  // load default executor plugin
  default_executor_ = std::unique_ptr<grasp_execution::moveit2::Executor>(
    executor_loader_->createUnmanagedInstance(
      "grasp_execution/DefaultExecutor"));
  default_executor_->load(moveit_cpp_, "");

  // Initialized octomap if specified
  grasp_execution::declare_or_get_param<bool>(
    load_octomap_requested_, "load_octomap", node, node->get_logger(), false);
}

MoveitCppGraspExecution::~MoveitCppGraspExecution()
{
  executor_loader_.reset();
  default_executor_.reset();
  // Exit everything in order
  for (auto & arm : arms_) {
    arm.second.planner.reset();
    arm.second.executors.clear();
  }
  moveit_cpp_.reset();
}

bool MoveitCppGraspExecution::start_world_geometry_monitor()
{
  if (!load_octomap_requested_) {
    RCLCPP_INFO(LOGGER, "World geometry monitor not started (load_octomap is false).");
    return true;
  }
  if (world_geometry_monitor_started_) {
    RCLCPP_INFO(LOGGER, "World geometry monitor already started.");
    return true;
  }

  moveit_cpp_->getPlanningSceneMonitor()->startWorldGeometryMonitor(
    "/collision_object", "/planning_scene_world", load_octomap_requested_);
  world_geometry_monitor_started_ = true;
  RCLCPP_INFO(LOGGER, "Started world geometry monitor (octomap + shape mask).");
  return true;
}

bool MoveitCppGraspExecution::init(const std::string & planning_group)
{
  prompt_job_start(
    LOGGER, "",
    "Initializing planning group: [" + planning_group + "].");
  if (!GraspExecutionInterface::init(planning_group)) {
    return false;
  }

  // Check if planner is already registered
  if (arms_.find(planning_group) == arms_.end()) {
    // Print out basic planning group info for debugging
    static constexpr double kCurrentStateTimeoutS = 1.0;
    moveit::core::RobotStatePtr state;
    if (!moveit_cpp_->getCurrentState(state, kCurrentStateTimeoutS)) {
      RCLCPP_ERROR(LOGGER, "Timed out waiting for current robot state");
      prompt_job_end(LOGGER, false);
      return false;
    }
    auto robot_model = state->getRobotModel();
    auto joint_model_group = robot_model->getJointModelGroup(planning_group);
    size_t dof = joint_model_group->getVariableCount();
    auto link_names = joint_model_group->getLinkModelNames();
    auto root_link = robot_model->getRootLinkName();

    if (dof < 1) {
      RCLCPP_ERROR(LOGGER, "Planning Group [%s] is empty", planning_group.c_str());
      prompt_job_end(LOGGER, false);
      return false;
    } else {
      RCLCPP_INFO(
        LOGGER,
        "Basic Info:\n"
        "Base frame: %s\n"
        "Joint Model Group: %s\n"
        "\tDoF: %lu\n"
        "\tfirst_link: %s\n"
        "\tend_link:  %s\n"
        "\tChain: %s\n"
        "\tEnd Effector: %s\n",
        root_link.c_str(), planning_group.c_str(), dof,
        link_names.front().c_str(), link_names.back().c_str(),
        (joint_model_group->isChain() ? "Yes" : "No"),
        (joint_model_group->isEndEffector() ? "Yes" : "No"));

      // Initialize another moveit2 Joint Model Group context
      arms_.emplace(
        planning_group,
        moveit2::JmgContext());

      // Initialize planner
      arms_[planning_group].planner =
        std::make_shared<moveit_cpp::PlanningComponent>(
        planning_group, moveit_cpp_);

      // Initialize gripper
      arms_[planning_group].default_ee.link = link_names.back();
      arms_[planning_group].default_ee.brand = "dummy";

      // Initialize dummy gripper driver
      auto dummy_gripper_driver = std::unique_ptr<gripper::GripperDriver>(
        gripper_driver_loader_->createUnmanagedInstance("grasp_execution/DummyGripperDriver"));
      if (dummy_gripper_driver->load("") != gripper::GripperDriver::Result::SUCCESS) {
        RCLCPP_ERROR(LOGGER, "Failed to load dummy gripper driver");
        prompt_job_end(LOGGER, false);
        return false;
      }
      arms_[planning_group].grippers["dummy"] = std::move(dummy_gripper_driver);

      prompt_job_end(LOGGER, true);
      return true;
    }
  } else {
    RCLCPP_WARN(LOGGER, "Planning Group [%s] already exist", planning_group.c_str());
    prompt_job_end(LOGGER, false);
    return false;
  }
}

bool MoveitCppGraspExecution::init_from_yaml(const std::string & path)
{
  if (!GraspExecutionInterface::init_from_yaml(path)) {
    return false;
  }

  for (auto & itr : this->get_workcell_context().groups) {
    // Initialize group (note: repeated group context loading will be ignored).
    if (!this->init(itr.first)) {
      return false;
    }
    const auto & group_name = itr.first;
    const auto & group = itr.second;

    // Initialize executors.
    // (note: repeated executor context loading will be ignored).
    for (auto & itr2 : group.executors) {
      if (!this->load_execution_method(
          group_name, itr2.first, itr2.second.plugin, itr2.second.controller))
      {
        return false;
      }
    }

    // Initialize gripper
    // (note: repeated gripper context loading will be ignored)
    for (auto & ee : group.end_effectors) {
      if (!this->load_ee(
          group_name, ee.first,
          ee.second.brand, ee.second.link, ee.second.clearance,
          ee.second.driver.plugin, ee.second.driver.controller))
      {
        return false;
      }
    }
  }
  return true;
}

bool MoveitCppGraspExecution::load_execution_method(
  const std::string & group_name,
  const std::string & execution_method,
  const std::string & execution_plugin,
  const std::string & execution_controller)
{
  if (!GraspExecutionInterface::load_execution_method(
      group_name, execution_method, execution_plugin, execution_controller))
  {
    return false;
  }

  // Check if arms are initialized
  if (arms_.find(group_name) == arms_.end()) {
    // Otherwise call the init function
    if (!this->init(group_name)) {
      return false;
    }
  }

  // Initialize customized execution method
  if (execution_method != "default" && !execution_method.empty()) {
    auto customized_executor = std::unique_ptr<grasp_execution::moveit2::Executor>(
      executor_loader_->createUnmanagedInstance(execution_plugin));
    if (!customized_executor) {
      RCLCPP_ERROR(LOGGER, "Failed to load executor plugin: %s", execution_plugin.c_str());
      return false;
    }
    customized_executor->load(moveit_cpp_, execution_controller);
    arms_[group_name].executors[execution_method] = std::move(customized_executor);
  }
  return true;
}

bool MoveitCppGraspExecution::load_ee(
  const std::string & group_name,
  const std::string & ee_name,
  const std::string & ee_brand,
  const std::string & ee_link,
  double ee_clearance,
  const std::string & ee_driver_plugin,
  const std::string & ee_driver_controller)
{
  if (!GraspExecutionInterface::load_ee(
      group_name, ee_name, ee_brand, ee_link, ee_clearance,
      ee_driver_plugin, ee_driver_controller))
  {
    return false;
  }

  // Check if arms are initialized
  if (arms_.find(group_name) == arms_.end()) {
    // Otherwise call the init function
    if (!this->init(group_name)) {
      return false;
    }
  }

  // Initialize customized execution method
  auto gripper_driver = std::unique_ptr<gripper::GripperDriver>(
    gripper_driver_loader_->createUnmanagedInstance(ee_driver_plugin));
  if (!gripper_driver) {
    RCLCPP_ERROR(LOGGER, "Failed to load gripper driver plugin: %s", ee_driver_plugin.c_str());
    return false;
  }
  if (gripper_driver->load(ee_driver_controller) != gripper::GripperDriver::Result::SUCCESS) {
    RCLCPP_ERROR(LOGGER, "Failed to load gripper driver for %s", ee_brand.c_str());
    return false;
  }
  if (gripper_driver->activate() != gripper::GripperDriver::Result::SUCCESS) {
    RCLCPP_ERROR(LOGGER, "Failed to activate gripper driver for %s", ee_brand.c_str());
    return false;
  }
  arms_[group_name].grippers[ee_brand] = std::move(gripper_driver);
  return true;
}

bool MoveitCppGraspExecution::init(
  const std::string & planning_group,
  const std::string & ee_link,
  const std::string & execution_method,
  const std::string & execution_type,
  const std::string & controller_name)
{
  if (!this->init(planning_group)) {
    return false;
  }

  // Reset the default ee link
  std::string & default_ee_link = arms_[planning_group].default_ee.link;
  if (ee_link != default_ee_link) {
    RCLCPP_WARN(
      LOGGER,
      MOVEIT_CONSOLE_COLOR_YELLOW
      "Assuming [%s] is rigidly attached to end_link [%s]"
      MOVEIT_CONSOLE_COLOR_RESET, ee_link.c_str(), default_ee_link.c_str());
    default_ee_link = ee_link;
  }

  bool execution_loaded = this->load_execution_method(
    planning_group, execution_method, execution_type, controller_name);
  if (!execution_loaded) {
    RCLCPP_ERROR(
      LOGGER,
      "Failed to load execution method '%s' for group '%s'",
      execution_method.c_str(), planning_group.c_str());
  }
  return execution_loaded;
}


void MoveitCppGraspExecution::register_target_object(
  const shape_msgs::msg::SolidPrimitive & target_object_shape,
  const geometry_msgs::msg::PoseStamped & target_object_pose,
  const int & index,
  const std::string & task_id,
  const std::vector<std::string> & disabled_links)
{
  // Add all targets into the scene
  moveit_msgs::msg::CollisionObject temp_collision_object;
  temp_collision_object.operation = temp_collision_object.ADD;

  const auto target_id =
    this->gen_target_object_id(target_object_shape, task_id, index);

  // Check if object already exists
  {    // Lock PlanningScene
    planning_scene_monitor::LockedPlanningSceneRW scene(moveit_cpp_->getPlanningSceneMonitor());
    if (scene->getWorld()->getObject(target_id)) {
      return;
    }
  }    // Unlock PlanningScene

  prompt_job_start(
    LOGGER, target_id,
    "Register object in the world");

  temp_collision_object.id = target_id;
  temp_collision_object.header.frame_id =
    target_object_pose.header.frame_id;

  temp_collision_object.primitives.clear();
  temp_collision_object.primitive_poses.clear();

  temp_collision_object.primitives.push_back(target_object_shape);
  temp_collision_object.primitive_poses.push_back(target_object_pose.pose);

  // // Print out all object poses as debug information
  // print_pose_ros(LOGGER, target.target_pose);

  auto tmp_pose = target_object_pose;
  to_frame(target_object_pose, tmp_pose, this->robot_frame_);

  // // Print out all object poses as debug information
  // print_pose_ros(LOGGER, tmp_pose);

  bool result;
  // Add object to planning scene
  {    // Lock PlanningScene
    planning_scene_monitor::LockedPlanningSceneRW scene(moveit_cpp_->getPlanningSceneMonitor());
    result = scene->processCollisionObjectMsg(temp_collision_object);

    if (!disabled_links.empty()) {
      auto & acm = scene->getAllowedCollisionMatrixNonConst();
      acm.setEntry(target_id, disabled_links, true);
    }
  }    // Unlock PlanningScene

  prompt_job_end(LOGGER, result);
}

std::string MoveitCppGraspExecution::register_target_object_mesh(
  const std::string & mesh_filepath,
  const geometry_msgs::msg::PoseStamped & target_object_pose,
  int index,
  const std::string & task_id,
  const std::vector<std::string> & disabled_links)
{
  std::string target_id;
  // Add all targets into the scene
  moveit_msgs::msg::CollisionObject temp_collision_object;
  temp_collision_object.operation = temp_collision_object.ADD;

  shapes::Mesh * m = shapes::createMeshFromResource(mesh_filepath);
  if (!m) {
    RCLCPP_ERROR(LOGGER, "Failed to load mesh from resource: %s", mesh_filepath.c_str());
    return "";
  }
  shape_msgs::msg::Mesh obj_mesh;
  shapes::ShapeMsg obj_mesh_msg;
  shapes::constructMsgFromShape(m, obj_mesh_msg);
  obj_mesh = boost::get<shape_msgs::msg::Mesh>(obj_mesh_msg);
  temp_collision_object.meshes.resize(1);

  target_id = gen_target_mesh_id(task_id, index);

  // Check if object already exists
  {    // Lock PlanningScene
    planning_scene_monitor::LockedPlanningSceneRW scene(moveit_cpp_->getPlanningSceneMonitor());
    if (scene->getWorld()->getObject(target_id)) {
      delete m;
      return "";
    }
  }    // Unlock PlanningScene

  prompt_job_start(
    LOGGER, target_id,
    "Register object in the world");

  temp_collision_object.id = target_id;
  temp_collision_object.header.frame_id =
    target_object_pose.header.frame_id;

  temp_collision_object.meshes.clear();
  temp_collision_object.mesh_poses.clear();

  temp_collision_object.meshes.push_back(obj_mesh);
  temp_collision_object.mesh_poses.push_back(target_object_pose.pose);

  // // Print out all object poses as debug information
  // print_pose_ros(LOGGER, target.target_pose);

  auto tmp_pose = target_object_pose;
  to_frame(target_object_pose, tmp_pose, this->robot_frame_);

  // // Print out all object poses as debug information
  // print_pose_ros(LOGGER, tmp_pose);

  bool result;
  // Add object to planning scene
  {    // Lock PlanningScene
    planning_scene_monitor::LockedPlanningSceneRW scene(moveit_cpp_->getPlanningSceneMonitor());
    result = scene->processCollisionObjectMsg(temp_collision_object);

    if (!disabled_links.empty()) {
      auto & acm = scene->getAllowedCollisionMatrixNonConst();
      acm.setEntry(target_id, disabled_links, true);
    }
  }    // Unlock PlanningScene

  prompt_job_end(LOGGER, result);
  delete m;
  return target_id;
}

geometry_msgs::msg::Pose MoveitCppGraspExecution::get_object_pose(
  const std::string & object_id) const
{
  {    // Lock PlanningScene
    planning_scene_monitor::LockedPlanningSceneRW scene(moveit_cpp_->getPlanningSceneMonitor());
    return detail::get_object_pose_from_world_object(
      scene->getWorld()->getObject(object_id), object_id, LOGGER);
  }    // Unlock PlanningScene
}

moveit::core::RobotStatePtr MoveitCppGraspExecution::get_curr_state() const
{
  static constexpr double kCurrentStateTimeoutS = 1.0;
  moveit::core::RobotStatePtr curr_state;
  if (!moveit_cpp_->getCurrentState(curr_state, kCurrentStateTimeoutS)) {
    RCLCPP_WARN(LOGGER, "Timed out waiting for current robot state in get_curr_state()");
  }
  return curr_state;
}

geometry_msgs::msg::PoseStamped MoveitCppGraspExecution::get_curr_pose(
  const std::string & link_name) const
{
  static constexpr double kCurrentStateTimeoutS = 1.0;
  moveit::core::RobotStatePtr state;
  if (!moveit_cpp_->getCurrentState(state, kCurrentStateTimeoutS) || !state) {
    throw std::runtime_error(
      "Failed to retrieve current robot state (timeout or null state) in get_curr_pose() for link '" +
      link_name + "'");
  }
  const auto & transform = state->getGlobalLinkTransform(link_name);
  ASSERT_ISOMETRY(transform);  // unsanitized input, could contain a non-isometry

  Eigen::Quaterniond q(transform.linear());
  geometry_msgs::msg::PoseStamped output_pose;
  output_pose.header.frame_id = state->getRobotModel()->getRootLinkName();
  output_pose.header.stamp = node_->now();
  output_pose.pose.position.x = transform.translation().x();
  output_pose.pose.position.y = transform.translation().y();
  output_pose.pose.position.z = transform.translation().z();
  output_pose.pose.orientation.x = q.x();
  output_pose.pose.orientation.y = q.y();
  output_pose.pose.orientation.z = q.z();
  output_pose.pose.orientation.w = q.w();
  return output_pose;
}

bool MoveitCppGraspExecution::move_to(
  const GraspExecutionContext & option,
  const geometry_msgs::msg::PoseStamped & pose,
  bool execute)
{
  auto & arm = arms_[option.planning_group];
  const auto & ee_link = (option.ee_link.empty() ? arm.default_ee.link : option.ee_link);

  // Flag for planning
  bool result = false;

  // Set the start state to the last point of the trajectory
  // if immediate execution is not needed
  if (!execute &&
    !arm.traj.empty())
  {
    arm.planner->setStartState(arm.traj.back()->getLastWayPoint());
  } else {
    arm.planner->setStartStateToCurrentState();
  }

  // Strategy 1
  RCLCPP_INFO(
    LOGGER, "\nStarting strategy 1: Cartesian move to destination with 1cm step. "
    "This works well within relatively empty space");
  robot_trajectory::RobotTrajectoryPtr traj;
  auto fraction = cartesian_to(
    option.planning_group, *arm.planner->getStartState(), {pose.pose},
    traj, option.ee_link, option.cartesian_step_size, 0);

  RCLCPP_INFO(LOGGER, "fraction: %f", fraction);
  result = (fraction == 1.0);

  // Strategy 2
  if (!result && static_cast<int>(traj->getWayPointCount()) > option.backtrack_steps) {
    RCLCPP_INFO(
      LOGGER,
      "\nStrategy 1 failed :<\n"
      "Starting strategy 2: Back track cartesian path 10cm"
      "to last viable point and start non-deterministic planning");

    arm.planner->setStartState(
      traj->getWayPoint(traj->getWayPointCount() - option.backtrack_steps));


    moveit_cpp::PlanningComponent::PlanSolution plan_solution;
    int count = 0;

    while (!plan_solution && count < option.hybrid_max_attempts) {
      arm.planner->setGoal(pose, ee_link);
      plan_solution = arm.planner->plan();  // PlanningComponent::PlanSolution
      count++;
    }

    if (plan_solution) {
      auto temp_traj = *traj;
      traj->clear();
      traj->append(temp_traj, 0, 0, temp_traj.getWayPointCount() - option.backtrack_steps);
      traj->append(*plan_solution.trajectory, 0, 1);

      trajectory_processing::IterativeParabolicTimeParameterization time_param;
      time_param.computeTimeStamps(*traj, 1.0);
      result = true;
    }
  }

  // Strategy 3
  if (!result) {
    RCLCPP_INFO(
      LOGGER,
      "\nStrategy 1 & 2 failed :<\n"
      "Starting strategy 3: Start over with non-deterministic planning");

    // Reset the start state
    if (!execute &&
      !arm.traj.empty())
    {
      arm.planner->setStartState(arm.traj.back()->getLastWayPoint());
    } else {
      arm.planner->setStartStateToCurrentState();
    }

    // Hardset to try 5 times
    moveit_cpp::PlanningComponent::PlanSolution plan_solution;
    int count = 0;

    while (!plan_solution && count < option.non_deterministic_max_attempts) {
      arm.planner->setGoal(pose, ee_link);
      plan_solution = arm.planner->plan();  // PlanningComponent::PlanSolution
      count++;
    }

    if (plan_solution) {
      traj = plan_solution.trajectory;
      result = true;
    }
  }

  // All strategies failed, exiting
  if (!result) {
    return false;
  }

  // Execute immediately
  if (execute) {
    RCLCPP_INFO(LOGGER, "Sending the trajectory for execution");
    moveit_cpp_->execute(option.planning_group, traj);  // blocked execution
  } else {
    arm.traj.push_back(traj);
  }
  return true;
}

bool MoveitCppGraspExecution::move_to(
  const float & cartesian_step_size,
  const int & backtrack_steps,
  const int & hybrid_max_attempts,
  const int & non_deterministic_max_attempts,
  const std::string & planning_group,
  const geometry_msgs::msg::PoseStamped & pose,
  const std::string & link,
  bool execute)
{
  auto & arm = arms_[planning_group];
  const auto & ee_link = (link.empty() ? arm.default_ee.link : link);

  // Flag for planning
  bool result = false;

  // Set the start state to the last point of the trajectory
  // if immediate execution is not needed
  if (!execute &&
    !arm.traj.empty())
  {
    arm.planner->setStartState(arm.traj.back()->getLastWayPoint());
  } else {
    arm.planner->setStartStateToCurrentState();
  }

  // Strategy 1
  RCLCPP_INFO(
    LOGGER, "\nStarting strategy 1: Cartesian move to destination with 1cm step. "
    "This works well within relatively empty space");
  robot_trajectory::RobotTrajectoryPtr traj;
  auto fraction = cartesian_to(
    planning_group, *arm.planner->getStartState(), {pose.pose},
    traj, link, cartesian_step_size, 0);

  RCLCPP_INFO(LOGGER, "fraction: %f", fraction);
  result = (fraction == 1.0);

  // Strategy 2
  if (!result && static_cast<int>(traj->getWayPointCount()) > backtrack_steps) {
    RCLCPP_INFO(
      LOGGER,
      "\nStrategy 1 failed :<\n"
      "Starting strategy 2: Back track cartesian path 10cm"
      "to last viable point and start non-deterministic planning");

    arm.planner->setStartState(
      traj->getWayPoint(traj->getWayPointCount() - backtrack_steps));


    moveit_cpp::PlanningComponent::PlanSolution plan_solution;
    int count = 0;

    while (!plan_solution && count < hybrid_max_attempts) {
      arm.planner->setGoal(pose, ee_link);
      plan_solution = arm.planner->plan();  // PlanningComponent::PlanSolution
      count++;
    }

    if (plan_solution) {
      auto temp_traj = *traj;
      traj->clear();
      traj->append(temp_traj, 0, 0, temp_traj.getWayPointCount() - backtrack_steps);
      traj->append(*plan_solution.trajectory, 0, 1);

      trajectory_processing::IterativeParabolicTimeParameterization time_param;
      time_param.computeTimeStamps(*traj, 1.0);
      result = true;
    }
  }

  // Strategy 3
  if (!result) {
    RCLCPP_INFO(
      LOGGER,
      "\nStrategy 1 & 2 failed :<\n"
      "Starting strategy 3: Start over with non-deterministic planning");

    // Reset the start state
    if (!execute &&
      !arm.traj.empty())
    {
      arm.planner->setStartState(arm.traj.back()->getLastWayPoint());
    } else {
      arm.planner->setStartStateToCurrentState();
    }

    // Hardset to try 5 times
    moveit_cpp::PlanningComponent::PlanSolution plan_solution;
    int count = 0;

    while (!plan_solution && count < non_deterministic_max_attempts) {
      arm.planner->setGoal(pose, ee_link);
      plan_solution = arm.planner->plan();  // PlanningComponent::PlanSolution
      count++;
    }

    if (plan_solution) {
      traj = plan_solution.trajectory;
      result = true;
    }
  }

  // All strategies failed, exiting
  if (!result) {
    return false;
  }

  // Execute immediately
  if (execute) {
    RCLCPP_INFO(LOGGER, "Sending the trajectory for execution");
    moveit_cpp_->execute(planning_group, traj);  // blocked execution
  } else {
    arm.traj.push_back(traj);
  }
  return true;
}

bool MoveitCppGraspExecution::move_to(
  const std::string & planning_group,
  const sensor_msgs::msg::JointState & state,
  bool execute)
{
  auto current_state = moveit_cpp_->getCurrentState();

  // JointState messages carry per-variable entries. Use setVariablePosition so
  // that each named variable is updated individually — this handles both
  // single-axis revolute joints and multi-axis joints correctly.
  const auto & var_names = current_state->getRobotModel()->getVariableNames();
  for (size_t i = 0;
    i < state.name.size() && i < state.position.size();
    i++)
  {
    if (std::find(var_names.begin(), var_names.end(), state.name[i]) != var_names.end()) {
      current_state->setVariablePosition(state.name[i], state.position[i]);
    }
  }
  return move_to(5, planning_group, *current_state, execute);
}

bool MoveitCppGraspExecution::move_to(
  const int & non_deterministic_max_attempts,
  const std::string & planning_group,
  const moveit::core::RobotState & state,
  bool execute)
{
  auto & arm = arms_[planning_group];
  // Set the start state to the last point of the trajectory
  // if immediate execution is not needed
  if (!execute &&
    !arm.traj.empty())
  {
    arm.planner->setStartState(arm.traj.back()->getLastWayPoint());
  } else {
    arm.planner->setStartStateToCurrentState();
  }

  // Hardset to try 5 times
  moveit_cpp::PlanningComponent::PlanSolution plan_solution;
  int count = 0;

  while (!plan_solution && count < non_deterministic_max_attempts) {
    arm.planner->setGoal(state);
    plan_solution = arm.planner->plan();  // PlanningComponent::PlanSolution
    count++;
  }

  if (plan_solution) {
    // Execute immediately
    if (execute) {
      RCLCPP_INFO(LOGGER, "Sending the trajectory for execution");
      return this->execute(plan_solution.trajectory);  // blocked execution
    } else {
      arm.traj.push_back(plan_solution.trajectory);
      return true;
    }
  } else {
    return false;
  }
}

bool MoveitCppGraspExecution::move_to(
  const int & non_deterministic_max_attempts,
  const std::string & planning_group,
  const std::string & named_state,
  bool execute)
{
  auto & arm = arms_[planning_group];
  // Set the start state to the last point of the trajectory
  // if immediate execution is not needed
  if (!execute &&
    !arm.traj.empty())
  {
    arm.planner->setStartState(arm.traj.back()->getLastWayPoint());
  } else {
    arm.planner->setStartStateToCurrentState();
  }

  // Hardset to try 5 times
  moveit_cpp::PlanningComponent::PlanSolution plan_solution;
  int count = 0;

  while (!plan_solution && count < non_deterministic_max_attempts) {
    arm.planner->setGoal(named_state);
    plan_solution = arm.planner->plan();
    count++;
  }

  if (plan_solution) {
    // Execute immediately
    if (execute) {
      RCLCPP_INFO(LOGGER, "Sending the trajectory for execution");
      return arm.planner->execute(true);  // blocked execution
    } else {
      arm.traj.push_back(plan_solution.trajectory);
      return true;
    }
  } else {
    return false;
  }
}

bool MoveitCppGraspExecution::cartesian_to(
  const std::string & planning_group,
  const std::vector<geometry_msgs::msg::Pose> & _waypoints,
  const std::string & _link, double step, double jump_threshold,
  bool execute)
{
  return cartesian_to(
    planning_group, _waypoints, _link, step, jump_threshold,
    CartesianPlanningOptions(), execute);
}

bool MoveitCppGraspExecution::cartesian_to(
  const std::string & planning_group,
  const std::vector<geometry_msgs::msg::Pose> & _waypoints,
  const std::string & _link, double step, double jump_threshold,
  const CartesianPlanningOptions & options,
  bool execute)
{
  auto & arm = arms_[planning_group];
  robot_trajectory::RobotTrajectoryPtr rt;

  // Get start state
  moveit::core::RobotState start_state(moveit_cpp_->getRobotModel());

  {    // Lock PlanningScene
    moveit_cpp_->getPlanningSceneMonitor()->updateFrameTransforms();

    // Set the start state to the last point of the trajectory
    // if immediate execution is not needed
    if (!execute &&
      !arm.traj.empty())
    {
      start_state = arm.traj.back()->getLastWayPoint();
    } else {
      planning_scene_monitor::LockedPlanningSceneRO lscene(
        moveit_cpp_->getPlanningSceneMonitor());
      start_state = lscene->getCurrentState();
    }
  }    // Unlock PlanningScene

  // Start Cartesian Planning
  auto fraction = cartesian_to(
    planning_group, start_state,
    _waypoints, rt, _link, step, jump_threshold, options);

  // Execute cartesian path
  if (fraction < 0.0) {
    RCLCPP_ERROR(LOGGER, "Cartesian planning failed due to invalid input.");
    return false;
  } else if (fraction > 0.0) {
    // Execute immediately
    if (execute) {
      RCLCPP_INFO(LOGGER, "Sending the trajectory for execution");
      moveit_cpp_->execute(planning_group, rt);
    } else {
      arm.traj.push_back(rt);
    }
    return true;
  } else {
    return false;
  }
}

/// Referenced from Move Group capability MoveGroupCartesianPathService::computeService
double MoveitCppGraspExecution::cartesian_to(
  const std::string & planning_group,
  moveit::core::RobotState & start_state,
  const std::vector<geometry_msgs::msg::Pose> & _waypoints,
  robot_trajectory::RobotTrajectoryPtr & traj,
  const std::string & _link, double step, double jump_threshold,
  const CartesianPlanningOptions & options)
{
  double fraction = 0.0;
  CartesianPlanStatus planning_status = CartesianPlanStatus::kNoSolution;

  RCLCPP_INFO(LOGGER, "Received request to compute Cartesian path");

  // Check if planning group valid
  if (const moveit::core::JointModelGroup * jmg =
    start_state.getJointModelGroup(planning_group))
  {
    const std::string & link_name =
      (_link.empty() ? arms_[planning_group].default_ee.link : _link);
    const auto * link_model = start_state.getLinkModel(link_name);

    const auto validation_result = validate_cartesian_request(
      _waypoints.size(), link_model != nullptr, step, jump_threshold);
    if (validation_result.status != CartesianPlanStatus::kOk) {
      RCLCPP_ERROR(LOGGER, "%s", validation_result.message.c_str());
      return -1.0;
    }

    EigenSTL::vector_Isometry3d waypoints(_waypoints.size());

    for (size_t i = 0; i < _waypoints.size(); i++) {
      tf2::fromMsg(_waypoints[i], waypoints[i]);
    }

    const auto constraint_config = build_cartesian_constraint_config(options);
    RCLCPP_INFO(
      LOGGER, "Cartesian options: avoid_collisions=%s path_constraints=%s planning_timeout=%.3fs"
      " max_ik_attempts=%d",
      options.avoid_collisions ? "true" : "false",
      constraint_config.has_path_constraints ? "active" : "none",
      options.planning_timeout, options.max_ik_attempts);
    if (options.planning_timeout > 0.0 || options.max_ik_attempts > 0) {
      RCLCPP_WARN(
        LOGGER,
        "planning_timeout/max_ik_attempts are not directly exposed by MoveIt CartesianInterpolator"
        " in this code path; parameters are accepted for diagnostics and forward compatibility.");
    }

    moveit::core::GroupStateValidityCallbackFn constraint_fn;
    std::unique_ptr<planning_scene_monitor::LockedPlanningSceneRO> ls;
    std::unique_ptr<kinematic_constraints::KinematicConstraintSet> kset;
    auto collision_rejections = std::make_shared<std::atomic<std::size_t>>(0U);
    auto constraint_rejections = std::make_shared<std::atomic<std::size_t>>(0U);
    if (constraint_config.build_constraint_fn) {
      ls.reset(new planning_scene_monitor::LockedPlanningSceneRO(moveit_cpp_->getPlanningSceneMonitor()));
      if (constraint_config.has_path_constraints) {
        kset.reset(new kinematic_constraints::KinematicConstraintSet((*ls)->getRobotModel()));
        kset->add(options.path_constraints, (*ls)->getTransforms());
      }

      constraint_fn = [
        planning_scene =
          (options.avoid_collisions ? static_cast<const planning_scene::PlanningSceneConstPtr &>(*ls)
        .get() : nullptr),
        constraint_set = (kset && !kset->empty() ? kset.get() : nullptr),
        collision_rejections,
        constraint_rejections](moveit::core::RobotState * state,
        const moveit::core::JointModelGroup * group, const double * ik_solution) {
        state->setJointGroupPositions(group, ik_solution);
        state->update();
        const bool is_collision_free =
          !planning_scene || !planning_scene->isStateColliding(*state, group->getName());
        if (!is_collision_free) {
          ++(*collision_rejections);
        }

        const bool satisfies_constraints =
          !constraint_set || constraint_set->decide(*state).satisfied;
        if (!satisfies_constraints) {
          ++(*constraint_rejections);
        }
        return is_collision_free && satisfies_constraints;
      };
    }

    bool global_frame = true;
    std::vector<moveit::core::RobotStatePtr> rstraj;
    fraction = moveit::core::CartesianInterpolator::computeCartesianPath(
      &start_state, jmg, rstraj, link_model, waypoints, global_frame,
      moveit::core::MaxEEFStep(step), moveit::core::JumpThreshold(
        jump_threshold), constraint_fn);

    traj = std::make_shared<robot_trajectory::RobotTrajectory>(
      moveit_cpp_->getPlanningSceneMonitor()->getRobotModel(), planning_group);
    for (const moveit::core::RobotStatePtr & traj_state : rstraj) {
      traj->addSuffixWayPoint(traj_state, 0.0);
    }

    // time trajectory
    // \todo optionally compute timing to move the eef with constant speed
    trajectory_processing::IterativeParabolicTimeParameterization time_param;
    time_param.computeTimeStamps(*traj, 1.0);

    const std::size_t collision_rejection_count = collision_rejections->load();
    const std::size_t constraint_rejection_count = constraint_rejections->load();
    if (fraction <= 0.0) {
      if ((options.avoid_collisions && collision_rejection_count > 0U) ||
        (constraint_config.has_path_constraints && constraint_rejection_count > 0U))
      {
        planning_status = CartesianPlanStatus::kCollisionFilteredOut;
      }
    } else {
      planning_status = CartesianPlanStatus::kOk;
    }

    RCLCPP_INFO(
      LOGGER,
      "Computed Cartesian path with %u points (followed %lf%%). rejection_counts:"
      " collision=%zu constraints=%zu status=%d",
      static_cast<unsigned int>(rstraj.size()), fraction * 100.0,
      collision_rejection_count, constraint_rejection_count, static_cast<int>(planning_status));
  } else {
    RCLCPP_ERROR(LOGGER, "Invalid planning group [%s] for Cartesian planning", planning_group.c_str());
    return -1.0;
  }
  return fraction;
}

bool MoveitCppGraspExecution::move_until_before_collide(
  const std::string & planning_group,
  const geometry_msgs::msg::PoseStamped & pose,
  const std::string & link, double step_size, int max_attempts,
  char axis,
  bool execute)
{
  auto next_waypoint = [&axis, &step_size](tf2::Vector3 & _origin) {
      switch (axis) {
        case 'x':
          _origin.m_floats[0] += step_size;
          break;
        case 'y':
          _origin.m_floats[1] += step_size;
          break;
        case 'z':
          _origin.m_floats[2] += step_size;
          break;
        default:
          return false;
      }
      return true;
    };
  if (pose.header.frame_id != robot_frame_) {
    RCLCPP_ERROR(LOGGER, "Please use global frame for this function");
    return false;
  }

  std::vector<geometry_msgs::msg::Pose> waypoints;

  auto temp_target_pose = pose;

  tf2::Transform base_to_ee;
  tf2::fromMsg(pose.pose, base_to_ee);

  tf2::Transform ee_w_clearance;
  ee_w_clearance.setIdentity();
  if (!next_waypoint(ee_w_clearance.getOrigin())) {
    return false;
  }
  for (int i = 0; i < max_attempts; i++) {
    tf2::toMsg(base_to_ee * ee_w_clearance, temp_target_pose.pose);
    waypoints.push_back(temp_target_pose.pose);
    next_waypoint(ee_w_clearance.getOrigin());
  }

  return cartesian_to(planning_group, waypoints, link, std::abs(step_size / 3), 0, execute);
}

void MoveitCppGraspExecution::attach_object_to_ee(
  const moveit_msgs::msg::CollisionObject & object,
  const std::string & ee_link)
{
  /*
  moveit_msgs::AttachedCollisionObject
  -----------------------------------
  string link_name
  moveit_msgs/CollisionObject object
  string[] touch_links
  trajectory_msgs/JointTrajectory detach_posture
  float64 weight
  */
  geometry_msgs::msg::PoseStamped ee_pose;
  geometry_msgs::msg::PoseStamped object_pose;

  object_pose.header.frame_id = object.header.frame_id;
  object_pose.pose = object.pose;

  to_frame(object_pose, ee_pose, ee_link);

  moveit_msgs::msg::AttachedCollisionObject attached_object;
  attached_object.link_name = ee_link;

  attached_object.object = object;
  attached_object.object.pose = ee_pose.pose;
  attached_object.object.operation = attached_object.object.ADD;

  // Attach object to ee_link
  {    // Lock PlanningScene
    planning_scene_monitor::LockedPlanningSceneRW scene(moveit_cpp_->getPlanningSceneMonitor());
    scene->processAttachedCollisionObjectMsg(attached_object);
  }    // Unlock PlanningScene
}

void MoveitCppGraspExecution::attach_object_to_ee(
  const std::string & target_id,
  const std::string & ee_link)
{
  moveit_msgs::msg::AttachedCollisionObject attach_object;

  attach_object.object.id = target_id;
  attach_object.link_name = ee_link;
  attach_object.object.operation = moveit_msgs::msg::CollisionObject::ADD;

  // Attach object to ee_link
  {    // Lock PlanningScene
    planning_scene_monitor::LockedPlanningSceneRW scene(moveit_cpp_->getPlanningSceneMonitor());
    scene->processAttachedCollisionObjectMsg(attach_object);
  }    // Unlock PlanningScene
}

void MoveitCppGraspExecution::detach_object_from_ee(
  const moveit_msgs::msg::CollisionObject & object,
  const std::string & /*ee_link*/)
{
  /*
  moveit_msgs::AttachedCollisionObject
  -----------------------------------
  string link_name
  moveit_msgs/CollisionObject object
  string[] touch_links
  trajectory_msgs/JointTrajectory detach_posture
  float64 weight
  */

  moveit_msgs::msg::AttachedCollisionObject detach_object;

  detach_object.object.id = object.id;
  detach_object.object.operation = object.REMOVE;

  // Add object to planning scene
  {    // Lock PlanningScene
    planning_scene_monitor::LockedPlanningSceneRW scene(moveit_cpp_->getPlanningSceneMonitor());
    scene->processAttachedCollisionObjectMsg(detach_object);
  }    // Unlock PlanningScene
}


void MoveitCppGraspExecution::detach_object_from_ee(
  const std::string & target_id,
  const std::string & ee_link)
{
  moveit_msgs::msg::AttachedCollisionObject detach_object;

  detach_object.object.id = target_id;
  detach_object.link_name = ee_link;
  detach_object.object.operation = moveit_msgs::msg::CollisionObject::REMOVE;

  // Attach object to ee_link
  {    // Lock PlanningScene
    planning_scene_monitor::LockedPlanningSceneRW scene(moveit_cpp_->getPlanningSceneMonitor());
    scene->processAttachedCollisionObjectMsg(detach_object);
  }    // Unlock PlanningScene
}

void MoveitCppGraspExecution::remove_object(
  const std::string & target_id)
{
  moveit_msgs::msg::CollisionObject object;
  object.id = target_id;
  object.operation = moveit_msgs::msg::CollisionObject::REMOVE;

  // Add object to planning scene
  {    // Lock PlanningScene
    planning_scene_monitor::LockedPlanningSceneRW scene(moveit_cpp_->getPlanningSceneMonitor());
    scene->processCollisionObjectMsg(object);

    auto & acm = scene->getAllowedCollisionMatrixNonConst();
    if (acm.hasEntry(target_id)) {
      acm.removeEntry(target_id);
    }
  }    // Unlock PlanningScene
}

bool MoveitCppGraspExecution::execute(
  const robot_trajectory::RobotTrajectoryPtr & traj,
  const std::string & method)
{
  if (method == "default" || method.empty()) {
    // Use default execution method
    return this->default_executor_->run(*traj);
  } else {
    const auto & group = traj->getGroupName();
    if (arms_.find(group) != arms_.end()) {
      auto & arm = arms_[group];
      if (arm.executors.find(method) != arm.executors.end()) {
        // Use customized execution method (per group)
        return arm.executors[method]->run(*traj);
      } else {
        RCLCPP_ERROR(
          LOGGER,
          "Method [%s] not found for group [%s]. "
          "Abort execution",
          method.c_str(), group.c_str());
        return false;
      }
    } else {
      RCLCPP_ERROR(
        LOGGER,
        "Group [%s] not initialized, cannot execute using custom method [%s]"
        "Abort execution",
        group.c_str(), method.c_str());
      return false;
    }
  }
}

void MoveitCppGraspExecution::cancel_execution(
  const std::string & group,
  const std::string & method)
{
  if (method == "default" || method.empty()) {
    if (default_executor_) {
      default_executor_->cancel();
    }
  } else {
    auto arm_it = arms_.find(group);
    if (arm_it != arms_.end()) {
      auto & arm = arm_it->second;
      auto exe_it = arm.executors.find(method);
      if (exe_it != arm.executors.end()) {
        exe_it->second->cancel();
      } else {
        RCLCPP_WARN(
          LOGGER,
          "Method [%s] not found for group [%s], cannot cancel execution",
          method.c_str(), group.c_str());
      }
    } else {
      RCLCPP_WARN(
        LOGGER,
        "Group [%s] not initialized, cannot cancel execution",
        group.c_str());
    }
  }
}

bool MoveitCppGraspExecution::squash_and_execute(
  const std::string & group,
  const std::string & method,
  const double velocity)
{
  auto & arm = arms_[group];
  squash_trajectories(group, velocity);
  while (!arm.traj.empty()) {
    auto traj = arm.traj.front();
    auto result = this->execute(traj, method);
    // this->print_trajectory_ros(LOGGER, traj);
    arm.traj.pop_front();
    if (!result) {
      return false;
    }
  }

  return true;
}

void MoveitCppGraspExecution::squash_trajectories(
  const std::string & planning_group,
  const double velocity,
  int start_idx, int end_idx,
  bool time_parameterization)
{
  auto & trajs = arms_[planning_group].traj;
  if (trajs.empty()) {
    return;
  }

  const size_t trajs_size = trajs.size();
  const int max_index = static_cast<int>(trajs_size - 1);
  int clamped_start = start_idx < 0 ? 0 : start_idx;
  int clamped_end = (end_idx == -1) ? max_index : end_idx;
  if (clamped_end < 0) {
    clamped_end = 0;
  } else if (clamped_end > max_index) {
    clamped_end = max_index;
  }
  if (clamped_start > max_index) {
    return;
  }
  const size_t start = static_cast<size_t>(clamped_start);
  const size_t end = static_cast<size_t>(clamped_end);
  const size_t size = trajs_size;
  if (start <= end) {
    RCLCPP_INFO(LOGGER, "Squashing trajectory...");
    while (trajs.size() > size - (end - start)) {
      trajs[start]->append(*trajs[start + 1], 0, 1);
      trajs.erase(trajs.begin() + start + 1);
    }
    RCLCPP_INFO(LOGGER, "Redo time parameterization...");
    if (time_parameterization) {
      // Strategy 1: TimeOptimization
      RCLCPP_INFO(LOGGER, "\nStrategy 1: Time Optimization");
      auto temp_traj = trajs[start];
      bool validity;
      trajectory_processing::TimeOptimalTrajectoryGeneration time_op_param;
      time_op_param.computeTimeStamps(*temp_traj, velocity);

      // TimeOptimization will cause waypoint to change
      // Thus we need to check the new trajectory collision validity
      {   // Lock PlanningScene
        planning_scene_monitor::LockedPlanningSceneRO ls(
          moveit_cpp_->getPlanningSceneMonitor());
        validity = ls->isPathValid(*temp_traj, planning_group, true);
      }   // Unlock PlanningScene

      if (validity) {
        trajs[start] = std::move(temp_traj);
        RCLCPP_INFO(LOGGER, "\nStrategy 1 succeeded!!");
      } else {
        // Strategy 2: Good old IterativeParabolicTimeParameterization
        RCLCPP_INFO(
          LOGGER, "\nStrategy 1 failed:<.\n"
          "Strategy 2: Good old IterativeParabolicTimeParameterization");
        trajectory_processing::IterativeParabolicTimeParameterization ip_time_param;
        ip_time_param.computeTimeStamps(*trajs[start], velocity);
      }
    }
  }
}

void MoveitCppGraspExecution::print_trajectory(
  const robot_trajectory::RobotTrajectoryPtr & traj,
  std::ostream & _out)
{
  _out << "Robot Trajectory:" << std::endl;
  _out << "Plannning Group: " << traj->getGroupName() << std::endl;
  _out << "Total waypoints: " << traj->getWayPointCount() << std::endl;

  _out << "Joints: ";
  for (const auto & joint : traj->getGroup()->getVariableNames()) {
    _out << joint << '\t';
  }

  _out << std::endl;

  _out << "t\t\tpos\t\tvel\t\taccel" << std::endl;
  int dof = traj->getGroup()->getVariableCount();
  const std::vector<int> & idx = traj->getGroup()->getVariableIndexList();
  for (size_t i = 0; i < traj->getWayPointCount(); i++) {
    _out << traj->getWayPointDurationFromStart(i) << "\t";
    _out << "|\t";
    for (int j = 0; j < dof; j++) {
      _out << traj->getWayPointPtr(i)->getVariablePosition(idx[j]) << "\t";
    }
    if (traj->getWayPointPtr(i)->hasVelocities()) {
      _out << "|\t";
      for (int j = 0; j < dof; j++) {
        _out << traj->getWayPointPtr(i)->getVariableVelocity(idx[j]) << "\t";
      }
    }
    if (traj->getWayPointPtr(i)->hasAccelerations()) {
      _out << "|\t";
      for (int j = 0; j < dof; j++) {
        _out << traj->getWayPointPtr(i)->getVariableAcceleration(idx[j]) << "\t";
      }
    }
    _out << std::endl;
  }
}

}  // namespace moveit2

}  // namespace grasp_execution
