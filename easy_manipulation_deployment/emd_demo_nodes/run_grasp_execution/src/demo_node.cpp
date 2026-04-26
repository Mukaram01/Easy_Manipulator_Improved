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

#include <chrono>
#include <memory>
#include <string>
#include <utility>
#include <vector>
#include <algorithm>
#include <filesystem>
#include <sstream>
#include <thread>
#include <set>
#include <regex>
#include <stdexcept>
#include <unordered_set>
#include <cctype>
#include <cmath>
#include <mutex>

#include <Eigen/Geometry>
#include <boost/algorithm/string/join.hpp>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "emd/grasp_execution/moveit2/moveit_cpp_if.hpp"
#include "emd/grasp_execution/utils.hpp"
#include "emd/end_effector/ee_execution_interface.hpp"

#include "emd_msgs/msg/grasp_task.hpp"
#include "emd_msgs/srv/grasp_request.hpp"

#include "moveit/macros/console_colors.h"
#include "sensor_msgs/msg/joint_state.hpp"
#include "moveit/collision_detection/collision_common.h"
#include "moveit/planning_scene_monitor/planning_scene_monitor.h"
#include "tf2_eigen/tf2_eigen.hpp"
#include "run_grasp_execution/grasp_precheck_collision_filter.hpp"


namespace grasp_execution
{

static const char GRASP_TASK_TOPIC[] = "grasp_tasks";

static const char GRASP_REQUEST_TOPIC[] = "grasp_requests";

static const char GRASP_EXECUTION_PACKAGE[] = "emd_grasp_execution";


static const std::vector<std::string> REQUIRED_FINGER_JOINTS = {
  "palm_finger_1_joint",
  "finger_1_joint_1",
  "finger_1_joint_2",
  "finger_1_joint_3",
  "palm_finger_2_joint",
  "finger_2_joint_1",
  "finger_2_joint_2",
  "finger_2_joint_3",
  "finger_middle_joint_1",
  "finger_middle_joint_2",
  "finger_middle_joint_3",
};

static const std::vector<std::string> REQUIRED_FINGER_LINK_FRAMES = {
  "finger_1_link_0",
  "finger_1_link_1",
  "finger_1_link_2",
  "finger_1_link_3",
  "finger_2_link_0",
  "finger_2_link_1",
  "finger_2_link_2",
  "finger_2_link_3",
  "finger_middle_link_0",
  "finger_middle_link_1",
  "finger_middle_link_2",
  "finger_middle_link_3",
};

struct GraspReadinessProfile
{
  bool require_robotiq_3f_checks{false};
  std::string description{"arm-only/non-finger"};
};

std::string to_lower_copy(std::string value)
{
  std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  return value;
}

bool is_robotiq_3f_end_effector(
  const std::string & ee_name,
  const grasp_execution::WorkcellContext::Gripper & ee)
{
  const std::string ee_name_lower = to_lower_copy(ee_name);
  const std::string ee_brand_lower = to_lower_copy(ee.brand);

  return ee_name_lower.find("robotiq_3f") != std::string::npos ||
         ee_brand_lower.find("robotiq_3f") != std::string::npos ||
         ee_brand_lower.find("3f_gripper") != std::string::npos;
}

GraspReadinessProfile detect_grasp_readiness_profile(
  const grasp_execution::WorkcellContext & workcell_context)
{
  std::vector<std::string> declared_ees;
  std::vector<std::string> robotiq_3f_ees;
  for (const auto & group : workcell_context.groups) {
    for (const auto & ee : group.second.end_effectors) {
      declared_ees.push_back(
        group.first + "/" + ee.first + " (brand=" + ee.second.brand +
        ", link=" + ee.second.link + ")");
      if (is_robotiq_3f_end_effector(ee.first, ee.second)) {
        robotiq_3f_ees.push_back(group.first + "/" + ee.first);
      }
    }
  }

  if (!robotiq_3f_ees.empty()) {
    std::ostringstream oss;
    for (size_t i = 0; i < robotiq_3f_ees.size(); ++i) {
      if (i > 0) {
        oss << ", ";
      }
      oss << robotiq_3f_ees[i];
    }
    return {true, "robotiq-3f finger (" + oss.str() + ")"};
  }

  if (declared_ees.empty()) {
    return {false, "arm-only/no declared end-effector"};
  }

  std::ostringstream oss;
  for (size_t i = 0; i < declared_ees.size(); ++i) {
    if (i > 0) {
      oss << ", ";
    }
    oss << declared_ees[i];
  }
  return {false, "non-3f end-effector(s): " + oss.str()};
}

bool wait_for_required_grasp_readiness(
  const rclcpp::Node::SharedPtr & node,
  const std::string & planning_frame,
  bool require_robotiq_3f_checks,
  const std::string & readiness_profile_description,
  std::chrono::seconds timeout = std::chrono::seconds(15))
{
  if (!require_robotiq_3f_checks) {
    RCLCPP_INFO(
      node->get_logger(),
      "Skipping Robotiq 3F startup readiness checks. Active readiness profile: %s.",
      readiness_profile_description.c_str());
    return true;
  }

  const std::string normalized_planning_frame = grasp_execution::sanitize_frame_id(planning_frame);
  if (normalized_planning_frame.empty()) {
    RCLCPP_ERROR(
      node->get_logger(),
      "Configured planning_frame is empty after trimming whitespace.");
    return false;
  }

  std::set<std::string> missing_joints(
    REQUIRED_FINGER_JOINTS.begin(), REQUIRED_FINGER_JOINTS.end());
  std::set<std::string> missing_frames(
    REQUIRED_FINGER_LINK_FRAMES.begin(), REQUIRED_FINGER_LINK_FRAMES.end());

  auto joint_state_sub = node->create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states", rclcpp::QoS(10),
    [&missing_joints](const sensor_msgs::msg::JointState::SharedPtr msg) {
      for (const auto & joint_name : msg->name) {
        missing_joints.erase(joint_name);
      }
    });

  tf2_ros::Buffer tf_buffer(node->get_clock());
  tf2_ros::TransformListener tf_listener(tf_buffer, node, false);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  const auto end_time = std::chrono::steady_clock::now() + timeout;
  auto next_progress_log = std::chrono::steady_clock::now() + std::chrono::seconds(1);
  while (std::chrono::steady_clock::now() < end_time) {
    executor.spin_some();

    for (auto it = missing_frames.begin(); it != missing_frames.end();) {
      if (tf_buffer.canTransform(normalized_planning_frame, *it, tf2::TimePointZero)) {
        it = missing_frames.erase(it);
      } else {
        ++it;
      }
    }

    if (missing_joints.empty() && missing_frames.empty()) {
      RCLCPP_INFO(node->get_logger(),
        "Readiness checks passed: all required finger joints and %s->finger_*_link_* TF frames are available.",
        normalized_planning_frame.c_str());
      return true;
    }

    if (std::chrono::steady_clock::now() >= next_progress_log) {
      RCLCPP_WARN(
        node->get_logger(),
        "Waiting for grasp readiness: missing_joints=%zu, missing_frames=%zu",
        missing_joints.size(), missing_frames.size());
      next_progress_log += std::chrono::seconds(1);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  std::ostringstream missing_joint_stream;
  bool first = true;
  for (const auto & joint_name : missing_joints) {
    if (!first) {
      missing_joint_stream << ", ";
    }
    missing_joint_stream << joint_name;
    first = false;
  }
  std::ostringstream missing_frame_stream;
  first = true;
  for (const auto & frame_name : missing_frames) {
    if (!first) {
      missing_frame_stream << ", ";
    }
    missing_frame_stream << frame_name;
    first = false;
  }
  RCLCPP_ERROR(
    node->get_logger(),
    "Timed out waiting for grasp readiness after %.1f s. Missing joints on /joint_states: [%s]. "
    "Missing TF transforms from %s->finger_*_link_*: [%s].",
    std::chrono::duration<double>(timeout).count(),
    missing_joint_stream.str().c_str(),
    normalized_planning_frame.c_str(),
    missing_frame_stream.str().c_str());
  return false;
}

bool validate_required_finger_link_frames_in_robot_description(
  const rclcpp::Node::SharedPtr & node,
  bool require_robotiq_3f_checks,
  const std::string & readiness_profile_description)
{
  if (!require_robotiq_3f_checks) {
    RCLCPP_INFO(
      node->get_logger(),
      "Skipping URDF Robotiq 3F finger-frame validation. Active readiness profile: %s.",
      readiness_profile_description.c_str());
    return true;
  }

  std::string robot_description;
  if (!node->get_parameter("robot_description", robot_description) || robot_description.empty()) {
    RCLCPP_ERROR(
      node->get_logger(),
      "robot_description parameter is missing; cannot validate finger link names.");
    return false;
  }

  std::unordered_set<std::string> urdf_links;
  static const std::regex link_regex(R"(<link\s+name\s*=\s*['"]([^'"]+)['"])", std::regex::icase);
  for (std::sregex_iterator it(robot_description.begin(), robot_description.end(), link_regex), end;
    it != end; ++it)
  {
    urdf_links.insert((*it)[1].str());
  }

  std::vector<std::string> missing_links;
  for (const auto & required_frame : REQUIRED_FINGER_LINK_FRAMES) {
    if (!urdf_links.count(required_frame)) {
      missing_links.push_back(required_frame);
    }
  }

  if (!missing_links.empty()) {
    std::ostringstream oss;
    for (size_t i = 0; i < missing_links.size(); ++i) {
      if (i > 0) {
        oss << ", ";
      }
      oss << missing_links[i];
    }
    RCLCPP_ERROR(
      node->get_logger(),
      "Finger link frame mismatch detected. Required frames missing from URDF: [%s].",
      oss.str().c_str());
    return false;
  }

  RCLCPP_INFO(
    node->get_logger(),
    "Finger link frame names match URDF links for the configured gripper model.");
  return true;
}

struct WorkspaceBounds
{
  bool enabled{false};
  double min_x{-1.0};
  double max_x{1.0};
  double min_y{-1.0};
  double max_y{1.0};
  double min_z{0.0};
  double max_z{1.5};
};

bool pose_within_workspace(
  const geometry_msgs::msg::PoseStamped & pose,
  const WorkspaceBounds & bounds)
{
  if (!bounds.enabled) {
    return true;
  }
  const auto & p = pose.pose.position;
  return (p.x >= bounds.min_x && p.x <= bounds.max_x) &&
         (p.y >= bounds.min_y && p.y <= bounds.max_y) &&
         (p.z >= bounds.min_z && p.z <= bounds.max_z);
}

bool is_valid_finite(double value)
{
  return std::isfinite(value);
}

std::string format_pose_xyz_quat_rpy(const geometry_msgs::msg::Pose & pose)
{
  tf2::Quaternion q_tmp{
    pose.orientation.x,
    pose.orientation.y,
    pose.orientation.z,
    pose.orientation.w};
  double yaw{0.0}, pitch{0.0}, roll{0.0};
  tf2::impl::getEulerYPR(q_tmp, yaw, pitch, roll);
  std::ostringstream oss;
  oss << "xyz=[" << pose.position.x << ", " << pose.position.y << ", " << pose.position.z <<
    "] quat=[" << pose.orientation.x << ", " << pose.orientation.y << ", " << pose.orientation.z <<
    ", " << pose.orientation.w << "] rpy=[" << roll << ", " << pitch << ", " << yaw << "]";
  return oss.str();
}

class Demo : public moveit2::MoveitCppGraspExecution
{
public:
  explicit Demo(
    const rclcpp::Node::SharedPtr & node,
    [[maybe_unused]] const std::string & package_name,
    const std::string & grasp_task_topic,
    const std::string & grasp_request_topic)
  : MoveitCppGraspExecution(node, 1, 1),
    node_(node)
  {
    end_effector_interface_ = std::make_shared<emd::EndEffectorExecutioninterface>();
    ee_context_.wait_for_completion = node_->declare_parameter<bool>(
      "ee_wait_for_completion", false);
    int delay_ms = node_->declare_parameter<int>("ee_post_command_delay_ms", 0);
    ee_context_.post_command_delay = std::chrono::milliseconds(delay_ms);
    grasp_execution::declare_or_get_param<std::string>(
      planning_frame_, "planning_frame", node, node->get_logger(), "world");
    planning_frame_ = grasp_execution::sanitize_frame_id(planning_frame_);
    if (planning_frame_.empty()) {
      throw std::runtime_error("Parameter 'planning_frame' is empty after trimming whitespace.");
    }

    // Configurable release pose: x-offset from the current EE position and
    // optional z-height override. Defaults preserve the original behaviour.
    // Use declare_or_get_param to avoid ParameterAlreadyDeclaredException when
    // these parameters are pre-declared by automatically_declare_parameters_from_overrides.
    grasp_execution::declare_or_get_param<double>(
      release_x_offset_, "release_x_offset", node, node->get_logger(), -0.3);
    grasp_execution::declare_or_get_param<bool>(
      release_use_grasp_z_, "release_use_grasp_z", node, node->get_logger(), true);
    grasp_execution::declare_or_get_param<bool>(
      workspace_bounds_.enabled,
      "grasp_workspace_filter.enabled",
      node,
      node->get_logger(),
      false);
    grasp_execution::declare_or_get_param<double>(
      workspace_bounds_.min_x,
      "grasp_workspace_filter.min_x",
      node,
      node->get_logger(),
      -1.0);
    grasp_execution::declare_or_get_param<double>(
      workspace_bounds_.max_x,
      "grasp_workspace_filter.max_x",
      node,
      node->get_logger(),
      1.0);
    grasp_execution::declare_or_get_param<double>(
      workspace_bounds_.min_y,
      "grasp_workspace_filter.min_y",
      node,
      node->get_logger(),
      -1.0);
    grasp_execution::declare_or_get_param<double>(
      workspace_bounds_.max_y,
      "grasp_workspace_filter.max_y",
      node,
      node->get_logger(),
      1.0);
    grasp_execution::declare_or_get_param<double>(
      workspace_bounds_.min_z,
      "grasp_workspace_filter.min_z",
      node,
      node->get_logger(),
      0.0);
    grasp_execution::declare_or_get_param<double>(
      workspace_bounds_.max_z,
      "grasp_workspace_filter.max_z",
      node,
      node->get_logger(),
      1.5);
    if (node_->has_parameter("grasp_precheck_allowed_touch_links")) {
      node_->get_parameter_or<std::vector<std::string>>(
        "grasp_precheck_allowed_touch_links",
        grasp_precheck_allowed_touch_links_,
        std::vector<std::string>{});
    } else {
      grasp_precheck_allowed_touch_links_ = node_->declare_parameter<std::vector<std::string>>(
        "grasp_precheck_allowed_touch_links",
        {"gripper_finger1_finger_tip_link", "gripper_finger2_finger_tip_link"});
    }
    if (node_->has_parameter("grasp_precheck_allowed_collision_ids")) {
      node_->get_parameter_or<std::vector<std::string>>(
        "grasp_precheck_allowed_collision_ids",
        grasp_precheck_allowed_collision_ids_,
        std::vector<std::string>{});
    } else {
      grasp_precheck_allowed_collision_ids_ = node_->declare_parameter<std::vector<std::string>>(
        "grasp_precheck_allowed_collision_ids", {"<octomap>"});
    }
    RCLCPP_INFO(
      node_->get_logger(),
      "Configured grasp_precheck_allowed_touch_links: [%s]",
      boost::algorithm::join(grasp_precheck_allowed_touch_links_, ", ").c_str());
    RCLCPP_INFO(
      node_->get_logger(),
      "Configured grasp_precheck_allowed_collision_ids: [%s]",
      boost::algorithm::join(grasp_precheck_allowed_collision_ids_, ", ").c_str());

    grasp_task_sub_ = node_->create_subscription<emd_msgs::msg::GraspTask>(
      grasp_task_topic, 10,
      [ = ](emd_msgs::msg::GraspTask::UniquePtr msg) {
        order_schedule(std::move(msg));
      });

    grasp_req_service_ = node_->create_service<emd_msgs::srv::GraspRequest>(
      grasp_request_topic,
      [ = ](
        const std::shared_ptr<rmw_request_id_t> req_header,
        const std::shared_ptr<emd_msgs::srv::GraspRequest::Request> req,
        const std::shared_ptr<emd_msgs::srv::GraspRequest::Response> res) -> void
      {
        (void)req_header;
        auto task = std::make_unique<emd_msgs::msg::GraspTask>();
        task->task_id = gen_uuid();
        task->grasp_targets = req->grasp_targets;
        const bool success = order_schedule(std::move(task), true);
        res->success = success;
        res->message = success ? "Execution completed successfully." : get_and_clear_last_failure_message();
      });
  }

  bool order_schedule(
    const emd_msgs::msg::GraspTask::SharedPtr & msg,
    bool blocking = false)
  {
    bool all_targets_success = true;
    // target id will be "#<shape>-<task_id>-<target-index>"

    // ------------------- Prepare object for grasping --------------------------
    for (size_t i = 0; i < msg->grasp_targets.size(); i++) {
      register_target_object(
        msg->grasp_targets[i].target_shape,
        msg->grasp_targets[i].target_pose,
        i,
        msg->task_id);
    }
    // -------------------------------------------------------------

    for (size_t i = 0; i < msg->grasp_targets.size(); i++) {
      auto grasp_target = std::make_shared<emd_msgs::msg::GraspTarget>(msg->grasp_targets[i]);

      auto target_id =
        gen_target_object_id(grasp_target->target_shape, msg->task_id, i);

      // Start planning workflow using planning schedule
      auto status = planning_scheduler.add_workflow(
        target_id,
        std::bind(
          &Demo::planning_workflow, this,
          std::move(grasp_target),
          std::placeholders::_1));

      // Check status of workflow
      check_status(status, target_id);

      if (blocking) {
        // Wait for the job to finish
        bool result = false;
        planning_scheduler.wait_till_complete(target_id, result);
        all_targets_success = all_targets_success && result;
      }
    }
    return all_targets_success;
  }

  void check_status(
    core::Workflow::Status status,
    std::string target_id)
  {
    switch (status) {
      case core::Workflow::Status::ONGOING:
        RCLCPP_INFO(
          node_->get_logger(),
          MOVEIT_CONSOLE_COLOR_YELLOW
          "New job [%s] started!!"
          MOVEIT_CONSOLE_COLOR_RESET, target_id.c_str());
        break;
      case core::Workflow::Status::QUEUED:
        RCLCPP_INFO(
          node_->get_logger(),
          MOVEIT_CONSOLE_COLOR_YELLOW
          "New job [%s] started!!"
          "No available planning worker, new job in queue."
          MOVEIT_CONSOLE_COLOR_RESET, target_id.c_str());
        break;
      case core::Workflow::Status::INVALID:
        RCLCPP_INFO(
          node_->get_logger(),
          MOVEIT_CONSOLE_COLOR_RED
          "New job [%s] is invalid, it could be already completed, ongoing or queued."
          "You can check with get_status(<workflow-id>), or use another <workflow-id>"
          MOVEIT_CONSOLE_COLOR_RESET, target_id.c_str());
        break;
      default:
        break;
    }
  }

  bool planning_workflow(
    const emd_msgs::msg::GraspTarget::SharedPtr & target,
    const std::string & target_id)
  {
    // Get home state
    moveit::core::RobotStatePtr home_state(get_curr_state());
    if (!home_state) {
      RCLCPP_ERROR(
        node_->get_logger(),
        "Failed to get current robot state for target %s",
        target_id.c_str());
      return false;
    }

    // Select grasp method based on end effector availability
    double clearance = 0.0;
    std::string moveit_link = "";
    std::string grasp_frame = "";
    std::string planning_group = "";

    const emd_msgs::msg::GraspMethod *selected_method = nullptr;
    for (const auto & method : target->grasp_methods) {
      for (auto & group : get_workcell_context().groups) {
        for (auto & ee : group.second.end_effectors) {
          if (ee.second.brand == method.ee_id) {
            selected_method = &method;
            moveit_link = ee.second.link;
            grasp_frame = ee.second.grasp_frame.empty() ? ee.second.link : ee.second.grasp_frame;
            clearance = ee.second.clearance;
            planning_group = group.first;
            break;
          }
        }
        if (selected_method) {
          break;
        }
      }
      if (selected_method) {
        break;
      }
    }

    if (!selected_method) {
      RCLCPP_ERROR(
        node_->get_logger(),
        "No valid end effector found for target %s",
        target_id.c_str());
      return false;
    }

    const auto & grasp_method = *selected_method;
    const std::string & ee_brand = grasp_method.ee_id;

    grasp_execution::GraspExecutionContext options;
    options.world_frame = planning_frame_;
    options.planning_group = planning_group;
    options.ee_link = moveit_link;
    options.move_to_collide_step_size = node_->get_parameter(
      "planning_strategy.cartesian_planning.collide_step_length").as_double();
    options.cartesian_step_size = static_cast<float>(node_->get_parameter(
        "planning_strategy.cartesian_planning.move_step_length").as_double());
    options.backtrack_steps = node_->get_parameter(
      "planning_strategy.cartesian_non_deterministic_hybrid.backtrack_steps").as_int();
    options.hybrid_max_attempts = node_->get_parameter(
      "planning_strategy.cartesian_non_deterministic_hybrid.max_planning_tries").as_int();
    options.non_deterministic_max_attempts = node_->get_parameter(
      "planning_strategy.non_deterministic.max_planning_tries").as_int();
    options.clearance = clearance;

    // Exit if brand name not found.
    if (moveit_link.empty()) {
      RCLCPP_ERROR(node_->get_logger(), "End effector brand: %s", ee_brand.c_str());
    }

    geometry_msgs::msg::PoseStamped release_pose;
    try {
      release_pose = get_curr_pose(moveit_link);
    } catch (const std::exception & ex) {
      RCLCPP_ERROR(node_->get_logger(), "Failed to get current pose for target %s: %s",
        target_id.c_str(), ex.what());
      return false;
    }

    if (grasp_method.grasp_poses.empty()) {
      RCLCPP_ERROR(
        node_->get_logger(),
        "No grasp poses provided by grasp method for target %s (ee: %s).",
        target_id.c_str(), ee_brand.c_str());
      return false;
    }

    bool result = false;
    geometry_msgs::msg::PoseStamped selected_moveit_grasp_pose;
    bool selected_moveit_grasp_pose_valid = false;
    bool any_candidate_passed_precheck = false;

    for (size_t pose_index = 0; pose_index < grasp_method.grasp_poses.size(); ++pose_index) {
      if (!rclcpp::ok()) {
        set_last_failure_message("Execution interrupted during candidate evaluation.");
        return false;
      }
      const auto & grasp_pose = grasp_method.grasp_poses[pose_index];
      geometry_msgs::msg::PoseStamped moveit_goal_pose;
      std::string rejection_reason;
      const bool precheck_ok = precheck_and_prepare_grasp_candidate(
        target_id, pose_index, grasp_method.grasp_poses.size(), planning_group, moveit_link, grasp_frame,
        grasp_pose, moveit_goal_pose, rejection_reason);
      if (!precheck_ok) {
        RCLCPP_WARN(
          node_->get_logger(),
          "Candidate %zu/%zu rejected for target %s: %s",
          pose_index + 1, grasp_method.grasp_poses.size(), target_id.c_str(), rejection_reason.c_str());
        continue;
      }
      any_candidate_passed_precheck = true;

      if (!pose_within_workspace(moveit_goal_pose, workspace_bounds_)) {
        RCLCPP_WARN(
          node_->get_logger(),
          "Rejected candidate %zu/%zu for target %s: outside workspace bounds "
          "[x:(%.3f, %.3f) y:(%.3f, %.3f) z:(%.3f, %.3f)].",
          pose_index + 1,
          grasp_method.grasp_poses.size(),
          target_id.c_str(),
          workspace_bounds_.min_x, workspace_bounds_.max_x,
          workspace_bounds_.min_y, workspace_bounds_.max_y,
          workspace_bounds_.min_z, workspace_bounds_.max_z);
        continue;
      }

      RCLCPP_INFO(
        node_->get_logger(),
        "Candidate %zu/%zu target=%s passed IK/collision precheck. Planning with moveit_link=%s.",
        pose_index + 1, grasp_method.grasp_poses.size(), target_id.c_str(), moveit_link.c_str());
      result = this->plan_and_execute_job(
        options,
        "Grasp location",
        target_id,
        moveit_goal_pose);

      if (result) {
        selected_moveit_grasp_pose = moveit_goal_pose;
        selected_moveit_grasp_pose_valid = true;
        RCLCPP_INFO(
          node_->get_logger(),
          "Grasp planning/execution succeeded for target %s using candidate %zu/%zu.",
          target_id.c_str(), pose_index + 1, grasp_method.grasp_poses.size());
        break;
      }

      RCLCPP_WARN(
        node_->get_logger(),
        "Grasp planning/execution failed for target %s using candidate %zu/%zu. Trying next candidate.",
        target_id.c_str(), pose_index + 1, grasp_method.grasp_poses.size());
    }

    if (!any_candidate_passed_precheck) {
      set_last_failure_message("All grasp pose candidates failed IK/collision precheck.");
      RCLCPP_ERROR(
        node_->get_logger(),
        "All %zu grasp pose candidates failed IK/collision precheck for target %s.",
        grasp_method.grasp_poses.size(), target_id.c_str());
      return false;
    }

    if (!selected_moveit_grasp_pose_valid) {
      set_last_failure_message("Execution failed after candidate planning attempts.");
      RCLCPP_ERROR(
        node_->get_logger(),
        "All %zu grasp pose candidates failed for target %s.",
        grasp_method.grasp_poses.size(), target_id.c_str());
      return false;
    }
    // ------------------- Attach grasp object to robot --------------------------
    prompt_job_start(
      node_->get_logger(), target_id,
      "Attaching to robot ee frame: [" + moveit_link + "]");

    end_effector_interface_->grasp_object(this, moveit_link, target_id, ee_context_);

    prompt_job_end(node_->get_logger(), true);

    // Apply configurable release pose offsets. The x_offset is applied to the
    // current EE position. Optionally the z-height is aligned to the grasp
    // pose so the object clears any surface obstacle on approach.
    release_pose.pose.position.x += release_x_offset_;
    if (release_use_grasp_z_) {
      release_pose.pose.position.z = selected_moveit_grasp_pose.pose.position.z;
    }
    release_pose.pose.orientation = selected_moveit_grasp_pose.pose.orientation;

    if(!this->plan_and_execute_job(
        options,
        "Grasp release",
        target_id,
        release_pose)){
          log_release_octomap_collision_diagnostic(target_id, planning_group);
          return false;
      }

    // ------------------- detach grasp object from robot --------------------------
    prompt_job_start(
      node_->get_logger(), target_id,
      "Detaching from robot ee frame: [" + moveit_link + "]");

    end_effector_interface_->release_object(this, moveit_link, target_id, ee_context_);

    prompt_job_end(node_->get_logger(), true);

    // ------------------- Move back to Home --------------------------
    prompt_job_start(
      node_->get_logger(), target_id,
      "Move back to home");
    result = move_to(
      options.non_deterministic_max_attempts,
      planning_group, *home_state);

    prompt_job_end(node_->get_logger(), result);

    if (!result) {
      return false;
    }
    // -------------------------------------------------------------

    // ------------------ Remove Object from world -------------------

    prompt_job_start(
      node_->get_logger(), target_id,
      "Remove object from world");

    remove_object(target_id);

    result = true;

    prompt_job_end(node_->get_logger(), result);
    return result;
  }

private:
  void set_last_failure_message(const std::string & message)
  {
    std::lock_guard<std::mutex> lock(last_failure_mutex_);
    last_failure_message_ = message;
  }

  std::string get_and_clear_last_failure_message()
  {
    std::lock_guard<std::mutex> lock(last_failure_mutex_);
    const std::string message = last_failure_message_.empty() ? "Execution failed." : last_failure_message_;
    last_failure_message_.clear();
    return message;
  }

  bool precheck_and_prepare_grasp_candidate(
    const std::string & target_id,
    size_t candidate_index,
    size_t candidate_total,
    const std::string & planning_group,
    const std::string & moveit_link,
    const std::string & grasp_frame,
    const geometry_msgs::msg::PoseStamped & original_candidate_pose,
    geometry_msgs::msg::PoseStamped & out_moveit_goal_pose,
    std::string & rejection_reason)
  {
    auto candidate_pose = original_candidate_pose;
    if (!normalize_quaternion(candidate_pose.pose.orientation)) {
      rejection_reason = "invalid candidate quaternion (NaN or zero length).";
      return false;
    }

    geometry_msgs::msg::PoseStamped desired_grasp_frame_pose;
    to_frame(candidate_pose, desired_grasp_frame_pose, this->robot_frame_);
    out_moveit_goal_pose = desired_grasp_frame_pose;

    auto current_state = get_curr_state();
    if (!current_state) {
      rejection_reason = "could not retrieve current robot state for IK precheck.";
      return false;
    }
    const auto robot_model = current_state->getRobotModel();
    const auto * jmg = robot_model->getJointModelGroup(planning_group);
    if (!jmg) {
      rejection_reason = "planning group not found in robot model.";
      return false;
    }

    if (grasp_frame != moveit_link) {
      const auto * moveit_link_model = robot_model->getLinkModel(moveit_link);
      const auto * grasp_frame_model = robot_model->getLinkModel(grasp_frame);
      if (!moveit_link_model || !grasp_frame_model) {
        RCLCPP_WARN(
          node_->get_logger(),
          "Candidate %zu/%zu target=%s: grasp frame conversion unavailable "
          "(moveit_link='%s' exists=%s, grasp_frame='%s' exists=%s). Falling back to legacy behaviour.",
          candidate_index + 1, candidate_total, target_id.c_str(),
          moveit_link.c_str(), moveit_link_model ? "true" : "false",
          grasp_frame.c_str(), grasp_frame_model ? "true" : "false");
      } else {
        const Eigen::Isometry3d world_moveit =
          current_state->getGlobalLinkTransform(moveit_link);
        const Eigen::Isometry3d world_grasp =
          current_state->getGlobalLinkTransform(grasp_frame);
        const Eigen::Isometry3d moveit_to_grasp = world_moveit.inverse() * world_grasp;
        out_moveit_goal_pose.pose = convert_grasp_frame_pose_to_moveit_link_pose(
          desired_grasp_frame_pose.pose, moveit_to_grasp);
      }
    }

    if (!is_pose_finite(out_moveit_goal_pose.pose)) {
      rejection_reason = "transformed moveit goal pose contains non-finite values.";
      return false;
    }

    moveit::core::RobotState ik_state(robot_model);
    ik_state.setToDefaultValues();
    constexpr double kIkTimeoutS = 0.1;
    const bool ik_ok = ik_state.setFromIK(jmg, out_moveit_goal_pose.pose, moveit_link, kIkTimeoutS);

    geometry_msgs::msg::PoseStamped current_moveit_pose;
    try {
      current_moveit_pose = get_curr_pose(moveit_link);
    } catch (const std::exception & ex) {
      rejection_reason = std::string("failed to read current moveit link pose: ") + ex.what();
      return false;
    }
    const double dx = out_moveit_goal_pose.pose.position.x - current_moveit_pose.pose.position.x;
    const double dy = out_moveit_goal_pose.pose.position.y - current_moveit_pose.pose.position.y;
    const double dz = out_moveit_goal_pose.pose.position.z - current_moveit_pose.pose.position.z;
    const double translation_distance = std::sqrt(dx * dx + dy * dy + dz * dz);

    RCLCPP_INFO(
      node_->get_logger(),
      "Candidate diag target=%s idx=%zu/%zu original_frame=%s original_pose=%s moveit_link=%s "
      "grasp_frame=%s transformed_moveit_pose=%s current_to_goal_distance=%.5f ik_ok=%s",
      target_id.c_str(), candidate_index + 1, candidate_total,
      original_candidate_pose.header.frame_id.c_str(),
      format_pose_xyz_quat_rpy(desired_grasp_frame_pose.pose).c_str(),
      moveit_link.c_str(), grasp_frame.c_str(),
      format_pose_xyz_quat_rpy(out_moveit_goal_pose.pose).c_str(),
      translation_distance, ik_ok ? "true" : "false");

    if (!ik_ok) {
      rejection_reason = "IK failed for transformed moveit goal.";
      return false;
    }

    planning_scene_monitor::LockedPlanningSceneRO scene(moveit_cpp_->getPlanningSceneMonitor());
    collision_detection::CollisionRequest req;
    req.contacts = true;
    req.max_contacts = 20;
    req.group_name = planning_group;
    collision_detection::CollisionResult res;
    scene->checkCollision(req, res, ik_state);
    if (res.collision) {
      std::vector<std::pair<std::string, std::string>> collision_pairs;
      for (const auto & contact_pair : res.contacts) {
        collision_pairs.emplace_back(contact_pair.first.first, contact_pair.first.second);
      }
      std::set<std::string> allowed_touch_links(
        grasp_precheck_allowed_touch_links_.begin(), grasp_precheck_allowed_touch_links_.end());
      std::set<std::string> allowed_collision_ids(
        grasp_precheck_allowed_collision_ids_.begin(), grasp_precheck_allowed_collision_ids_.end());
      allowed_collision_ids.insert(target_id);
      allowed_collision_ids.insert("#" + target_id);

      const auto filter_result = run_grasp_execution::filter_grasp_precheck_collision_pairs(
        collision_pairs, allowed_touch_links, allowed_collision_ids);
      if (!filter_result.allowed_pairs.empty() && filter_result.invalid_pairs.empty()) {
        RCLCPP_INFO(
          node_->get_logger(),
          "Candidate %zu/%zu target=%s allowed expected grasp contact: %s",
          candidate_index + 1, candidate_total, target_id.c_str(),
          boost::algorithm::join(filter_result.allowed_pairs, ", ").c_str());
      }
      if (!filter_result.invalid_pairs.empty()) {
        rejection_reason = "IK solution in collision: " +
          boost::algorithm::join(filter_result.invalid_pairs, ", ");
        RCLCPP_WARN(
          node_->get_logger(),
          "Candidate %zu/%zu target=%s rejected: %s",
          candidate_index + 1, candidate_total, target_id.c_str(), rejection_reason.c_str());
        return false;
      }
    }

    return true;
  }

  bool is_pose_finite(const geometry_msgs::msg::Pose & pose) const
  {
    return is_valid_finite(pose.position.x) && is_valid_finite(pose.position.y) &&
           is_valid_finite(pose.position.z) && is_valid_finite(pose.orientation.x) &&
           is_valid_finite(pose.orientation.y) && is_valid_finite(pose.orientation.z) &&
           is_valid_finite(pose.orientation.w);
  }

  void log_release_octomap_collision_diagnostic(
    const std::string & target_id,
    const std::string & planning_group)
  {
    auto current_state = get_curr_state();
    if (!current_state) {
      return;
    }
    planning_scene_monitor::LockedPlanningSceneRO scene(moveit_cpp_->getPlanningSceneMonitor());
    collision_detection::CollisionRequest req;
    req.contacts = true;
    req.max_contacts = 50;
    req.group_name = planning_group;
    collision_detection::CollisionResult res;
    scene->checkCollision(req, res, *current_state);
    if (!res.collision) {
      return;
    }

    bool attached_octomap_contact_found = false;
    const std::string attached_target = "#" + target_id;
    for (const auto & contact_pair : res.contacts) {
      const auto & first = contact_pair.first.first;
      const auto & second = contact_pair.first.second;
      const bool has_octomap =
        first.find("octomap") != std::string::npos || second.find("octomap") != std::string::npos;
      const bool has_target =
        first.find(target_id) != std::string::npos || second.find(target_id) != std::string::npos ||
        first.find(attached_target) != std::string::npos ||
        second.find(attached_target) != std::string::npos;
      if (has_octomap && has_target) {
        attached_octomap_contact_found = true;
        break;
      }
    }

    if (attached_octomap_contact_found) {
      RCLCPP_ERROR(
        node_->get_logger(),
        "Release planning failed because attached target object is still colliding with "
        "octomap/world geometry. This usually means the picked object's world/octomap "
        "representation was not cleared or allowed after attach.");
    }
  }

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<emd::EndEffectorExecutioninterface> end_effector_interface_;
  emd::EndEffectorExecutionContext ee_context_;
  rclcpp::Subscription<emd_msgs::msg::GraspTask>::SharedPtr grasp_task_sub_;
  rclcpp::Service<emd_msgs::srv::GraspRequest>::SharedPtr grasp_req_service_;
  std::string planning_frame_;
  double release_x_offset_{-0.3};
  bool release_use_grasp_z_{true};
  WorkspaceBounds workspace_bounds_;
  std::vector<std::string> grasp_precheck_allowed_touch_links_;
  std::vector<std::string> grasp_precheck_allowed_collision_ids_;
  std::mutex last_failure_mutex_;
  std::string last_failure_message_;
};

}  // namespace grasp_execution

class DemoLifecycleNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit DemoLifecycleNode(const rclcpp::NodeOptions & options)
  : rclcpp_lifecycle::LifecycleNode("grasp_execution_demo_node", "", options)
  {
    if (!this->has_parameter("workcell_context")) {
      this->declare_parameter<std::string>("workcell_context", "");
    }
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State &)
  {
    // Create a standard rclcpp::Node to interface with MoveIt and other ROS 2 APIs.
    // The lifecycle node itself cannot be directly used where a rclcpp::Node is
    // required, so we construct a new node with a unique name in the same
    // namespace. Parameters are automatically declared from overrides to mirror
    // the lifecycle node behaviour.
    try {
      rclcpp::NodeOptions base_options;
      base_options.automatically_declare_parameters_from_overrides(true);
      const auto base_node_name = std::string(this->get_name()) + "_worker";
      base_node_ = std::make_shared<rclcpp::Node>(
        base_node_name, this->get_namespace(), base_options);
      demo_ = std::make_shared<grasp_execution::Demo>(
        base_node_, grasp_execution::GRASP_EXECUTION_PACKAGE,
        grasp_execution::GRASP_TASK_TOPIC, grasp_execution::GRASP_REQUEST_TOPIC);
      std::string workcell_context_filepath =
        this->get_parameter("workcell_context").as_string();
      if (!std::filesystem::path(workcell_context_filepath).is_absolute()) {
        workcell_context_filepath =
          (std::filesystem::path(
             ament_index_cpp::get_package_share_directory("run_grasp_execution")) /
           workcell_context_filepath)
            .string();
      }
      if (!demo_->init_from_yaml(workcell_context_filepath)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize workcell context from '%s'",
          workcell_context_filepath.c_str());
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
      }
      std::string configured_planning_frame;
      if (this->has_parameter("planning_frame")) {
        this->get_parameter_or<std::string>("planning_frame", configured_planning_frame, "world");
      } else {
        configured_planning_frame = this->declare_parameter<std::string>("planning_frame", "world");
      }
      const auto planning_frame = grasp_execution::sanitize_frame_id(configured_planning_frame);
      if (planning_frame.empty()) {
        RCLCPP_ERROR(
          this->get_logger(),
          "Configured planning_frame is empty after trimming whitespace.");
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
      }
      base_node_->set_parameter(rclcpp::Parameter("planning_frame", planning_frame));

      const auto readiness_profile =
        grasp_execution::detect_grasp_readiness_profile(demo_->get_workcell_context());
      RCLCPP_INFO(
        this->get_logger(),
        "Startup readiness profile selected from workcell context: %s.",
        readiness_profile.description.c_str());

      if (!grasp_execution::validate_required_finger_link_frames_in_robot_description(
          base_node_, readiness_profile.require_robotiq_3f_checks, readiness_profile.description))
      {
        RCLCPP_ERROR(this->get_logger(),
          "URDF finger link validation failed; frame names must exactly match the installed gripper model.");
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
      }
      const auto readiness_timeout_s = this->declare_parameter<int>(
        "startup_readiness_timeout_s", 15);
      if (!grasp_execution::wait_for_required_grasp_readiness(
          base_node_, planning_frame, readiness_profile.require_robotiq_3f_checks,
          readiness_profile.description, std::chrono::seconds(readiness_timeout_s)))
      {
        RCLCPP_ERROR(this->get_logger(),
          "Startup readiness checks failed; aborting configure before enabling octomap/shape masking.");
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
      }
      if (!demo_->start_world_geometry_monitor()) {
        RCLCPP_ERROR(this->get_logger(),
          "Failed to start world geometry monitor after readiness checks.");
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
      }
    } catch (const std::exception & ex) {
      RCLCPP_ERROR(this->get_logger(), "Exception during configure: %s", ex.what());
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
    }
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State &)
  {
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State &)
  {
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State &)
  {
    demo_.reset();
    base_node_.reset();
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp::Node::SharedPtr get_base_node() const
  {
    return base_node_;
  }

private:
  std::shared_ptr<grasp_execution::Demo> demo_;
  rclcpp::Node::SharedPtr base_node_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto node = std::make_shared<DemoLifecycleNode>(node_options);

  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  // Also spin the internal ROS node used by the Demo so that grasp task
  // subscriptions and service callbacks are actually processed.
  if (auto base_node = node->get_base_node()) {
    executor.add_node(base_node->get_node_base_interface());
  }
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
