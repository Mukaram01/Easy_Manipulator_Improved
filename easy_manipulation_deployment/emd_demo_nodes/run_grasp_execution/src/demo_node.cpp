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

bool wait_for_required_grasp_readiness(
  const rclcpp::Node::SharedPtr & node,
  std::chrono::seconds timeout = std::chrono::seconds(15))
{
  std::set<std::string> missing_joints(REQUIRED_FINGER_JOINTS.begin(), REQUIRED_FINGER_JOINTS.end());
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
      if (tf_buffer.canTransform("world", *it, tf2::TimePointZero)) {
        it = missing_frames.erase(it);
      } else {
        ++it;
      }
    }

    if (missing_joints.empty() && missing_frames.empty()) {
      RCLCPP_INFO(node->get_logger(),
        "Readiness checks passed: all required finger joints and world->finger_*_link_* TF frames are available.");
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
    "Missing TF transforms from world->finger_*_link_*: [%s].",
    std::chrono::duration<double>(timeout).count(),
    missing_joint_stream.str().c_str(),
    missing_frame_stream.str().c_str());
  return false;
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

    // Configurable release pose: x-offset from the current EE position and
    // optional z-height override. Defaults preserve the original behaviour.
    // Use declare_or_get_param to avoid ParameterAlreadyDeclaredException when
    // these parameters are pre-declared by automatically_declare_parameters_from_overrides.
    grasp_execution::declare_or_get_param<double>(
      release_x_offset_, "release_x_offset", node, node->get_logger(), -0.3);
    grasp_execution::declare_or_get_param<bool>(
      release_use_grasp_z_, "release_use_grasp_z", node, node->get_logger(), true);

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
        order_schedule(std::move(task), true);
        res->success = true;
      });
  }

  void order_schedule(
    const emd_msgs::msg::GraspTask::SharedPtr & msg,
    bool blocking = false)
  {
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
        bool result;
        planning_scheduler.wait_till_complete(target_id, result);
      }
    }
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
      RCLCPP_ERROR(node_->get_logger(), "Failed to get current robot state for target %s", target_id.c_str());
      return false;
    }

    // Select grasp method based on end effector availability
    double clearance = 0.0;
    std::string ee_link = "";
    std::string planning_group = "";

    const emd_msgs::msg::GraspMethod *selected_method = nullptr;
    for (const auto & method : target->grasp_methods) {
      for (auto & group : get_workcell_context().groups) {
        for (auto & ee : group.second.end_effectors) {
          if (ee.second.brand == method.ee_id) {
            selected_method = &method;
            ee_link = ee.second.link;
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
      RCLCPP_ERROR(node_->get_logger(), "No valid end effector found for target %s", target_id.c_str());
      return false;
    }

    const auto & grasp_method = *selected_method;
    const std::string & ee_brand = grasp_method.ee_id;

    grasp_execution::GraspExecutionContext options;
    options.world_frame = "world";
    options.planning_group = planning_group;
    options.ee_link = ee_link;
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
    if (ee_link.empty()) {
      RCLCPP_ERROR(node_->get_logger(), "End effector brand: %s", ee_brand.c_str());
    }

    geometry_msgs::msg::PoseStamped release_pose;
    try {
      release_pose = get_curr_pose(ee_link);
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
    const geometry_msgs::msg::PoseStamped * selected_grasp_pose = nullptr;

    for (size_t pose_index = 0; pose_index < grasp_method.grasp_poses.size(); ++pose_index) {
      const auto & grasp_pose = grasp_method.grasp_poses[pose_index];

      result = this->plan_and_execute_job(
        options,
        "Grasp location",
        target_id,
        grasp_pose);

      if (result) {
        selected_grasp_pose = &grasp_pose;
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

    if (!selected_grasp_pose) {
      RCLCPP_ERROR(
        node_->get_logger(),
        "All %zu grasp pose candidates failed for target %s.",
        grasp_method.grasp_poses.size(), target_id.c_str());
      return false;
    }
    // ------------------- Attach grasp object to robot --------------------------
    prompt_job_start(
      node_->get_logger(), target_id,
      "Attaching to robot ee frame: [" + ee_link + "]");

    end_effector_interface_->grasp_object(this, ee_link, target_id, ee_context_);

    prompt_job_end(node_->get_logger(), true);

    // Apply configurable release pose offsets. The x_offset is applied to the
    // current EE position. Optionally the z-height is aligned to the grasp
    // pose so the object clears any surface obstacle on approach.
    geometry_msgs::msg::PoseStamped base_grasp_pose;
    to_frame(*selected_grasp_pose, base_grasp_pose, this->robot_frame_);
    release_pose.pose.position.x += release_x_offset_;
    if (release_use_grasp_z_) {
      release_pose.pose.position.z = base_grasp_pose.pose.position.z;
    }
    release_pose.pose.orientation = base_grasp_pose.pose.orientation;

    if(!this->plan_and_execute_job(
        options,
        "Grasp release",
        target_id,
        release_pose)){
          return false;
      }

    // ------------------- detach grasp object from robot --------------------------
    prompt_job_start(
      node_->get_logger(), target_id,
      "Detaching from robot ee frame: [" + ee_link + "]");

    end_effector_interface_->release_object(this, ee_link, target_id, ee_context_);

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
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<emd::EndEffectorExecutioninterface> end_effector_interface_;
  emd::EndEffectorExecutionContext ee_context_;
  rclcpp::Subscription<emd_msgs::msg::GraspTask>::SharedPtr grasp_task_sub_;
  rclcpp::Service<emd_msgs::srv::GraspRequest>::SharedPtr grasp_req_service_;
  double release_x_offset_{-0.3};
  bool release_use_grasp_z_{true};
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
    // required, so we construct a new node that shares the same name and
    // namespace. Parameters are automatically declared from overrides to mirror
    // the lifecycle node behaviour.
    try {
      rclcpp::NodeOptions base_options;
      base_options.automatically_declare_parameters_from_overrides(true);
      base_node_ = std::make_shared<rclcpp::Node>(
        this->get_name(), this->get_namespace(), base_options);
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
      const auto readiness_timeout_s = this->declare_parameter<int>("startup_readiness_timeout_s", 15);
      if (!grasp_execution::wait_for_required_grasp_readiness(
          base_node_, std::chrono::seconds(readiness_timeout_s)))
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
